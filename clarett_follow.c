// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the deferred workers: the
 * settings persist, the notification relay, front-panel follow and the
 * optional meter poll.
 */
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include "clarett.h"

/*
 * Optional periodic GET_METER. The device needs no host heartbeat, and the
 * Level Meter control reads the meters on demand (cached for
 * CLARETT_METER_CACHE_MS), so by default an idle card issues no mailbox
 * commands. The worker also hosts the optional monitor_poll backstop.
 */
static int meter_poll_ms;
module_param(meter_poll_ms, int, 0444);
MODULE_PARM_DESC(meter_poll_ms,
		 "Period (ms) of an optional periodic GET_METER, which also "
		 "drives monitor_poll (default 0: none; the meters are read on "
		 "demand).");

/*
 * Debounced flash persist. clarett_write_u8() schedules this
 * CLARETT_SAVE_DELAY_MS after a control change, so a burst (a slider drag)
 * coalesces into one DATA_CMD{PERSIST}, which writes the device's live config
 * to its own NVRAM. The device owns the state, as with scarlett2: settings
 * survive a power cycle without the host re-applying them.
 */
void clarett_save_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, save_work.work);
	int err = clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);

	if (err)
		dev_warn_ratelimited(&c->pci->dev,
				     "config persist (DATA_CMD{%u}) failed: %d\n",
				     FCP_ACTIVATE_PERSIST, err);
}

void clarett_notify_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, notify_work);
	u32 ev = atomic_xchg(&c->notify_bits, 0);

	if (!ev)
		return;

	/*
	 * The controls live in fcp-server, so a notification is only relayed;
	 * it re-reads whatever it owns.
	 */
	clarett_hwdep_notify(c, ev);
	if (ev & (NOTIFY_EV_MONITOR | NOTIFY_EV_DIM_MUTE))
		clarett_schedule_follow(c);

	dev_dbg(&c->pci->dev, "async notification handled: 0x%x\n", ev);
}

/*
 * Monitor-region change poll, a backstop for a front-panel change that raises
 * no device event: read the monitor region every meter_poll_ms tick (so it
 * needs that set too) and relay, and run hw_gain_follow, only when the bytes
 * change. Off by default, since the event relay covers every front-panel
 * control seen on the Clarett and Red units. The Clarett region (24, len 92)
 * covers the monitor mute/dim flags, the master volume pair at 32/33, the
 * HW-enable bits and the knob; a model can widen it
 * (clarett_model.monitor_cfg_len).
 */
static bool monitor_poll;
module_param(monitor_poll, bool, 0644);
MODULE_PARM_DESC(monitor_poll,
		 "Poll the monitor config region and act when it changes: "
		 "relay a notification and drive hw_gain_follow. A backstop "
		 "for changes that raise no device event; runs in the "
		 "meter_poll_ms worker, so needs that set too (default off).");

/*
 * Keep the SW gain of every output under HARDWARE control equal to the
 * front-panel knob.
 *
 * The device applies the knob in its own signal path and never writes it back:
 * turning it moves byte 112 only, and the per-output gains (out_gains[].offset)
 * stay wherever software left them. So the GUI's fader would not follow the
 * knob, and switching an output from HW to SW would jump to a stale volume.
 * (The USB models' scarlett2 driver synthesises the same link.)
 *
 * Writing the knob's value into each HW-controlled output's SW gain fixes both
 * with no userspace change: fcp-server re-reads the byte, and the stored value
 * already matches when control returns to software. The write is inaudible
 * while the knob owns the level.
 *
 * Not persisted (clarett_write_u8_nosave): it mirrors the knob rather than
 * recording user intent, and committing the flash on every knob movement would
 * wear it. Writes are change-gated, so a stationary knob costs nothing.
 *
 * `cfg` is the monitor region from clarett_monitor_fetch(): offset
 * MONITOR_CFG_OFFSET, at least MONITOR_CFG_LEN long, spanning the gains, the
 * HW-enable bits and the knob.
 */
static bool hw_gain_follow = true;
module_param(hw_gain_follow, bool, 0644);
MODULE_PARM_DESC(hw_gain_follow,
		 "Track the front-panel volume knob into the SW gain of "
		 "every output set to HW, so the GUI fader follows it and a "
		 "HW->SW toggle does not jump (default on; matches the USB "
		 "models' scarlett2 behaviour). 0 leaves the stored SW gains "
		 "untouched.");

/*
 * Byte `off` as fetched into the monitor-region buffer, or -1 if it falls
 * outside that window.
 */
static int clarett_cfg_byte(const u8 *cfg, u32 off)
{
	if (off < MONITOR_CFG_OFFSET ||
	    off >= MONITOR_CFG_OFFSET + MONITOR_CFG_LEN)
		return -1;
	return cfg[off - MONITOR_CFG_OFFSET];
}

static void clarett_hw_gain_follow(struct clarett *c, const u8 *cfg)
{
	int mon = clarett_cfg_byte(cfg, MONITOR_VOLUME_OFFSET);
	int i;

	if (mon < 0)
		return;

	for (i = 0; i < c->model->n_out_gains; i++) {
		u32 gain_off = c->model->out_gains[i].offset;
		u32 hwen_off = HWEN_GAIN_OFFSET + (i / 2) * 4;
		u8 hwen_bit = 1u << (i % 2);
		int hwen = clarett_cfg_byte(cfg, hwen_off);
		int gain = clarett_cfg_byte(cfg, gain_off);
		int err;

		/* a model whose offsets escape the polled window */
		if (hwen < 0 || gain < 0)
			continue;
		/*
		 * Only outputs whose enable-hardware-gain bit is set; the rest
		 * are software's.
		 */
		if (!(hwen & hwen_bit))
			continue;
		if (gain == mon)
			continue;

		err = clarett_write_u8_nosave(c, gain_off, mon,
					      OUT_GAIN_ACTIVATE);
		if (err) {
			dev_warn_ratelimited(&c->pci->dev,
					     "hw-gain follow: writing output %d gain @%u failed: %d\n",
					     i, gain_off, err);
			return;
		}
		dev_dbg(&c->pci->dev,
			"hw-gain follow: output %d gain @%u -> 0x%02x\n",
			i, gain_off, mon);
	}
}

/*
 * Meter-source follow (8PreX). fcp-server's "Meter Source" enum writes only the
 * selector byte @184, but the front-panel meters are routed by the per-source
 * channel tables @136/146/156, which the device does not derive from it:
 * without this the selector LED moves while the meters keep showing the old
 * bank. Write the selected source's three per-band tables and commit with
 * activate 8, which also commits the selector byte fcp-server staged. No-op on
 * models with no meter-source control and for an unknown source value. Called
 * from the hwdep CMD path on a SET_DATA @184.
 */
void clarett_meter_source_follow(struct clarett *c, u8 source)
{
	const struct clarett_model *m = c->model;
	const struct clarett_meter_source *ms = NULL;
	int i, err;

	for (i = 0; i < m->n_meter_sources; i++)
		if (m->meter_sources[i].value == source) {
			ms = &m->meter_sources[i];
			break;
		}
	if (!ms)
		return;

	err = clarett_set_data(c, METER_TABLE_L_OFFSET, METER_TABLE_LEN,
			       ms->tbl[0]);
	if (!err)
		err = clarett_set_data(c, METER_TABLE_M_OFFSET,
				       METER_TABLE_LEN, ms->tbl[1]);
	if (!err)
		err = clarett_set_data(c, METER_TABLE_H_OFFSET,
				       METER_TABLE_LEN, ms->tbl[2]);
	if (!err)
		err = clarett_data_cmd(c, METER_SOURCE_ACTIVATE);
	if (err)
		dev_warn_ratelimited(&c->pci->dev,
				     "meter-source follow (%s) failed: %d\n",
				     ms->name, err);
	else
		dev_dbg(&c->pci->dev, "meter-source follow: %s (0x%02x)\n",
			ms->name, source);
}

/*
 * Fetch the monitor config region into buf (MONITOR_CFG_MAX_LEN). Returns its
 * length, or <0.
 */
static int clarett_monitor_fetch(struct clarett *c, u8 *buf)
{
	u32 len = c->model->monitor_cfg_len ? : MONITOR_CFG_LEN;
	u8 req[8];		/* GET_DATA {u32 offset, u32 len} */
	int err;

	if (WARN_ON_ONCE(len > MONITOR_CFG_MAX_LEN))
		len = MONITOR_CFG_MAX_LEN;

	clarett_put_le32(req, MONITOR_CFG_OFFSET);
	clarett_put_le32(req + 4, len);

	/* copies the payload out under mbox_lock, unlike reading resp_buf */
	err = clarett_fcp_cmd(c, FCP_GET_DATA, req, sizeof(req), buf, len);
	return err ? err : len;
}

/*
 * hw_gain_follow on an event. Scheduled when a relayed notification carries the
 * monitor or dim/mute bit, once when the card comes up (the stored SW gains are
 * whatever the last session left), and when fcp-server commits a SW/HW switch,
 * which the device does not announce. Delayed by notify_ms and a no-op while
 * already queued, so a turning knob (dozens of events a second) costs at most
 * one region read per notify_ms.
 */
void clarett_follow_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett,
					 follow_work.work);
	u8 buf[MONITOR_CFG_MAX_LEN];

	if (hw_gain_follow && clarett_monitor_fetch(c, buf) > 0)
		clarett_hw_gain_follow(c, buf);
}

/*
 * A model with no output gains to follow (the Red range) has nothing to fetch;
 * on a Red, bits 21-22 are input events, not the monitor and dim/mute ones the
 * caller tests for.
 */
void clarett_schedule_follow(struct clarett *c)
{
	unsigned long delay = msecs_to_jiffies(clarett_hwdep_notify_ms());

	if (hw_gain_follow && c->model->n_out_gains && READ_ONCE(c->ctl_ready))
		schedule_delayed_work(&c->follow_work, delay);
}

static void clarett_monitor_poll(struct clarett *c)
{
	u8 buf[MONITOR_CFG_MAX_LEN];
	bool changed, first;
	int len;

	len = clarett_monitor_fetch(c, buf);
	if (len < 0) {
		dev_dbg(&c->pci->dev, "monitor poll: GET_DATA failed: %d\n",
			len);
		return;
	}

	first = !c->mon_snap_valid;
	changed = !first && memcmp(c->mon_snap, buf, len);
	if (changed) {
		/* Which bytes moved: "@off old->new", first few only. */
		char why[96];
		int n = 0, shown = 0;
		u32 i;

		why[0] = 0;
		for (i = 0; i < len && shown < 6; i++)
			if (c->mon_snap[i] != buf[i]) {
				n += scnprintf(why + n, sizeof(why) - n,
					       " @%u %02x->%02x",
					       MONITOR_CFG_OFFSET + i,
					       c->mon_snap[i], buf[i]);
				shown++;
			}
		dev_dbg(&c->pci->dev, "monitor poll: region changed:%s\n", why);
	}
	memcpy(c->mon_snap, buf, len);
	c->mon_snap_valid = true;

	if (!first && !changed)
		return;

	if (changed) {
		/*
		 * Usually the event relay got there first; a duplicate costs
		 * fcp-server one re-read. This catches a change that raised no
		 * event (the Red's Meter Source is one).
		 */
		clarett_hwdep_notify(c, NOTIFY_EV_MONITOR | NOTIFY_EV_DIM_MUTE);
	}

	/*
	 * Also on the FIRST poll: until the knob next moves, the stored SW
	 * gains are whatever the last session left, and a HW->SW switch would
	 * jump.
	 */
	if (hw_gain_follow)
		clarett_hw_gain_follow(c, buf);
}

/* Start the optional periodic GET_METER, if meter_poll_ms asks for one. */
void clarett_meter_poll_start(struct clarett *c)
{
	if (meter_poll_ms > 0)
		schedule_delayed_work(&c->meter_work,
				      msecs_to_jiffies(meter_poll_ms));
}

void clarett_meter_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, meter_work.work);
	int n = c->hwdep_n_meter_slots;

	if (meter_poll_ms > 0) {
		if (n > 0 && c->hwdep_meter_levels) {
			/*
			 * The meter control exists (fcp-server set the map):
			 * fill its cache and stamp hwdep_meter_polled, so the
			 * control's .get serves the cache instead of sending a
			 * GET_METER of its own.
			 */
			/* {pad:u16=0, num_meters:u16, magic:u32=1} */
			u8 req[8];

			clarett_put_le16(req, 0);
			clarett_put_le16(req + 2, n);
			clarett_put_le32(req + 4, 1);
			mutex_lock(&c->hwdep_lock);
			if (!clarett_fcp_cmd(c, FCP_GET_METER, req, sizeof(req),
					     (u8 *)c->hwdep_meter_levels,
					     n * sizeof(__le32)))
				c->hwdep_meter_polled = jiffies ? jiffies : 1;
			mutex_unlock(&c->hwdep_lock);
		} else {
			/* no meter control yet: a bare GET_METER */
			static const u8 meter_req[8] = {
				0x00, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00
			};

			clarett_fcp(c, FCP_GET_METER, meter_req,
				    sizeof(meter_req));
		}

		/* the optional backstop, streaming or not */
		if (monitor_poll && READ_ONCE(c->ctl_ready))
			clarett_monitor_poll(c);

		clarett_meter_poll_start(c);
	}
}
