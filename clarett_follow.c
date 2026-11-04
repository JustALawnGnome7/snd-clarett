// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the deferred workers: the
 * settings persist, the notification relay and front-panel follow.
 */
#include <linux/pci.h>
#include <linux/workqueue.h>
#include "clarett.h"

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
 * `cfg` is the monitor region from clarett_monitor_fetch(): MONITOR_CFG_LEN
 * bytes from MONITOR_CFG_OFFSET, spanning the gains, the HW-enable bits and the
 * knob.
 */
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

		/* a model whose offsets escape the fetched window */
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

/* Fetch the monitor config region into buf (MONITOR_CFG_LEN bytes). */
static int clarett_monitor_fetch(struct clarett *c, u8 *buf)
{
	u8 req[8];		/* GET_DATA {u32 offset, u32 len} */

	clarett_put_le32(req, MONITOR_CFG_OFFSET);
	clarett_put_le32(req + 4, MONITOR_CFG_LEN);

	/* copies the payload out under mbox_lock, unlike reading resp_buf */
	return clarett_fcp_cmd(c, FCP_GET_DATA, req, sizeof(req), buf,
			       MONITOR_CFG_LEN);
}

/*
 * The knob follow, run on an event: scheduled when a relayed notification
 * carries the monitor or dim/mute bit, once when the card comes up (the stored
 * SW gains are whatever the last session left), and when fcp-server commits a
 * SW/HW switch, which the device does not announce. Delayed by
 * CLARETT_NOTIFY_MS and a no-op while already queued, so a turning knob costs
 * at most one region read per CLARETT_NOTIFY_MS.
 */
void clarett_follow_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett,
					 follow_work.work);
	u8 buf[MONITOR_CFG_LEN];

	if (!clarett_monitor_fetch(c, buf))
		clarett_hw_gain_follow(c, buf);
}

/*
 * A model with no output gains to follow (the Red range) has nothing to fetch;
 * on a Red, bits 21-22 are input events, not the monitor and dim/mute ones the
 * caller tests for.
 */
void clarett_schedule_follow(struct clarett *c)
{
	unsigned long delay = msecs_to_jiffies(CLARETT_NOTIFY_MS);

	if (c->model->n_out_gains && READ_ONCE(c->ctl_ready))
		schedule_delayed_work(&c->follow_work, delay);
}
