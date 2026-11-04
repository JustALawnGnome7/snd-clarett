// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — PCM data plane (full
 * duplex).
 *
 * The hardware is ONE full-duplex DMA engine over two ring blocks in a single
 * contiguous coherent buffer (c->stream_buf): block 0 (0x200 = TX/playback)
 * then block 1 (0x300 = RX/capture). It raises a period on the 0x300 cause
 * register and waits for the host to ACK by reading it;
 * clarett_stream_service() (clarett_engine.c) is that ACK loop and the shared
 * frame clock for both directions. Each 0x300 event carries the frames the
 * engine advanced (ctr delta * CLARETT_CTR_FRAMES); clarett_pcm_tick() then
 * drains the RX ring into the capture ALSA buffer (behind the write pointer)
 * and refills the TX ring from the playback ALSA buffer (ahead of the read
 * pointer).
 *
 * Two ALSA substreams share the one engine: whichever prepares first arms it
 * full-duplex, the other attaches at the current clock. The engine clocks only
 * with both rings in one contiguous buffer, so TX is armed even for a capture
 * alone (it plays silence until a playback stream fills it).
 */
#include <linux/dma-mapping.h>
#include <linux/math64.h>
#include <linux/log2.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <sound/core.h>
/* snd_kcontrol_new — the driver-owned "Clock Source" control */
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "clarett.h"

/* rebuilt at prepare when the period changes cadence */
static void clarett_build_rings(struct clarett *c);

/*
 * Period-derived cadence: the RX IRQ cadence follows the negotiated ALSA
 * period, so a DAW can pick a small device buffer (down to one 16-frame
 * fragment). Prepare rebuilds the descriptor ring at the matching marker
 * cadence; both directions are locked to one period (DAWs use a single duplex
 * buffer size anyway).
 *
 * The 0x300 counter free-runs in 16-frame units regardless of marker spacing,
 * so the servicer's advance is correct at every cadence, and the counter's
 * wrap/recovery window stays 4096 frames (~85 ms) however fine the period. The
 * floor is one 16-frame fragment (CLARETT_DYN_MIN_FRAMES, every descriptor
 * IRQ-flagged), the hardware minimum. With PipeWire holding the card, the
 * duplex lock holds an app to PipeWire's period.
 *
 * The period is only half of what an app feels as latency; the buffer is
 * bounded too — a power-of-two fraction of the ring down to
 * CLARETT_MIN_BUFFER_FRAMES, and at most CLARETT_MAX_PERIODS of the app's own
 * period.
 */
/* one fragment = 16 frames (cadence 1): the floor */
#define CLARETT_DYN_MIN_FRAMES	CLARETT_FRAG_FRAMES

/*
 * The clock source sent with SET_CLOCK at each stream arm and whenever the
 * "Clock Source" control changes (the enums are in clarett.h). On an external
 * source the PCM rate must match the external clock's.
 */
static int clarett_clock_source(struct clarett *c)
{
	return READ_ONCE(c->clock_src);
}

/*
 * "Clock Source", the one control this driver owns. The clock source is not a
 * config byte: it exists only in the SET_CLOCK payload {u32 rate, u32 source},
 * and only this driver knows the rate. alsa-scarlett-gui renders any element
 * named "Clock Source" as a drop-down. Its value is per card, kept across
 * reloads and re-attaches by alsactl's saved state.
 */
static int clarett_clock_src_info(struct snd_kcontrol *kctl,
				  struct snd_ctl_elem_info *uinfo)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;

	unsigned int i;

	/*
	 * Not snd_ctl_enum_info(): the names live in a struct array, not a flat
	 * char* table.
	 */
	uinfo->type  = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = m->n_clock_srcs;
	i = uinfo->value.enumerated.item;
	if (i >= m->n_clock_srcs)
		i = m->n_clock_srcs - 1;
	uinfo->value.enumerated.item = i;
	strscpy(uinfo->value.enumerated.name, m->clock_srcs[i].name,
		sizeof(uinfo->value.enumerated.name));
	return 0;
}

static int clarett_clock_src_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;
	int cur = clarett_clock_source(c);
	u8 i;

	for (i = 0; i < m->n_clock_srcs; i++) {
		if (m->clock_srcs[i].value == cur) {
			ucontrol->value.enumerated.item[0] = i;
			return 0;
		}
	}
	/* only listed values are stored, so this is not reached */
	ucontrol->value.enumerated.item[0] = 0;
	return 0;
}

static int clarett_clock_src_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;
	unsigned int i = ucontrol->value.enumerated.item[0];
	u32 rate = READ_ONCE(c->cur_rate);
	u8 clk[8];
	int value;

	if (i >= m->n_clock_srcs)
		return -EINVAL;
	value = m->clock_srcs[i].value;
	if (value == clarett_clock_source(c))
		return 0;

	WRITE_ONCE(c->clock_src, value);

	/*
	 * Apply it at once, streaming or not, at the rate the device runs
	 * (cur_rate): a desktop holds a PCM open permanently, so waiting for
	 * the next arm would leave the control without effect. An absent
	 * external source then drops Sync Status to Unlocked, which is the
	 * device reporting the truth. Each arm re-sends the stored source.
	 */
	clarett_put_le32(clk, rate ? rate : CLARETT_DEFAULT_RATE);
	clarett_put_le32(clk + 4, value);
	if (clarett_fcp(c, FCP_SET_CLOCK, clk, sizeof(clk)))
		dev_dbg(&c->pci->dev, "clock source %s: SET_CLOCK failed\n",
			m->clock_srcs[i].name);
	return 1;
}

static const struct snd_kcontrol_new clarett_clock_src_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name  = "Clock Source",
	.info  = clarett_clock_src_info,
	.get   = clarett_clock_src_get,
	.put   = clarett_clock_src_put,
};

int clarett_add_clock_control(struct clarett *c)
{
	if (!c->model->n_clock_srcs)
		return 0;
	return snd_ctl_add(c->card, snd_ctl_new1(&clarett_clock_src_ctl, c));
}

/*
 * The ring geometry is derived per model at runtime (clarett.h,
 * clarett_pcm_rx_samples() &c.).
 */

/*
 * Constant capability template; the per-model geometry fields (channels,
 * buffer/period bytes, periods_max) and the rate set (rates/rate_min/rate_max,
 * per the model's max_rate) are filled in clarett_pcm_open().
 *
 * BATCH: .pointer advances a whole period at a time, not continuously; without
 * the flag a timer-scheduled client (PipeWire outside Pro Audio) keeps less
 * than a period queued and underruns every period.
 */
static const struct snd_pcm_hardware clarett_pcm_hw = {
	.info             = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			    SNDRV_PCM_INFO_MMAP_VALID |
			    SNDRV_PCM_INFO_BLOCK_TRANSFER |
			    SNDRV_PCM_INFO_BATCH,
	/* interleaved, 24-bit MSB-justified */
	.formats          = SNDRV_PCM_FMTBIT_S32_LE,
	.periods_min      = 2,
};

/*
 * Pointer to the block-1 (capture) RX sample area inside the contiguous
 * hardware buffer: [TX table][TX samples][RX table][RX samples].
 */
static u8 *clarett_rx_area(struct clarett *c)
{
	return (u8 *)c->stream_buf + clarett_stream_rx_off(c);
}

/* RX capture sample-ring size in bytes (the largest capture ALSA buffer). */
static size_t clarett_rx_ring_bytes(struct clarett *c)
{
	return clarett_stream_rx_area_bytes(c);
}

/*
 * Pointer to the block-0 (playback) TX sample area, and its size (== the
 * playback ALSA buffer).
 */
static u8 *clarett_tx_area(struct clarett *c)
{
	return (u8 *)c->stream_buf + clarett_stream_tx_off(c);
}

static size_t clarett_tx_ring_bytes(struct clarett *c)
{
	return clarett_stream_tx_area_bytes(c);
}

/*
 * Silence the whole TX device area (all slots, padding included), so an idle
 * playback side never loops stale audio out of the DACs.
 */
static void clarett_zero_tx(struct clarett *c)
{
	memset(clarett_tx_area(c), 0, clarett_pcm_tx_dev_bytes(c));
}

/* STREAM_INFO speed band for a sample rate: 0 single, 1 double, 2 quad. */
static unsigned int clarett_speed_band(unsigned int rate)
{
	return rate > 96000 ? 2 : rate > 48000 ? 1 : 0;
}

/*
 * How many leading capture channels does the device actually write at this
 * rate? ADAT S/MUX removes the upper ADAT channels at double and quad speed (8
 * -> 4 -> 2 per port); the frame stride is unchanged, so the removed channels
 * are a contiguous tail of the frame. Read from the device at probe; see
 * clarett.rx_live[].
 */
static u8 clarett_rx_live_channels(struct clarett *c, unsigned int rate)
{
	return min(c->rx_live[clarett_speed_band(rate)],
		   c->model->capture_channels);
}

/*
 * Latch how much of each capture frame the device actually fills at this rate,
 * for clarett_rx_drain() to blank on the way out. Called from prepare, once the
 * rate is negotiated.
 *
 * The engine keeps writing a sparse residue (one sample every 32 frames) into
 * some removed channels, so the blanking has to happen per period, on the
 * frames handed to ALSA.
 */
static void clarett_set_rx_live(struct clarett *c, unsigned int rate)
{
	u8 chans = c->model->capture_channels;
	u8 live  = clarett_rx_live_channels(c, rate);

	WRITE_ONCE(c->rx_dead_bytes, (u32)(chans - live) * 4);
	WRITE_ONCE(c->rx_live_bytes, (u32)live * 4);
	dev_dbg(&c->pci->dev, "rx: %u of %u capture channels live at %u Hz\n",
		live, chans, rate);
}

/*
 * Drain nframes of captured audio from the RX device area (starting at hardware
 * ring frame `pos`) into the contiguous ALSA buffer (starting at that stream's
 * own frame `apos` — the two differ by the capture direction's attach base
 * and wrap independently). The RX area is a table of NDESC fragment SLOTS of
 * c->rx_slot bytes; ring frame f lives in slot (f/FRAG_FRAMES) at byte
 * (f%FRAG_FRAMES)*frame within that slot, so with page-safe padding the copy
 * gathers per fragment across the gaps. FRAG_FRAMES divides the ring, so a
 * chunk clipped to the fragment boundary also handles the ring wrap.
 *
 * `abuf` is the ALSA buffer in frames, which is NOT the ring: it is a
 * power-of-two divisor of it, so the destination wraps one or more times per
 * ring pass and has to be clipped separately from the source.
 *
 * On models with clarett_model.rx_loopback_at set, each frame is reordered on
 * the way: the loopback pair moves to the end and the channels after it move
 * down two, so ALSA channel n is the router's PCM n.
 */
#define CLARETT_LOOPBACK_BYTES	(2 * 4)

static void clarett_rx_drain(struct clarett *c, u8 *alsa, u32 apos, u32 pos,
			     u32 nframes, u32 abuf)
{
	u8 *ring = clarett_rx_area(c);
	u32 frame = (u32)c->model->capture_channels * 4;
	u32 slot  = c->rx_slot;
	u32 ring_frames = CLARETT_STREAM_NDESC * CLARETT_FRAG_FRAMES;
	/* S/MUX-removed tail; see clarett_set_rx_live() */
	u32 dead = READ_ONCE(c->rx_dead_bytes);
	u32 live = READ_ONCE(c->rx_live_bytes);
	/* loopback pair's byte offset in a device frame */
	u32 lb   = (u32)c->model->rx_loopback_at * 4;

	/*
	 * With the reorder, the dead device tail lands two channels lower in
	 * the ALSA frame (the loopback pair, always live, now follows it).
	 */
	if (lb)
		live -= CLARETT_LOOPBACK_BYTES;

	while (nframes) {
		/* frame within its fragment */
		u32 fio   = pos % CLARETT_FRAG_FRAMES;
		/* up to the fragment (and ring) boundary */
		u32 chunk = min(nframes, CLARETT_FRAG_FRAMES - fio);
		u8 *src, *dst;

		/* and up to the ALSA buffer wrap */
		chunk = min(chunk, abuf - apos);
		src = ring + (size_t)(pos / CLARETT_FRAG_FRAMES) * slot +
		      (size_t)fio * frame;
		dst = alsa + (size_t)apos * frame;

		if (!lb) {
			memcpy(dst, src, (size_t)chunk * frame);
		} else {
			/* channels after the pair */
			u32 tail = frame - lb - CLARETT_LOOPBACK_BYTES;
			u8 *s = src, *d = dst;
			u32 i;

			for (i = 0; i < chunk; i++, s += frame, d += frame) {
				memcpy(d, s, lb);
				memcpy(d + lb, s + lb + CLARETT_LOOPBACK_BYTES,
				       tail);
				memcpy(d + lb + tail, s + lb,
				       CLARETT_LOOPBACK_BYTES);
			}
		}

		/* blank the channels S/MUX removed (see the residue above) */
		if (dead) {
			u8 *d = dst + live;
			u32 i;

			for (i = 0; i < chunk; i++, d += frame)
				memset(d, 0, dead);
		}
		pos += chunk;
		if (pos == ring_frames)
			pos = 0;
		apos += chunk;
		if (apos == abuf)
			apos = 0;
		nframes -= chunk;
	}
}

/*
 * Fill nframes of playback audio into the TX device area (starting at hardware
 * ring frame `pos`) from the contiguous ALSA playback buffer (starting at that
 * stream's own frame `apos`): the mirror of clarett_rx_drain. Ring frame f
 * lives in slot (f/ff) at byte (f%ff)*frame, where ff is the TX fragment (16
 * frames, fewer on a stream too wide for the engine's per-descriptor read,
 * clarett_tx_frag_frames()).
 *
 * `abuf` is the ALSA buffer in frames, a power-of-two divisor of the ring, so
 * the source wraps within a fill and is clipped separately. A runway longer
 * than the buffer TILES it around the ring, and since abuf divides the ring,
 * ring frame f is always sourced from the buffer frame due when the engine
 * reaches f.
 *
 * A NULL `alsa` writes silence instead (apos/abuf still bound the chunks,
 * harmlessly).
 */
static void clarett_tx_fill(struct clarett *c, const u8 *alsa, u32 apos,
			    u32 pos, u32 nframes, u32 abuf)
{
	u8 *ring = clarett_tx_area(c);
	u32 frame = (u32)c->model->playback_channels * 4;
	u32 slot  = c->tx_slot;
	/* 16, or fewer on a wide stream */
	u32 ff    = clarett_tx_frag_frames(c->model->playback_channels);
	u32 ring_frames = CLARETT_STREAM_NDESC * ff;

	while (nframes) {
		/* frame within its fragment */
		u32 fio   = pos % ff;
		/* up to the fragment (and ring) boundary */
		u32 chunk = min(nframes, ff - fio);
		u8 *dst = ring + (size_t)(pos / ff) * slot +
			  (size_t)fio * frame;

		/* and up to the ALSA buffer wrap */
		chunk = min(chunk, abuf - apos);

		if (alsa)
			memcpy(dst, alsa + (size_t)apos * frame,
			       (size_t)chunk * frame);
		else
			memset(dst, 0, (size_t)chunk * frame);
		pos += chunk;
		if (pos == ring_frames)
			pos = 0;
		apos += chunk;
		if (apos == abuf)
			apos = 0;
		nframes -= chunk;
	}
}

/*
 * Guard: keep the playback fill this many frames clear of the engine's current
 * TX read position, so a DMA read is never torn. One fragment (the unit the
 * engine reads) is enough: the fill covers the whole ring ahead of the read,
 * so near the read position it rewrites frames with the data they already
 * hold. A larger guard only loses audio from a client keeping a short queue: a
 * frame written less than the guard ahead of the read is never copied and
 * plays as silence. It is not a latency term (the buffer and the reported
 * delay do not change with it) but a deadline: a frame has to be in the ALSA
 * buffer this many frames, plus up to one period, before the engine reaches
 * its slot.
 */
#define CLARETT_TX_GUARD_FRAMES	CLARETT_FRAG_FRAMES		/* 16 frames */

/*
 * Smallest ALSA buffer offered. The buffer is a power-of-two divisor of the
 * 4096-frame ring, which keeps ALSA frame k and ring frame k wrapping
 * together. 128 frames (2.7 ms at 48 kHz) is a clean floor, and the one
 * CLARETT_MAX_PERIODS works against.
 */
#define CLARETT_MIN_BUFFER_FRAMES	128

/*
 * Largest ALSA buffer offered, in frames; 0 (the default) = the whole
 * 4096-frame ring. An optional hard cap below what the period rule allows.
 * Latency is bounded by that rule (CLARETT_MAX_PERIODS): alsa-lib resolves
 * BUFFER_SIZE with set_last, so an app that pins only the period would
 * otherwise be handed the largest buffer allowed.
 */
static unsigned int max_buffer;
module_param(max_buffer, uint, 0644);
MODULE_PARM_DESC(max_buffer,
		 "Largest ALSA buffer offered, in frames (rounded down to a "
		 "power of two, floored at 128; default 0 = the full ring: "
		 "4096 frames, 2048 on a Red). Buffers are otherwise limited "
		 "to four periods of the app's own period, so this is only "
		 "needed as a hard cap.");

/*
 * The advertised buffer ceiling in frames: the ring, or the max_buffer override
 * rounded down to a power of two so the ceiling is itself an attainable value
 * under the pow2 constraint.
 */
static u32 clarett_buffer_max_frames(u32 ring_frames)
{
	u32 want = READ_ONCE(max_buffer);

	if (!want || want >= ring_frames)
		return ring_frames;
	if (want < CLARETT_MIN_BUFFER_FRAMES)
		want = CLARETT_MIN_BUFFER_FRAMES;
	return rounddown_pow_of_two(want);
}

/*
 * Playback fill: refresh the whole runway ahead of the engine (from GUARD past
 * the read position to just before it), so a lagged tick can never underfill
 * and the current read is never torn. Caller holds pcm_lock. Runs on every
 * period tick AND whenever the app has written since the last fill (.ack ->
 * tx_dirty -> clarett_pcm_tx_refill).
 *
 * The tick alone is not enough: it runs BEFORE period_elapsed, so the app's new
 * audio for the region the engine plays next lands only after it. Refilling
 * mid-period only rewrites frames with the data they already hold, so it
 * cannot tear anything.
 */
/*
 * First fill after START. If the engine has already reached the guard window of
 * frame 0 — START landed just before a period event and the servicer only got
 * here after it — frame 0 can no longer be written in time. Push the anchor
 * on by whole periods (staying on an event boundary) while .pointer still
 * reports 0, so hw_ptr never steps backwards. Rare, and it costs one period of
 * start latency instead of the stream's first frames.
 */
static void clarett_play_prime(struct clarett *c,
			       struct snd_pcm_substream *ps, u32 guard)
{
	if (c->play_primed)
		return;
	c->play_primed = true;
	while (c->pcm_frames <= c->play_base &&
	       c->pcm_frames + guard > c->play_base)
		WRITE_ONCE(c->play_base,
			   c->play_base + ps->runtime->period_size);
}

/*
 * How many frames the app has written past engine frame `from` (its appl_ptr
 * distance), or 0 if it is behind the engine (underrun).
 */
static snd_pcm_uframes_t clarett_play_ahead(struct clarett *c,
					    struct snd_pcm_substream *ps,
					    u64 from, u32 abuf)
{
	snd_pcm_uframes_t boundary = ps->runtime->boundary;
	snd_pcm_uframes_t appl = READ_ONCE(ps->runtime->control->appl_ptr);
	u64 spos;
	snd_pcm_uframes_t ahead;

	/* this stream's frame at `from` */
	div64_u64_rem(from - c->play_base, boundary, &spos);
	ahead = appl >= spos ? appl - spos : appl + boundary - spos;
	/* the app is behind the engine (underrun): nothing to copy */
	if (ahead > abuf)
		ahead = 0;
	return ahead;
}

static void clarett_play_fill(struct clarett *c)
{
	struct snd_pcm_substream *ps = c->pcm_play_sub;
	u32 frame, ring, abuf, guard, n, written;
	u64 from, q, aq;
	u32 start, astart;

	/* before the copy: an .ack that lands during it re-arms */
	atomic_set(&c->tx_dirty, 0);
	if (!ps || !ps->runtime->dma_area)
		return;
	/*
	 * Stopped (drained or dropped): silence the runway the fill queued, or
	 * the engine can wrap back into the last period before close does it.
	 * A drain stops only once the engine has read every frame. Done before
	 * the running test, so a quick restart (xrun recovery) also starts from
	 * a silent ring.
	 */
	if (READ_ONCE(c->tx_silence)) {
		WRITE_ONCE(c->tx_silence, false);
		clarett_zero_tx(c);
	}
	/*
	 * Pairs with the smp_store_release() in clarett_pcm_trigger(): once
	 * the stream is seen running, the play_base it was anchored at is
	 * visible too.
	 */
	if (!smp_load_acquire(&c->play_running))
		return;

	frame = (u32)c->model->playback_channels * 4;
	ring  = clarett_tx_ring_bytes(c) / frame;
	abuf  = ps->runtime->buffer_size;	/* <= ring, and divides it */
	guard = CLARETT_TX_GUARD_FRAMES;
	if (ring <= guard)
		return;

	clarett_play_prime(c, ps, guard);

	/*
	 * The whole runway, from the guard to just behind the read position —
	 * but only as far as the app has actually written (appl_ptr); the rest
	 * is silence. Copying past appl_ptr would queue whatever the buffer
	 * held a lap earlier, and the engine plays that whenever the app falls
	 * behind or stops: when a stream ends, the last period again, one
	 * buffer later. Right after START the anchor lies ahead of the runway's
	 * start; begin there, leaving the frames before frame 0 silent.
	 */
	from = c->pcm_frames + guard;
	n = ring - guard;
	if (c->play_base > from) {
		u64 lead = c->play_base - from;

		if (lead >= n)
			return;
		from = c->play_base;
		n -= lead;
	}
	written = min_t(u32, n, clarett_play_ahead(c, ps, from, abuf));
	q = from;
	aq = from - c->play_base;
	/* hardware ring position */
	start = do_div(q, ring);
	/* same frame in the ALSA buffer */
	astart = do_div(aq, abuf);

	clarett_tx_fill(c, ps->runtime->dma_area, astart, start, written, abuf);
	if (written < n) {
		q = from + written;
		clarett_tx_fill(c, NULL, 0, do_div(q, ring), n - written, abuf);
	}
}

/*
 * Servicer, between period events: refill if the app has written since the last
 * fill.
 */
void clarett_pcm_tx_refill(struct clarett *c)
{
	if (!atomic_read(&c->tx_dirty))
		return;
	mutex_lock(&c->pcm_lock);
	clarett_play_fill(c);
	mutex_unlock(&c->pcm_lock);
}

/*
 * Capture drain: the add_frames the engine just wrote, from the current ring
 * position (slot-aware) to the matching position in the stream's own ALSA
 * buffer (offset by where it attached).
 *
 * Clamped to the ALSA buffer and skipped forward to the newest n frames: after
 * a stall longer than the buffer the app has lost the rest anyway (ALSA xruns
 * on the hw_ptr jump), but it gets the most recent audio.
 */
static void clarett_cap_drain(struct clarett *c, struct snd_pcm_substream *cs,
			      u32 add_frames)
{
	u32 frame = (u32)c->model->capture_channels * 4;
	u32 ring  = clarett_rx_ring_bytes(c) / frame;
	/* <= ring, and divides it */
	u32 abuf  = cs->runtime->buffer_size;
	u32 n     = min(add_frames, abuf);
	/* lag longer than the buffer: keep the NEWEST n */
	u32 skip  = add_frames - n;
	u64 q = c->pcm_frames + skip;
	u64 aq = c->pcm_frames + skip - c->pcm_base;
	u32 pos = do_div(q, ring);
	u32 apos = do_div(aq, abuf);

	clarett_rx_drain(c, cs->runtime->dma_area, apos, pos, n, abuf);
}

/*
 * Whether a stream `frames` into its own clock has entered a new period since
 * *last; records the new period index if so.
 */
static bool clarett_period_crossed(u64 frames, snd_pcm_uframes_t period_size,
				   u64 *last)
{
	u64 period = div_u64(frames, period_size);

	if (period == *last)
		return false;
	*last = period;
	return true;
}

/*
 * From the servicer on every 0x300 period event, with the frames the engine
 * advanced since the last one. One engine clock drives both directions: copy
 * the frames the engine just WROTE from the RX ring to the capture buffer, and
 * refill the TX ring AHEAD of its read pointer from the playback buffer.
 *
 * pcm_lock serialises the copies against hw_free, which clears the substream
 * pointer and frees the ALSA buffer; period_elapsed is called after unlocking
 * (the substream itself lives until close).
 */
void clarett_pcm_tick(struct clarett *c, u32 add_frames)
{
	struct snd_pcm_substream *cs, *ps;
	bool cap_elapsed = false, play_elapsed = false;

	if (!READ_ONCE(c->stream_run) || !add_frames)
		return;

	mutex_lock(&c->pcm_lock);

	cs = c->pcm_sub;
	ps = c->pcm_play_sub;

	if (cs && cs->runtime->dma_area)
		clarett_cap_drain(c, cs, add_frames);

	c->pcm_frames += add_frames;

	/* fill from where the engine is NOW, after the advance */
	clarett_play_fill(c);

	/*
	 * Deliver period boundaries only between trigger START and STOP (the
	 * *_running gates). Period indices are counted on each direction's own
	 * clock (pcm_frames - base), matching what .pointer reports.
	 */
	if (cs && READ_ONCE(c->pcm_running))
		cap_elapsed =
			clarett_period_crossed(c->pcm_frames - c->pcm_base,
					       cs->runtime->period_size,
					       &c->pcm_last_period);
	if (ps && READ_ONCE(c->play_running) && c->pcm_frames >= c->play_base)
		play_elapsed =
			clarett_period_crossed(c->pcm_frames - c->play_base,
					       ps->runtime->period_size,
					       &c->play_last_period);

	mutex_unlock(&c->pcm_lock);

	/*
	 * Outside the lock (period_elapsed takes the stream lock; the
	 * substreams live until close).
	 */
	if (cap_elapsed)
		snd_pcm_period_elapsed(cs);
	if (play_elapsed)
		snd_pcm_period_elapsed(ps);
}

/*
 * Duplex period lock: once either direction has pinned the session's period
 * (clarett_pcm_hw_params -> c->lock_period), the other must take the same, so
 * both share the one RX marker cadence the engine is armed with.
 */
static int clarett_rule_lock_period(struct snd_pcm_hw_params *params,
				    struct snd_pcm_hw_rule *rule)
{
	struct clarett *c = rule->private;
	u32 locked = READ_ONCE(c->lock_period);
	struct snd_interval *ps =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval t;

	if (!locked)
		return 0;
	snd_interval_any(&t);
	t.min = t.max = locked;
	t.integer = 1;
	return snd_interval_refine(ps, &t);
}

/*
 * Duplex rate lock: the engine has a single sample clock, so once the other
 * direction is configured (c->lock_rate) this one may only take the same rate.
 * Keyed on the OTHER direction only, so a lone client can still renegotiate.
 */
static int clarett_rule_lock_rate(struct snd_pcm_hw_params *params,
				  struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_substream *ss = rule->private;
	struct clarett *c = snd_pcm_substream_chip(ss);
	u32 other = READ_ONCE(c->lock_rate[!ss->stream]);
	struct snd_interval *r =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval t;

	if (!other)
		return 0;
	snd_interval_any(&t);
	t.min = t.max = other;
	t.integer = 1;
	return snd_interval_refine(r, &t);
}

/*
 * Most periods the ALSA buffer may hold — except where the period is so small
 * that this many of them fall below CLARETT_MIN_BUFFER_FRAMES, where the floor
 * wins (a 16-frame period may have 8).
 *
 * This bounds latency for an app that pins only the period: alsa-lib resolves
 * BUFFER_SIZE with set_last, so the app is handed the largest buffer allowed,
 * and tying that to its own period keeps latency proportional to its request.
 * A fixed ceiling would shortchange clients that ask for more: JUCE asks for
 * four periods of its block and reads and writes whole blocks, so a buffer
 * below its block garbles the audio without ever reporting an xrun. Four is
 * what both JUCE and PipeWire request.
 */
#define CLARETT_MAX_PERIODS	4

/*
 * BUFFER_SIZE <= max(CLARETT_MAX_PERIODS * period, floor), taken at the largest
 * period still allowed.
 */
static int clarett_rule_buffer_by_period(struct snd_pcm_hw_params *params,
					 struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *ps =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval *bs =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	struct snd_interval t;

	if (ps->max > UINT_MAX / CLARETT_MAX_PERIODS)
		return 0;	/* period still unbounded: nothing to say yet */
	snd_interval_any(&t);
	t.max = max_t(unsigned int, CLARETT_MAX_PERIODS * ps->max,
		      CLARETT_MIN_BUFFER_FRAMES);
	t.integer = 1;
	return snd_interval_refine(bs, &t);
}

/*
 * The same relation read the other way, so refinement converges from either
 * side: a buffer above the floor needs a period of at least
 * 1/CLARETT_MAX_PERIODS of it.
 */
static int clarett_rule_period_by_buffer(struct snd_pcm_hw_params *params,
					 struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *bs =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	struct snd_interval *ps =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval t;

	if (bs->min <= CLARETT_MIN_BUFFER_FRAMES)
		return 0;
	snd_interval_any(&t);
	t.min = DIV_ROUND_UP(bs->min, CLARETT_MAX_PERIODS);
	t.integer = 1;
	return snd_interval_refine(ps, &t);
}

/*
 * Advertised rate set. 44.1 and 48 kHz (single speed) are always offered;
 * 88.2/96 (double) and 176.4/192 (quad) are added up to the model's
 * clarett_model.max_rate. All six are SET_CLOCK enums the device lists.
 */
static unsigned int clarett_rate_caps(struct clarett *c, unsigned int *rmin,
				      unsigned int *rmax)
{
	unsigned int cap = c->model->max_rate;
	unsigned int rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000;

	*rmin = 44100;
	*rmax = 48000;
	if (cap >= 88200) {
		rates |= SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000;
		*rmax = 96000;
	}
	if (cap >= 176400) {
		rates |= SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000;
		*rmax = 192000;
	}
	return rates;
}

/*
 * The hw constraints and rules of an open substream, on top of the limits
 * clarett_pcm_open() put in runtime->hw. frame is the substream's frame stride
 * in bytes, buf its buffer ceiling.
 */
static int clarett_pcm_constrain(struct snd_pcm_substream *ss, u32 frame,
				 size_t buf)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	int err;

	/* Lock both directions to one rate. */
	err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				  clarett_rule_lock_rate, ss,
				  SNDRV_PCM_HW_PARAM_RATE, -1);
	if (err < 0)
		return err;

	/*
	 * The buffer is a power-of-two FRAME count between
	 * CLARETT_MIN_BUFFER_FRAMES and the ring, so it divides the ring and
	 * ALSA frame k and ring frame k wrap together. In frames, not bytes:
	 * the frame stride is not a power of two on most of the line.
	 */
	err = snd_pcm_hw_constraint_minmax(runtime,
					   SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					   (size_t)CLARETT_MIN_BUFFER_FRAMES *
					   frame, buf);
	if (err < 0)
		return err;
	err = snd_pcm_hw_constraint_pow2(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	if (err < 0)
		return err;
	/*
	 * The period becomes the RX marker cadence (irq_descs = period/16),
	 * which must divide CLARETT_STREAM_NDESC; a power of two does.
	 */
	err = snd_pcm_hw_constraint_pow2(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	if (err < 0)
		return err;
	/*
	 * At most CLARETT_MAX_PERIODS of the app's own period (above the
	 * floor), in both directions.
	 */
	err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
				  clarett_rule_buffer_by_period, NULL,
				  SNDRV_PCM_HW_PARAM_PERIOD_SIZE, -1);
	if (err < 0)
		return err;
	err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
				  clarett_rule_period_by_buffer, NULL,
				  SNDRV_PCM_HW_PARAM_BUFFER_SIZE, -1);
	if (err < 0)
		return err;
	/* Lock both directions to one period. */
	return snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
				   clarett_rule_lock_period, c,
				   SNDRV_PCM_HW_PARAM_PERIOD_SIZE, -1);
}

static int clarett_pcm_open(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u8 chans   = play ? c->model->playback_channels :
			    c->model->capture_channels;
	u32 frame  = (u32)chans * 4;
	/*
	 * Both directions get the SHORTER ring's ceiling. A Red's playback ring
	 * is half its capture ring, so otherwise their default periods differ,
	 * and two clients configuring at once fail the period lock.
	 */
	u32 tx_frame = (u32)c->model->playback_channels * 4;
	u32 rx_frame = (u32)c->model->capture_channels * 4;
	u32 ring_frames = min_t(u32, clarett_tx_ring_bytes(c) / tx_frame,
				clarett_rx_ring_bytes(c) / rx_frame);
	size_t buf = (size_t)ring_frames * frame;
	/* minimum period: the finest cadence (CLARETT_DYN_MIN_FRAMES) */
	u32 min_period = CLARETT_DYN_MIN_FRAMES * frame;

	runtime->hw = clarett_pcm_hw;
	if (play)
		/* .ack must see every appl_ptr move */
		runtime->hw.info |= SNDRV_PCM_INFO_SYNC_APPLPTR;
	runtime->hw.rates           =
		clarett_rate_caps(c, &runtime->hw.rate_min,
				  &runtime->hw.rate_max);
	runtime->hw.channels_min     = chans;
	runtime->hw.channels_max     = chans;
	/*
	 * Ceiling: the ring, unless max_buffer lowers it. What an app that pins
	 * only the period actually receives is bounded lower still, by the
	 * period rule clarett_pcm_constrain() adds (CLARETT_MAX_PERIODS).
	 */
	buf = (size_t)clarett_buffer_max_frames(buf / frame) * frame;

	runtime->hw.buffer_bytes_max = buf;
	runtime->hw.period_bytes_min = min_period;
	runtime->hw.period_bytes_max = buf / 2;	/* periods_min = 2 */
	runtime->hw.periods_max      = buf / min_period;

	return clarett_pcm_constrain(ss, frame, buf);
}

/*
 * Pin the session's shared period the first time a direction is configured, and
 * record this direction's rate. The other direction's open() rules read these
 * and constrain it to match, so both arm the one marker cadence at the one
 * sample rate.
 */
static int clarett_pcm_hw_params(struct snd_pcm_substream *ss,
				 struct snd_pcm_hw_params *params)
{
	struct clarett *c = snd_pcm_substream_chip(ss);

	WRITE_ONCE(c->lock_rate[ss->stream], params_rate(params));
	WRITE_ONCE(c->lock_period, params_period_size(params));
	return 0;
}

/*
 * Detach a direction and, once BOTH are gone, stop the shared engine. Clearing
 * the substream pointer under pcm_lock (and draining any in-flight tick)
 * guarantees clarett_pcm_tick() stops touching this runtime's dma_area before
 * the core frees it. On playback teardown re-silence the TX ring so a surviving
 * capture stream does not loop stale playback audio out the DACs.
 */
static void clarett_pcm_detach(struct clarett *c, struct snd_pcm_substream *ss)
{
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;

	mutex_lock(&c->pcm_lock);
	WRITE_ONCE(c->lock_rate[ss->stream], 0);
	if (play) {
		c->pcm_play_sub = NULL;
		WRITE_ONCE(c->play_running, false);
		/* after removal the ring (devres) may be freed */
		if (!c->hw_gone)
			clarett_zero_tx(c);
	} else {
		c->pcm_sub = NULL;
		WRITE_ONCE(c->pcm_running, false);
	}
	mutex_unlock(&c->pcm_lock);

	if (!c->pcm_sub && !c->pcm_play_sub) {
		/* last user out: release the shared-period lock */
		WRITE_ONCE(c->lock_period, 0);
		/* and tear the engine down */
		clarett_engine_stop(c);
	}
}

static int clarett_pcm_close(struct snd_pcm_substream *ss)
{
	clarett_pcm_detach(snd_pcm_substream_chip(ss), ss);
	return 0;
}

/* The core calls hw_free() ahead of releasing the managed DMA buffer. */
static int clarett_pcm_hw_free(struct snd_pcm_substream *ss)
{
	clarett_pcm_detach(snd_pcm_substream_chip(ss), ss);
	return 0;
}

/*
 * System sleep. By the time the device suspends, the core has suspended every
 * running stream (trigger SUSPEND), so nothing reports periods; stop the engine
 * itself, which a stream otherwise keeps until it closes. The substreams stay
 * attached, and the first re-prepare after resume finds the engine down and
 * arms it, at a fresh clock. Silence the TX ring too: the re-armed engine reads
 * it before the new servicer's first fill.
 */
void clarett_pcm_suspend(struct clarett *c)
{
	clarett_engine_stop(c);
	mutex_lock(&c->pcm_lock);
	if (c->stream_buf)
		clarett_zero_tx(c);
	mutex_unlock(&c->pcm_lock);
}

/*
 * After resume, give the device the Clock Source again at the rate it last ran,
 * as a control write does: a stream arm would send it anyway, but an idle card
 * should not wait for one.
 */
void clarett_pcm_resume(struct clarett *c)
{
	u32 rate = READ_ONCE(c->cur_rate);
	u8 clk[8];

	if (!c->model->n_clock_srcs)
		return;
	clarett_put_le32(clk, rate ? rate : CLARETT_DEFAULT_RATE);
	clarett_put_le32(clk + 4, clarett_clock_source(c));
	if (clarett_fcp(c, FCP_SET_CLOCK, clk, sizeof(clk)))
		dev_dbg(&c->pci->dev, "resume: SET_CLOCK failed\n");
}

/*
 * Stream-config handshake, run immediately before every engine arm: the device
 * resets its stream config when idle, so nothing established earlier survives
 * to arm time. SET_CLOCK{rate, source}, then the sync queries. Non-fatal: log
 * and proceed even if a command errors.
 */
static void clarett_stream_handshake(struct clarett *c, unsigned int rate)
{
	u8 clk[8];
	int e_clk, e_en1, e_en2, e_commit;
	int clk_src = clarett_clock_source(c);

	if (!rate)
		rate = CLARETT_DEFAULT_RATE;
	/* published at /proc/asound/cardN/clarett */
	WRITE_ONCE(c->cur_rate, rate);
	clarett_put_le32(clk,     rate);
	clarett_put_le32(clk + 4, clk_src);

	e_clk = clarett_fcp(c, FCP_SET_CLOCK, clk, sizeof(clk));

	/*
	 * The sync queries, in order 0x6004, 0x6002, 0x6005 (see FCP_SYNC_READ
	 * in clarett.h).
	 */
	e_en1    = clarett_fcp(c, FCP_SYNC_READ, NULL, 0);
	e_en2    = clarett_fcp(c, FCP_GET_62, NULL, 0);
	e_commit = clarett_fcp(c, FCP_SYNC_RATE, NULL, 0);

	dev_dbg(&c->pci->dev,
		"stream-handshake: SET_CLOCK{%u,%u}=%d 0x6004=%d 0x6002=%d 0x6005=%d\n",
		rate, clk_src, e_clk, e_en1, e_en2, e_commit);
}

/*
 * Attach the substream to the shared engine clock, and claim the arm if it is
 * the first direction in (*arm). Fails if the engine already runs at another
 * rate.
 */
static int clarett_pcm_attach(struct clarett *c, struct snd_pcm_substream *ss,
			      bool *arm)
{
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 other;

	mutex_lock(&c->pcm_lock);
	/*
	 * Backstop for clarett_rule_lock_rate, which two directions configured
	 * at once can both pass.
	 */
	other = c->lock_rate[!ss->stream];
	if (c->stream_on && other && other != ss->runtime->rate) {
		mutex_unlock(&c->pcm_lock);
		dev_dbg(&c->pci->dev,
			"%s at %u Hz refused: the engine runs at %u Hz\n",
			play ? "playback" : "capture", ss->runtime->rate,
			other);
		return -EINVAL;
	}
	*arm = !c->stream_on;
	if (*arm) {
		/*
		 * CLAIM THE ARM UNDER THE LOCK. The stream handshake runs
		 * mailbox commands before the engine is up, so if stream_on
		 * were published only at the end, two prepares in that window
		 * (a duplex client starting both directions at once) would both
		 * arm, and the second clarett_engine_run() would orphan the
		 * first servicer thread. engine_arm sets it again (idempotent).
		 */
		c->stream_on = true;
		/* first direction in: the shared clock starts here */
		c->pcm_frames = 0;
	}
	if (play) {
		c->pcm_play_sub = ss;
		c->play_base = c->pcm_frames;
		c->play_last_period = 0;
		WRITE_ONCE(c->play_running, false);
	} else {
		c->pcm_sub = ss;
		c->pcm_base = c->pcm_frames;
		c->pcm_last_period = 0;
		WRITE_ONCE(c->pcm_running, false);
	}
	mutex_unlock(&c->pcm_lock);
	return 0;
}

/*
 * First direction in: size the RX marker cadence to the period, then run the
 * stream handshake, arm the engine and start its servicer.
 */
static void clarett_pcm_arm(struct clarett *c, unsigned int rate)
{
	dma_addr_t r0, r1;

	/*
	 * Match the RX marker cadence to the shared period before arming, so
	 * the engine raises a 0x300 period per ALSA period. The engine is not
	 * armed yet, so the table can be rebuilt.
	 */
	if (c->lock_period) {
		u32 descs = clamp_t(u32, c->lock_period / CLARETT_FRAG_FRAMES,
				    1, CLARETT_STREAM_NDESC / 2);

		if (descs != c->irq_descs) {
			c->irq_descs = descs;
			clarett_build_rings(c);
			/*
			 * descriptor table visible to the device before we arm
			 */
			wmb();
		}
	}

	/*
	 * First direction in: arm the full-duplex engine. block 0 (TX) base,
	 * block 1 (RX) base — each points at its descriptor table.
	 */
	r0 = c->stream_dma;
	r1 = c->stream_dma + clarett_stream_r1_off(c);

	/*
	 * Stream-config handshake immediately before arming (handshake ->
	 * program regs -> arm).
	 */
	clarett_stream_handshake(c, rate);
	clarett_engine_arm(c, r0, r1);

	/*
	 * ACK from here, not the trigger: the engine stalls within ms of the
	 * arm if unserviced. The triggers only gate whether periods are
	 * reported to ALSA (pcm_running / play_running).
	 */
	WRITE_ONCE(c->stream_run, true);
	clarett_engine_run(c);
}

/*
 * Prepare a direction and, if the shared full-duplex engine is not already
 * running (the other direction armed it), arm it. The engine is ONE full-duplex
 * stream (block 0 = TX/playback, block 1 = RX/capture) with a single frame
 * clock; whichever direction prepares first arms it and the other just attaches
 * at the current clock position. Re-preparing an already-armed direction (xrun
 * recovery) does not re-arm.
 *
 * Either way the direction records its attach point in the shared clock: ALSA
 * resets hw_ptr to 0 on every prepare, so .pointer must report from there, or
 * a stream joining a running engine would see an enormous hw_ptr jump and xrun
 * again and again.
 */
static int clarett_pcm_prepare(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	bool arm;
	int err;

	err = clarett_pcm_attach(c, ss, &arm);
	if (err)
		return err;

	/*
	 * Latch this rate's live capture width so the drain blanks the
	 * S/MUX-removed tail.
	 */
	if (!play)
		clarett_set_rx_live(c, ss->runtime->rate);

	if (!arm) /* engine already armed: just attached at the current clock */
		return 0;

	clarett_pcm_arm(c, ss->runtime->rate);
	return 0;
}

/*
 * Atomic context (stream lock held): only flip this direction's servicing gate.
 * The engine is already armed and the kthread running from prepare(); START
 * begins reporting that direction's periods from the current clock, STOP halts
 * them. Full teardown happens in close()/hw_free, which can sleep.
 */
static int clarett_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u64 now = READ_ONCE(c->pcm_frames);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (play) {
			/*
			 * Frame 0 plays one period after the clock START sees.
			 * pcm_frames lags the engine by up to a period, so a
			 * frame anchored at it may already be past the read
			 * position. One period on is the next event boundary:
			 * still ahead of the engine, and it keeps this stream's
			 * period boundaries on events (off them, a two-period
			 * client would refill behind the engine). .pointer
			 * reports 0 until then, so the wait is in the delay.
			 */
			WRITE_ONCE(c->play_base,
				   now + ss->runtime->period_size);
			c->play_last_period = 0;
			c->play_primed = false;
			/* anchor visible before running */
			smp_store_release(&c->play_running, true);
			/* fill the prefilled buffer now, not a tick later */
			atomic_set(&c->tx_dirty, 1);
		} else {
			c->pcm_last_period = div_u64(now - c->pcm_base,
						     ss->runtime->period_size);
			WRITE_ONCE(c->pcm_running, true);
		}
		return 0;
	/*
	 * System sleep: the core suspends a running stream before the device's
	 * own suspend callback, which then tears the engine down
	 * (clarett_pcm_suspend()). There is no RESUME (the hardware advertises
	 * none), so the app re-prepares after resume and that prepare arms the
	 * engine again.
	 */
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		if (play) {
			WRITE_ONCE(c->play_running, false);
			WRITE_ONCE(c->tx_silence, true);
			/* the servicer silences the ring on its next poll */
			atomic_set(&c->tx_dirty, 1);
		} else {
			WRITE_ONCE(c->pcm_running, false);
		}
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t clarett_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u64 base = play ? READ_ONCE(c->play_base) : READ_ONCE(c->pcm_base);
	u64 now = READ_ONCE(c->pcm_frames);
	u64 frames;

	/*
	 * Playback anchored ahead of the engine (START): nothing has played
	 * yet.
	 */
	if (now < base)
		return 0;
	frames = now - base;
	/* position on THIS stream's clock (frames since it attached) */
	return do_div(frames, ss->runtime->buffer_size);
}

/*
 * Atomic context (stream lock held), called whenever the app moves appl_ptr.
 * Only flag it: the copy is up to a ring of audio and needs pcm_lock, so the
 * servicer does it on its next poll (clarett_pcm_tx_refill), within a few
 * hundred microseconds. Playback advertises SNDRV_PCM_INFO_SYNC_APPLPTR so mmap
 * clients report appl_ptr through the kernel and this fires for them too.
 */
static int clarett_pcm_ack(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);

	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK)
		atomic_set(&c->tx_dirty, 1);
	return 0;
}

static const struct snd_pcm_ops clarett_pcm_ops = {
	.open      = clarett_pcm_open,
	.close     = clarett_pcm_close,
	.hw_params = clarett_pcm_hw_params,
	.hw_free   = clarett_pcm_hw_free,
	.prepare   = clarett_pcm_prepare,
	.trigger   = clarett_pcm_trigger,
	.pointer   = clarett_pcm_pointer,
	.ack       = clarett_pcm_ack,
};

/*
 * Build both static descriptor tables in the contiguous hardware buffer ([TX
 * ring][RX ring], each = [table][samples]). Each entry is a bare 8-byte LE
 * fragment bus address, strided by the fragment slot (tx_slot / rx_slot). The
 * LAST entry carries the wrap flag in its low bits (TX 0x01, RX 0x03), and
 * there is NO zero terminator. dma_alloc_coherent returns zeroed memory, so the
 * TX samples are already silence and the RX sample area starts clean.
 */
static void clarett_build_rings(struct clarett *c)
{
	size_t tbl     = clarett_pcm_tbl_bytes();

	size_t tx_ring = clarett_pcm_tx_ring(c);
	u32 tx_frag = clarett_tx_frag_bytes(c->model->playback_channels);
	/* TX descriptor stride: audio bytes, or a padded slot */
	u32 tx_slot = c->tx_slot;
	/* RX descriptor stride: audio bytes, or a padded slot */
	u32 rx_slot = c->rx_slot;
	__le64 *tx_tbl = (__le64 *)c->stream_buf;
	__le64 *rx_tbl = (__le64 *)((u8 *)c->stream_buf + tx_ring);
	dma_addr_t tx_smp = c->stream_dma + tbl;
	/* page-aligned RX sample area */
	dma_addr_t rx_smp = c->stream_dma + clarett_stream_rx_off(c);
	unsigned int i;

	for (i = 0; i < CLARETT_STREAM_NDESC; i++) {
		/* slotted: non-contiguous when padded */
		tx_tbl[i] = cpu_to_le64(tx_smp + (u64)i * tx_slot);
		/* slotted: fragments non-contiguous when padded */
		rx_tbl[i] = cpu_to_le64(rx_smp + (u64)i * rx_slot);
		/*
		 * Periodic RX IRQ marker: the engine raises a counted 0x300
		 * period when it consumes an IRQ-flagged descriptor. Every
		 * clarett_irq_descs(c)-th one (default 16; prepare sets it from
		 * the chosen ALSA period). TX flags only its last entry.
		 */
		if ((i + 1) % clarett_irq_descs(c) == 0)
			rx_tbl[i] |= cpu_to_le64(CLARETT_DESC_IRQ);
	}
	tx_tbl[CLARETT_STREAM_NDESC - 1] |= cpu_to_le64(CLARETT_DESC_WRAP_TX);
	rx_tbl[CLARETT_STREAM_NDESC - 1] |= cpu_to_le64(CLARETT_DESC_WRAP_RX);

	dev_dbg(&c->pci->dev,
		"descriptor rings: %u entries, tx audio=0x%x slot=0x%x, rx audio=0x%x slot=0x%x, RX IRQ every %u desc (%u frames/period)\n",
		CLARETT_STREAM_NDESC, tx_frag, tx_slot,
		clarett_frag_bytes(c->model->capture_channels), rx_slot,
		clarett_irq_descs(c), clarett_irq_period_frames(c));
}

int clarett_create_pcm(struct clarett *c)
{
	struct snd_pcm *pcm;
	size_t rxbuf, txbuf, prealloc;
	int err;

	/* 1 playback, 1 capture */
	err = snd_pcm_new(c->card, c->model->name, 0, 1, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = c;
	strscpy(pcm->name, c->model->name, sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &clarett_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &clarett_pcm_ops);
	c->pcm = pcm;

	c->stream_size = clarett_stream_total_bytes(c);
	c->stream_buf = dmam_alloc_coherent(&c->pci->dev, c->stream_size,
					    &c->stream_dma, GFP_KERNEL);
	if (!c->stream_buf)
		return -ENOMEM;
	clarett_build_rings(c);

	/*
	 * Preallocate the larger ring's size, so the widest case fits. The ALSA
	 * buffers are only copied to and from the hardware rings, so they are
	 * vmalloc, not device memory: after a removal the card is freed when
	 * userspace closes it, when the PCI device is already gone.
	 */
	rxbuf = clarett_rx_ring_bytes(c);
	txbuf = clarett_tx_ring_bytes(c);
	prealloc = max(rxbuf, txbuf);
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       prealloc, prealloc);

	/*
	 * Debug: the probe summary in clarett_probe() already states that a PCM
	 * registered and how wide it is; this adds the ring/buffer detail.
	 */
	dev_dbg(&c->pci->dev,
		"PCM registered (playback %uch / capture %uch, S32_LE @%u, bufs tx=%zu rx=%zu B @%pad)\n",
		c->model->playback_channels, c->model->capture_channels,
		CLARETT_PCM_RATE, txbuf, rxbuf, &c->stream_dma);

	return 0;
}
