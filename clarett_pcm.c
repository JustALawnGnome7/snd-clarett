// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett (Thunderbolt) — PCM data plane (full duplex).
 *
 * The hardware is ONE full-duplex DMA engine over two ring blocks in a single contiguous coherent buffer
 * (c->stream_buf): block 0 (0x200 = TX/playback) then block 1 (0x300 = RX/capture). It raises a period on
 * the 0x300 cause register and waits for the host to ACK by reading it; clarett_stream_service()
 * (clarett_main.c) is that ACK loop and the shared frame clock for both directions. Each 0x300 event
 * carries the frames the engine advanced (ctr delta * CLARETT_CTR_FRAMES); clarett_pcm_tick() then drains
 * the RX ring into the capture ALSA buffer (behind the write pointer) and refills the TX ring from the
 * playback ALSA buffer (ahead of the read pointer).
 *
 * Two ALSA substreams share the one engine: whichever prepares first arms it full-duplex, the other
 * attaches at the current clock. The engine ONLY clocks when both rings live in one contiguous buffer
 * with r1 = r0 + ring (proven by the engine-start probe), which is why TX is always armed even for
 * capture-only (it plays silence until a playback stream fills it).
 *
 * Calibration caveat (clarett.h): frames-per-0x300-event uses CLARETT_CTR_FRAMES; verified on the 2Pre.
 * Clocking and period flow are independent of that constant.
 */
#include <linux/dma-mapping.h>
#include <linux/math64.h>
#include <linux/log2.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <sound/core.h>
#include <sound/control.h>	/* snd_kcontrol_new — the driver-owned "Clock Source" control */
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "clarett.h"

static void clarett_build_rings(struct clarett *c);	/* rebuilt at prepare when dyn_period changes cadence */

/*
 * Replay the vendor's pre-arm re-init batch in the stream handshake (see clarett_stream_handshake).
 * TESTED ON A 4Pre AND IT CHANGES NOTHING: the device accepts every command (err=0) but the engine
 * still raises one period event with ctr=0. Default OFF because it overlaps the probe-time bring-up
 * that is documented to wedge GET_DATA when re-run on an armed device, and it has no demonstrated
 * benefit to weigh against that. Kept as a lever for retesting on other models.
 */
static bool stream_batch;
module_param(stream_batch, bool, 0644);
MODULE_PARM_DESC(stream_batch,
		 "Replay the vendor's pre-arm re-init batch (INIT_2, subsystem enables 1-8, count "
		 "queries, 0x004001 x6) before arming the stream engine (default off; no effect on a 4Pre).");

/*
 * dyn_period: derive the RX IRQ cadence from the negotiated ALSA period instead of the fixed 256-frame
 * default, so a DAW can pick a smaller device buffer (down to one 16-frame fragment). It rebuilds the
 * descriptor ring at a finer marker cadence; both directions are locked to one period (option a: DAWs use a
 * single duplex buffer size anyway).
 *
 * HARDWARE-VERIFIED (8Pre): the 0x300 counter free-runs in 16-frame units regardless of marker
 * spacing — its per-event step scaled proportionally with the cadence across a 64x range (+0x01/+0x04/+0x08/
 * +0x10/+0x40 at cadence 1/4/8/16/64), so the servicer's step*CLARETT_CTR_FRAMES advance stays correct with no
 * change, and a 1 kHz reference captured at 1000.0 Hz (no drift) at every one. Cadence 1 (EVERY descriptor
 * IRQ-flagged, 8x the vendor's ~14 density) held stable over 132k periods / ~44 s with rekicks=0, wraps=0,
 * late=0 — the engine tolerates maximum flag density. Consequently the counter's wrap/recovery window is 0x100
 * units = 4096 frames = ~85 ms at EVERY cadence, so a finer period does not reduce the scheduling-gap tolerance
 * (it rode through the platform's ~42 ms SMI freeze at cadence 4, coalesced periods recovered exactly, wraps=0).
 *
 * ON by default. The floor is one 16-frame fragment (CLARETT_DYN_MIN_FRAMES = CLARETT_FRAG_FRAMES, cadence 1),
 * the hardware minimum and verified above — a 16x drop from the old 256 floor. dyn_period=0 restores the fixed
 * 256-frame cadence. NOTE: with PipeWire adopting the card it arms the engine first and the duplex lock coerces
 * the app to PipeWire's quantum, so a DAW controls the buffer size only once PipeWire has released the card
 * (standard pro-audio setup).
 *
 * The period is only half of what an app feels as latency. The total ALSA buffer used to be PINNED to the
 * 4096-frame ring, so an app that keeps its buffer full ran 85 ms of playback latency no matter how small a
 * period it asked for; it is now any power-of-two fraction of the ring down to CLARETT_MIN_BUFFER_FRAMES, and
 * at most CLARETT_MAX_PERIODS of the app's own period.
 */
#define CLARETT_DYN_MIN_FRAMES	CLARETT_FRAG_FRAMES	/* one fragment = 16 frames (cadence 1); the verified floor */
static bool dyn_period = true;
module_param(dyn_period, bool, 0444);
MODULE_PARM_DESC(dyn_period,
		 "Derive the RX IRQ cadence from the chosen ALSA period, lowering the minimum device buffer from "
		 "256 to 16 frames (default on; verified drift-free at cadence 1-64). 0 = fixed 256-frame cadence. "
		 "A DAW controls the period only once PipeWire has released the card.");

/*
 * Override the highest sample rate the PCM advertises. Default 0 = use the per-model confirmed cap
 * (clarett_model.max_rate): single speed (44.1/48) everywhere, plus double/quad on models where the
 * high-rate data plane is hardware-confirmed (the 2Pre, to 192 kHz). Set this to opt a NOT-yet-confirmed
 * model into the higher rates for testing: the transport already sends SET_CLOCK{rate,Internal} for any
 * rate and the stream width is rate-independent, but confirm with a known tone (correct pitch on the
 * analogue channel) before trusting a rate on an ADAT model, where double/quad speed is unverified.
 */
static unsigned int max_rate;
module_param(max_rate, uint, 0444);
MODULE_PARM_DESC(max_rate,
		 "Override the highest advertised sample rate for ALL models: 48000, 96000, or 192000. "
		 "0 (default) uses each model's hardware-confirmed cap. 44.1 and 48 kHz are always offered.");

/*
 * Clock source sent with SET_CLOCK at each stream arm: 24=Internal, 0=ADAT, 3=S/PDIF on every model
 * (the 2Pre XML claims 4 for S/PDIF; measured, 4 locks to any external source and 3 is the one that
 * tracks S/PDIF alone — see clarett.h). The 8PreX alone adds 1=ADAT 2 and 2=Wordclock, both untested.
 * Default Internal. Set to 0 to slave to an incoming ADAT clock (needed to receive a digital ADAT input
 * cleanly). The source is NOT a config-space byte — it lives only in the SET_CLOCK payload — so it
 * cannot be mapped as an fcp-server global control, and there is no clock-source ALSA control yet.
 *
 * PER-CARD, indexed by ALSA card number (the one /proc/asound/cards shows), because a two-card rig needs
 * one master and one slave: feeding one Clarett's ADAT output into another's input requires the source
 * card on Internal and the sink card on ADAT, and a scalar parameter would slave both. Writable at
 * runtime; takes effect at the next stream arm. The negotiated PCM rate must match the external clock's
 * rate when the source is not Internal.
 */
static int clock_source[SNDRV_CARDS] = { [0 ... SNDRV_CARDS - 1] = CLARETT_CLOCK_INTERNAL };
/* NULL count, not a &num: with a count the sysfs read shows only the entries explicitly set at load
 * (nothing at all by default), which makes the live setting unreadable. NULL shows the whole array. */
module_param_array(clock_source, int, NULL, 0644);
MODULE_PARM_DESC(clock_source,
		 "Per-card SET_CLOCK source at stream arm, indexed by ALSA card number: 24=Internal "
		 "(default), 0=ADAT, 3=S/PDIF (8PreX also has 1=ADAT 2, 2=Wordclock). PCM rate must match "
		 "the external clock when not Internal.");

/* The clock source in force for this card; falls back to Internal for a card number past the array. */
static int clarett_clock_source(struct clarett *c)
{
	int n = c->card->number;

	return (n >= 0 && n < SNDRV_CARDS) ? clock_source[n] : CLARETT_CLOCK_INTERNAL;
}

static void clarett_set_clock_source(struct clarett *c, int value)
{
	int n = c->card->number;

	if (n >= 0 && n < SNDRV_CARDS)
		clock_source[n] = value;
}

/*
 * "Clock Source" — the one control this driver owns rather than leaving to fcp-server.
 *
 * It cannot go through fcp-server's map: the clock source is NOT a config-space byte (the [XML]
 * <clocking> element carries no offset-bytes), it exists only in the SET_CLOCK payload, which is
 * {u32 rate, u32 source} — and fcp-server has no idea what sample rate the device is running at, so it
 * cannot issue that command safely. This driver already sends SET_CLOCK at every stream arm and knows
 * the negotiated rate, so the selection belongs here. alsa-scarlett-gui renders any element named
 * "Clock Source" as a drop-down with no changes needed on its side.
 *
 * Backed by the per-card clock_source[] module parameter so the control and the sysfs knob stay one
 * value; a sysfs write still works but bypasses the control's change notification.
 */
static int clarett_clock_src_info(struct snd_kcontrol *kctl, struct snd_ctl_elem_info *uinfo)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;

	unsigned int i;

	/* Not snd_ctl_enum_info(): the names live in a struct array, not a flat char* table. */
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

static int clarett_clock_src_get(struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;
	int cur = clarett_clock_source(c);
	u8 i;

	/* Report the matching entry; a raw value set out-of-band via sysfs may not be in the list, in
	 * which case fall back to entry 0 (Internal) rather than an out-of-range index. */
	for (i = 0; i < m->n_clock_srcs; i++) {
		if (m->clock_srcs[i].value == cur) {
			ucontrol->value.enumerated.item[0] = i;
			return 0;
		}
	}
	dev_dbg(&c->pci->dev, "clock source %d not in this model's list; reporting %s\n",
		cur, m->clock_srcs[0].name);
	ucontrol->value.enumerated.item[0] = 0;
	return 0;
}

static int clarett_clock_src_put(struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct clarett *c = snd_kcontrol_chip(kctl);
	const struct clarett_model *m = c->model;
	unsigned int i = ucontrol->value.enumerated.item[0];
	int value;

	if (i >= m->n_clock_srcs)
		return -EINVAL;
	value = m->clock_srcs[i].value;
	if (value == clarett_clock_source(c))
		return 0;

	clarett_set_clock_source(c, value);

	/*
	 * Apply it now if nothing is streaming, so Sync Status reflects the choice immediately instead of
	 * waiting for the next stream arm — that is what makes the drop-down feel live in the GUI. The rate
	 * here is nominal (nothing is streaming); the arm sends the negotiated one. While a stream IS
	 * running we deliberately do not touch the device: re-clocking mid-stream would tear the audio, so
	 * the change lands at the next arm.
	 */
	if (!READ_ONCE(c->stream_on)) {
		u8 clk[8];

		clarett_put_le32(clk, CLARETT_DEFAULT_RATE);
		clarett_put_le32(clk + 4, value);
		if (clarett_fcp(c, FCP_SET_CLOCK, clk, sizeof(clk)))
			dev_dbg(&c->pci->dev, "clock source %s: SET_CLOCK failed\n",
				m->clock_srcs[i].name);
	}
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
 * Per-ring geometry is derived per-model at runtime (clarett.h: clarett_pcm_rx_samples() &c.). The ALSA
 * buffer is pinned to exactly the RX sample-area size so the RX ring and the ALSA ring share one geometry
 * (CLARETT_STREAM_NDESC descriptors, 1:1 byte offsets) — the per-period copy is then a straight offset copy
 * with no rescaling.
 */

/*
 * Constant capability template; the per-model geometry fields (channels, buffer/period bytes,
 * periods_max) and the rate set (rates/rate_min/rate_max, per the max_rate cap) are filled in
 * clarett_pcm_open().
 */
static const struct snd_pcm_hardware clarett_pcm_hw = {
	.info             = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			    SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats          = SNDRV_PCM_FMTBIT_S32_LE,	/* interleaved, 24-bit MSB-justified */
	.periods_min      = 2,
};

/* Pointer to the block-1 (capture) RX sample area inside the contiguous hardware buffer. Descriptor mode:
 * [TX table][TX samples][RX table][RX samples]. Flat mode: [TX samples][RX samples]. clarett_stream_rx_off()
 * returns the right offset for the model's mode. */
static u8 *clarett_rx_area(struct clarett *c)
{
	return (u8 *)c->stream_buf + clarett_stream_rx_off(c);
}

/* RX capture sample-ring size in bytes (== the capture ALSA buffer), per mode. */
static size_t clarett_rx_ring_bytes(struct clarett *c)
{
	return clarett_stream_rx_area_bytes(c);
}

/* Pointer to the block-0 (playback) TX sample area, and its size (== the playback ALSA buffer). */
static u8 *clarett_tx_area(struct clarett *c)
{
	return (u8 *)c->stream_buf + clarett_stream_tx_off(c);
}
static size_t clarett_tx_ring_bytes(struct clarett *c)
{
	return clarett_stream_tx_area_bytes(c);
}

/* Silence the whole TX device area (all slots, including any inter-fragment padding): playback-idle,
 * capture-only must not loop stale playback audio out the DACs. */
static void clarett_zero_tx(struct clarett *c)
{
	memset(clarett_tx_area(c), 0, clarett_pcm_tx_dev_bytes(c));
}

/*
 * How many leading capture channels does the device actually write at this rate? ADAT S/MUX removes the
 * upper ADAT channels at double and quad speed (8 -> 4 -> 2 per port); the frame stride is unchanged, so
 * the removed channels are a contiguous tail of the frame. See clarett_model.rx_live_{mid,high}.
 */
static u8 clarett_rx_live_channels(struct clarett *c, unsigned int rate)
{
	const struct clarett_model *m = c->model;
	u8 live = m->capture_channels;

	if (rate > 96000) {
		if (m->rx_live_high)
			live = m->rx_live_high;
	} else if (rate > 48000) {
		if (m->rx_live_mid)
			live = m->rx_live_mid;
	}
	return min(live, m->capture_channels);
}

/*
 * Latch how much of each capture frame the device actually fills at this rate, for clarett_rx_drain() to
 * blank on the way out. Called from prepare, once the rate is negotiated.
 *
 * The removed channels are NOT left untouched by the engine, which an earlier version of this assumed:
 * hardware shows it keeps depositing a sparse residue — one non-zero sample every 32 frames, an impulse
 * train at -25 dBFS — into channels it dropped at the immediately preceding speed tier (ADAT 5-8 at
 * double, ADAT 3-4 at quad), while channels dropped a full tier earlier get nothing at all. So blanking
 * the ring once cannot hold; it has to happen per period, on the frames handed to ALSA.
 */
static void clarett_set_rx_live(struct clarett *c, unsigned int rate)
{
	u8 chans = c->model->capture_channels;
	u8 live  = clarett_rx_live_channels(c, rate);

	WRITE_ONCE(c->rx_dead_bytes, (u32)(chans - live) * 4);
	WRITE_ONCE(c->rx_live_bytes, (u32)live * 4);
	dev_dbg(&c->pci->dev, "rx: %u of %u capture channels live at %u Hz\n", live, chans, rate);
}

/*
 * Drain nframes of captured audio from the RX device area (starting at hardware ring frame `pos`) into the
 * contiguous ALSA buffer (starting at that stream's own frame `apos` — the two differ by the capture
 * direction's attach base and wrap independently). The RX area is a table of NDESC fragment SLOTS of
 * c->rx_slot bytes; ring frame f lives in slot (f/FRAG_FRAMES) at byte (f%FRAG_FRAMES)*frame within that
 * slot. When rx_slot == audio-bytes/fragment (the contiguous default) this is just a linear copy; when
 * padded (scatter-gather experiment) it gathers per fragment across the gaps. FRAG_FRAMES divides the ring,
 * so a chunk clipped to the fragment boundary also handles the ring wrap.
 *
 * `abuf` is the ALSA buffer in frames, which is NOT the ring: it is a power-of-two divisor of it, so the
 * destination wraps one or more times per ring pass and has to be clipped separately from the source.
 */
static void clarett_rx_drain(struct clarett *c, u8 *alsa, u32 apos, u32 pos, u32 nframes, u32 abuf)
{
	u8 *ring = clarett_rx_area(c);
	u32 frame = (u32)c->model->capture_channels * 4;
	u32 slot  = c->rx_slot;
	u32 ring_frames = CLARETT_STREAM_NDESC * CLARETT_FRAG_FRAMES;
	u32 dead = READ_ONCE(c->rx_dead_bytes);		/* S/MUX-removed tail; see clarett_set_rx_live() */
	u32 live = READ_ONCE(c->rx_live_bytes);

	while (nframes) {
		u32 fio   = pos % CLARETT_FRAG_FRAMES;			/* frame within its fragment */
		u32 chunk = min(nframes, CLARETT_FRAG_FRAMES - fio);	/* up to the fragment (and ring) boundary */

		chunk = min(chunk, abuf - apos);			/* and up to the ALSA buffer wrap */

		memcpy(alsa + (size_t)apos * frame,
		       ring + (size_t)(pos / CLARETT_FRAG_FRAMES) * slot + (size_t)fio * frame,
		       (size_t)chunk * frame);

		/* Blank the channels S/MUX removed at this rate: the engine still drops a sparse residue in
		 * them, which a full-width capture would otherwise record as an impulse train. */
		if (dead) {
			u8 *d = alsa + (size_t)apos * frame + live;
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
 * Fill nframes of playback audio into the TX device area (starting at hardware ring frame `pos`) from the
 * contiguous ALSA playback buffer (starting at that stream's own frame `apos`). Exact mirror of
 * clarett_rx_drain with source/destination swapped: the TX area is NDESC fragment SLOTS of c->tx_slot
 * bytes; ring frame f lives in slot (f/FRAG_FRAMES) at byte (f%FRAG_FRAMES)*frame within that slot. When
 * tx_slot == audio-bytes/fragment (tx_frag_pad=0) this degenerates to the old linear copy; when padded it
 * scatters per fragment across the gaps (matching the vendor's non-contiguous TX ring). FRAG_FRAMES divides
 * the ring, so a chunk clipped to the fragment boundary also handles the ring wrap.
 *
 * `abuf` is the ALSA buffer in frames, a power-of-two divisor of the ring rather than the ring itself, so
 * the source wraps one or more times across a single fill and is clipped separately. Filling a runway
 * longer than the buffer therefore TILES it around the ring, which is what keeps the whole-runway strategy
 * below correct: ring frame f is always sourced from the buffer frame that is due to play when the engine
 * reaches f, because both advance by the same delta and abuf divides the ring.
 */
static void clarett_tx_fill(struct clarett *c, const u8 *alsa, u32 apos, u32 pos, u32 nframes, u32 abuf)
{
	u8 *ring = clarett_tx_area(c);
	u32 frame = (u32)c->model->playback_channels * 4;
	u32 slot  = c->tx_slot;
	u32 ring_frames = CLARETT_STREAM_NDESC * CLARETT_FRAG_FRAMES;

	while (nframes) {
		u32 fio   = pos % CLARETT_FRAG_FRAMES;			/* frame within its fragment */
		u32 chunk = min(nframes, CLARETT_FRAG_FRAMES - fio);	/* up to the fragment (and ring) boundary */

		chunk = min(chunk, abuf - apos);			/* and up to the ALSA buffer wrap */

		memcpy(ring + (size_t)(pos / CLARETT_FRAG_FRAMES) * slot + (size_t)fio * frame,
		       alsa + (size_t)apos * frame,
		       (size_t)chunk * frame);
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
 * Guard: keep the playback fill this many frames clear of the engine's current TX read position, so a
 * concurrent DMA read is never torn by our write. It only has to exceed the engine's advance during ONE
 * fill memcpy (a few frames — a ~64 KB copy is single-digit microseconds), NOT the whole inter-tick gap:
 * the fill covers the ENTIRE ring ahead of the read, so even a lagged tick reads frames a prior tick
 * already filled. It must also stay <= the app's steady-state lead (>= one ALSA period, min 256 frames),
 * or the fill's near edge reads not-yet-written data — the PipeWire skipping. 64 frames satisfies both. */
#define CLARETT_TX_GUARD_FRAMES	(4 * CLARETT_FRAG_FRAMES)	/* 64 frames */

/*
 * Smallest ALSA buffer offered. The buffer used to be PINNED to the 4096-frame ring, which set playback
 * latency for any app that keeps the buffer full — measured on an 8Pre at a 16-frame period: delay 4000
 * frames, 83 ms, against the 1.75 ms the DAW believed it had asked for. The buffer is now a power-of-two
 * divisor of the ring instead, which keeps ALSA frame k and ring frame k wrapping coherently.
 *
 * The floor is twice CLARETT_TX_GUARD_FRAMES, and the factor of two is the point rather than the value:
 * an app's steady-state lead cannot exceed the buffer, the fill refuses to write within GUARD frames of
 * the engine's read position, and the guard has to stay BELOW that lead or the fill's near edge reads
 * frames the app has not written yet — the skipping this guard was introduced to cure.
 */
#define CLARETT_MIN_BUFFER_FRAMES	(2 * CLARETT_TX_GUARD_FRAMES)	/* 128 frames */

/*
 * The guard as a lever. Its job is ANTI-TEARING: keeping the fill clear of the engine's current read
 * position. What it is not is a latency term — the ALSA buffer and the reported delay are unchanged
 * across its whole range (measured identical at 64/128/256 on an 8Pre at a 512-frame buffer). It sets a
 * deadline instead: a frame has to be in the ALSA buffer roughly this many frames before the engine
 * reaches its ring slot, or the engine plays whatever was already there.
 *
 * The text here used to claim the opposite risk — that a guard EXCEEDING the app's steady-state lead
 * would make the fill's near edge hand the engine frames the app had not written — and advised turning
 * the guard DOWN if skipping appeared. Neither half is supported by measurement. On hardware (8Pre), via
 * a device-internal digital loopback carrying a sample-counter ramp checked frame by frame: a client
 * pinning its lead at 48 frames against a 64-frame guard was indistinguishable from one leading by 128,
 * with break counts tracking the client's OWN underrun count and not the lead at all. And with a normal
 * full-buffer client every guard from 16 to 96 came out equally clean, at a 32- and a 64-frame period
 * alike (single-digit breaks throughout). So there is no measured basis for lowering it, and none for
 * raising it either.
 *
 * UNRESOLVED, and the reason the floor stays at one fragment rather than being raised: the same sweep
 * driven by a SYNTHETIC client that pins a short lead showed guard 16 and 32 tearing catastrophically —
 * tens of thousands of whole-buffer skip/repeat pairs per 25 s — while 64 stayed in single digits, 3/3.
 * That did not reproduce with an ordinary client at either period, so it is unattributed, and plausibly
 * an artifact of that client rather than a property of the driver. Do not act on it without a retest,
 * and note that everything above was measured through a host with a large periodic firmware stall.
 *
 * Clamped per fill to half the ALSA buffer, so the guard is satisfied by any app that keeps its buffer
 * even half full, and floored at one fragment. If skipping appears at a small buffer, the honest fix is
 * to bound the fill by the app's appl_ptr instead of assuming a lead.
 */
static unsigned int tx_guard = CLARETT_TX_GUARD_FRAMES;
module_param(tx_guard, uint, 0644);
MODULE_PARM_DESC(tx_guard,
		 "Frames the playback fill stays clear of the engine's TX read position (default 64, "
		 "clamped to half the ALSA buffer, floored at one 16-frame fragment). Measured to have "
		 "no effect on reported latency; see clarett_pcm.c before changing it.");

/*
 * Largest ALSA buffer offered, in frames; 0 = the whole 4096-frame ring, the default.
 *
 * An optional hard cap, and no longer the latency control. What it used to be for is now done by the
 * period rule (CLARETT_MAX_PERIODS): alsa-lib's snd_pcm_hw_params_choose() resolves every parameter with
 * set_first (the minimum) EXCEPT BUFFER_SIZE, which it resolves with set_last, so an app that pins only
 * the period — most of them, including DAWs that display a period count they never actually request — is
 * handed the largest buffer allowed. With nothing but the ring as the limit, a DAW at a 16-frame period ran
 * 85 ms of playback latency. The rule ties that largest value to the app's own period instead.
 *
 * It used to default to CLARETT_MIN_BUFFER_FRAMES, which made the ceiling equal the floor and PINNED every
 * client at 128 frames whatever it asked for — and that broke a client moving audio in blocks larger than
 * the pin (see CLARETT_MAX_PERIODS). Setting it now caps every client below what the rule would allow; a
 * value at the floor reinstates the pin. Lowering it does not break PipeWire, measured on hardware twice
 * (2Pre and 8Pre): PipeWire negotiates exactly the advertised ceiling at every value down to the floor and
 * adapts by shrinking the ALSA node's period — its graph quantum never moves.
 *
 * The floor is measured, not assumed. Swept 128..4096 on a stall-free host against the widest device in
 * the range (60 capture channels at 48 kHz, client at SCHED_FIFO), every ceiling delivered its EXACT
 * expected period count with no client overrun, no late tick, no engine overrun and no bad read. The
 * governing quantity is the servicer's worst-case excess over the nominal period, which measured
 * ~205-233 us and — the part that matters — did NOT scale with the period, so it is scheduling overhead
 * rather than anything the buffer size influences. Against a 2.7 ms buffer that is a ~11x margin, so the
 * default is not marginal even at the widest geometry.
 *
 * To decide whether a given host needs a larger ceiling, do NOT read gapmax directly: it tracks the
 * NOMINAL period (period/rate) plus that ~220 us, so the same figure means health at a large period and
 * ruin at a small one. Divide by the nominal period first. The unambiguous signals are readmax (tens of
 * microseconds on a healthy host; tens of MILLISECONDS means a platform freeze caught mid-readl) and the
 * late/overrun/badread counters. A host showing a ~40 ms readmax needs a buffer above its stall — ask the
 * app for one; with this left at the ring every size up to 85 ms is available — and no smaller buffer will
 * help it, because the stall is not a buffering problem.
 */
static unsigned int max_buffer;
module_param(max_buffer, uint, 0644);
MODULE_PARM_DESC(max_buffer,
		 "Largest ALSA buffer offered, in frames (rounded down to a power of two, floored at "
		 "128; default 0 = the full 4096-frame ring). Buffers are otherwise limited to four "
		 "periods of the app's own period, so this is only needed as a hard cap.");

/* The advertised buffer ceiling in frames: the ring, or the max_buffer override rounded down to a power
 * of two so the ceiling is itself an attainable value under the pow2 constraint. */
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
 * Called from the servicer kthread on every 0x300 period event with the number of frames the engine
 * advanced since the last event (the 0x300 counter delta * CLARETT_CTR_FRAMES — self-calibrating to the
 * real hardware period). One engine clock drives BOTH directions of the full-duplex ring:
 *
 *   capture  — copy the add_frames the engine just WROTE from the RX ring into the capture ALSA buffer
 *              (behind the engine's write pointer).
 *   playback — refill the TX ring AHEAD of the engine's read pointer from the playback ALSA buffer, so
 *              the audio the app queued is in place before the engine reads it.
 *
 * Both rings map frame k at (k mod ring_frames), and the ALSA buffers are pinned to the same frame count,
 * so ring offset == ALSA offset in each direction. The pcm_lock serialises these copies against hw_free,
 * which clears the substream pointer and frees the ALSA buffer; period_elapsed is called after unlocking
 * (the substream object itself lives until close).
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

	/* Capture drain: the add_frames the engine just wrote, from the current ring position (slot-aware) to
	 * the matching position in the stream's own ALSA buffer (offset by where it attached). */
	if (cs && cs->runtime->dma_area) {
		u32 frame = (u32)c->model->capture_channels * 4;
		u32 ring  = clarett_rx_ring_bytes(c) / frame;
		u32 abuf  = cs->runtime->buffer_size;		/* <= ring, and divides it */
		u32 n     = min(add_frames, abuf);
		u32 skip  = add_frames - n;			/* lag longer than the buffer: keep the NEWEST n */
		u64 q = c->pcm_frames + skip;
		u64 aq = c->pcm_frames + skip - c->pcm_base;
		u32 pos = do_div(q, ring);
		u32 apos = do_div(aq, abuf);

		/*
		 * Clamped to the ALSA buffer rather than the ring, and skipped forward to the newest n frames.
		 * The app has lost the rest either way (ALSA will xrun on the hw_ptr jump), but it must be
		 * handed the most recent audio, not the oldest. Reachable in normal operation: the ~42 ms
		 * platform freeze advances the engine ~2000 frames, far past a small buffer.
		 */
		clarett_rx_drain(c, cs->runtime->dma_area, apos, pos, n, abuf);
	}

	/* Playback fill: refresh the whole runway ahead of the engine (from GUARD past the read position to
	 * just before it), so a lagged tick can never underfill and the current read is never torn. */
	if (ps && ps->runtime->dma_area && READ_ONCE(c->play_running)) {
		u32 frame = (u32)c->model->playback_channels * 4;
		u32 ring  = clarett_tx_ring_bytes(c) / frame;
		u32 abuf  = ps->runtime->buffer_size;		/* <= ring, and divides it */
		u32 guard = clamp_t(u32, READ_ONCE(tx_guard), CLARETT_FRAG_FRAMES, abuf / 2);

		if (ring > guard) {
			u64 q = c->pcm_frames + guard;
			u64 aq = c->pcm_frames + guard - c->play_base;
			u32 start = do_div(q, ring);		/* hardware ring position */
			u32 astart = do_div(aq, abuf);		/* same frame in the ALSA buffer */

			/* Still the whole runway: with abuf dividing the ring the fill tiles the buffer, so a
			 * lagged tick reads audio at most one buffer stale instead of a whole ring pass. */
			clarett_tx_fill(c, ps->runtime->dma_area, astart, start, ring - guard, abuf);
		}
	}

	c->pcm_frames += add_frames;

	/* Deliver period boundaries only between trigger START and STOP (the *_running gates). Period indices
	 * are counted on each direction's own clock (pcm_frames - base), matching what .pointer reports. */
	if (cs && READ_ONCE(c->pcm_running)) {
		u64 period = div_u64(c->pcm_frames - c->pcm_base, cs->runtime->period_size);

		if (period != c->pcm_last_period) {
			c->pcm_last_period = period;
			cap_elapsed = true;
		}
	}
	if (ps && READ_ONCE(c->play_running)) {
		u64 period = div_u64(c->pcm_frames - c->play_base, ps->runtime->period_size);

		if (period != c->play_last_period) {
			c->play_last_period = period;
			play_elapsed = true;
		}
	}

	mutex_unlock(&c->pcm_lock);

	/* Outside the lock (period_elapsed takes the stream lock; the substreams live until close). */
	if (cap_elapsed)
		snd_pcm_period_elapsed(cs);
	if (play_elapsed)
		snd_pcm_period_elapsed(ps);
}

/*
 * dyn_period duplex lock (option a): once either direction has pinned the session's period (clarett_pcm_hw_params
 * -> c->lock_period), constrain the other direction's period to the same frame count. DAWs drive the card as a
 * single duplex device with one buffer size, so this takes nothing real away; it guarantees the two directions
 * share the one RX marker cadence the engine is armed with. No lock pinned yet -> leave the period free.
 */
static int clarett_rule_lock_period(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
	struct clarett *c = rule->private;
	u32 locked = READ_ONCE(c->lock_period);
	struct snd_interval *ps = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval t;

	if (!locked)
		return 0;
	snd_interval_any(&t);
	t.min = t.max = locked;
	t.integer = 1;
	return snd_interval_refine(ps, &t);
}

/*
 * Most periods the ALSA buffer may hold — except where the period is so small that this many of them fall
 * below CLARETT_MIN_BUFFER_FRAMES, where the floor wins (a 16-frame period may have 8).
 *
 * This is what bounds latency for an app that pins only the period: alsa-lib resolves BUFFER_SIZE with
 * set_last, so such an app is handed the largest buffer the constraints allow, and tying that largest value
 * to the period it chose keeps its latency proportional to its own request. A fixed ceiling cannot do that
 * without also shortchanging clients that ask for more. Pinned at 128 frames (the old max_buffer default),
 * it gave a JUCE client — which asks for four periods of its block size and then reads and writes whole
 * blocks — 4 x 32 = 128 frames for any block of 64 or more. From a 128-frame block up, its read-then-write
 * loop overran capture and underran playback every cycle, and because JUCE sets the stop threshold to the
 * boundary nothing stopped the stream: it was heard as garbled, repeating audio rather than as dropouts.
 *
 * Four is what both JUCE and PipeWire request, so each gets exactly what it asks for.
 */
#define CLARETT_MAX_PERIODS	4

/* BUFFER_SIZE <= max(CLARETT_MAX_PERIODS * period, floor), taken at the largest period still allowed. */
static int clarett_rule_buffer_by_period(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *ps = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval *bs = hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	struct snd_interval t;

	if (ps->max > UINT_MAX / CLARETT_MAX_PERIODS)
		return 0;			/* period still unbounded: nothing to say yet */
	snd_interval_any(&t);
	t.max = max_t(unsigned int, CLARETT_MAX_PERIODS * ps->max, CLARETT_MIN_BUFFER_FRAMES);
	t.integer = 1;
	return snd_interval_refine(bs, &t);
}

/* The same relation read the other way, so refinement converges from either side: a buffer above the
 * floor needs a period of at least 1/CLARETT_MAX_PERIODS of it. */
static int clarett_rule_period_by_buffer(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *bs = hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
	struct snd_interval *ps = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
	struct snd_interval t;

	if (bs->min <= CLARETT_MIN_BUFFER_FRAMES)
		return 0;
	snd_interval_any(&t);
	t.min = DIV_ROUND_UP(bs->min, CLARETT_MAX_PERIODS);
	t.integer = 1;
	return snd_interval_refine(ps, &t);
}

/*
 * Advertised rate set. 44.1 and 48 kHz (single speed) are always offered; 88.2/96 (double) and 176.4/192
 * (quad) are added up to the effective cap — the max_rate module override if set, else the model's
 * hardware-confirmed clarett_model.max_rate. All six are SET_CLOCK enums the device lists.
 */
static unsigned int clarett_rate_caps(struct clarett *c, unsigned int *rmin, unsigned int *rmax)
{
	unsigned int cap = max_rate ? max_rate : c->model->max_rate;
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

static int clarett_pcm_open(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u8 chans   = play ? c->model->playback_channels : c->model->capture_channels;
	size_t buf = play ? clarett_tx_ring_bytes(c) : clarett_rx_ring_bytes(c);
	u32 frame  = (u32)chans * 4;
	/*
	 * Minimum period. Fixed path: one 256-frame hardware IRQ period (CLARETT_IRQ_DESCS descriptors) — a fixed
	 * tiny period forced PipeWire into a rigid 5 ms cadence it serviced badly (audible skipping), so a floor
	 * plus a step lets it pick a comfortable larger period. dyn_period lowers the floor to the finest verified
	 * cadence (CLARETT_DYN_MIN_FRAMES) and derives the marker cadence from whatever period the app picks
	 * (clarett_pcm_prepare). The BUFFER is pinned to the ring on the fixed path; under dyn_period it is any
	 * power-of-two fraction of it, which is what keeps ALSA frame k and ring frame k wrapping together.
	 */
	u32 min_frames = dyn_period ? CLARETT_DYN_MIN_FRAMES : (CLARETT_IRQ_DESCS * CLARETT_FRAG_FRAMES);
	u32 min_period = min_frames * frame;
	int err;

	runtime->hw = clarett_pcm_hw;
	runtime->hw.rates            = clarett_rate_caps(c, &runtime->hw.rate_min, &runtime->hw.rate_max);
	runtime->hw.channels_min     = chans;
	runtime->hw.channels_max     = chans;
	/* Ceiling: the ring, unless max_buffer lowers it. What an app that pins only the period actually
	 * receives is bounded lower still, by the period rule registered below (CLARETT_MAX_PERIODS). */
	if (dyn_period)
		buf = (size_t)clarett_buffer_max_frames(buf / frame) * frame;

	runtime->hw.buffer_bytes_max = buf;
	runtime->hw.period_bytes_min = min_period;
	runtime->hw.period_bytes_max = buf / 2;			/* periods_min = 2 */
	runtime->hw.periods_max      = buf / min_period;

	if (dyn_period) {
		/*
		 * The buffer is a power-of-two FRAME count between CLARETT_MIN_BUFFER_FRAMES and the ring.
		 * Pow2 is what makes it divide the 4096-frame ring, so ALSA frame k and ring frame k wrap
		 * together and the per-period copies stay a straight mapping (clarett_rx_drain/tx_fill take
		 * the buffer separately from the ring for exactly this). Constrained in FRAMES, not bytes:
		 * the frame stride is channels*4, which is not a power of two on most of the line.
		 */
		err = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
						   (size_t)CLARETT_MIN_BUFFER_FRAMES * frame, buf);
		if (err < 0)
			return err;
		err = snd_pcm_hw_constraint_pow2(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE);
		if (err < 0)
			return err;
		/*
		 * The period becomes the RX marker cadence (irq_descs = period/16), which must divide
		 * CLARETT_STREAM_NDESC so the markers place evenly to the wrap. A power-of-two frame count
		 * guarantees that (16..2048 all divide the 4096-frame ring), and DAW/PipeWire buffers are pow2.
		 */
		err = snd_pcm_hw_constraint_pow2(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
		if (err < 0)
			return err;
		/* At most CLARETT_MAX_PERIODS of the app's own period (above the floor), in both directions. */
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
		/* Lock both directions to one period (option a). */
		return snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					   clarett_rule_lock_period, c,
					   SNDRV_PCM_HW_PARAM_PERIOD_SIZE, -1);
	}
	/* Legacy fixed cadence: buffer pinned to the ring, period a whole number of 256-frame hardware
	 * periods. Left as it was — this path exists as the known-good fallback. */
	err = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, buf, buf);
	if (err < 0)
		return err;
	return snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, min_period);
}

/*
 * Pin the session's shared period the first time a direction is configured (dyn_period only). The other
 * direction's open() rule reads this and is constrained to match, so both arm the one marker cadence.
 */
static int clarett_pcm_hw_params(struct snd_pcm_substream *ss, struct snd_pcm_hw_params *params)
{
	struct clarett *c = snd_pcm_substream_chip(ss);

	if (dyn_period)
		WRITE_ONCE(c->lock_period, params_period_size(params));
	return 0;
}

/*
 * Detach a direction and, once BOTH are gone, stop the shared engine. Clearing the substream pointer
 * under pcm_lock (and draining any in-flight tick) guarantees clarett_pcm_tick() stops touching this
 * runtime's dma_area before the core frees it. On playback teardown re-silence the TX ring so a
 * surviving capture stream does not loop stale playback audio out the DACs.
 */
static void clarett_pcm_detach(struct clarett *c, struct snd_pcm_substream *ss)
{
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;

	mutex_lock(&c->pcm_lock);
	if (play) {
		c->pcm_play_sub = NULL;
		WRITE_ONCE(c->play_running, false);
		clarett_zero_tx(c);
	} else {
		c->pcm_sub = NULL;
		WRITE_ONCE(c->pcm_running, false);
	}
	mutex_unlock(&c->pcm_lock);

	if (!c->pcm_sub && !c->pcm_play_sub) {
		WRITE_ONCE(c->lock_period, 0);	/* last user out: release the shared-period lock (dyn_period) */
		clarett_engine_stop(c);		/* and tear the engine down */
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
 * Stage-1 stream-config handshake. The VM re-issues SET_CLOCK + the
 * no-arg session lifecycle (0x6004 ×2 / 0x6005) in-session, immediately before every engine arm. The device
 * resets its stream config when idle, so nothing established earlier survives to PCM-arm time and this
 * handshake has to run at every arm. It is deliberately the re-run-safe SUBSET, not a full bring-up.
 * Process context, so the mailbox is safe.
 *
 * Stage 2: the per-channel CONFIG_PUSH burst (model->stream_tx_ids / stream_rx_ids) re-declares which physical
 * inputs feed which DMA stream channels — without it the engine arms cleanly but no samples are routed in and
 * 0x300 never ticks (periods=0). Wire order (from a 2Pre stream-start capture): SET_CLOCK, GET_6.2, GET_7.2, push tx
 * ids, GET_7.3, push rx ids, then the lifecycle commands. Non-fatal: log and proceed even if a command errors.
 */
static void clarett_stream_handshake(struct clarett *c, unsigned int rate)
{
	const struct clarett_model *m = c->model;
	u8 clk[8], id[2];
	int e_clk, e_en1, e_en2, e_commit, pushes = 0, push_err = 0, batch_err = 0, i;
	int clk_src = clarett_clock_source(c);

	if (!rate)
		rate = CLARETT_DEFAULT_RATE;
	WRITE_ONCE(c->cur_rate, rate);		/* published at /proc/asound/cardN/clarett */
	clarett_put_le32(clk,     rate);
	clarett_put_le32(clk + 4, clk_src);

	e_clk = clarett_fcp(c, FCP_SET_CLOCK, clk, sizeof(clk));

	/* Per-channel routing push (skipped if the model has no captured ids). */
	if (m->n_stream_tx_ids || m->n_stream_rx_ids) {
		clarett_fcp(c, FCP_GET_62, NULL, 0);
		clarett_fcp(c, FCP_GET_72, NULL, 0);
		for (i = 0; i < m->n_stream_tx_ids; i++) {
			id[0] = m->stream_tx_ids[i]; id[1] = 0;
			push_err |= clarett_fcp(c, FCP_CONFIG_PUSH, id, sizeof(id));
			pushes++;
		}
		clarett_fcp(c, FCP_GET_73, NULL, 0);
		for (i = 0; i < m->n_stream_rx_ids; i++) {
			id[0] = m->stream_rx_ids[i]; id[1] = 0;
			push_err |= clarett_fcp(c, FCP_CONFIG_PUSH, id, sizeof(id));
			pushes++;
		}
	}

	/*
	 * The vendor's pre-arm RE-INIT batch (from a 4Pre boot-to-stream capture). Our engine
	 * state at arm is byte-identical to the vendor's — its failing arms read 0x218=0xe
	 * 0x21c=0xd->0xe 0x318=0x3 0x31c=0x3 and so do we — and it arms and stalls exactly as we do
	 * four times over. What it does differently is issue this batch, then re-arm once, after
	 * which the 0x300 counter advances. The commands look like a subset of probe-time bring-up
	 * (subsystem enables + count queries), re-issued per stream start; semantics are not decoded,
	 * so this is a verbatim replay. ids 1..8 and idx 0..5 are as observed on the 4Pre.
	 */
	if (stream_batch) {
		static const u32 count_queries[] = { 0x001000, 0x002000, 0x003000, 0x004000 };
		u8 arg[4];

		clarett_fcp(c, FCP_INIT_2, NULL, 0);
		for (i = 1; i <= 8; i++) {
			arg[0] = i; arg[1] = 0;
			batch_err |= clarett_fcp(c, FCP_INIT_1, arg, 2);
		}
		clarett_fcp(c, FCP_INIT_2, NULL, 0);
		for (i = 0; i < (int)ARRAY_SIZE(count_queries); i++)
			batch_err |= clarett_fcp(c, count_queries[i], NULL, 0);
		for (i = 0; i < 6; i++) {
			clarett_put_le32(arg, i);
			batch_err |= clarett_fcp(c, 0x004001, arg, 4);
		}
	}

	/*
	 * The pre-arm triple, in the vendor's order: 0x6004, 0x6002, 0x6005 — NOT 0x6004 twice.
	 * Every occurrence of these opcodes in the 4Pre boot-to-stream capture is that triple (sometimes
	 * doubled for full duplex, which is where the old "VM issues twice" note came from), and the
	 * triple at 21:58:18.70 is what immediately precedes the one arm that streams: the vendor
	 * arms and fails exactly as we do — 0x110=7, one period event, 0x110=0 + 0x100=0xf, retry —
	 * four times over, then issues this batch, re-arms once, and the counter starts advancing.
	 */
	e_en1    = clarett_fcp(c, FCP_STREAM_ENABLE, NULL, 0);
	e_en2    = clarett_fcp(c, FCP_GET_62, NULL, 0);
	e_commit = clarett_fcp(c, FCP_STREAM_COMMIT, NULL, 0);

	dev_dbg(&c->pci->dev,
		 "stream-handshake: SET_CLOCK{%u,%u}=%d CONFIG_PUSH=%d(err=%d) batch=%s(err=%d) "
		 "0x6004=%d 0x6002=%d 0x6005=%d\n",
		 rate, clk_src, e_clk, pushes, push_err, stream_batch ? "yes" : "off", batch_err,
		 e_en1, e_en2, e_commit);
}

/*
 * Prepare a direction and, if the shared full-duplex engine is not already running (the other direction
 * armed it), arm it. The engine is ONE full-duplex stream (block 0 = TX/playback, block 1 = RX/capture)
 * with a single frame clock; whichever direction prepares first arms it and the other just attaches at
 * the current clock position. Re-preparing an already-armed direction (xrun recovery) does not re-arm —
 * the free-running engine is undisturbed. Process context, so the mailbox handshake is safe here.
 *
 * Either way the direction records its attach point in the shared clock. ALSA resets hw_ptr to 0 on every
 * prepare, so .pointer must report from there; without the base, a stream attaching to an already-running
 * engine (a second direction, or the SAME one recovering from an xrun) saw its first .pointer return the
 * engine's absolute position mod buffer_size. That reads as an enormous hw_ptr jump and the core xruns it
 * within a tick — which then re-prepares, and xruns again. Audio stayed dead until every substream closed
 * and the engine was torn down (a module reload, in practice).
 */
static int clarett_pcm_prepare(struct snd_pcm_substream *ss)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	bool arm;
	dma_addr_t r0, r1;

	mutex_lock(&c->pcm_lock);
	arm = !c->stream_on;
	if (arm) {
		/*
		 * CLAIM THE ARM UNDER THE LOCK. c->stream_on used to be published only at the END of
		 * clarett_engine_arm(), several milliseconds later — clarett_stream_handshake() runs ~20
		 * mailbox commands in between — so two prepares landing inside that window both decided they
		 * were first. Both then armed the engine AND called clarett_engine_run(), whose unconditional
		 * `c->stream_svc = kthread_run(...)` overwrote the first thread's pointer, orphaning a
		 * SCHED_FIFO kthread that nothing could ever kthread_stop(). On rmmod that orphan keeps
		 * executing module text while devres frees it: a panic, not a warning.
		 *
		 * Reproduced by starting `arecord &` and `aplay` together at dyn_period cadence 4: two
		 * `engine armed` lines 240 us apart, two servicers, and only one `stopped` line at teardown.
		 * PipeWire spaces its two prepares widely enough to have hidden this; a DAW opening duplex
		 * would not. Publishing here is safe: engine_arm sets it again (idempotent), engine_stop's
		 * `if (!c->stream_on) return` still holds, and the only other reader is the 0x400 notify gate,
		 * which merely starts suppressing relays a few ms earlier.
		 */
		c->stream_on = true;
		c->pcm_frames = 0;	/* first direction in: the shared clock starts here */
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

	/* Latch this rate's live capture width so the drain blanks the S/MUX-removed tail. */
	if (!play)
		clarett_set_rx_live(c, ss->runtime->rate);

	if (!arm)			/* engine already armed: just attached at the current clock */
		return 0;

	/*
	 * dyn_period: match the RX marker cadence to the negotiated (shared) period before arming, so the
	 * engine raises a 0x300 period at the app's buffer granularity. Rebuild the descriptor ring at the new
	 * cadence — the engine is not yet armed on this (first-direction) path, so the table is quiescent. The
	 * cadence must divide NDESC; the pow2 period constraint in open() already guarantees that.
	 */
	if (dyn_period && c->lock_period) {
		u32 descs = clamp_t(u32, c->lock_period / CLARETT_FRAG_FRAMES, 1, CLARETT_STREAM_NDESC / 2);

		if (descs != c->irq_descs) {
			c->irq_descs = descs;
			clarett_build_rings(c);
			wmb();		/* descriptor table visible to the device before we arm */
		}
	}

	/* First direction in: arm the full-duplex engine. block 0 (TX) base, block 1 (RX) base — in
	 * descriptor mode each points at its table; r1 offset is clarett_stream_r1_off() for the mode. */
	r0 = c->stream_dma;
	r1 = c->stream_dma + clarett_stream_r1_off(c);

	/* In-session stream-config handshake immediately before arming, matching the VM (handshake ->
	 * program regs -> arm). Establishes the device's stream routing/mode for this session. */
	clarett_stream_handshake(c, ss->runtime->rate);
	clarett_engine_arm(c, r0, r1);

	/*
	 * ACK from here, not the trigger: the engine bursts immediately after arm and stalls within ms if
	 * unserviced, so the servicer must ACK 0x300 the instant it is armed. The triggers only gate whether
	 * completed periods are reported to ALSA (pcm_running / play_running).
	 */
	WRITE_ONCE(c->stream_run, true);
	clarett_engine_run(c);
	return 0;
}

/*
 * Atomic context (stream lock held): only flip this direction's servicing gate. The engine is already
 * armed and the kthread running from prepare(); START begins reporting that direction's periods from the
 * current clock, STOP halts them. Full teardown happens in close()/hw_free, which can sleep.
 */
static int clarett_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct clarett *c = snd_pcm_substream_chip(ss);
	bool play = ss->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u64 base = play ? c->play_base : c->pcm_base;
	u64 period = div_u64(c->pcm_frames - base, ss->runtime->period_size);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (play) {
			c->play_last_period = period;
			WRITE_ONCE(c->play_running, true);
		} else {
			c->pcm_last_period = period;
			WRITE_ONCE(c->pcm_running, true);
		}
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		if (play)
			WRITE_ONCE(c->play_running, false);
		else
			WRITE_ONCE(c->pcm_running, false);
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
	u64 frames = READ_ONCE(c->pcm_frames) - base;

	/* Position on THIS stream's clock (frames since it attached), % buffer_size — 64-bit-safe
	 * (do_div takes a u32 divisor; buffer_size fits easily). */
	return do_div(frames, ss->runtime->buffer_size);
}

static const struct snd_pcm_ops clarett_pcm_ops = {
	.open      = clarett_pcm_open,
	.close     = clarett_pcm_close,
	.hw_params = clarett_pcm_hw_params,
	.hw_free   = clarett_pcm_hw_free,
	.prepare   = clarett_pcm_prepare,
	.trigger   = clarett_pcm_trigger,
	.pointer   = clarett_pcm_pointer,
};

/*
 * Build both static descriptor tables in the contiguous hardware buffer ([TX ring][RX ring], each =
 * [table][samples]). Each entry is a bare 8-byte LE fragment bus address, 0x100-aligned; the per-direction
 * fragment is clarett_frag_bytes(channels) (a whole-frame, 0x100-aligned span). The LAST entry carries the
 * wrap flag in its low bits (TX 0x01, RX 0x03) — matching the live 8PreX vendor table from the RAM dump —
 * and there is NO zero terminator. dma_alloc_coherent returns zeroed memory, so the TX samples are already
 * silence and the RX sample area (the engine's write target) starts clean.
 *
 * No pre-fill is needed: the engine reads the TABLE (valid, non-null entries -> it clocks) and writes
 * audio INTO the sample area, so a clean zeroed sample area is correct and fault-free.
 */
static void clarett_build_rings(struct clarett *c)
{
	size_t tbl     = clarett_pcm_tbl_bytes();

	size_t tx_ring = clarett_pcm_tx_ring(c);
	u32 tx_frag = clarett_frag_bytes(c->model->playback_channels);
	u32 tx_slot = c->tx_slot;		/* TX descriptor stride: audio bytes, or a padded slot */
	u32 rx_slot = c->rx_slot;		/* RX descriptor stride: audio bytes, or a padded slot (experiment) */
	__le64 *tx_tbl = (__le64 *)c->stream_buf;
	__le64 *rx_tbl = (__le64 *)((u8 *)c->stream_buf + tx_ring);
	dma_addr_t tx_smp = c->stream_dma + tbl;
	dma_addr_t rx_smp = c->stream_dma + clarett_stream_rx_off(c);	/* page-aligned RX sample area */
	unsigned int i;

	for (i = 0; i < CLARETT_STREAM_NDESC; i++) {
		tx_tbl[i] = cpu_to_le64(tx_smp + (u64)i * tx_slot);	/* slotted: non-contiguous when padded */
		rx_tbl[i] = cpu_to_le64(rx_smp + (u64)i * rx_slot);	/* slotted: fragments non-contiguous when padded */
		/* Periodic RX IRQ marker: the engine raises a counted 0x300 period when it
		 * consumes an IRQ-flagged descriptor. Every clarett_irq_descs(c)-th one (default 16, matching
		 * the vendor's ~14-descriptor cadence; dyn_period tightens it to the chosen ALSA period).
		 * TX carries no periodic marker (vendor TX flags only the last). */
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

	err = snd_pcm_new(c->card, c->model->name, 0, 1, 1, &pcm);	/* 1 playback, 1 capture */
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

	/* Each direction caps its own buffer at its ring in open() (and may pick any power-of-two fraction
	 * of it); preallocate the larger so the widest case fits. The ALSA buffers are plain memory we
	 * memcpy to/from the hardware rings (not DMA'd directly). */
	rxbuf = clarett_rx_ring_bytes(c);
	txbuf = clarett_tx_ring_bytes(c);
	prealloc = max(rxbuf, txbuf);
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV, &c->pci->dev, prealloc, prealloc);

	/* Debug: the probe summary in clarett_probe() already states that a PCM registered and how wide
	 * it is; the ring/buffer detail here is bring-up instrumentation. */
	dev_dbg(&c->pci->dev,
		 "PCM registered (playback %uch / capture %uch, S32_LE @%u, bufs tx=%zu rx=%zu B @%pad)\n",
		 c->model->playback_channels, c->model->capture_channels, CLARETT_PCM_RATE,
		 txbuf, rxbuf, &c->stream_dma);

	return 0;
}
