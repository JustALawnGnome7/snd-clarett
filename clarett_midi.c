// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett (Thunderbolt) — DIN MIDI (ALSA rawmidi).
 *
 * The MIDI transport is register PIO through a single memory-mapped MIDI UART at BAR0 REG_MIDI_DATA
 * (0x58c) — NOT the FCP mailbox, NOT the audio DMA ring. Reverse-engineered from three 2Pre
 * MMIO captures; the framing is:
 *
 *   TX  write (count << 24) | (b2 << 16) | (b1 << 8) | b0  to REG_MIDI_DATA, where count = the number of
 *       valid MIDI bytes 1..3 packed low->high in transmit order. `count` is a plain BYTE COUNT (the SysEx
 *       F7 terminator is a lone count=1 word), NOT a USB-MIDI CIN. Chunking is by raw 3-byte grouping of
 *       the outgoing byte stream, independent of message boundaries — so the TX path just hands the device
 *       whatever bytes ALSA has queued, three at a time. The vendor reads REG_MIDI_STATUS before each write
 *       (a TX-ready gate, or advisory — unconfirmed); we mirror that read.
 *
 *   RX  read REG_MIDI_DATA one byte per read = (valid << 24) | byte, valid = bit24 (MIDI_RX_VALID); a read
 *       returns 0 when the RX FIFO is empty. RX is interrupt-driven (an idle device does zero MMIO): the
 *       shared IRQ summary REG_MIDI_STATUS carries a MIDI-RX-pending code in its low byte, the driver drains
 *       REG_MIDI_DATA to empty, then writes MIDI_IRQ_ACK_VAL to REG_MIDI_ACK to clear the interrupt.
 *
 * This is the simplest possible transport — no DMA, no descriptor rings, no mailbox round-trips — and it is
 * independent of the FCP control session, so MIDI works regardless of control-plane state. DIN MIDI is
 * line-wide across the Clarett Thunderbolt range (the 2Pre carries it too), so the rawmidi is not model-gated.
 *
 * Open follow-ups (harmless first-cut assumptions, flagged inline): which MSI vector carries the MIDI RX
 * interrupt (we drain from the ISR on ANY vector, so it does not matter for correctness); whether the
 * pre-TX REG_MIDI_STATUS read is a required ready-gate (a large SysEx dump could overrun the device FIFO —
 * midi_tx_pace_us throttles if so); and whether the RX interrupt needs an explicit enable at input-open
 * (assumed already enabled by the bring-up's REG_IRQ0_ENABLE write).
 */
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/rawmidi.h>
#include "clarett.h"

static bool enable_midi = true;
module_param(enable_midi, bool, 0444);
MODULE_PARM_DESC(enable_midi,
		 "Register the DIN MIDI rawmidi device (the register UART at BAR0 0x58c). Default on.");

/*
 * Optional per-word delay (us) after each MIDI TX register write. Default 0. MIDI is 3125 bytes/s at the
 * wire, but the register FIFO could be shallower than a burst; if a large SysEx dump overruns it (garbled
 * output), a few us here paces the writes to the UART drain rate. The pre-write REG_MIDI_STATUS read may
 * already be the device's own flow control, in which case this stays 0.
 */
static int midi_tx_pace_us;
module_param(midi_tx_pace_us, int, 0644);
MODULE_PARM_DESC(midi_tx_pace_us,
		 "Delay (us) after each MIDI TX register write to throttle large SysEx (default 0 = none).");

/*
 * Drain the RX FIFO, called from the ISR (clarett_irq) on any vector — MIDI RX is register PIO and which
 * MSI vector signals it is unconfirmed, so draining unconditionally is both correct and prevents a stuck
 * MIDI interrupt from livelocking whichever vector it lands on. Bytes are pushed to the input substream only
 * while it is open and triggered, but the FIFO is ALWAYS drained and acked (an undrained FIFO keeps the
 * interrupt asserted). Runs in hard IRQ context; snd_rawmidi_receive() is designed for that.
 *
 * A failed PCIe read returns 0xffffffff, which has MIDI_RX_VALID set and a 0xff data byte — treating it as
 * data would spin forever emitting 0xff, so bail on it (device gone / transiently unreachable). The guard
 * count bounds the drain per interrupt; anything still pending re-interrupts.
 */
void clarett_midi_irq(struct clarett *c)
{
	struct snd_rawmidi_substream *ss;
	bool deliver, got = false;
	int guard = 64;

	/*
	 * Serialise: this runs for every MSI vector, and while streaming the period vectors fire on other
	 * CPUs concurrently with vec0 — two drainers of the one-byte FIFO would interleave and reorder the
	 * bytes (corrupting multi-byte messages). Hardirq-only, so plain spin_lock (local IRQs already off).
	 */
	spin_lock(&c->midi_rx_lock);

	ss = READ_ONCE(c->midi_in);
	deliver = ss && READ_ONCE(c->midi_in_up);

	while (guard-- > 0) {
		u32 v = clarett_rl(c, REG_MIDI_DATA);
		u8 byte;

		if (v == 0xffffffff)		/* failed transaction — never a real byte */
			break;
		if (!(v & MIDI_RX_VALID))	/* FIFO empty */
			break;
		byte = v & MIDI_RX_BYTE_MASK;
		got = true;
		if (deliver)
			snd_rawmidi_receive(ss, &byte, 1);
	}

	if (got)
		clarett_wl(c, REG_MIDI_ACK, MIDI_IRQ_ACK_VAL);

	spin_unlock(&c->midi_rx_lock);
}

/*
 * Drain the rawmidi output buffer into the TX register, up to 3 bytes per write. Runs in process context
 * (scheduled from the output trigger) so it can pull an arbitrarily long stream without spinning in atomic
 * context. snd_rawmidi_transmit() takes up to `count` RAW bytes irrespective of MIDI message boundaries —
 * exactly the device's 3-byte grouping — and combines peek+ack, so each call consumes what it returns.
 */
static void clarett_midi_tx_work(struct work_struct *w)
{
	struct clarett *c = container_of(w, struct clarett, midi_tx_work);
	struct snd_rawmidi_substream *ss = READ_ONCE(c->midi_out);
	u8 buf[3];

	if (!ss)
		return;

	while (READ_ONCE(c->midi_out_up)) {
		u32 word;
		int n, i;

		if (pci_dev_is_disconnected(c->pci))
			break;

		n = snd_rawmidi_transmit(ss, buf, sizeof(buf));
		if (n <= 0)			/* nothing more queued (a later write re-triggers us) */
			break;

		word = (u32)n << MIDI_TX_COUNT_SHIFT;
		for (i = 0; i < n; i++)
			word |= (u32)buf[i] << (i * 8);

		clarett_rl(c, REG_MIDI_STATUS);	/* vendor reads status before each write (ready gate?) */
		clarett_wl(c, REG_MIDI_DATA, word);

		if (midi_tx_pace_us)
			udelay(midi_tx_pace_us);
	}
}

static int clarett_midi_out_open(struct snd_rawmidi_substream *ss)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_out, ss);
	return 0;
}

static int clarett_midi_out_close(struct snd_rawmidi_substream *ss)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_out_up, false);
	cancel_work_sync(&c->midi_tx_work);	/* no TX work touches this substream after close */
	WRITE_ONCE(c->midi_out, NULL);
	return 0;
}

/* Atomic context (rawmidi runtime lock held): flip the gate and kick the (process-context) drain. */
static void clarett_midi_out_trigger(struct snd_rawmidi_substream *ss, int up)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_out_up, !!up);
	if (up)
		schedule_work(&c->midi_tx_work);
}

static int clarett_midi_in_open(struct snd_rawmidi_substream *ss)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_in, ss);
	return 0;
}

static int clarett_midi_in_close(struct snd_rawmidi_substream *ss)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_in_up, false);
	WRITE_ONCE(c->midi_in, NULL);
	return 0;
}

/* Atomic context: gate whether the ISR pushes received bytes to this substream. The RX interrupt itself is
 * assumed already enabled by the bring-up (REG_IRQ0_ENABLE); if RX never fires on hardware, an explicit
 * device enable write likely belongs here. */
static void clarett_midi_in_trigger(struct snd_rawmidi_substream *ss, int up)
{
	struct clarett *c = ss->rmidi->private_data;

	WRITE_ONCE(c->midi_in_up, !!up);
}

static const struct snd_rawmidi_ops clarett_midi_out_ops = {
	.open    = clarett_midi_out_open,
	.close   = clarett_midi_out_close,
	.trigger = clarett_midi_out_trigger,
};

static const struct snd_rawmidi_ops clarett_midi_in_ops = {
	.open    = clarett_midi_in_open,
	.close   = clarett_midi_in_close,
	.trigger = clarett_midi_in_trigger,
};

int clarett_create_midi(struct clarett *c)
{
	struct snd_rawmidi_substream *s;
	struct snd_rawmidi *rmidi;
	char name[40];
	int err;

	if (!enable_midi)
		return 0;

	/* device index 0 in the rawmidi namespace (independent of the PCM device index); 1 out + 1 in. */
	err = snd_rawmidi_new(c->card, "Clarett MIDI", 0, 1, 1, &rmidi);
	if (err < 0)
		return err;

	snprintf(name, sizeof(name), "%s MIDI", c->model->name);
	strscpy(rmidi->name, name, sizeof(rmidi->name));
	rmidi->info_flags = SNDRV_RAWMIDI_INFO_OUTPUT | SNDRV_RAWMIDI_INFO_INPUT |
			    SNDRV_RAWMIDI_INFO_DUPLEX;
	rmidi->private_data = c;
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_OUTPUT, &clarett_midi_out_ops);
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_INPUT, &clarett_midi_in_ops);

	list_for_each_entry(s, &rmidi->streams[SNDRV_RAWMIDI_STREAM_OUTPUT].substreams, list)
		strscpy(s->name, name, sizeof(s->name));
	list_for_each_entry(s, &rmidi->streams[SNDRV_RAWMIDI_STREAM_INPUT].substreams, list)
		strscpy(s->name, name, sizeof(s->name));

	spin_lock_init(&c->midi_rx_lock);
	INIT_WORK(&c->midi_tx_work, clarett_midi_tx_work);
	WRITE_ONCE(c->rmidi, rmidi);	/* set LAST: the ISR uses this as the "MIDI live" gate */

	/* Debug: the probe summary in clarett_probe() already reports that MIDI came up. */
	dev_dbg(&c->pci->dev, "MIDI registered (%s, 1 in + 1 out, register UART @0x%03x)\n",
		name, REG_MIDI_DATA);
	return 0;
}

/* Cancel the TX drain before the card (and c) are freed. Safe to call on a card that never created MIDI. */
void clarett_midi_stop(struct clarett *c)
{
	if (!READ_ONCE(c->rmidi))
		return;
	WRITE_ONCE(c->midi_out_up, false);
	WRITE_ONCE(c->midi_in_up, false);
	cancel_work_sync(&c->midi_tx_work);
}
