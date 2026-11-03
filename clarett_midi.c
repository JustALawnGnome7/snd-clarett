// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — DIN MIDI (ALSA rawmidi).
 *
 * The MIDI transport is register PIO through one memory-mapped UART at BAR0
 * REG_MIDI_DATA (0x58c), not the FCP mailbox or the audio DMA. Both directions
 * move the same packed word, (count << 24) | (b2 << 16) |
 * (b1 << 8) | b0, where count = the number of valid MIDI bytes 1..3 packed
 * low->high in wire order. `count` is a plain BYTE COUNT (the SysEx F7
 * terminator is a lone count=1 word), NOT a USB-MIDI CIN, and words are chunked
 * by raw byte grouping, independent of message boundaries.
 *
 *   TX  write a word to REG_MIDI_DATA, up to three queued bytes at a time.
 *       Every write is gated on MIDI_TX_READY in REG_MIDI_STATUS: the FIFO is
 *       nine words deep and discards a word written while it is full.
 *
 *   RX  read a word from REG_MIDI_DATA; count 0 means the RX FIFO is
 *       empty. The device decides how many bytes a word carries: the 2Pre
 *       returns one byte per read, while the 8PreX also returns count=2
 *       words. RX is interrupt-driven (an idle device does zero MMIO): the
 *       shared IRQ summary REG_MIDI_STATUS carries a MIDI-RX-pending code in
 *       its low byte, the driver drains REG_MIDI_DATA to empty, then writes
 *       MIDI_IRQ_ACK_VAL to REG_MIDI_ACK to clear the interrupt.
 *
 * It is independent of the FCP control session. Every Clarett Thunderbolt model
 * has DIN MIDI; the Red range has none, so the rawmidi is created only where
 * clarett_model.has_midi is set. Which MSI vector carries the RX interrupt is
 * not known, so the ISR drains on every vector.
 */
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/rawmidi.h>
#include "clarett.h"

static bool enable_midi = true;
module_param(enable_midi, bool, 0444);
MODULE_PARM_DESC(enable_midi,
		 "Register the DIN MIDI rawmidi device (the register UART at "
		 "BAR0 0x58c). Default on.");

/*
 * TX flow control. The FIFO drains at the DIN wire rate, 3125 bytes/s or ~960
 * us per 3-byte word. The poll interval is a fraction of a word time; the
 * timeout is many, so it expires only if the UART has stopped draining.
 */
#define CLARETT_MIDI_TX_POLL_US     200
#define CLARETT_MIDI_TX_TIMEOUT_US  100000

/*
 * Runaway bound on the RX drain loop, above the FIFO's measured 139 bytes, so a
 * full FIFO always empties in one pass.
 */
#define CLARETT_MIDI_RX_GUARD       256

/*
 * Drain the RX FIFO, handing the bytes to ss, or discarding them when ss is
 * NULL, and acknowledge the RX interrupt if anything was read. Caller holds
 * midi_rx_lock.
 *
 * The FIFO must be emptied in ONE pass: a non-empty FIFO raises no fresh
 * interrupt, and a pass that reads data but skips the ack leaves the interrupt
 * unarmed. A word whose count exceeds MIDI_WORD_BYTES is not data; that
 * includes a failed PCIe read (0xffffffff), which would otherwise spin emitting
 * 0xff.
 */
static void clarett_midi_drain(struct clarett *c,
			       struct snd_rawmidi_substream *ss)
{
	bool got = false;
	int guard = CLARETT_MIDI_RX_GUARD;

	while (guard > 0) {
		u32 v = clarett_rl(c, REG_MIDI_DATA);
		unsigned int n = v >> MIDI_COUNT_SHIFT, i;
		u8 bytes[MIDI_WORD_BYTES];

		/* FIFO empty */
		if (!n)
			break;
		/* failed transaction — never a real word */
		if (n > MIDI_WORD_BYTES)
			break;
		for (i = 0; i < n; i++)
			bytes[i] = v >> (i * 8);
		guard -= n;
		got = true;
		if (ss)
			snd_rawmidi_receive(ss, bytes, n);
	}

	if (got)
		clarett_wl(c, REG_MIDI_ACK, MIDI_IRQ_ACK_VAL);
}

/*
 * From the ISR, on every vector. The FIFO is drained and acked whether or not
 * the input is open, since the ack is what re-arms the interrupt; the bytes go
 * to the substream only while it is open and triggered.
 */
void clarett_midi_irq(struct clarett *c)
{
	struct snd_rawmidi_substream *ss;

	/*
	 * Serialise: this runs for every MSI vector, and while streaming the
	 * period vectors fire on other CPUs concurrently with vec0. Two
	 * drainers of the one FIFO would reorder the bytes. Also taken by
	 * clarett_midi_resume(), with IRQs off.
	 */
	spin_lock(&c->midi_rx_lock);

	ss = READ_ONCE(c->midi_in);
	if (!READ_ONCE(c->midi_in_up) || READ_ONCE(c->midi_rx_discard))
		ss = NULL;
	clarett_midi_drain(c, ss);

	spin_unlock(&c->midi_rx_lock);
}

/*
 * Wait for the TX FIFO to accept another word. Sleeps rather than busy-waits: a
 * full FIFO takes a word time to free a slot, which is most of a millisecond.
 * Process context only.
 */
static int clarett_midi_tx_wait(struct clarett *c)
{
	u32 st;

	return read_poll_timeout(clarett_rl, st, st & MIDI_TX_READY,
				 CLARETT_MIDI_TX_POLL_US,
				 CLARETT_MIDI_TX_TIMEOUT_US,
				 false, c, REG_MIDI_STATUS);
}

/*
 * Drain the rawmidi output buffer into the TX register, up to 3 bytes per
 * write, in process context (scheduled from the output trigger).
 * snd_rawmidi_transmit() takes raw bytes regardless of message boundaries,
 * which is the device's grouping, and consumes what it returns. MIDI_TX_READY
 * is checked BEFORE taking bytes, so a stalled device leaves the stream queued.
 */
static void clarett_midi_tx_work(struct work_struct *w)
{
	struct clarett *c = container_of(w, struct clarett, midi_tx_work);
	struct snd_rawmidi_substream *ss = READ_ONCE(c->midi_out);
	u8 buf[MIDI_WORD_BYTES];

	if (!ss)
		return;

	while (READ_ONCE(c->midi_out_up)) {
		u32 word;
		int n, i;

		if (pci_dev_is_disconnected(c->pci))
			break;

		if (clarett_midi_tx_wait(c)) {
			dev_warn_ratelimited(&c->pci->dev,
					     "MIDI TX FIFO stalled; output paused\n");
			break;
		}

		n = snd_rawmidi_transmit(ss, buf, sizeof(buf));
		/* nothing more queued (a later write re-triggers us) */
		if (n <= 0)
			break;

		word = (u32)n << MIDI_COUNT_SHIFT;
		for (i = 0; i < n; i++)
			word |= (u32)buf[i] << (i * 8);

		clarett_wl(c, REG_MIDI_DATA, word);
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
	/* no TX work touches this substream after close */
	cancel_work_sync(&c->midi_tx_work);
	WRITE_ONCE(c->midi_out, NULL);
	return 0;
}

/*
 * Atomic context (rawmidi runtime lock held): flip the gate and kick the
 * (process-context) drain.
 */
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

/*
 * Atomic context: gate whether the ISR pushes received bytes to this substream.
 * The RX interrupt is enabled at attach (REG_IRQ0_ENABLE).
 */
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
	struct snd_rawmidi_str *out, *in;
	struct snd_rawmidi *rmidi;
	char name[40];
	int err;

	if (!enable_midi || !c->model->has_midi)
		return 0;

	/*
	 * device index 0 in the rawmidi namespace (independent of the PCM
	 * device index); 1 out + 1 in.
	 */
	err = snd_rawmidi_new(c->card, "Clarett MIDI", 0, 1, 1, &rmidi);
	if (err < 0)
		return err;

	snprintf(name, sizeof(name), "%s MIDI", c->model->name);
	strscpy(rmidi->name, name, sizeof(rmidi->name));
	rmidi->info_flags = SNDRV_RAWMIDI_INFO_OUTPUT |
			    SNDRV_RAWMIDI_INFO_INPUT |
			    SNDRV_RAWMIDI_INFO_DUPLEX;
	rmidi->private_data = c;
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_OUTPUT,
			    &clarett_midi_out_ops);
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_INPUT,
			    &clarett_midi_in_ops);

	out = &rmidi->streams[SNDRV_RAWMIDI_STREAM_OUTPUT];
	in = &rmidi->streams[SNDRV_RAWMIDI_STREAM_INPUT];
	list_for_each_entry(s, &out->substreams, list)
		strscpy(s->name, name, sizeof(s->name));
	list_for_each_entry(s, &in->substreams, list)
		strscpy(s->name, name, sizeof(s->name));

	spin_lock_init(&c->midi_rx_lock);
	INIT_WORK(&c->midi_tx_work, clarett_midi_tx_work);
	/* set LAST: the ISR uses this as the "MIDI live" gate */
	WRITE_ONCE(c->rmidi, rmidi);

	/* debug: the probe summary already reports MIDI */
	dev_dbg(&c->pci->dev,
		"MIDI registered (%s, 1 in + 1 out, register UART @0x%03x)\n",
		name, REG_MIDI_DATA);
	return 0;
}

/*
 * Cancel the TX drain before the card (and c) are freed. Safe to call on a card
 * that never created MIDI.
 */
void clarett_midi_stop(struct clarett *c)
{
	if (!READ_ONCE(c->rmidi))
		return;
	WRITE_ONCE(c->midi_out_up, false);
	WRITE_ONCE(c->midi_in_up, false);
	cancel_work_sync(&c->midi_tx_work);
}

/*
 * System sleep. Unlike clarett_midi_stop() the trigger gates stay as the open
 * substreams left them, so MIDI carries on after resume without a reopen:
 * suspend only parks the TX drain (bytes still queued in the rawmidi buffer
 * stay there), and resume restarts it.
 *
 * RX is muted across the sleep (midi_rx_discard): after a sleep the unit's RX
 * FIFO delivers a burst of stale bytes (old buffer contents) at the wake. The
 * ISR discards until the re-attach is done, and resume drains what is left
 * before delivery starts again. Bytes sent to the unit during the sleep are
 * lost either way.
 */
void clarett_midi_suspend(struct clarett *c)
{
	if (!READ_ONCE(c->rmidi))
		return;
	WRITE_ONCE(c->midi_rx_discard, true);
	cancel_work_sync(&c->midi_tx_work);
}

void clarett_midi_resume(struct clarett *c)
{
	unsigned long flags;

	if (!READ_ONCE(c->rmidi))
		return;

	spin_lock_irqsave(&c->midi_rx_lock, flags);
	clarett_midi_drain(c, NULL);
	WRITE_ONCE(c->midi_rx_discard, false);
	spin_unlock_irqrestore(&c->midi_rx_lock, flags);

	if (READ_ONCE(c->midi_out_up))
		schedule_work(&c->midi_tx_work);
}
