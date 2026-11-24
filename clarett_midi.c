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
 *       Writes are gated on MIDI_TX_READY in REG_MIDI_STATUS: the FIFO holds
 *       16 words and discards a word written while it is full, and the bit
 *       is set only while more than REG_MIDI_TX_THRESH words are free, so
 *       MIDI_TX_BURST words may follow each check. The output trigger writes
 *       until the queue is empty or the FIFO is full; in the latter case it
 *       sets MIDI_TX_IRQ_EN, and the interrupt raised when the FIFO is ready
 *       again carries on, clearing the enable once nothing is left. No
 *       polling, and no MMIO while nothing is queued.
 *
 *   RX  read a word from REG_MIDI_DATA; count 0 means the RX FIFO is
 *       empty. The device decides how many bytes a word carries: the 2Pre
 *       returns one byte per read, while the 8PreX also returns count=2
 *       words. RX is interrupt-driven (an idle device does zero MMIO).
 *
 * Both directions share one interrupt (0x100 bit28) and one acknowledgment:
 * the handler drains RX, refills TX, then writes MIDI_IRQ_ACK_VAL to
 * REG_MIDI_ACK once.
 *
 * It is independent of the FCP control session. Every Clarett Thunderbolt model
 * has DIN MIDI; the Red range has none, so the rawmidi is created only for a
 * unit that reports MIDI ports (clarett_read_midi_ports()), or, if it does not
 * answer, whose model table says it has them.
 */
#include <linux/io.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <sound/core.h>
#include <sound/rawmidi.h>
#include "clarett.h"

/*
 * Runaway bound on the RX drain loop, above the most the device buffers (128
 * one-byte words, then packed words to 159 bytes in all), so a full FIFO
 * always empties in one pass.
 */
#define CLARETT_MIDI_RX_GUARD       256

/*
 * Drain the RX FIFO, handing the bytes to ss, or discarding them when ss is
 * NULL. Returns whether anything was read; the caller acknowledges. Caller
 * holds midi_rx_lock.
 *
 * The FIFO must be emptied in ONE pass: a non-empty FIFO raises no fresh
 * interrupt, and a pass that reads data but skips the ack leaves the interrupt
 * unarmed. A word whose count exceeds MIDI_WORD_BYTES is not data; that
 * includes a failed PCIe read (0xffffffff), which would otherwise spin emitting
 * 0xff.
 */
static bool clarett_midi_drain(struct clarett *c,
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

	return got;
}

/*
 * Write queued output to the TX FIFO, up to 3 bytes per word, until the queue
 * is empty or the FIFO is full, then set MIDI_TX_IRQ_EN if bytes remain and
 * clear it if not. Caller holds midi_tx_lock with IRQs off; atomic.
 *
 * snd_rawmidi_transmit() takes raw bytes regardless of message boundaries,
 * which is the device's grouping, and consumes what it returns, so it is called
 * only once the FIFO is known to have room: MIDI_TX_READY guarantees
 * MIDI_TX_BURST free words. If the FIFO becomes ready between the last check
 * and setting the enable, the enable raises the interrupt at once, so no
 * wakeup is lost. A failed read (the device gone) stops TX.
 */
static void clarett_midi_tx_fill(struct clarett *c)
{
	struct snd_rawmidi_substream *ss = c->midi_out;
	bool more = ss && c->midi_out_up && !c->midi_tx_parked;
	u8 buf[MIDI_WORD_BYTES];

	while (more) {
		u32 st = clarett_rl(c, REG_MIDI_STATUS);
		int k;

		if (st == ~0u) {
			more = false;
			break;
		}
		if (!(st & MIDI_TX_READY))
			break;
		for (k = 0; k < MIDI_TX_BURST; k++) {
			int n = snd_rawmidi_transmit(ss, buf, sizeof(buf));
			u32 word;
			int i;

			if (n <= 0) {
				more = false;
				break;
			}
			word = (u32)n << MIDI_COUNT_SHIFT;
			for (i = 0; i < n; i++)
				word |= (u32)buf[i] << (i * 8);
			clarett_wl(c, REG_MIDI_DATA, word);
		}
	}

	if (more != c->midi_tx_irq) {
		clarett_wl(c, more ? REG_MIDI_ACK : REG_MIDI_CLR,
			   MIDI_TX_IRQ_EN);
		c->midi_tx_irq = more;
	}
}

/*
 * From the interrupt sweep, when 0x100 bit28 says the MIDI block has an
 * interrupt pending: received data, or the TX FIFO ready again while
 * MIDI_TX_IRQ_EN is set. RX is drained whether or not the input is open (the
 * bytes go to the substream only while it is open and triggered), TX is
 * refilled, and the interrupt is acknowledged once for both, every time: an
 * unacknowledged one stays pending and the device raises no other.
 */
void clarett_midi_irq(struct clarett *c)
{
	struct snd_rawmidi_substream *ss;

	/*
	 * Serialise against clarett_midi_resume()'s drain (IRQs off there):
	 * two drainers of the one FIFO would reorder the bytes.
	 */
	spin_lock(&c->midi_rx_lock);
	ss = READ_ONCE(c->midi_in);
	if (!READ_ONCE(c->midi_in_up) || READ_ONCE(c->midi_rx_discard))
		ss = NULL;
	clarett_midi_drain(c, ss);
	spin_unlock(&c->midi_rx_lock);

	spin_lock(&c->midi_tx_lock);
	if (c->midi_tx_irq)
		clarett_midi_tx_fill(c);
	spin_unlock(&c->midi_tx_lock);

	clarett_wl(c, REG_MIDI_ACK, MIDI_IRQ_ACK_VAL);
}

/* Open/close the output gate and refill (or stop) TX; any context. */
static void clarett_midi_tx_set(struct clarett *c, bool up)
{
	unsigned long flags;

	spin_lock_irqsave(&c->midi_tx_lock, flags);
	WRITE_ONCE(c->midi_out_up, up);
	clarett_midi_tx_fill(c);
	spin_unlock_irqrestore(&c->midi_tx_lock, flags);
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
	unsigned long flags;

	/* stop TX; the interrupt touches this substream only under the lock */
	spin_lock_irqsave(&c->midi_tx_lock, flags);
	WRITE_ONCE(c->midi_out_up, false);
	clarett_midi_tx_fill(c);
	WRITE_ONCE(c->midi_out, NULL);
	spin_unlock_irqrestore(&c->midi_tx_lock, flags);
	return 0;
}

/*
 * Called outside the rawmidi runtime lock, possibly atomic: write what fits
 * now, and leave the rest to the transmit-ready interrupt.
 */
static void clarett_midi_out_trigger(struct snd_rawmidi_substream *ss, int up)
{
	clarett_midi_tx_set(ss->rmidi->private_data, !!up);
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

	if (!c->has_midi)
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
	spin_lock_init(&c->midi_tx_lock);
	/* set LAST: the ISR uses this as the "MIDI live" gate */
	WRITE_ONCE(c->rmidi, rmidi);

	/* debug: the probe summary already reports MIDI */
	dev_dbg(&c->pci->dev,
		"MIDI registered (%s, 1 in + 1 out, register UART @0x%03x)\n",
		name, REG_MIDI_DATA);
	return 0;
}

/*
 * Stop TX and clear its interrupt enable before teardown. Safe to call on a
 * card that never created MIDI.
 */
void clarett_midi_stop(struct clarett *c)
{
	if (!READ_ONCE(c->rmidi))
		return;
	WRITE_ONCE(c->midi_in_up, false);
	clarett_midi_tx_set(c, false);
}

/*
 * System sleep. Unlike clarett_midi_stop() the trigger gates stay as the open
 * substreams left them, so MIDI carries on after resume without a reopen:
 * suspend only parks TX and clears its interrupt enable (bytes still queued in
 * the rawmidi buffer stay there), and resume restarts it.
 *
 * RX is muted across the sleep (midi_rx_discard): after a sleep the unit's RX
 * FIFO delivers a burst of stale bytes (old buffer contents) at the wake. The
 * ISR discards until the re-attach is done, and resume drains what is left
 * before delivery starts again. Bytes sent to the unit during the sleep are
 * lost either way.
 */
void clarett_midi_suspend(struct clarett *c)
{
	unsigned long flags;

	if (!READ_ONCE(c->rmidi))
		return;
	WRITE_ONCE(c->midi_rx_discard, true);
	spin_lock_irqsave(&c->midi_tx_lock, flags);
	c->midi_tx_parked = true;
	clarett_midi_tx_fill(c);
	spin_unlock_irqrestore(&c->midi_tx_lock, flags);
}

void clarett_midi_resume(struct clarett *c)
{
	unsigned long flags;

	if (!READ_ONCE(c->rmidi))
		return;

	spin_lock_irqsave(&c->midi_rx_lock, flags);
	if (clarett_midi_drain(c, NULL))
		clarett_wl(c, REG_MIDI_ACK, MIDI_IRQ_ACK_VAL);
	WRITE_ONCE(c->midi_rx_discard, false);
	spin_unlock_irqrestore(&c->midi_rx_lock, flags);

	spin_lock_irqsave(&c->midi_tx_lock, flags);
	c->midi_tx_parked = false;
	clarett_midi_tx_fill(c);
	spin_unlock_irqrestore(&c->midi_tx_lock, flags);
}
