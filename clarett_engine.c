// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the data-plane engine
 * (arm, run, stop) and the stream servicer that drives it.
 */
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include "clarett.h"

/*
 * The data-plane DMA engine is flow-controlled: it raises a period on the 0x300
 * cause register, with an MSI, and waits for the host to ack it by reading it
 * (read-to-clear); unserviced, it stalls after a few descriptors. The interrupt
 * sweep (clarett_irq_sweep()) acks it and counts the frames; vec0's IRQ thread
 * (clarett_svc_thread()) drives the PCM path (clarett_pcm_tick) from what the
 * sweep collected, and a per-stream timer (clarett_svc_backstop()) covers an
 * interrupt that never came.
 */
/*
 * Log a servicer telemetry line at info if it reports something wrong, else at
 * debug: a desktop holds a PCM open for as long as it is up, so a healthy line
 * would otherwise repeat every 2 s indefinitely.
 */
#define clarett_svc_log(c, bad, fmt, ...)				\
	do {								\
		if (bad)						\
			dev_info(&(c)->pci->dev, fmt, ##__VA_ARGS__);	\
		else							\
			dev_dbg(&(c)->pci->dev, fmt, ##__VA_ARGS__);	\
	} while (0)

/*
 * From the sweep, every 0x100 read while a stream runs: the summary's own
 * telemetry. Under irq_lock.
 */
void clarett_engine_irq_summary(struct clarett *c, u32 sum)
{
	struct clarett_irq_stats *st = &c->irq_st;

	st->cause_or |= sum;
	if (sum & IRQ0_CAUSE_TX_UNDERRUN)
		st->underruns++;
	/*
	 * A bit nobody has characterised. Worth one line at info, since every
	 * capture of every model has only ever shown the known ones, and the
	 * open question (whether bits 4 and 5 are stream errors) is settled the
	 * first time this prints.
	 */
	if ((sum & ~IRQ0_CAUSE_KNOWN) && !st->cause_told) {
		st->cause_told = true;
		dev_info(&c->pci->dev,
			 "stream-cause: 0x100=0x%08x has unknown bits 0x%08x\n",
			 sum, sum & ~IRQ0_CAUSE_KNOWN);
	}
}

/* The event's timing: the gap since the last one, and the worst step. */
static void clarett_irq_timing(struct clarett *c, u32 step)
{
	struct clarett_irq_stats *st = &c->irq_st;
	ktime_t now = ktime_get();
	u64 gap = ktime_us_delta(now, st->last_ev);

	st->last_ev = now;
	if (gap > st->gap_max_us)
		st->gap_max_us = gap;
	if (gap > clarett_tick_late_us(c))
		st->gap_late++;
	if (step > st->step_max)
		st->step_max = step;
}

/*
 * From the sweep, with a 0x300 sample (reading it acked the engine): if it
 * carries a period event, add the frames the engine advanced to what the
 * servicer has still to take, and wake it. Under irq_lock.
 */
void clarett_engine_irq_period(struct clarett *c, u32 c2)
{
	struct clarett_irq_stats *st = &c->irq_st;
	u32 ctr = c2 & CLARETT_CTR_MASK;
	u32 step;

	/*
	 * Reject only samples with bits outside CLARETT_CTR_KNOWN (in practice
	 * a dead link's all-ones reads); CLARETT_CTR_OVERRUN samples carry a
	 * valid counter and are kept. The next accepted sample's modular delta
	 * recovers the advance across a dropped one.
	 */
	if (c2 & ~CLARETT_CTR_KNOWN) {
		st->bad_reads++;
		st->bad_or |= c2;
		if (st->bad_reads <= 8)
			dev_info(&c->pci->dev,
				 "stream-badread[%u]: 0x300=0x%08x (unknown bits 0x%08x)\n",
				 st->bad_reads, c2, c2 & ~CLARETT_CTR_KNOWN);
		return;
	}
	if (!(c2 & CLARETT_CTR_EVENT))
		return;

	/* the first events of a session: the raw counter values */
	if (atomic_read(&c->stream_periods) < 8)
		dev_dbg(&c->pci->dev, "stream-ev[%d]: 0x300=0x%08x\n",
			atomic_read(&c->stream_periods), c2);

	if (c2 & CLARETT_CTR_OVERRUN)
		st->overruns++;
	/*
	 * Frames since the last event = the MODULAR counter delta (mod
	 * CLARETT_CTR_MOD) * CLARETT_CTR_FRAMES. A wrap and a late event are
	 * the same arithmetic, so a late one makes up the frames it missed;
	 * losing them would leave pcm_frames behind the engine, outside the TX
	 * guard.
	 */
	step = (ctr - c->irq_ctr) & (CLARETT_CTR_MOD - 1);
	if (!step) {
		/* a whole multiple of the modulus: the advance is unknown */
		step = c->stream_ctr_step;
		if (st->seen)
			st->wraps++;
	}
	if (step)
		c->stream_ctr_step = step;
	c->irq_ctr = ctr;
	st->seen = true;

	clarett_irq_timing(c, step);
	atomic_inc(&c->stream_periods);
	c->irq_pending += step;
	/* the caller wakes the IRQ thread (clarett_irq, clarett_svc_kick) */
	atomic_set(&c->svc_wake, 1);
}

/*
 * Take the counter units the sweep collected since the last call, and the
 * counter value they bring the frame clock to (*ctr).
 */
static u32 clarett_svc_take(struct clarett *c, u32 *ctr)
{
	unsigned long flags;
	u32 units;

	spin_lock_irqsave(&c->irq_lock, flags);
	units = c->irq_pending;
	c->irq_pending = 0;
	*ctr = c->irq_ctr;
	spin_unlock_irqrestore(&c->irq_lock, flags);
	return units;
}

/* Copy the sweep's figures into the window, and start its next window. */
static void clarett_svc_collect(struct clarett *c, struct clarett_svc_stats *st)
{
	struct clarett_irq_stats *is = &c->irq_st;
	unsigned long flags;

	spin_lock_irqsave(&c->irq_lock, flags);
	st->gap_max_us = is->gap_max_us;
	st->read_us_max = is->sweep_us_max;
	st->gap_late = is->gap_late;
	st->step_max = is->step_max;
	is->gap_max_us = 0;
	is->sweep_us_max = 0;
	is->gap_late = 0;
	is->step_max = 0;
	st->wraps = is->wraps;
	st->bad_reads = is->bad_reads;
	st->bad_or = is->bad_or;
	st->overruns = is->overruns;
	st->underruns = is->underruns;
	st->cause_or = is->cause_or;
	spin_unlock_irqrestore(&c->irq_lock, flags);
	st->backstops = atomic_read(&c->svc_backstops);
}

/* Fold the current window into the run-wide worst case. */
static void clarett_svc_fold(struct clarett_svc_stats *st)
{
	if (st->gap_max_us > st->run_gap_max_us)
		st->run_gap_max_us = st->gap_max_us;
	if (st->read_us_max > st->run_read_us_max)
		st->run_read_us_max = st->read_us_max;
	if (st->step_max > st->run_step_max)
		st->run_step_max = st->step_max;
	st->run_late += st->gap_late;
}

/*
 * Close a 2-second window: log it, fold it into the run, start the next. The
 * line is at info only when the window had a late tick, a device-side period
 * overrun, a playback underrun, a rejected sample or an event only the
 * backstop found. Every window, without
 * the mailbox trace:
 *   echo 'format "stream-svc:" +p' |
 *     tee /sys/kernel/debug/dynamic_debug/control
 */
static void clarett_svc_window(struct clarett *c, struct clarett_svc_stats *st)
{
	bool bad = st->gap_late || st->bad_reads != st->prev_bad_reads ||
		   st->overruns != st->prev_overruns ||
		   st->underruns != st->prev_underruns ||
		   st->backstops != st->prev_backstops;

	clarett_svc_log(c, bad,
			"stream-svc: periods=%d ctr=0x%x wraps=%u gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u underrun=%u backstop=%u\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, st->gap_max_us, st->read_us_max,
			st->gap_late, st->step_max, st->bad_reads, st->bad_or,
			st->overruns, st->underruns, st->backstops);

	clarett_svc_fold(st);
	st->prev_bad_reads = st->bad_reads;
	st->prev_overruns = st->overruns;
	st->prev_underruns = st->underruns;
	st->prev_backstops = st->backstops;

	st->gap_max_us = 0;
	st->read_us_max = 0;
	st->gap_late = 0;
	st->step_max = 0;
}

/*
 * One summary per stream, at info only if the run had anything wrong with it.
 * ptr0/ptr1 are the two engines' levels (STREAM_OFF_PTR): zero means that
 * engine never fetched anything.
 */
static void clarett_svc_summary(struct clarett *c, struct clarett_svc_stats *st)
{
	void __iomem *bar = c->bar0;

	/* fold the final (partial) window in before summarising */
	clarett_svc_fold(st);

	clarett_svc_log(c, st->run_late || st->bad_reads || st->overruns ||
			st->underruns || st->backstops,
			"stream-svc: stopped (periods=%d ctr=0x%x wraps=%u ptr0=0x%x ptr1=0x%x; "
			"run gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u underrun=%u backstop=%u cause=0x%08x)\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
			readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
			st->run_gap_max_us, st->run_read_us_max, st->run_late,
			st->run_step_max, st->bad_reads, st->bad_or,
			st->overruns, st->underruns, st->backstops,
			st->cause_or);
}

/*
 * The longest a running engine can go with no period event: four times the
 * widest event spacing (a 1024-frame period at 44.1 kHz, ~23 ms).
 */
#define CLARETT_SILENT_RUN_MS	100

/*
 * Warn about a stream that ran with no period event at all: the engine took
 * the arm, then never moved audio, and the app gets -EIO at its wait timeout.
 * Seen after long arm/stop churn on one host; it survives a module reload, and
 * a power cycle of the unit clears it. Nothing in such a run trips the
 * anomaly gates of clarett_svc_summary(), so without this the log stays
 * silent. Not for a device that left the bus, nor for an engine stopped
 * before it could have raised an event.
 */
static void clarett_svc_check_silent(struct clarett *c,
				     const struct clarett_svc_stats *st)
{
	s64 ran_ms;

	if (pci_dev_is_disconnected(c->pci) || !st->run_start ||
	    atomic_read(&c->stream_periods))
		return;
	ran_ms = ktime_ms_delta(ktime_get(), st->run_start);
	if (ran_ms < CLARETT_SILENT_RUN_MS)
		return;
	dev_warn_ratelimited(&c->pci->dev,
			     "stream engine raised no period in %lld ms; if every stream does this, power-cycle the unit\n",
			     ran_ms);
}

/*
 * Wake vec0's IRQ thread from outside vec0's own handler: the backstop timer,
 * the other vectors' handlers, and the release of the stream. The thread runs
 * again if it is already running, so a wake is never lost.
 */
void clarett_svc_kick(struct clarett *c)
{
	if (READ_ONCE(c->irq_ready))
		irq_wake_thread(pci_irq_vector(c->pci, 0), &c->irq_ctx[0]);
}

/* The backstop's spacing: a few nominal periods, never under the floor. */
static ktime_t clarett_svc_backstop_interval(const struct clarett *c)
{
	u64 us = clarett_nominal_period_us(c) * CLARETT_SVC_BACKSTOP_PERIODS;

	return us_to_ktime(max_t(u64, us, CLARETT_SVC_BACKSTOP_MIN_US));
}

/*
 * The backstop, a per-stream timer: if no period event arrived since its last
 * expiry, run the sweep here, reading 0x300 whether or not 0x100 flags it. That
 * covers a lost interrupt. It also wakes the IRQ thread when a telemetry
 * window is due, so a stream whose engine raises nothing still logs.
 *
 * An all-ones read is a FAILED PCIe transaction, never data: the device left
 * the bus (stop re-arming; engine stop and remove follow) or the link is
 * transiently unreachable (tens of ms of all-ones reads happen on some hosts).
 */
static enum hrtimer_restart clarett_svc_backstop(struct hrtimer *t)
{
	struct clarett *c = container_of(t, struct clarett, svc_timer);
	int periods = atomic_read(&c->stream_periods);

	if (!READ_ONCE(c->stream_run))
		return HRTIMER_NORESTART;
	if (periods == c->svc_bs_periods) {
		if (clarett_irq_sweep(c, true) == 0xffffffff) {
			if (pci_dev_is_disconnected(c->pci))
				return HRTIMER_NORESTART;
			if (!c->svc_bs_warned) {
				c->svc_bs_warned = true;
				dev_warn(&c->pci->dev,
					 "0x100 read returned ~0 with the device still attached: the link is transiently unreachable (PCIe power management?)\n");
			}
		} else if (atomic_read(&c->stream_periods) != periods) {
			atomic_inc(&c->svc_backstops);
		}
	}
	c->svc_bs_periods = atomic_read(&c->stream_periods);
	if (atomic_read(&c->svc_wake) ||
	    time_after(jiffies, READ_ONCE(c->svc_next_log)))
		clarett_svc_kick(c);
	hrtimer_forward_now(t, clarett_svc_backstop_interval(c));
	return HRTIMER_RESTART;
}

/*
 * vec0's IRQ thread: takes the frames the interrupt sweep collected and runs
 * the PCM tick (copies, period_elapsed), and closes a telemetry window every
 * 2 s. Playback the app writes is copied by .ack, and the position between
 * events is read by .pointer, not here. Process context; it reads no register.
 */
irqreturn_t clarett_svc_thread(int irq, void *dev_id)
{
	struct clarett_irqctx *ic = dev_id;
	struct clarett *c = ic->c;
	u32 units, ctr;

	atomic_set(&c->svc_wake, 0);
	if (!READ_ONCE(c->stream_run))
		return IRQ_HANDLED;
	units = clarett_svc_take(c, &ctr);
	if (units)
		clarett_pcm_tick(c, units * CLARETT_CTR_FRAMES, ctr);
	if (time_after(jiffies, c->svc_next_log)) {
		clarett_svc_collect(c, &c->svc_st);
		clarett_svc_window(c, &c->svc_st);
		WRITE_ONCE(c->svc_next_log,
			   jiffies + msecs_to_jiffies(2000));
	}
	return IRQ_HANDLED;
}

void clarett_engine_init(struct clarett *c)
{
	hrtimer_setup(&c->svc_timer, clarett_svc_backstop, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
}

/*
 * Program the data-plane engine over caller-provided descriptor-table bases. r0
 * = block-0 (TX/playback) table base, r1 = block-1 (RX/capture) table base;
 * pass 0 to skip a block. Leaves the engine armed but paused: it prefetches a
 * few descriptors and waits for the interrupt sweep to ack 0x300 (gated by
 * stream_run).
 */
static void clarett_engine_program(struct clarett *c, dma_addr_t r0,
				   dma_addr_t r1)
{
	unsigned long flags;
	void __iomem *bar = c->bar0;

	/*
	 * Ack the cause block (0x100=0xf), 0x108, the global enable (0x20c=1)
	 * BEFORE the geometry and base writes, per-block geometry and bases,
	 * 0x10c, then 0x110=7. DMA starts only on the 0x110=7 arm, which stays
	 * last, so there is no null-base fault. The mailbox side (SET_CLOCK,
	 * the sync queries) is clarett_stream_handshake(), run before this.
	 */
	/* 0x100 = 0xf: ack cause block */
	writel(0xf, bar + REG_IRQ0_CAUSE);
	/* 0x108 */
	writel(0x10, bar + REG_STREAM_IRQ_CFG);
	/* 0x20c global enable (before geometry) */
	writel(1, bar + STREAM_BLK0 + STREAM_OFF_CTRL);

	if (r0) {
		/* 0x204 */
		writel(c->playback_ch,
		       bar + STREAM_BLK0 + STREAM_OFF_CHANS);
		writel(clarett_period_bytes(c->playback_ch),
		       /* 0x208 period */
		       bar + STREAM_BLK0 + STREAM_OFF_SIZE);
		writel(upper_32_bits(r0),
		       /* 0x214 */
		       bar + STREAM_BLK0 + STREAM_OFF_BASE_HI);
		/* 0x210 (low last) */
		writel(lower_32_bits(r0),
		       bar + STREAM_BLK0 + STREAM_OFF_BASE_LO);
	}
	if (r1) {
		/* 0x304 */
		writel(c->capture_ch,
		       bar + STREAM_BLK1 + STREAM_OFF_CHANS);
		writel(clarett_period_bytes(c->capture_ch),
		       /* 0x308 period */
		       bar + STREAM_BLK1 + STREAM_OFF_SIZE);
		writel(upper_32_bits(r1),
		       /* 0x314 */
		       bar + STREAM_BLK1 + STREAM_OFF_BASE_HI);
		/* 0x310 */
		writel(lower_32_bits(r1),
		       bar + STREAM_BLK1 + STREAM_OFF_BASE_LO);
	}

	/* 0x10c */
	writel(0x1e70700, bar + REG_STREAM_IRQ_CFG2);

	/* the sweep counts this stream from zero, before it raises an event */
	spin_lock_irqsave(&c->irq_lock, flags);
	c->irq_ctr = 0;
	c->stream_ctr_step = 0;
	c->irq_pending = 0;
	memset(&c->irq_st, 0, sizeof(c->irq_st));
	c->irq_st.last_ev = ktime_get();
	atomic_set(&c->stream_periods, 0);
	spin_unlock_irqrestore(&c->irq_lock, flags);

	/* 0x110 arm (0x0 is stream-stop) */
	writel(0x7, bar + REG_STREAM_IRQ_ARM);

	c->stream_on = true;
}

/*
 * Arm the engine. Sleeps — process context only, which is where every caller
 * already is.
 */
void clarett_engine_arm(struct clarett *c, dma_addr_t r0, dma_addr_t r1)
{
	void __iomem *bar = c->bar0;

	clarett_engine_program(c, r0, r1);

	/* post-arm: whether the bases latched, where the pointers start */
	dev_dbg(&c->pci->dev,
		"engine armed: blk0 chans=%u size=0x%x base=%08x:%08x ctrl=%08x ptr=%08x | blk1 chans=%u size=0x%x base=%08x:%08x ctrl=%08x ptr=%08x (model chans tx=%u rx=%u)\n",
		readl(bar + STREAM_BLK0 + STREAM_OFF_CHANS),
		readl(bar + STREAM_BLK0 + STREAM_OFF_SIZE),
		readl(bar + STREAM_BLK0 + STREAM_OFF_BASE_HI),
		readl(bar + STREAM_BLK0 + STREAM_OFF_BASE_LO),
		readl(bar + STREAM_BLK0 + STREAM_OFF_CTRL),
		readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
		readl(bar + STREAM_BLK1 + STREAM_OFF_CHANS),
		readl(bar + STREAM_BLK1 + STREAM_OFF_SIZE),
		readl(bar + STREAM_BLK1 + STREAM_OFF_BASE_HI),
		readl(bar + STREAM_BLK1 + STREAM_OFF_BASE_LO),
		readl(bar + STREAM_BLK1 + STREAM_OFF_CTRL),
		readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
		c->playback_ch, c->capture_ch);
}

/*
 * Release the stream: from here the interrupt sweep acks the engine and the
 * IRQ thread services it. Sweep once at once: an event whose 0x100 bit a sweep
 * consumed before the release raises no further interrupt until 0x300 is read.
 */
void clarett_engine_run(struct clarett *c)
{
	c->stream_ctr = 0;
	memset(&c->svc_st, 0, sizeof(c->svc_st));
	c->svc_st.run_start = ktime_get();
	c->svc_bs_periods = -1;
	c->svc_bs_warned = false;
	atomic_set(&c->svc_backstops, 0);
	WRITE_ONCE(c->svc_next_log, jiffies + msecs_to_jiffies(2000));

	WRITE_ONCE(c->stream_run, true);
	clarett_irq_sweep(c, true);
	if (atomic_read(&c->svc_wake))
		clarett_svc_kick(c);
	hrtimer_start(&c->svc_timer, clarett_svc_backstop_interval(c),
		      HRTIMER_MODE_REL);
}

/*
 * Halt the engine before the ring buffer is freed (a bad/continued DMA would
 * fault the IOMMU).
 */
void clarett_engine_stop(struct clarett *c)
{
	/*
	 * CLAIM THE TEARDOWN UNDER pcm_lock. Both directions can close at once,
	 * and clarett_pcm_detach() tests "last one out" after dropping the
	 * lock, so two callers can arrive here; exactly one proceeds (the
	 * mirror of the arm claim in clarett_pcm_attach()).
	 *
	 * The waits below MUST run outside pcm_lock: the IRQ thread takes it in
	 * clarett_pcm_tick(), so waiting for it under the lock would deadlock.
	 */
	mutex_lock(&c->pcm_lock);
	if (!c->stream_on) {
		mutex_unlock(&c->pcm_lock);
		return;
	}
	c->stream_on = false;
	/* stop the interrupt sweep acking 0x300 and waking the thread */
	WRITE_ONCE(c->stream_run, false);
	mutex_unlock(&c->pcm_lock);

	hrtimer_cancel(&c->svc_timer);
	/* a sweep or tick already under way finishes; none starts after */
	if (READ_ONCE(c->irq_ready))
		synchronize_irq(pci_irq_vector(c->pci, 0));
	clarett_svc_collect(c, &c->svc_st);
	clarett_svc_summary(c, &c->svc_st);
	clarett_svc_check_silent(c, &c->svc_st);

	/* disable ring 0 */
	clarett_wl(c, STREAM_BLK0 + STREAM_OFF_CTRL, 0);
	/* disable ring 1 */
	clarett_wl(c, STREAM_BLK1 + STREAM_OFF_CTRL, 0);
	clarett_wl(c, REG_STREAM_IRQ_ARM, 0);
	clarett_wl(c, REG_STREAM_IRQ_CFG, 0);
	/* flush posted writes */
	clarett_rl(c, STREAM_BLK0 + STREAM_OFF_CTRL);
}

/*
 * Deregister the response buffer and stop the device mastering the bus.
 * REG_DMA_ADDR_LO/HI are a standing registration that nothing expires, so left
 * in place across an unload they point the device at freed memory. Bus master
 * goes off first, so the device cannot write through a half-zeroed address.
 * Call only after all mailbox traffic has stopped.
 */
void clarett_quiesce_dma(struct pci_dev *pci, void __iomem *bar0)
{
	pci_clear_master(pci);
	if (!bar0)
		return;
	writel(0, bar0 + REG_DMA_ADDR_LO);
	writel(0, bar0 + REG_DMA_ADDR_HI);
	/* flush posted writes before the buffer is freed */
	readl(bar0 + REG_DMA_ADDR_LO);
}
