// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — the data-plane engine
 * (arm, run, stop) and the stream servicer that drives it.
 */
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include "clarett.h"

/*
 * The data-plane DMA engine is flow-controlled: it raises a period on the 0x300
 * cause register, with an MSI, and waits for the host to ack it by reading it
 * (read-to-clear); unserviced, it stalls after a few descriptors. The interrupt
 * sweep (clarett_irq_sweep()) acks it and counts the frames; the stream
 * servicer below, a kthread, drives the PCM path (clarett_pcm_tick) from what
 * the sweep collected.
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
 * Servicer telemetry: the 2-second window and the run-wide summary. The event
 * figures are the interrupt sweep's (struct clarett_irq_stats), copied in at
 * each window (clarett_svc_collect()).
 */
struct clarett_svc_stats {
	u32 wraps;
	u32 bad_reads;
	/* cumulative OR of every unusable 0x100/0x300 sample (badbits) */
	u32 bad_or;
	/*
	 * Cumulative OR of every 0x100 summary read (IRQ0_CAUSE_KNOWN says
	 * which bits are understood).
	 */
	u32 cause_or;
	/* events flagged CLARETT_CTR_OVERRUN (device saw a missed ack) */
	u32 overruns;
	/*
	 * 0x100 reads showing IRQ0_CAUSE_TX_UNDERRUN (the playback engine ran
	 * dry). A count of samples, not of separate events.
	 */
	u32 underruns;
	/*
	 * Previous window's values of the cumulative counters, so a window can
	 * tell whether IT was bad.
	 */
	u32 prev_bad_reads, prev_overruns, prev_underruns;
	/*
	 * Run-wide worst case, for the one-line summary at stop (the windows
	 * reset their own maxima).
	 */
	u64 run_gap_max_us, run_read_us_max;
	u32 run_late, run_step_max;
	/* when the servicer first saw stream_run (0 = never) */
	ktime_t run_start;
	/*
	 * Event lateness. The period counters cannot show it: they accumulate
	 * the hardware counter delta, so a late event is made up on the next
	 * one and the totals stay smooth while the audio glitches. What
	 * matters is the wall-clock gap between events, against a nominal that
	 * follows the period and the rate (clarett_tick_late_us(); do not
	 * reintroduce a fixed threshold). step_max is the same signal without a
	 * clock: a doubled step is a missed event.
	 */
	u64 gap_max_us;
	u32 gap_late, step_max;
	/*
	 * readmax = the longest interrupt sweep (us). If it is close to
	 * gapmax, the stall was inside a readl() (the link or the whole host
	 * stalled).
	 */
	u64 read_us_max;
};

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
	atomic_set(&c->svc_wake, 1);
	wake_up(&c->svc_wq);
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
 * overrun, a playback underrun or a rejected sample. Every window, without
 * the mailbox trace:
 *   echo 'format "stream-svc:" +p' |
 *     tee /sys/kernel/debug/dynamic_debug/control
 */
static void clarett_svc_window(struct clarett *c, struct clarett_svc_stats *st)
{
	bool bad = st->gap_late || st->bad_reads != st->prev_bad_reads ||
		   st->overruns != st->prev_overruns ||
		   st->underruns != st->prev_underruns;

	clarett_svc_log(c, bad,
			"stream-svc: periods=%d ctr=0x%x wraps=%u gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u underrun=%u\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, st->gap_max_us, st->read_us_max,
			st->gap_late, st->step_max, st->bad_reads, st->bad_or,
			st->overruns, st->underruns);

	clarett_svc_fold(st);
	st->prev_bad_reads = st->bad_reads;
	st->prev_overruns = st->overruns;
	st->prev_underruns = st->underruns;

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
			st->underruns,
			"stream-svc: stopped (periods=%d ctr=0x%x wraps=%u ptr0=0x%x ptr1=0x%x; "
			"run gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u underrun=%u cause=0x%08x)\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
			readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
			st->run_gap_max_us, st->run_read_us_max, st->run_late,
			st->run_step_max, st->bad_reads, st->bad_or,
			st->overruns, st->underruns, st->cause_or);
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
				     const struct clarett_svc_stats *st,
				     bool gone)
{
	s64 ran_ms;

	if (gone || !st->run_start || atomic_read(&c->stream_periods))
		return;
	ran_ms = ktime_ms_delta(ktime_get(), st->run_start);
	if (ran_ms < CLARETT_SILENT_RUN_MS)
		return;
	dev_warn_ratelimited(&c->pci->dev,
			     "stream engine raised no period in %lld ms; if every stream does this, power-cycle the unit\n",
			     ran_ms);
}

/*
 * Sleep until the sweep collects a period event, or CLARETT_SVC_FALLBACK_US,
 * whichever comes first. Returns true if it did; on a timeout the caller runs
 * the backstop.
 *
 * atomic_xchg consumes the flag inside the condition, so an event collected
 * while the servicer is busy is not missed: the wait returns at once.
 */
static bool clarett_svc_wait(struct clarett *c)
{
	return wait_event_hrtimeout(c->svc_wq,
				    atomic_xchg(&c->svc_wake, 0) ||
				    kthread_should_stop(),
				    us_to_ktime(CLARETT_SVC_FALLBACK_US)) == 0;
}

/*
 * The backstop: no period event for CLARETT_SVC_FALLBACK_US, so run the sweep
 * here, reading 0x300 whether or not 0x100 flags it. That covers a lost
 * interrupt, and an event whose summary bit an earlier sweep consumed before
 * the stream was released. Returns false if the read failed: the device has
 * left the bus (*gone set) or the link is transiently unreachable.
 */
static bool clarett_svc_backstop(struct clarett *c, bool *gone, bool *warned)
{
	if (clarett_irq_sweep(c, true) != 0xffffffff)
		return true;
	/*
	 * An all-ones read is a FAILED PCIe transaction, never data. Either
	 * the device left the bus -- then park, never return early: a kthread
	 * that exits before kthread_stop() runs makes kthread_stop() (from the
	 * remove path) use the reaped task -- or the link is transiently
	 * unreachable (tens of ms of all-ones reads happen on some hosts).
	 */
	if (pci_dev_is_disconnected(c->pci)) {
		*gone = true;
	} else if (!*warned) {
		*warned = true;
		dev_warn(&c->pci->dev,
			 "0x100 read returned ~0 with the device still attached: the link is transiently unreachable (PCIe power management?)\n");
	}
	return false;
}

/*
 * The stream servicer: takes the frames the interrupt sweep collected and
 * runs the PCM tick (copies, period_elapsed). Playback the app writes is
 * copied by .ack, and the position between events is read by .pointer, not
 * here. It reads no cause register itself except through the backstop.
 */
static int clarett_stream_service(void *data)
{
	struct clarett *c = data;
	unsigned long next_log = jiffies + msecs_to_jiffies(2000);
	struct clarett_svc_stats st = { };
	/* the sweep collected a period event */
	bool woken = false;
	/*
	 * set when the device leaves the bus: park the loop, never return early
	 * (kthread_stop UAF)
	 */
	bool gone = false, warned = false;

	while (!kthread_should_stop()) {
		u32 units, ctr;

		/*
		 * bus gone (below): idle here, never self-exit, until
		 * kthread_stop() reaps us
		 */
		if (gone) {
			usleep_range(1000, 2000);
			continue;
		}
		if (!READ_ONCE(c->stream_run)) {
			usleep_range(300, 600);
			continue;
		}
		if (!st.run_start)
			st.run_start = ktime_get();

		if (!woken && !clarett_svc_backstop(c, &gone, &warned)) {
			usleep_range(100, 200);
			continue;
		}
		woken = false;

		units = clarett_svc_take(c, &ctr);
		if (units)
			clarett_pcm_tick(c, units * CLARETT_CTR_FRAMES, ctr);
		if (time_after(jiffies, next_log)) {
			clarett_svc_collect(c, &st);
			clarett_svc_window(c, &st);
			next_log = jiffies + msecs_to_jiffies(2000);
		}
		woken = clarett_svc_wait(c);
	}
	clarett_svc_collect(c, &st);
	clarett_svc_summary(c, &st);
	clarett_svc_check_silent(c, &st, gone);
	return 0;
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
 * Start the stream servicer kthread. The caller flips stream_run to let the
 * interrupt sweep ack the engine.
 */
void clarett_engine_run(struct clarett *c)
{
	/*
	 * Never start a second servicer: the pointer is the ONLY handle
	 * kthread_stop() has, so overwriting it orphans a SCHED_FIFO thread
	 * that faults on module text after rmmod. The arm claim in
	 * clarett_pcm_attach() should make this unreachable.
	 */
	if (WARN_ON_ONCE(c->stream_svc))
		return;

	c->stream_ctr = 0;
	c->stream_svc = kthread_run(clarett_stream_service, c, "clarett-svc");
	if (IS_ERR(c->stream_svc)) {
		dev_warn(&c->pci->dev, "stream servicer failed to start: %ld\n",
			 PTR_ERR(c->stream_svc));
		c->stream_svc = NULL;
		return;
	}
	/*
	 * Low real-time priority: if userspace preempts the servicer, the TX
	 * fill falls behind and the engine plays stale audio with no XRUN. It
	 * sleeps until the next period interrupt (CLARETT_SVC_FALLBACK_US at
	 * the latest), so it cannot starve the system.
	 */
	sched_set_fifo_low(c->stream_svc);
}

/*
 * Halt the engine before the ring buffer is freed (a bad/continued DMA would
 * fault the IOMMU).
 */
void clarett_engine_stop(struct clarett *c)
{
	struct task_struct *svc;

	/*
	 * CLAIM THE TEARDOWN UNDER pcm_lock. Both directions can close at once,
	 * and clarett_pcm_detach() tests "last one out" after dropping the
	 * lock, so two callers can arrive here. A second kthread_stop() on the
	 * same, already reaped task blocks forever. Taking stream_on and the
	 * handle together under the lock lets exactly one caller proceed (the
	 * mirror of the arm claim in clarett_pcm_attach()).
	 *
	 * kthread_stop() MUST run outside pcm_lock: the servicer takes it in
	 * clarett_pcm_tick(), so stopping it under the lock would deadlock.
	 */
	mutex_lock(&c->pcm_lock);
	if (!c->stream_on) {
		mutex_unlock(&c->pcm_lock);
		return;
	}
	c->stream_on = false;
	/* stop the interrupt sweep acking 0x300 */
	WRITE_ONCE(c->stream_run, false);
	svc = c->stream_svc;
	/* take the handle: only this caller may stop it */
	c->stream_svc = NULL;
	mutex_unlock(&c->pcm_lock);

	if (svc)
		/* stop acking 0x300 before the engine is torn down */
		kthread_stop(svc);
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
