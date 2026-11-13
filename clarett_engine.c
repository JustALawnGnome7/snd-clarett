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
 * The stream servicer. The data-plane DMA engine is flow-controlled: it raises
 * a period on the 0x300 cause register and waits for the host to ACK it by
 * reading it (read-to-clear); unserviced, it stalls after a few descriptors.
 * This kthread polls the cause block for the whole stream and drives the PCM
 * path (clarett_pcm_tick) from it.
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

/* Servicer telemetry: the 2-second window and the run-wide summary. */
struct clarett_svc_stats {
	u32 wraps;
	u32 bad_reads;
	/* cumulative OR of every rejected 0x300 sample (badbits) */
	u32 bad_or;
	/*
	 * Cumulative OR of every 0x100 summary read (IRQ0_CAUSE_KNOWN says
	 * which bits are understood), and whether an unknown one has been
	 * reported for this stream. Samples taken while a command is in flight
	 * are not included: the vec0 ISR owns 0x100 then.
	 */
	u32 cause_or;
	bool cause_told;
	/* events flagged CLARETT_CTR_OVERRUN (device saw a missed ack) */
	u32 overruns;
	/*
	 * Previous window's values of the cumulative counters, so a window can
	 * tell whether IT was bad.
	 */
	u32 prev_bad_reads, prev_overruns;
	/*
	 * Run-wide worst case, for the one-line summary at stop (the windows
	 * reset their own maxima).
	 */
	u64 run_gap_max_us, run_read_us_max;
	u32 run_late, run_step_max;
	bool seen;
	/* when the servicer first saw stream_run (0 = never) */
	ktime_t run_start;
	/*
	 * Tick lateness. The period counters cannot show it: they accumulate
	 * the hardware counter delta, so a late servicer catches up on the next
	 * tick and the totals stay smooth while the audio glitches. What
	 * matters is the wall-clock gap between events, against a nominal that
	 * follows the period and the rate (clarett_tick_late_us(); do not
	 * reintroduce a fixed threshold). step_max is the same signal without a
	 * clock: a doubled step is a missed poll. Reset each log window.
	 */
	ktime_t last_ev;
	u64 gap_max_us;
	u32 gap_late, step_max;
	/*
	 * readmax = longest a single MMIO read block took (us). If it is close
	 * to gapmax, the stall was inside a readl() (the link or the whole host
	 * stalled); if it stays small while gapmax spikes, the thread was
	 * descheduled between reads.
	 */
	u64 read_us_max;
};

/*
 * Read the cause blocks the servicer owns and return 0x300.
 *
 * With no command in flight it also reads 0x100, which asserts bit31 every
 * period (read-to-clear), and 0x500. 0x400 is the vec0 ISR's alone. While a
 * command is in flight (cmd_inflight), 0x100 and 0x500 belong to the mailbox,
 * so only the stream blocks 0x300/0x200 are read; the mailbox's cause sweep
 * skips those two while streaming (clarett_mailbox.c).
 */
static u32 clarett_svc_read_causes(struct clarett *c,
				   struct clarett_svc_stats *st)
{
	void __iomem *bar = c->bar0;
	bool busy = atomic_read(&c->cmd_inflight);
	/* time the read block: is a stall inside a readl()? */
	ktime_t rt0 = ktime_get();
	u64 rus;
	u32 c2;

	if (!busy) {
		/* the 0x100 summary: block bits, and the stop class in 0-3 */
		u32 sum = readl(bar + REG_IRQ0_CAUSE);

		st->cause_or |= sum;
		/*
		 * A bit nobody has characterised. Worth one line at info,
		 * since every capture of every model has only ever shown the
		 * known ones, and the open question (whether bits 4 and 5 are
		 * stream errors) is settled the first time this prints.
		 */
		if ((sum & ~IRQ0_CAUSE_KNOWN) && !st->cause_told) {
			st->cause_told = true;
			dev_info(&c->pci->dev,
				 "stream-cause: 0x100=0x%08x has unknown bits 0x%08x\n",
				 sum, sum & ~IRQ0_CAUSE_KNOWN);
		}
	}
	/* 0x300 read-to-clear = period event + counter (owned) */
	c2 = readl(bar + STREAM_BLK1);
	/* 0x200 TX cause (owned) */
	readl(bar + STREAM_BLK0);
	if (!busy)
		/* 0x500 IRQ summary (0x400 belongs to the vec0 ISR) */
		readl(bar + 0x500);
	rus = ktime_us_delta(ktime_get(), rt0);
	if (rus > st->read_us_max)
		st->read_us_max = rus;
	return c2;
}

/* The window's timing: the gap since the last event, and the worst step. */
static void clarett_svc_timing(struct clarett *c, struct clarett_svc_stats *st,
			       u32 step)
{
	ktime_t now = ktime_get();
	u64 gap = ktime_to_us(ktime_sub(now, st->last_ev));

	st->last_ev = now;
	if (gap > st->gap_max_us)
		st->gap_max_us = gap;
	if (gap > clarett_tick_late_us(c))
		st->gap_late++;
	if (step > st->step_max)
		st->step_max = step;
}

/*
 * One 0x300 period event: advance the frame clock by the counter delta and
 * run the PCM tick. Returns false if the sample was rejected, in which case
 * the caller skips the rest of this pass.
 */
static bool clarett_svc_event(struct clarett *c, u32 c2,
			      struct clarett_svc_stats *st)
{
	u32 ctr = c2 & CLARETT_CTR_MASK;
	u32 step;

	/* the first events of a session: the raw counter values */
	if (atomic_read(&c->stream_periods) < 8)
		dev_dbg(&c->pci->dev, "stream-ev[%d]: 0x300=0x%08x\n",
			atomic_read(&c->stream_periods), c2);

	/*
	 * Frames since the last event = the MODULAR counter delta (mod
	 * CLARETT_CTR_MOD) * CLARETT_CTR_FRAMES. A wrap and a late poll are the
	 * same arithmetic, so a delayed tick makes up the frames it missed;
	 * losing them would leave pcm_frames behind the engine, outside the TX
	 * guard.
	 *
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
		return false;
	}
	if (c2 & CLARETT_CTR_OVERRUN)
		st->overruns++;
	step = (ctr - c->stream_ctr) & (CLARETT_CTR_MOD - 1);
	if (!step) {
		/* a whole multiple of the modulus: the advance is unknown */
		step = c->stream_ctr_step;
		if (st->seen)
			st->wraps++;
	}
	if (step)
		c->stream_ctr_step = step;
	c->stream_ctr = ctr;
	st->seen = true;

	clarett_svc_timing(c, st, step);

	atomic_inc(&c->stream_periods);
	/* advance by real captured frames (no-op if idle) */
	clarett_pcm_tick(c, step * CLARETT_CTR_FRAMES);
	return true;
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
 * overrun or a rejected sample. Every window, without the mailbox trace:
 *   echo 'format "stream-svc:" +p' |
 *     tee /sys/kernel/debug/dynamic_debug/control
 */
static void clarett_svc_window(struct clarett *c, struct clarett_svc_stats *st)
{
	bool bad = st->gap_late || st->bad_reads != st->prev_bad_reads ||
		   st->overruns != st->prev_overruns;

	clarett_svc_log(c, bad,
			"stream-svc: periods=%d ctr=0x%x wraps=%u gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, st->gap_max_us, st->read_us_max,
			st->gap_late, st->step_max, st->bad_reads, st->bad_or,
			st->overruns);

	clarett_svc_fold(st);
	st->prev_bad_reads = st->bad_reads;
	st->prev_overruns = st->overruns;

	st->gap_max_us = 0;
	st->read_us_max = 0;
	st->gap_late = 0;
	st->step_max = 0;
}

/*
 * One summary per stream, at info only if the run had anything wrong with it.
 * The ring pointers show whether the engine walked the descriptor table at all.
 */
static void clarett_svc_summary(struct clarett *c, struct clarett_svc_stats *st)
{
	void __iomem *bar = c->bar0;

	/* fold the final (partial) window in before summarising */
	clarett_svc_fold(st);

	clarett_svc_log(c, st->run_late || st->bad_reads || st->overruns,
			"stream-svc: stopped (periods=%d ctr=0x%x wraps=%u ptr0=0x%x ptr1=0x%x; "
			"run gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u cause=0x%08x)\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st->wraps, readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
			readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
			st->run_gap_max_us, st->run_read_us_max, st->run_late,
			st->run_step_max, st->bad_reads, st->bad_or,
			st->overruns, st->cause_or);
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
 * Sleep until the next period interrupt, or CLARETT_SVC_REFILL_US, whichever
 * comes first. Returns true if the ISR flagged a period event, which is what
 * tells the caller to read the device; on a plain timeout it only tops
 * playback up, which costs no register access.
 *
 * atomic_xchg consumes the flag inside the condition, so an event that lands
 * between the servicer's 0x300 read and this wait is not missed: it returns at
 * once and costs one read that finds nothing.
 */
static bool clarett_svc_wait(struct clarett *c)
{
	return wait_event_hrtimeout(c->svc_wq,
				    atomic_xchg(&c->svc_wake, 0) ||
				    kthread_should_stop(),
				    us_to_ktime(CLARETT_SVC_REFILL_US)) == 0;
}

static int clarett_stream_service(void *data)
{
	struct clarett *c = data;
	unsigned long next_log = jiffies + msecs_to_jiffies(2000);
	struct clarett_svc_stats st = { .last_ev = ktime_get() };
	/* when 0x300 was last read, against CLARETT_SVC_FALLBACK_US */
	ktime_t last_read = ktime_get();
	/*
	 * the ISR flagged a period event. True to start, so the first pass
	 * reads 0x300: an armed engine stays paused until it does.
	 */
	bool woken = true;
	/*
	 * set when the device leaves the bus: park the loop, never return early
	 * (kthread_stop UAF)
	 */
	bool gone = false;

	while (!kthread_should_stop()) {
		u32 c2;

		/*
		 * bus gone (below): idle here, never self-exit, until
		 * kthread_stop() reaps us
		 */
		if (gone) {
			usleep_range(1000, 2000);
			continue;
		}

		/*
		 * Gate ACKing on stream_run. An armed engine stays paused
		 * until 0x300 is read (read-to-clear is the period ACK), so
		 * not reading it keeps the engine quiescent until stream_run.
		 */
		if (!READ_ONCE(c->stream_run)) {
			usleep_range(300, 600);
			continue;
		}
		if (!st.run_start)
			st.run_start = ktime_get();

		/*
		 * Read the device only when there is a period event to
		 * collect. Reading 0x300 is also how a period is acknowledged,
		 * so an engine whose interrupt never arrives is covered by the
		 * backstop rather than by polling.
		 */
		if (!woken && ktime_us_delta(ktime_get(), last_read) <
			      CLARETT_SVC_FALLBACK_US) {
			/* playback the app wrote since: no register access */
			clarett_pcm_tx_refill(c);
			woken = clarett_svc_wait(c);
			continue;
		}
		woken = false;
		last_read = ktime_get();

		c2 = clarett_svc_read_causes(c, &st);
		/*
		 * An all-ones read is a FAILED PCIe transaction, never data
		 * (it would look like a period event with a garbage counter).
		 * Either the device left the bus, or the link is transiently
		 * unreachable (tens of ms of all-ones reads happen on some
		 * hosts): drop the sample and let the next good read's
		 * modular delta recover the advance.
		 */
		if (c2 == 0xffffffff) {
			if (pci_dev_is_disconnected(c->pci)) {
				/*
				 * The device left the bus. Do NOT return: a
				 * kthread that exits before kthread_stop() runs
				 * makes kthread_stop() (from the remove path)
				 * use the reaped task. Park until stopped.
				 */
				gone = true;
				continue;
			}
			if (!st.bad_reads++)
				dev_warn(&c->pci->dev,
					 "0x300 read returned ~0 with the device still attached: the link is transiently unreachable (PCIe power management?). Dropping the sample.\n");
			usleep_range(100, 200);
			continue;
		}

		if ((c2 & CLARETT_CTR_EVENT) &&
		    !clarett_svc_event(c, c2, &st)) {
			usleep_range(100, 200);
			continue;
		}
		/* between events: copy playback the app wrote since */
		clarett_pcm_tx_refill(c);
		if (time_after(jiffies, next_log)) {
			clarett_svc_window(c, &st);
			next_log = jiffies + msecs_to_jiffies(2000);
		}
		woken = clarett_svc_wait(c);
	}
	clarett_svc_summary(c, &st);
	clarett_svc_check_silent(c, &st, gone);
	return 0;
}

/*
 * Program the data-plane engine over caller-provided descriptor-table bases. r0
 * = block-0 (TX/playback) table base, r1 = block-1 (RX/capture) table base;
 * pass 0 to skip a block. Leaves the engine armed but paused: it prefetches a
 * few descriptors and waits for the servicer to ACK 0x300 (gated by
 * stream_run).
 */
static void clarett_engine_program(struct clarett *c, dma_addr_t r0,
				   dma_addr_t r1)
{
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
		writel(c->model->playback_channels,
		       bar + STREAM_BLK0 + STREAM_OFF_CHANS);
		writel(clarett_period_bytes(c->model->playback_channels),
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
		writel(c->model->capture_channels,
		       bar + STREAM_BLK1 + STREAM_OFF_CHANS);
		writel(clarett_period_bytes(c->model->capture_channels),
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
		c->model->playback_channels, c->model->capture_channels);
}

/*
 * Start the persistent 0x300 servicer kthread. The caller flips stream_run to
 * release ACKing.
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

	atomic_set(&c->stream_periods, 0);
	c->stream_ctr = 0;
	c->stream_ctr_step = 0;
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
	/* stop the servicer ACKing 0x300 */
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
