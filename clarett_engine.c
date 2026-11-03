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

static int tx_trace;
module_param(tx_trace, int, 0644);
MODULE_PARM_DESC(tx_trace,
		 "Diagnostic (off by default): every Nth 0x300 period, log the "
		 "engine's block-0 (TX) and block-1 (RX) DMA pointer regs "
		 "(0x218/0x318) alongside the ctr step and our fill clock "
		 "(pcm_frames). N = the value (e.g. tx_trace=32 logs one line "
		 "per 32 periods; 1 = every period). Shows whether the engine "
		 "consumes TX faster than the fill. Runtime-writable.");

/*
 * Persistent servicing engine. The data-plane DMA engine is flow-controlled: it
 * raises a period on the 0x300 cause register and waits for the host to ACK by
 * reading it (read-to-clear). Without that the engine does a few descriptors
 * and stalls; serviced, it clocks for the stream's lifetime. This kthread polls
 * the cause block for the whole stream and drives the PCM path
 * (clarett_pcm_tick) from it.
 */
/*
 * Log one servicer telemetry line at info if the window was BAD, else at debug.
 * The healthy case is the common case and it repeats every 2 s for as long as a
 * stream is open — which, with enable_pcm on, is "as long as the machine is
 * up" (PipeWire adopts the card and holds a PCM). See the default-quiet
 * rationale at the window log below.
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
	/*
	 * Tick-lateness telemetry. The period counters cannot show this: they
	 * accumulate the HARDWARE ctr delta, so a late servicer catches up on
	 * the next tick and the totals stay perfectly smooth while the audio
	 * glitches. What matters is the wall-clock gap between period events
	 * — nominal is CLARETT_CTR_FRAMES*step/rate, which is NOT a constant:
	 * the IRQ cadence follows to the negotiated ALSA period, so it spans
	 * 0.33 ms (cadence 1) to tens of ms, and the rate varies too.
	 * clarett_tick_late_us() derives the threshold from both — do not
	 * reintroduce a fixed one. A gap far over nominal means the TX refill
	 * landed late and the engine read ring content the app had not been
	 * copied into yet. step_max is the same signal without a clock: the
	 * counter delta per event is normally one period's worth, so a doubled
	 * step IS a missed poll. Reset each log window.
	 */
	ktime_t last_ev;
	u64 gap_max_us;
	u32 gap_late, step_max;
	/*
	 * readmax = longest a single MMIO read-block took (us). Tells a stall's
	 * mechanism: if readmax ~= gapmax, the time is spent INSIDE a readl()
	 * (the TB link stalling a read completion, or the whole host frozen);
	 * if readmax stays small while gapmax spikes, the thread was
	 * descheduled BETWEEN reads.
	 */
	u64 read_us_max;
};

/*
 * Read the cause blocks the servicer owns and return 0x300.
 *
 * Cause-register ownership. When no command is in flight the servicer sweeps
 * all five blocks (0x100, 0x300, 0x200, 0x400, 0x500): 0x100 asserts bit31 per
 * period (read-to-clear) and must be serviced. But 0x100 (mailbox DONE), 0x400
 * (mailbox command phase) and 0x500 (IRQ summary) belong to the MAILBOX while
 * a command is in flight: the servicer runs at ~6.6 kHz, and read-clearing
 * 0x400 mid-command corrupts the command-phase handshake, so the command
 * times out. During a command the servicer therefore reads ONLY the stream
 * period blocks it owns (0x300/0x200); the mailbox sweep reciprocally skips
 * those two while streaming (clarett_mailbox.c). Every mailbox command sets
 * cmd_inflight, so the guard covers them all.
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

	if (!busy)
		/* 0x100 per-period cause (bit31) + mailbox DONE */
		readl(bar + REG_IRQ0_CAUSE);
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

/*
 * One 0x300 period event: advance the frame clock by the counter delta and
 * run the PCM tick. Returns false if the sample was rejected, in which case
 * the caller skips the rest of this pass.
 */
static bool clarett_svc_event(struct clarett *c, u32 c2,
			      struct clarett_svc_stats *st)
{
	void __iomem *bar = c->bar0;
	u32 ctr = c2 & CLARETT_CTR_MASK;
	ktime_t now;
	u64 gap;
	u32 step;

	/*
	 * First events of a session: raw counter values (the frame advance is
	 * ctr-delta driven). Debug: eight lines per arm is noise on a working
	 * driver.
	 */
	if (atomic_read(&c->stream_periods) < 8)
		dev_dbg(&c->pci->dev, "stream-ev[%d]: 0x300=0x%08x\n",
			atomic_read(&c->stream_periods), c2);

	/*
	 * Frames captured since the last event = ctr delta *
	 * CLARETT_CTR_FRAMES, where the delta is the MODULAR difference
	 * (CLARETT_CTR_MOD). A counter wrap and a late poll are the same
	 * arithmetic, so this recovers the frames a delayed tick has to make
	 * up instead of throwing them away — losing them would put pcm_frames
	 * permanently behind the engine, so the TX guard would no longer cover
	 * where the engine reads.
	 *
	 * Reject only samples carrying bits outside CLARETT_CTR_KNOWN — in
	 * practice the all-ones reads of a dead link. CLARETT_CTR_OVERRUN
	 * samples carry a valid counter and are kept (see clarett.h). Dropping
	 * is safe whatever an unknown bit means: the next accepted sample's
	 * modular difference recovers the advance across the gap.
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

	now = ktime_get();
	gap = ktime_to_us(ktime_sub(now, st->last_ev));
	st->last_ev = now;
	if (gap > st->gap_max_us)
		st->gap_max_us = gap;
	if (gap > clarett_tick_late_us(c))
		st->gap_late++;
	if (step > st->step_max)
		st->step_max = step;

	if (tx_trace > 0 &&
	    (atomic_read(&c->stream_periods) % tx_trace) == 0) {
		/* 0x218 engine TX read pos */
		u32 txptr = readl(bar + STREAM_BLK0 + STREAM_OFF_PTR);
		/* 0x318 engine RX write pos */
		u32 rxptr = readl(bar + STREAM_BLK1 + STREAM_OFF_PTR);

		dev_info(&c->pci->dev,
			 "tx-trace[%d]: ctr=0x%x step=%u frames=%u | TXptr=0x%08x RXptr=0x%08x | fill=pcm_frames=%llu\n",
			 atomic_read(&c->stream_periods), ctr, step,
			 step * CLARETT_CTR_FRAMES, txptr, rxptr,
			 (unsigned long long)c->pcm_frames);
	}

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
 * Close a 2-second window: log it, fold it into the run, start the next.
 *
 * DEFAULT-QUIET: a stream is open for as long as PipeWire holds the card, so
 * an unconditional line would log every 2 s indefinitely. Print at info only
 * when the window saw something actually wrong — a late tick (how a platform
 * stall presents), a device-side period overrun, or a rejected sample — and
 * leave the healthy case to dynamic debug. To get every window back without
 * also enabling the ~24 Hz mailbox trace, match this statement by its format:
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

static int clarett_stream_service(void *data)
{
	struct clarett *c = data;
	void __iomem *bar = c->bar0;
	unsigned long next_log = jiffies + msecs_to_jiffies(2000);
	struct clarett_svc_stats st = { .last_ev = ktime_get() };
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
		 * Gate ACKing on stream_run. The engine is armed (and
		 * prefetches a few descriptors) by clarett_engine_arm, but
		 * stays paused until stream_run is set — reading 0x300
		 * (read-to-clear) IS the period ACK that releases it, so
		 * withholding the read keeps the engine quiescent between
		 * prepare and START. The probe path sets stream_run at start.
		 */
		if (!READ_ONCE(c->stream_run)) {
			usleep_range(300, 600);
			continue;
		}

		c2 = clarett_svc_read_causes(c, &st);
		/*
		 * An all-ones read is a FAILED PCIe transaction, never data.
		 * bit31 is set in ~0, so it would otherwise present as a period
		 * event carrying a garbage counter, and the frame advance
		 * derived from it would wreck pcm_frames.
		 *
		 * Two cases. The device really left the bus (cable pulled, unit
		 * switched off) — stop. Or the link is TRANSIENTLY
		 * unreachable while the device is still attached (windows of
		 * tens of ms in which every MMIO read returns ~0 have been seen
		 * on some hosts). Drop the sample and let the next good read's
		 * modular difference recover the whole advance.
		 */
		if (c2 == 0xffffffff) {
			if (pci_dev_is_disconnected(c->pci)) {
				/*
				 * The device left the bus. Do NOT return from
				 * the threadfn here: this kthread is the target
				 * of kthread_stop() in clarett_engine_stop(),
				 * and a kthread that exits on its own BEFORE
				 * kthread_stop() runs makes kthread_stop()
				 * dereference the already-reaped task — a
				 * use-after-free that oopses in kthread_stop()
				 * from the pciehp remove thread. Park instead
				 * (the `gone` check at the loop top idles
				 * without touching the dead device) and let
				 * kthread_stop() set kthread_should_stop() to
				 * end the loop and reap us cleanly.
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
		/*
		 * Between period events: copy playback audio the app wrote
		 * since the tick (.ack).
		 */
		clarett_pcm_tx_refill(c);
		if (time_after(jiffies, next_log)) {
			clarett_svc_window(c, &st);
			next_log = jiffies + msecs_to_jiffies(2000);
		}
		usleep_range(100, 200);
	}
	/* fold the final (partial) window in before summarising */
	clarett_svc_fold(&st);

	/*
	 * One summary per stream, same default-quiet rule as the window line:
	 * at info only if the run had anything wrong with it, so a clean
	 * session leaves the log untouched from arm to teardown. PTR tells
	 * whether the engine walked the descriptor table at all.
	 */
	clarett_svc_log(c, st.run_late || st.bad_reads || st.overruns,
			"stream-svc: stopped (periods=%d ctr=0x%x wraps=%u ptr0=0x%x ptr1=0x%x; "
			"run gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u)\n",
			atomic_read(&c->stream_periods), c->stream_ctr,
			st.wraps, readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
			readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
			st.run_gap_max_us, st.run_read_us_max, st.run_late,
			st.run_step_max, st.bad_reads, st.bad_or, st.overruns);
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
	 * The register-only arm sequence: ack the cause block (0x100=0xf),
	 * 0x108, the global enable (0x20c=1) BEFORE the geometry/base writes,
	 * per-block geometry and bases, 0x10c, then 0x110=7. DMA only starts on
	 * the 0x110=7 arm, which stays last, so there is no null-base fault;
	 * 0x300 then starts ticking. No FCP commands here: the mailbox side
	 * (SET_CLOCK, sync queries) is clarett_stream_handshake(), run from PCM
	 * prepare before this. A null base arg skips that block.
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

	/*
	 * Post-arm state: whether the bases latched and where the per-block
	 * pointers start. Debug, not info: this path runs every time an app
	 * opens the card.
	 */
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
	 * Never start a second servicer. The pointer is the ONLY handle
	 * kthread_stop() has, so overwriting it orphans a SCHED_FIFO thread
	 * that polls the BAR forever and faults on module text after rmmod.
	 * clarett_pcm_prepare() claims the arm under pcm_lock so this cannot be
	 * reached, but a silent thread leak is far too expensive to leave
	 * guarded by one caller's discipline.
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
	 * Real-time priority for the servicer: it ACKs 0x300 and refills the TX
	 * ring every period, and if userspace preempts it the TX fill falls
	 * behind and the engine replays stale audio — audible skips despite
	 * no XRUN. It sleeps ~150 us between polls (work is microseconds), so
	 * it yields constantly and cannot starve the system; SCHED_FIFO just
	 * guarantees it runs promptly when a period is due. Low RT priority so
	 * it sits above normal tasks but below any other real-time thread.
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
	 * CLAIM THE TEARDOWN UNDER pcm_lock. Both directions can close
	 * simultaneously, and clarett_pcm_detach() makes its "last one out"
	 * test AFTER dropping pcm_lock, so two callers can both observe both
	 * substreams NULL and both arrive here. If both read the same
	 * stream_svc and both called kthread_stop(), the second would stop a
	 * task that has already exited and been reaped, blocking forever — an
	 * unkillable closing process, and a module that cannot be unloaded.
	 *
	 * Taking stream_on and the servicer handle together under the lock
	 * makes exactly one caller proceed; the loser returns at the stream_on
	 * test. This is the teardown mirror of the arm claim in
	 * clarett_pcm_prepare().
	 *
	 * kthread_stop() MUST run outside pcm_lock: the servicer calls
	 * clarett_pcm_tick(), which takes pcm_lock, so stopping it while
	 * holding the lock trades this hang for a deadlock.
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
 * Deregister the response-buffer owner and stop the device mastering the bus.
 * REG_DMA_ADDR_LO/HI are a standing registration — the armed device keeps the
 * host address and DMAs responses into it asynchronously, and nothing in the
 * protocol expires it. Left in place across an unload it is a dangling DMA
 * target (freed memory) for the rest of the power cycle. Bus master goes off
 * first so the device cannot issue a write to a half-zeroed (torn) address;
 * the zeroed registration is what the next same-power-cycle session inherits.
 * Call only after all mailbox traffic has stopped — a command issued after
 * this can never have its response land (the gated cycle times out).
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
