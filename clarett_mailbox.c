// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — FCP mailbox transport.
 *
 * One transaction: fill the request mailbox (cmd/size+seq/error/pad/data), ring
 * the doorbell, wait until the device has accepted the command and its response
 * DMA has landed, then acknowledge.
 */
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include "clarett.h"

/*
 * system_wq is deprecated from 6.17, which warns on first use and points at the
 * unbound system_dfl_wq. The deferred flash save has no CPU affinity to keep,
 * so it belongs there.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
#define CLARETT_SAVE_WQ	system_dfl_wq
#else
#define CLARETT_SAVE_WQ	system_unbound_wq
#endif

/*
 * Two completion cycles. With MSI up, a command completes on the device's own
 * 0x400 phase bits, delivered by the vec0 ISR: bit0 when the request is
 * accepted, bit1 when its response has landed. Without MSI (allocation or
 * request_irq failed, which the driver survives), it falls back to polling
 * 0x100 for DONE, sweeping the cause blocks and polling the response buffer.
 */

/*
 * Why the trailing ack waits for the response. The doorbell ack (0x408=2) means
 * "response consumed, buffer free": the device DMAs its response AFTER it
 * reports completion, and an ack before it lands makes the device refuse the
 * whole session (err=3 on every later command). So each command zeroes the
 * response header before submit (a repeated opcode must not match a stale
 * echo), waits for THIS command's echoed opcode in resp_buf, and only then
 * acks. A response that never arrives is never acked.
 *
 * A long reply lands in pieces, header first, so the payload area is poisoned
 * before submit too, and after the echo the wait continues until the last
 * bytes the header's size promises are no longer poison
 * (clarett_resp_tail_wait).
 *
 * resp_trace logs one line per command: the latencies, the echoed seq, the FCP
 * status word (resp+8) and the size.
 */
static bool resp_trace;
module_param(resp_trace, bool, 0444);
MODULE_PARM_DESC(resp_trace,
		 "Log per-command response telemetry: DONE and "
		 "response-landing latencies, echo, FCP status (resp+8), "
		 "size. Use with meter_poll_ms=0. Default 0.");

/*
 * Deadline for a command's response DMA to land, separate from
 * CLARETT_MBOX_TIMEOUT_MS (the DONE wait). A response that misses it is retired
 * by the next command (clarett_retire_lost()).
 */
static uint resp_timeout_ms = CLARETT_MBOX_TIMEOUT_MS;
module_param(resp_timeout_ms, uint, 0644);
MODULE_PARM_DESC(resp_timeout_ms,
		 "Deadline (ms) for a command's response DMA to land before "
		 "the trailing ack is withheld (default 100). Diagnostic: "
		 "raise it to find out whether a slow command's response is "
		 "merely LATE or never sent at all. resp_trace=1 prints the "
		 "landing latency.");

/*
 * The response deadline for one command: probe's override while it asks for
 * readiness, the long flash deadline for the FLASH category, else
 * resp_timeout_ms.
 */
static unsigned int clarett_resp_ms(const struct clarett *c, u32 opcode)
{
	if (c->resp_deadline_ms)
		return c->resp_deadline_ms;
	if ((opcode & ~CMD_EXEC_FLAG) >> 12 == 0x004)
		return CLARETT_FLASH_TIMEOUT_MS;
	return resp_timeout_ms;
}

/*
 * True for a stream period-cause block (0x200 TX / 0x300 RX) while the PCM
 * engine is streaming. The mailbox skips these in its DONE sweep so it does not
 * read-to-clear a period event the stream servicer is waiting for (an audible
 * gap). When idle they read 0 and are swept normally.
 */
static inline bool clarett_stream_cause(const struct clarett *c, u16 reg)
{
	return c->stream_on && (reg == STREAM_BLK0 || reg == STREAM_BLK1);
}

/*
 * Cause blocks the mailbox sweep leaves alone: the stream blocks while
 * streaming, and 0x400 whenever the vec0 ISR is up, which is then its only
 * reader (the ISR records the phase bits in mbox_phase).
 */
static inline bool clarett_skip_cause(const struct clarett *c, u16 reg)
{
	return clarett_stream_cause(c, reg) ||
	       (reg == REG_NOTIFY_CAUSE && c->irq_ready);
}

/* The FCP status word of the response that has just landed. */
static u32 clarett_resp_status(const struct clarett *c)
{
	return clarett_get_le32((const u8 *)c->resp_buf + FCP_RESP_STATUS_OFF);
}

/*
 * Wait for THIS command's response to land in resp_buf. Returns the matched
 * echo word (CMD_EXEC_FLAG | opcode — never zero), or 0 if nothing landed
 * before the deadline; *land_us gets the submit->landing latency (-1 on
 * timeout). Spins briefly (a landing, when it happens, is expected within ~150
 * us), then backs off to sleeps; called under mbox_lock, sleeping is fine.
 */
static u32 clarett_resp_wait(struct clarett *c, u32 exp_echo, ktime_t t_submit,
			     s64 *land_us)
{
	const u8 *r = c->resp_buf;
	unsigned int ms = clarett_resp_ms(c, exp_echo);
	unsigned long deadline = jiffies + msecs_to_jiffies(max(ms, 1u));
	ktime_t spin_until = ktime_add_us(ktime_get(), 500);
	u32 echo;

	for (;;) {
		/* order the DMAed response before we read resp_buf */
		dma_rmb();
		echo = r[FCP_RESP_ECHO_OFF] |
		       r[FCP_RESP_ECHO_OFF + 1] << 8 |
		       r[FCP_RESP_ECHO_OFF + 2] << 16 |
		       (u32)r[FCP_RESP_ECHO_OFF + 3] << 24;
		if (echo == exp_echo) {
			*land_us = ktime_us_delta(ktime_get(), t_submit);
			return echo;
		}
		if (time_after(jiffies, deadline)) {
			*land_us = -1;
			return 0;
		}
		if (ktime_before(ktime_get(), spin_until))
			cpu_relax();
		else
			usleep_range(20, 50);
	}
}

/*
 * Payload poison, and the wait for the tail of the response it makes possible.
 * Posted writes from the device land in order, so once the last payload bytes
 * the header announces are no longer poison, everything before them has landed
 * too. Real data can equal the poison, so the wait is bounded; on expiry the
 * bytes are taken as they are.
 */
#define CLARETT_RESP_POISON	0xa5
#define CLARETT_RESP_TAIL_US	2000

static void clarett_resp_tail_wait(struct clarett *c)
{
	const u8 *r = c->resp_buf;
	ktime_t until = ktime_add_us(ktime_get(), CLARETT_RESP_TAIL_US);
	size_t size, end, from, i;

	/* the header has landed: read its size field after it */
	dma_rmb();
	size = r[FCP_RESP_SIZE_OFF] | r[FCP_RESP_SIZE_OFF + 1] << 8;
	if (!size)
		/* not every command fills the size field */
		return;
	end = min_t(size_t, FCP_RESP_DATA_OFF + size, c->resp_size);
	from = max_t(size_t, end >= 4 ? end - 4 : 0, FCP_RESP_DATA_OFF);

	for (;;) {
		/* re-read the tail as the device's DMA fills it */
		dma_rmb();
		for (i = from; i < end; i++)
			if (r[i] != CLARETT_RESP_POISON)
				return;
		if (ktime_after(ktime_get(), until)) {
			dev_dbg(&c->pci->dev,
				"response tail still poison after %d us (size %zu); taking it as landed\n",
				CLARETT_RESP_TAIL_US, size);
			return;
		}
		cpu_relax();
	}
}

/*
 * Retire a command whose response missed its deadline, before the next command
 * is sent. A "lost" response is a late one, and until it is acknowledged the
 * device refuses every later command (err=3, no payload) whatever sequence
 * number it carries; acking it once it lands restores the session, with the
 * sequence continuing from the lost command. So: wait up to that command's own
 * deadline, ack and carry on; if it never lands (a device asked too early
 * after power-up answers nothing), start the sequence over at 0. Returns true
 * if it waited and nothing landed. Caller holds mbox_lock.
 */
static bool clarett_retire_lost(struct clarett *c)
{
	const u8 *r = c->resp_buf;
	unsigned long deadline;
	u32 echo;

	if (!c->lost_pending)
		return false;
	c->lost_pending = false;
	deadline = jiffies +
		   msecs_to_jiffies(max(clarett_resp_ms(c, c->lost_echo), 1u));
	for (;;) {
		/* re-read the late response's header as it lands */
		dma_rmb();
		echo = clarett_get_le32(r + FCP_RESP_ECHO_OFF);
		if (echo == c->lost_echo &&
		    clarett_get_le16(r + FCP_RESP_SEQ_OFF) == c->lost_seq) {
			clarett_wl(c, REG_DOORBELL, DOORBELL_ACK);
			c->seq = c->lost_seq + 1;
			dev_warn_ratelimited(&c->pci->dev,
					     "FCP op=0x%06x seq=%u: late response acknowledged\n",
					     c->lost_echo & ~CMD_EXEC_FLAG,
					     c->lost_seq);
			return false;
		}
		if (time_after(jiffies, deadline))
			break;
		usleep_range(200, 400);
	}
	c->seq = 0;
	return true;
}

/* One mailbox transaction's progress, shared by the steps below. */
struct clarett_fcp_txn {
	u32 opcode;
	ktime_t t_submit;
	/* 0x100 as last read; IRQ_DONE_BIT set once the command is accepted */
	u32 cause, first_cause;
	/* the echoed command word once the response has landed, else 0 */
	u32 resp_echo;
	u32 fcp_status;
	s64 done_us, land_us, acc_us;
	int polls;
};

/*
 * Prepare the response buffer, write the request into the mailbox and ring
 * the doorbell. Caller holds mbox_lock.
 */
static void clarett_mbox_submit(struct clarett *c, struct clarett_fcp_txn *t,
				const u8 *data, u16 len)
{
	int i;

	/* zero the header: a repeated opcode must not match a stale echo */
	memset(c->resp_buf, 0, FCP_RESP_DATA_OFF);
	/* and poison the payload, for clarett_resp_tail_wait */
	memset((u8 *)c->resp_buf + FCP_RESP_DATA_OFF, CLARETT_RESP_POISON,
	       c->resp_size - FCP_RESP_DATA_OFF);
	/* both in memory before the doorbell lets the device overwrite them */
	dma_wmb();

	clarett_wl(c, REG_MBOX + MBOX_CMD, CMD_EXEC_FLAG | t->opcode);
	clarett_wl(c, REG_MBOX + MBOX_SIZESEQ, ((u32)c->seq << 16) | len);
	clarett_wl(c, REG_MBOX + MBOX_ERROR, 0);
	clarett_wl(c, REG_MBOX + MBOX_PAD, 0);

	/*
	 * payload, written as little-endian 32-bit words (last word
	 * zero-padded)
	 */
	for (i = 0; i < len; i += 4) {
		u32 w = 0;
		int j;

		for (j = 0; j < 4 && i + j < len; j++)
			w |= (u32)data[i + j] << (8 * j);
		clarett_wl(c, REG_MBOX + MBOX_DATA + i, w);
	}

	/*
	 * Mark the command in flight BEFORE submitting, so the ISR records its
	 * phase bits rather than relaying them. Held until just before
	 * mutex_unlock, to cover an MSI delivered late.
	 */
	reinit_completion(&c->mbox_accepted);
	reinit_completion(&c->mbox_landed);
	atomic_set(&c->mbox_phase, 0);
	atomic_set(&c->cmd_inflight, 1);

	t->t_submit = ktime_get();
	clarett_wl(c, REG_DOORBELL, DOORBELL_SUBMIT);
}

/*
 * Wait for the command on the 0x400 phase bits the vec0 ISR completes:
 * accepted, then landed. Both deadlines follow the response deadline
 * (clarett_resp_ms(): resp_timeout_ms, ready_resp_ms while probe asks, longer
 * for a flash command). A command whose response has not landed is not acked
 * here; the next command retires it first (clarett_retire_lost()).
 */
static void clarett_mbox_wait_phase(struct clarett *c,
				    struct clarett_fcp_txn *t)
{
	unsigned int ms = clarett_resp_ms(c, t->opcode);
	unsigned long tmo = msecs_to_jiffies(max(ms, 1u));
	u32 exp_echo = CMD_EXEC_FLAG | t->opcode;

	t->polls = 1;
	if (!wait_for_completion_timeout(&c->mbox_accepted, tmo))
		return;
	t->acc_us = ktime_us_delta(ktime_get(), t->t_submit);
	/* accepted: the call succeeds, as DONE did */
	t->cause = IRQ_DONE_BIT;
	if (wait_for_completion_timeout(&c->mbox_landed, tmo))
		t->resp_echo = clarett_resp_wait(c, exp_echo, t->t_submit,
						 &t->land_us);
	if (t->resp_echo) {
		clarett_resp_tail_wait(c);
		/* read the status word only after the response has landed */
		dma_rmb();
		t->fcp_status = clarett_resp_status(c);
		clarett_wl(c, REG_DOORBELL, DOORBELL_ACK);
	} else {
		dev_warn_ratelimited(&c->pci->dev,
				     "FCP op=0x%06x seq=%u: accepted, response not landed; ack deferred\n",
				     t->opcode, c->seq);
	}
}

/*
 * Polled fallback (no MSI): once DONE shows in 0x100, sweep the remaining
 * cause blocks (0x400 holds the command phase and is read-to-clear), do one
 * confirming full sweep, then wait for the response and ack. Sweep only AFTER
 * DONE: sweeping the phase register while the command is processing makes it
 * time out. An incomplete command is never acknowledged.
 */
static void clarett_mbox_wait_polled(struct clarett *c,
				     struct clarett_fcp_txn *t)
{
	static const u16 sweep[] = {
		REG_IRQ0_CAUSE, STREAM_BLK1, STREAM_BLK0,
		REG_NOTIFY_CAUSE, 0x500
	};
	unsigned long deadline;
	int s;

	deadline = jiffies + msecs_to_jiffies(CLARETT_MBOX_TIMEOUT_MS);
	do {
		/* read-to-clear */
		t->cause = clarett_rl(c, REG_IRQ0_CAUSE);
		if (!t->polls)
			t->first_cause = t->cause;
		t->polls++;
		if (t->cause & IRQ_DONE_BIT)
			break;
		cpu_relax();
	} while (time_before(jiffies, deadline));

	if (!(t->cause & IRQ_DONE_BIT))
		return;
	t->done_us = ktime_us_delta(ktime_get(), t->t_submit);
	/*
	 * The sweeps skip the stream blocks while streaming: a read here would
	 * steal a period event from the servicer (clarett_stream_cause()).
	 */
	for (s = 1; s < ARRAY_SIZE(sweep); s++)
		if (!clarett_skip_cause(c, sweep[s]))
			/* rest of the DONE sweep */
			clarett_rl(c, sweep[s]);
	for (s = 0; s < ARRAY_SIZE(sweep); s++)
		if (!clarett_skip_cause(c, sweep[s]))
			/* confirming full sweep */
			clarett_rl(c, sweep[s]);
	t->resp_echo = clarett_resp_wait(c, CMD_EXEC_FLAG | t->opcode,
					 t->t_submit, &t->land_us);
	if (t->resp_echo) {
		/* the header has landed; a long reply's tail may not have */
		clarett_resp_tail_wait(c);
		/*
		 * Command-level failures are in the FCP status word (resp+8),
		 * not the mailbox error register: a rejected write lands a
		 * nonzero status while DONE and the echo look fine.
		 */
		dma_rmb();
		t->fcp_status = clarett_resp_status(c);
		clarett_wl(c, REG_DOORBELL, DOORBELL_ACK);
	} else {
		dev_warn_ratelimited(&c->pci->dev,
				     "FCP op=0x%06x seq=%u: response never landed; ack withheld\n",
				     t->opcode, c->seq);
	}
}

/*
 * Per-transaction trace, at dev_dbg since the meters are read up to 20 times a
 * second. resp_trace adds the response telemetry.
 */
static void clarett_mbox_trace(struct clarett *c,
			       const struct clarett_fcp_txn *t, bool by_phase)
{
	const u8 *r = c->resp_buf;

	dev_dbg(&c->pci->dev,
		"FCP op=0x%06x seq=%u %s first_cause=0x%08x polls=%d done=%d phase=0x%x acc=%lldus land=%lldus status=0x%08x\n",
		t->opcode, c->seq, by_phase ? "phase" : "done", t->first_cause,
		t->polls, !!(t->cause & IRQ_DONE_BIT),
		atomic_read(&c->mbox_phase), t->acc_us, t->land_us,
		t->fcp_status);

	if (!resp_trace)
		return;
	if (t->resp_echo) {
		u32 size = r[FCP_RESP_SIZE_OFF] | r[FCP_RESP_SIZE_OFF + 1] << 8;
		u32 rseq = r[FCP_RESP_SEQ_OFF] | r[FCP_RESP_SEQ_OFF + 1] << 8;

		dev_info(&c->pci->dev,
			 "FCPr op=0x%06x seq=%u done=%lldus resp=%lldus rseq=%u err=%u size=%u phase=0x%x\n",
			 t->opcode, c->seq, t->done_us, t->land_us, rseq,
			 r[FCP_RESP_STATUS_OFF], size,
			 atomic_read(&c->mbox_phase));
		/* the payload head of every answered non-meter command */
		if (size && t->opcode != FCP_GET_METER)
			dev_info(&c->pci->dev, "FCPr   payload=%*ph\n",
				 (int)min(size, 32u), r + FCP_RESP_DATA_OFF);
	} else {
		dev_info(&c->pci->dev,
			 "FCPr op=0x%06x seq=%u done=%lldus resp=NONE (ack withheld) phase=0x%x\n",
			 t->opcode, c->seq, t->done_us,
			 atomic_read(&c->mbox_phase));
	}
}

/*
 * Copy the response payload out while still holding the lock (resp_buf is
 * stable until the next command, which can't start before we unlock).
 */
static void clarett_mbox_copy_out(struct clarett *c,
				  const struct clarett_fcp_txn *t,
				  u8 *resp_out, u16 resp_len, u16 *size_out,
				  u32 *status_out)
{
	const u8 *r = c->resp_buf;
	u16 have;

	/* read the size and payload only after the response has landed */
	dma_rmb();
	have = r[FCP_RESP_SIZE_OFF] | r[FCP_RESP_SIZE_OFF + 1] << 8;
	/*
	 * Honour the response's own size. The device answers some commands
	 * with LESS than the caller asked for — MUX_READ caps every reply at
	 * 28 entries (112 bytes) however large a count is requested — and
	 * copying the full requested length then hands the caller whatever
	 * the PREVIOUS command left in resp_buf: stale DMA content presented
	 * as device data, and via the hwdep a kernel-memory disclosure.
	 * Zero-fill the tail so a short reply is unmistakably short. size == 0
	 * is left alone: not every command fills the field, and a zero-length
	 * answer is judged by outcome (see the note in __clarett_fcp()).
	 */
	if (have && have < resp_len) {
		memcpy(resp_out, r + FCP_RESP_DATA_OFF, have);
		memset(resp_out + have, 0, resp_len - have);
		dev_dbg(&c->pci->dev,
			"op 0x%06x: short response %u < %u requested\n",
			t->opcode, have, resp_len);
	} else {
		memcpy(resp_out, r + FCP_RESP_DATA_OFF, resp_len);
	}
	if (size_out)
		*size_out = have;
	if (status_out)
		*status_out = t->fcp_status;
}

/*
 * Wedge detection, for probe's report of a device that never became ready. No
 * response at all, or one echoing a sequence number that is not ours, both mean
 * the device is still answering an earlier unretired command.
 */
static void clarett_mbox_check_wedge(struct clarett *c,
				     const struct clarett_fcp_txn *t)
{
	u16 rseq;

	if (!t->resp_echo) {
		c->mbox_wedged = true;
		return;
	}
	/* read the echoed seq only after the response has landed */
	dma_rmb();
	rseq = ((const u8 *)c->resp_buf)[FCP_RESP_SEQ_OFF] |
	       ((const u8 *)c->resp_buf)[FCP_RESP_SEQ_OFF + 1] << 8;
	c->mbox_wedged = rseq != c->seq;
}

/*
 * The result. A command with no answer of its own fails: not accepted or not
 * answered in time (-ETIMEDOUT), or answered with another command's sequence
 * number (-EIO); handing back the response buffer then would give the caller
 * poison or a stale reply as if it were data.
 *
 * NOTE: fcp_status (resp+8) is NOT a clean pass/fail on this device — INIT_1
 * on an already initialised device returns a nonzero status yet the command
 * works. So it does not fail the call by itself; it is surfaced in the dev_dbg
 * trace and via resp_trace. The one case it does decide is a caller asking for
 * data and getting a nonzero status with none: a refusal.
 */
static int clarett_mbox_result(struct clarett *c,
			       const struct clarett_fcp_txn *t,
			       const u8 *resp_out, u16 resp_len)
{
	if (!(t->cause & IRQ_DONE_BIT) || !t->resp_echo)
		return -ETIMEDOUT;
	if (c->mbox_wedged)
		return -EIO;
	if (resp_out && resp_len && t->fcp_status &&
	    !clarett_get_le16((const u8 *)c->resp_buf + FCP_RESP_SIZE_OFF))
		return -EIO;
	return 0;
}

/*
 * Advance the sequence number for a command the device answered as ours. One
 * with no answer — accepted but not landed, or not seen accepted at all (it
 * may yet be, late) — is retired by the next command (clarett_retire_lost(),
 * which sets the sequence). Except a command not accepted straight after a
 * retire found nothing: the device is not answering, so don't make every later
 * command wait out a second deadline as well. Otherwise — another command's
 * answer — start over at 0: measured on a Clarett asked too early after
 * power-up, re-asking as seq 1 stays refused (err=3, stale echoed seq)
 * indefinitely, re-asking as seq 0 is answered.
 */
static void clarett_mbox_next_seq(struct clarett *c,
				  const struct clarett_fcp_txn *t,
				  bool retire_failed)
{
	if (t->resp_echo && !c->mbox_wedged) {
		c->seq++;
	} else if (!t->resp_echo &&
		   ((t->cause & IRQ_DONE_BIT) || !retire_failed)) {
		c->lost_pending = true;
		c->lost_echo = CMD_EXEC_FLAG | t->opcode;
		c->lost_seq = c->seq;
	} else {
		c->seq = 0;
	}
}

/*
 * Core mailbox transaction. If resp_out/resp_len are given, the response
 * *payload* (the bytes after the 16-byte echoed header) is copied out under
 * mbox_lock on success — race-free, unlike reading c->resp_buf after the call
 * — and size_out/status_out, if given, get the response's size and FCP
 * status. clarett_fcp() is the response-less wrapper; clarett_fcp_cmd() is the
 * hwdep CMD path that needs the payload; clarett_fcp_read() also returns the
 * size and status.
 */
static int __clarett_fcp(struct clarett *c, u32 opcode, const u8 *data,
			 u16 len, u8 *resp_out, u16 resp_len, u16 *size_out,
			 u32 *status_out)
{
	struct clarett_fcp_txn t = {
		.opcode = opcode,
		.done_us = -1,
		.land_us = -1,
		.acc_us = -1,
	};
	bool by_phase, retire_failed;
	int ret;

	if (len > CLARETT_MBOX_DATA_MAX)
		return -EINVAL;

	/*
	 * A removed device answers every MMIO read with ~0 and swallows writes,
	 * so the gated cycle would wait out its full timeout for a response
	 * that can never land — once per command, for every command still
	 * queued when the Thunderbolt cable is pulled or the unit is switched
	 * off. Fail fast instead; the PCI core sets this the moment the link
	 * goes.
	 */
	if (pci_dev_is_disconnected(c->pci))
		return -ENODEV;

	mutex_lock(&c->mbox_lock);
	/* torn down: resp_buf and the BAR may already be freed */
	if (c->hw_gone) {
		mutex_unlock(&c->mbox_lock);
		return -ENODEV;
	}

	retire_failed = clarett_retire_lost(c);
	clarett_mbox_submit(c, &t, data, len);

	by_phase = c->irq_ready;
	if (by_phase)
		clarett_mbox_wait_phase(c, &t);
	else
		clarett_mbox_wait_polled(c, &t);

	/*
	 * No mailbox register is read back: the error channel is resp+8 in the
	 * DMA buffer, and a read here would race the device's response DMA.
	 */
	clarett_mbox_trace(c, &t, by_phase);

	clarett_mbox_check_wedge(c, &t);
	ret = clarett_mbox_result(c, &t, resp_out, resp_len);

	/* only on success, and only if the caller asked */
	if (!ret && resp_out && resp_len)
		clarett_mbox_copy_out(c, &t, resp_out, resp_len, size_out,
				      status_out);

	clarett_mbox_next_seq(c, &t, retire_failed);

	/* completion window closed; idle-gap events may resume */
	atomic_set(&c->cmd_inflight, 0);

	mutex_unlock(&c->mbox_lock);
	return ret;
}

/*
 * Read-to-clear the cause blocks between probe's readiness attempts, so each
 * starts from a clean slate.
 */
void clarett_mbox_clear_causes(struct clarett *c)
{
	static const u16 causes[] = {
		REG_IRQ0_CAUSE, STREAM_BLK1, STREAM_BLK0,
		REG_NOTIFY_CAUSE, 0x500
	};
	int i;

	mutex_lock(&c->mbox_lock);
	for (i = 0; i < ARRAY_SIZE(causes); i++)
		if (!clarett_skip_cause(c, causes[i]))
			clarett_rl(c, causes[i]);
	mutex_unlock(&c->mbox_lock);
}

int clarett_fcp(struct clarett *c, u32 opcode, const u8 *data, u16 len)
{
	return __clarett_fcp(c, opcode, data, len, NULL, 0, NULL, NULL);
}

/*
 * hwdep FCP_IOCTL_CMD: run one command and return `resp_len` bytes of its
 * response payload.
 */
int clarett_fcp_cmd(struct clarett *c, u32 opcode, const u8 *req, u16 req_len,
		    u8 *resp, u16 resp_len)
{
	return __clarett_fcp(c, opcode, req, req_len, resp, resp_len,
			     NULL, NULL);
}

/*
 * As clarett_fcp_cmd(), also returning the response's size field and FCP status
 * (on success only).
 */
int clarett_fcp_read(struct clarett *c, u32 opcode, const u8 *req, u16 req_len,
		     u8 *resp, u16 resp_len, u16 *size, u32 *status)
{
	return __clarett_fcp(c, opcode, req, req_len, resp, resp_len,
			     size, status);
}

/* SET_DATA: write `len` bytes into the config space at `offset`. */
int clarett_set_data(struct clarett *c, u32 offset, u32 len, const u8 *val)
{
	u8 buf[8 + CLARETT_MAX_PAYLOAD];

	if (len > CLARETT_MAX_PAYLOAD)
		return -EINVAL;
	clarett_put_le32(buf, offset);
	clarett_put_le32(buf + 4, len);
	memcpy(buf + 8, val, len);
	return clarett_fcp(c, FCP_SET_DATA, buf, 8 + len);
}

/*
 * DATA_CMD: commit a preceding SET_DATA (activate == the control's command
 * number).
 */
int clarett_data_cmd(struct clarett *c, u32 activate)
{
	u8 buf[4];

	clarett_put_le32(buf, activate);
	return clarett_fcp(c, FCP_DATA_CMD, buf, 4);
}

/*
 * Schedule the debounced NVRAM commit (DATA_CMD{FCP_ACTIVATE_PERSIST}) so a
 * config change survives a power cycle: a burst (a slider drag) becomes one
 * save CLARETT_SAVE_DELAY_MS after the LAST change. Gated on ctl_ready, so the
 * probe-time monitor-enable write does not commit flash on every load. Called
 * from the in-kernel write path and from the hwdep relay when fcp-server
 * commits a change (clarett_hwdep_cmd): the device persists some config by
 * itself (routing) but not the rest (output gains, S/PDIF source).
 */
void clarett_schedule_persist(struct clarett *c)
{
	if (READ_ONCE(c->ctl_ready))
		mod_delayed_work(CLARETT_SAVE_WQ, &c->save_work,
				 msecs_to_jiffies(CLARETT_SAVE_DELAY_MS));
}

/*
 * Write one config byte and commit it. DATA_CMD{activate} applies the change
 * live but not across a power cycle; persisting is a separate debounced
 * DATA_CMD{FCP_ACTIVATE_PERSIST}.
 */
static int __clarett_write_u8(struct clarett *c, u32 offset, u8 val,
			      u32 activate, bool save)
{
	int err;

	if (offset >= CLARETT_CONFIG_SIZE)
		return -EINVAL;

	err = clarett_set_data(c, offset, 1, &val);
	if (err)
		return err;
	err = clarett_data_cmd(c, activate);
	if (err)
		return err;

	c->shadow[offset] = val;
	/* write-through: the shadow now matches hardware */
	set_bit(offset, c->shadow_known);

	/* nosave writes are mirrors, not user intent, and skip the persist */
	if (save)
		clarett_schedule_persist(c);
	return 0;
}

int clarett_write_u8(struct clarett *c, u32 offset, u8 val, u32 activate)
{
	return __clarett_write_u8(c, offset, val, activate, true);
}

/*
 * As clarett_write_u8, but without the NVRAM save: for bytes the driver keeps
 * as a mirror of other state (the knob-following SW gains,
 * clarett_hw_gain_follow), which would otherwise commit flash on every knob
 * movement.
 */
int clarett_write_u8_nosave(struct clarett *c, u32 offset, u8 val, u32 activate)
{
	return __clarett_write_u8(c, offset, val, activate, false);
}

/*
 * Read-modify-write the `mask` bits of a shared config byte to the value in
 * `val`, then commit. Used for the command-3 enable bytes (72/73) that pack one
 * bit per output: the shadow MUST have been seeded from the device first
 * (clarett_seed_shadow) so the other outputs' bits survive.
 */
int clarett_write_bits(struct clarett *c, u32 offset, u8 mask, u8 val,
		       u32 activate)
{
	u8 updated;

	if (offset >= CLARETT_CONFIG_SIZE)
		return -EINVAL;

	updated = (c->shadow[offset] & ~mask) | (val & mask);
	if (updated == c->shadow[offset])
		return 0;
	return clarett_write_u8(c, offset, updated, activate);
}
