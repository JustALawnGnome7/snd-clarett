// SPDX-License-Identifier: GPL-2.0-only
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — PCI bring-up and data-plane engine.
 *
 * Supports the Clarett Thunderbolt line (2Pre / 4Pre / 8Pre / 8PreX) and the Red range (8Line),
 * detected at probe. Provides the mixer control plane (through the FCP hwdep + fcp-server),
 * PCM capture and playback, and DIN MIDI.
 *
 * Clean-room: built from black-box observation of the device's host interface and
 * Focusrite's device XML descriptors; no vendor driver code was used.
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>		/* sched_set_fifo_low() — real-time priority for the stream servicer */
#include <linux/bitmap.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/initval.h>
#include "clarett.h"

#define PCI_VENDOR_FOCUSRITE   0x1cb5
#define PCI_DEVICE_CLARETT     0x0002

static bool stream_probe;
module_param(stream_probe, bool, 0444);
MODULE_PARM_DESC(stream_probe,
		 "Diagnostic: after probe, program the ring registers with a driver buffer and watch "
		 "for vec1/vec2 IRQs + DMA-pointer movement (off by default; excludes enable_pcm).");

static bool enable_pcm = true;
module_param(enable_pcm, bool, 0444);
MODULE_PARM_DESC(enable_pcm,
		 "Register the PCM devices (playback + capture, S32_LE, descriptor-ring engine driven "
		 "by the 0x300 servicer). Default on. Set 0 for a control-only card, or when using "
		 "stream_probe (mutually exclusive).");

static int tx_trace;
module_param(tx_trace, int, 0644);
MODULE_PARM_DESC(tx_trace,
		 "Diagnostic (off by default): every Nth 0x300 period, log the engine's block-0 (TX) and "
		 "block-1 (RX) DMA pointer regs (0x218/0x318) alongside the ctr step and our fill clock "
		 "(pcm_frames). N = the value (e.g. tx_trace=32 logs one line per 32 periods; 1 = every "
		 "period). Shows whether the engine consumes TX faster than the fill. Runtime-writable.");

/*
 * Host bring-up ("arm") policy at probe: THE DRIVER NEVER ARMS.
 *
 * A device that has been set up once restores its session from flash at power-up: reads, input metering
 * and control writes all work with no host bring-up. Every unit that has been through Focusrite Control
 * is in that state, and a host-side bring-up would reset the user's routing and mixer to the factory
 * default. Probe waits for the flash-persisted session to answer (clarett_detect_model) and detects the
 * model from it, arming nothing.
 *
 * If the device never answers within wait_ready_ms, probe does NOT fall back to a placeholder, which
 * would pass a not-ready device off as a working card. It fails loudly; reload to retry.
 */

/*
 * Total budget for the readiness retry (see the loop in probe). It has to cover a device still waking
 * from a cold power-up, which can take tens of seconds of quiet. It is only meaningful with the long
 * quiet between attempts (CLARETT_READY_RETRY_MS) — the same budget spent polling tightly recovers
 * nothing.
 *
 * Costs a warm attach nothing: the first attempt answers in ~90 us and the budget is never touched.
 * Probe is asynchronous, so the worst case does not stall the PCI hotplug worker.
 *
 * Writable at runtime because it is read only inside probe, and a Thunderbolt device re-probes on every
 * power cycle — so a write applies to the next attach without needing the card free.
 */
static unsigned int wait_ready_ms = 100000;
module_param(wait_ready_ms, uint, 0644);
MODULE_PARM_DESC(wait_ready_ms,
		 "Total budget (ms) for the readiness retry at probe before giving up "
		 "(default 100000). A warm device answers the first attempt and never spends it.");

/*
 * Leave the device untouched for this long after attach, before the pre-mailbox init.
 *
 * A first touch ~140 ms after enumeration can fail, while one at 1 s or later has not. 3 s is margin over
 * that, not a tuned value; the mechanism is not established.
 *
 * When it does fail, the command completes (DONE raised) but never DMAs a response, so the trailing ack
 * is withheld (acking an unlanded response makes the device refuse the session), and the device is left
 * holding that command unretired, answering it in place of every later one (stale rseq, err=3). Nothing
 * over the mailbox recovers that, which is why this is a don't-touch window rather than a retry.
 *
 * Every probe pays it, including a reload or sysfs rebind: unbinding disables the PCI device and
 * re-enabling brings it back in whatever state a fresh attach is in.
 */
static unsigned int settle_ms = 3000;
module_param(settle_ms, uint, 0644);
MODULE_PARM_DESC(settle_ms,
		 "Leave the device untouched for this long (ms) after attach before the first init "
		 "(default 3000). Touching a freshly attached device too early can wedge its mailbox until "
		 "it is power-cycled. 0 disables the wait.");

/*
 * RX fragment slot stride. A fragment that straddles a 4 KB page is mis-framed by the device's DMA: with
 * the fragments packed contiguously, the capture drifts its channel alignment by 8 bytes per page. Giving
 * each RX fragment its own page-safe SLOT (a power of two, so it divides the page) over a page-aligned
 * area keeps every fragment page-contained. Default (-1) = auto = roundup_pow_of_two(fragment). 0 = packed
 * contiguously (drifts; diagnostic). >0 = fragment audio bytes + this many.
 */
static int rx_frag_pad = -1;
module_param(rx_frag_pad, int, 0444);
MODULE_PARM_DESC(rx_frag_pad,
		 "RX fragment slot: -1 = auto page-safe pow2 (default), 0 = contiguous (drifts; "
		 "diagnostic), >0 = audio bytes + this padding.");

static int tx_frag_pad = -1;
module_param(tx_frag_pad, int, 0444);
MODULE_PARM_DESC(tx_frag_pad,
		 "TX fragment slot (mirror of rx_frag_pad): -1 = auto page-safe pow2 (default), "
		 "0 = contiguous (garbles playback on models whose fragment is not a power of two; "
		 "diagnostic), >0 = audio bytes + this padding.");

static int meter_poll_ms = CLARETT_METER_POLL_MS;
module_param(meter_poll_ms, int, 0444);
MODULE_PARM_DESC(meter_poll_ms,
		 "Period (ms) of the GET_METER host heartbeat, as Focusrite Control issues while connected. "
		 "Default 40; 0 disables it (diagnostic).");

/*
 * Relay 0x400 notifications to userspace while a stream runs. Off by default, which suppresses the
 * relay for the duration of any stream (see the stream_on gate in clarett_irq) to save the mailbox
 * traffic of fcp-server's re-reads; monitor_poll keeps the monitor controls live meanwhile.
 * Runtime-writable.
 */
static bool notify_while_streaming;
module_param(notify_while_streaming, bool, 0644);
MODULE_PARM_DESC(notify_while_streaming,
		 "Relay device notifications to userspace while streaming (default 0: suppressed for the "
		 "duration of a stream, with monitor_poll covering the monitor controls). 1 lets fcp-server "
		 "re-read its notifiable controls mid-stream, at the cost of that mailbox traffic.");

static int dma_bits = 32;
module_param(dma_bits, int, 0444);
MODULE_PARM_DESC(dma_bits,
		 "DMA coherent mask width (bits). Default 32. Lower it (e.g. 31 -> <2 GB, 30 -> <1 GB) to "
		 "force the stream buffer lower in the address space (diagnostic).");

static bool monitor_enables = true;
module_param(monitor_enables, bool, 0444);
MODULE_PARM_DESC(monitor_enables,
		 "At probe, write the monitor HW-enable bits (0x48/0x49, cmd3) so global Mute/Dim affect "
		 "Monitor Out 1-2. Default true; 0 leaves the device's own setting.");

static bool seed_dump;
module_param(seed_dump, bool, 0444);
MODULE_PARM_DESC(seed_dump,
		 "One-shot dump of the full seeded config shadow [0,256) at probe (16 lines); diff two "
		 "dumps to locate a setting's offset. Default 0.");

static bool premailbox_reads = true;
module_param(premailbox_reads, bool, 0444);
MODULE_PARM_DESC(premailbox_reads,
		 "Read the full pre-mailbox BAR0 register set at attach (caps/serial/fw-header/"
		 "cause-blocks/0x514/0x58c) before the first FCP command. Default true; 0 reads only "
		 "what the driver uses (diagnostic).");

static bool error_probe;
module_param(error_probe, bool, 0444);
MODULE_PARM_DESC(error_probe,
		 "Diagnostic: at probe, send a few deliberately MALFORMED FCP commands (bad offset, zero "
		 "length, unknown opcode) alongside a valid GET_DATA and log each response's error word "
		 "(resp+8) and size. Off by default (sends junk commands). Combine with meter_poll_ms=0, or "
		 "the heartbeat's responses interleave with the probe's.");

static const struct clarett_model clarett_8prex, clarett_2pre, clarett_4pre, clarett_8pre,
				  red_8line;	/* defined below; chosen by clarett_detect_model() */

/*
 * Model selection: THE DEVICE DECIDES, always. There is deliberately no override.
 *
 * The whole line shares PCI id 1cb5:0002 and presents a byte-identical PRE-MAILBOX surface (MMIO
 * regs, config space, the fw-info header, even the dummy serial), and the Thunderbolt DROM's model
 * name is not reliably visible either (whether a Thunderbolt 2 unit is enumerated as a kernel-managed
 * Thunderbolt device depends on the host firmware). But the device reports its own stream geometry:
 * GET_7.1{band 0} answers {u16 playback_ch, u16 capture_ch}, unique per model.
 *
 * Everything downstream is sized from c->model: channel counts, DMA ring and descriptor geometry,
 * fragment strides, routing and mixer tables, meter layout, the fcp-server map slug. A wrong model
 * is therefore not a cosmetic mislabel, it is a card that streams the wrong width into wrongly
 * strided rings — which is why a mismatch REFUSES TO REGISTER rather than fall back to a plausible
 * guess, and why no guess is available as a module parameter either. If a genuinely new model
 * appears, the probe error prints the raw geometry pair to add to the table below.
 */

/*
 * Ask the device who it is: GET_7.1{band 0} returns {u16 playback_ch, u16 capture_ch}
 * (+16 more bytes, meaning open), a pair unique per model. Runs before the meter heartbeat
 * starts, so nothing else touches resp_buf between the (landed-gated) completion and the parse.
 * Returns NULL if the query fails or the pair matches no known model.
 *
 * A NULL return is fatal to probe either way — there is no fallback model — but *collapsed
 * distinguishes WHY, because the two need opposite things from the user: true = the GET path is
 * dead (transport error, or a status=3/size=0 refusal), which usually means "not ready yet,
 * retry"; false = a VALID
 * reply whose geometry is not in the table, i.e. genuinely unknown hardware, which needs a new
 * clarett_model entry. The non-quiet warn on each path prints the raw pair for exactly that.
 */
static const struct clarett_model *clarett_detect_model(struct clarett *c, bool *collapsed,
							bool quiet)
{
	static const struct clarett_model *const models[] = {
		&clarett_2pre, &clarett_4pre, &clarett_8pre, &clarett_8prex,
		&red_8line,
	};
	static const u8 band0;
	const u8 *r = c->resp_buf;
	u16 pb, cap, size;
	u8 status;
	int i, err;

	*collapsed = false;

	/*
	 * Every early return logs the raw response (when not quiet), so a first attach of a new
	 * model pins the cause (transport vs. status vs. unmatched) in one line and surfaces the
	 * actual pair to add to the table.
	 */
	err = clarett_fcp(c, FCP_GET_71, &band0, 1);
	if (err) {
		if (!quiet)
			dev_warn(&c->pci->dev,
				 "model auto-detect: GET_7.1 transport failed (%d)\n", err);
		*collapsed = true;	/* GET didn't complete at all — the path is dead */
		return NULL;
	}
	dma_rmb();	/* order the DMAed response before we read resp_buf */
	status = r[FCP_RESP_STATUS_OFF];
	size = r[FCP_RESP_SIZE_OFF] | r[FCP_RESP_SIZE_OFF + 1] << 8;
	pb  = r[FCP_RESP_DATA_OFF]     | r[FCP_RESP_DATA_OFF + 1] << 8;
	cap = r[FCP_RESP_DATA_OFF + 2] | r[FCP_RESP_DATA_OFF + 3] << 8;
	if (status != FCP_RESP_ERR_OK || size < 4) {
		if (!quiet)
			dev_warn(&c->pci->dev,
				 "model auto-detect: GET_7.1 bad response (status=%u size=%u raw playback=%u capture=%u)\n",
				 status, size, pb, cap);
		*collapsed = true;	/* GET refused (status=3/size=0) */
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(models); i++)
		if (models[i]->playback_channels == pb &&
		    models[i]->capture_channels == cap)
			return models[i];

	if (!quiet)
		dev_warn(&c->pci->dev,
			 "model auto-detect: unrecognized stream geometry (playback=%u capture=%u) — unknown model, needs a clarett_model entry\n",
			 pb, cap);
	return NULL;
}

void clarett_wl(struct clarett *c, u32 off, u32 val)
{
	writel(val, c->bar0 + off);
}

u32 clarett_rl(struct clarett *c, u32 off)
{
	return readl(c->bar0 + off);
}

static void clarett_hw_init(struct clarett *c)
{
	void __iomem *bar = c->bar0;

	/*
	 * Pre-mailbox init: 0x510/0x500 device enable, the DMA-response address, and 0x104 (interrupt
	 * causes). By default (premailbox_reads) this also reads the full attach register set — caps,
	 * 0x4/0x8/0x514/0x58c, all cause blocks and the whole fw-info header — in the order, and with the
	 * ~1-8 ms spacing between register groups, of a normal attach. readl() returns are discarded except
	 * serial/fw (kept for the probe summary). Process context, so sleeping is fine.
	 */
	if (premailbox_reads) {
		u32 r000, r004, r008, r514, r58c_a, r58c_b;

		readl(bar + REG_INFO);			/* 0x8000 — first touch */
		r004 = readl(bar + 0x004);
		r008 = readl(bar + 0x008);
		r000 = readl(bar + REG_CAPS);		/* 0x000 */
		c->serial_hi = readl(bar + REG_SERIAL_HI);	/* 0x014 (hi before lo) */
		c->serial_lo = readl(bar + REG_SERIAL_LO);	/* 0x010 */
		usleep_range(7000, 7200);		/* ~7 ms */
		r514 = readl(bar + 0x514);
		usleep_range(1300, 1400);		/* ~1.30 ms */
		clarett_wl(c, 0x510, 0x8);
		r58c_a = readl(bar + 0x58c);		/* back-to-back with the write */
		usleep_range(780, 850);			/* ~0.78 ms */
		clarett_wl(c, 0x500, 0x8);
		usleep_range(880, 950);			/* ~0.88 ms */
		clarett_wl(c, REG_IRQ0_ENABLE, 0xf000003f);	/* 0x104 — BEFORE the DMA addr */
		usleep_range(5640, 5800);		/* ~5.64 ms */
		clarett_wl(c, REG_DMA_ADDR_LO, lower_32_bits(c->resp_dma));
		usleep_range(1850, 1950);		/* ~1.85 ms */
		clarett_wl(c, REG_DMA_ADDR_HI, upper_32_bits(c->resp_dma));
		usleep_range(3200, 3350);		/* ~3.22 ms */
		{
			/*
			 * Read-to-clear cause blocks (0x100, 0x300, 0x200, 0x400, 0x500). On a cold boot they
			 * report and clear whatever the device latched at power-on; clearing them is also what
			 * makes the Analogue 2 gain LED flash briefly at probe.
			 */
			u32 c100 = readl(bar + REG_IRQ0_CAUSE);
			u32 c300 = readl(bar + STREAM_BLK1);
			u32 c200 = readl(bar + STREAM_BLK0);
			u32 c400 = readl(bar + REG_NOTIFY_CAUSE);
			u32 c500 = readl(bar + 0x500);

			dev_dbg(&c->pci->dev,
				"pre-mailbox causes: 0x100=0x%08x 0x300=0x%08x 0x200=0x%08x 0x400=0x%08x 0x500=0x%08x\n",
				c100, c300, c200, c400, c500);
		}
		usleep_range(8220, 8400);		/* ~8.22 ms */
		r58c_b = readl(bar + 0x58c);
		dev_dbg(&c->pci->dev,
			"pre-mailbox regs: caps(0x0)=0x%08x 0x4=0x%08x 0x8=0x%08x 0x514=0x%08x 0x58c=0x%08x/0x%08x\n",
			r000, r004, r008, r514, r58c_a, r58c_b);
		c->fw_app  = readl(bar + REG_INFO + 0x00);	/* 0x8000 */
		c->fw_fpga = readl(bar + REG_INFO + 0x04);	/* 0x8004 */
		readl(bar + REG_INFO + 0x08);		/* rest of the 8-word fw-info header */
		readl(bar + REG_INFO + 0x0c);
		readl(bar + REG_INFO + 0x10);
		readl(bar + REG_INFO + 0x14);
		readl(bar + REG_INFO + 0x18);
		readl(bar + REG_INFO + 0x1c);
	} else {
		/*
		 * Read-minimal variant (premailbox_reads=0, diagnostic). 0x510/0x500 are the device-open
		 * writes (global clock/converter-subsystem enable); the DMA-response address goes to
		 * REG_DMA_ADDR_LO/HI (HI = bus-address high 32 bits); 0x104 enables the interrupt causes.
		 */
		c->serial_lo = readl(bar + REG_SERIAL_LO);
		c->serial_hi = readl(bar + REG_SERIAL_HI);
		c->fw_app    = readl(bar + REG_INFO + 0);
		c->fw_fpga   = readl(bar + REG_INFO + 4);

		clarett_wl(c, 0x510, 0x8);
		clarett_wl(c, 0x500, 0x8);
		clarett_wl(c, REG_DMA_ADDR_LO, lower_32_bits(c->resp_dma));
		clarett_wl(c, REG_DMA_ADDR_HI, upper_32_bits(c->resp_dma));
		clarett_wl(c, REG_IRQ0_ENABLE, 0xf000003f);
	}

	memset(c->shadow, 0, sizeof(c->shadow));
}

/*
 * Error-code probe (error_probe, diagnostic). Send a valid GET_DATA plus deliberately malformed and
 * unsupported commands, and log each response's error word (resp+8) and size. Reads resp+8 directly
 * because the BAR MBOX_ERROR word reads 0; the real error is in the DMAed response.
 */
static void clarett_error_probe(struct clarett *c)
{
	static const struct {
		const char *name;
		u32 opcode;
		u8 data[8];
		u16 len;
	} cmds[] = {
		/* Status codes (resp+8) on a working session, one per validation stage:
		 *   valid → 0; bad parameter (offset out of range / oversized length) → 1;
		 *   unsupported category (ESP_DFU 0x009) → 4; unknown opcode or sub-op in a
		 *   supported category → 7. Zero-length and past-the-end reads are accepted (0).
		 * A refused session answers everything identically: err=3, size=0, seq=0 (the
		 * request seq is not echoed), opcode echoed. */
		{ "GET_DATA{24,4} valid",     FCP_GET_DATA, { 24,0,0,0,  4,0,0,0 }, 8 },
		{ "GET_DATA bad-offset",      FCP_GET_DATA, { 0,0,0xff,0xff, 4,0,0,0 }, 8 },
		{ "unknown opcode 0x0000ff",  0x0000ff,     { 0 }, 0 },
		/* Queries that answer with real data on a working session: READ_SEG (segment open),
		 * GET_7/GET_6 device queries, CONFIG_PUSH{id} (a per-id name query: 0x1e -> "ADAT 8"),
		 * and GET_DATA{0xc8} in the persistent appspace. */
		{ "READ_SEG{0,8}",            FCP_READ_SEG, { 0,0,0,0,  8,0,0,0 }, 8 },
		{ "GET_7.1{0}",               0x007001,     { 0 }, 1 },
		{ "GET_6.2",                  0x006002,     { 0 }, 0 },
		{ "CONFIG_PUSH{0x1e}",        0x005000,     { 0x1e, 0 }, 2 },
		{ "GET_DATA{0xc8,8}",         FCP_GET_DATA, { 0xc8,0,0,0, 8,0,0,0 }, 8 },
		/* Further validation stages: length errors, an unknown sub-op inside a supported
		 * category, and a command in an unsupported one. Read-only — no side effects. */
		{ "GET_DATA zero-len",        FCP_GET_DATA, { 24,0,0,0,  0,0,0,0 }, 8 },
		{ "GET_DATA oversized-len",   FCP_GET_DATA, { 0,0,0,0,  0,0,1,0 }, 8 },   /* len=0x10000 */
		{ "GET_DATA overrun-end",     FCP_GET_DATA, { 0xf8,0,0,0, 0x40,0,0,0 }, 8 }, /* off ok, off+len past end */
		{ "unknown sub-op in MUX",    0x0030ff,     { 0 }, 0 },   /* supported cat 0x003, bogus sub-op */
		{ "unknown sub-op in MIX",    0x0020ff,     { 0 }, 0 },   /* supported cat 0x002, bogus sub-op */
		{ "unsupported cat ESP_DFU",  0x009000,     { 0 }, 0 },   /* CAP_READ: 0x009 NOT SUPPORTED */
		{ "unsupported cat ESP_DFU1", 0x009001,     { 0 }, 0 },
	};
	const u8 *r = c->resp_buf;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cmds); i++) {
		u32 exp_echo = CMD_EXEC_FLAG | cmds[i].opcode;
		u16 exp_seq = c->seq;		/* the seq clarett_fcp will stamp on this command */
		unsigned long deadline;
		u32 echo = 0, err = 0;
		u16 size = 0, rseq = 0;
		bool landed = false;

		/*
		 * The device DMAs its response ASYNCHRONOUSLY, a little after the BAR done bit clarett_fcp
		 * polls — so reading resp_buf immediately races and catches the PREVIOUS command's late
		 * response. Wait for THIS command's response by matching the echoed opcode (unique per command
		 * after the memset). A refusal does not echo the request seq (it writes seq=0), so match the
		 * echo ONLY; a timeout is a genuine no-response. Requires meter_poll_ms=0 (else the heartbeat
		 * shares resp_buf and bumps seq).
		 */
		memset(c->resp_buf, 0, 32);
		ret = clarett_fcp(c, cmds[i].opcode, cmds[i].data, cmds[i].len);

		deadline = jiffies + msecs_to_jiffies(50);
		do {
			dma_rmb();
			echo = r[0] | r[1] << 8 | r[2] << 16 | r[3] << 24;
			rseq = r[FCP_RESP_SEQ_OFF] | r[FCP_RESP_SEQ_OFF + 1] << 8;
			if (echo == exp_echo) {
				landed = true;
				break;
			}
			usleep_range(200, 300);
		} while (time_before(jiffies, deadline));

		size = r[4] | r[5] << 8;
		err  = r[8] | r[9] << 8 | r[10] << 16 | r[11] << 24;
		dev_info(&c->pci->dev,
			 "error_probe: %-20s fcp_ret=%d %s echo=0x%08x seq=%u(exp %u) err=%u size=%u payload=%*ph\n",
			 cmds[i].name, ret, landed ? "RESP" : "NO-RESP",
			 echo, rseq, exp_seq, err, size, 8, r + FCP_RESP_DATA_OFF);
	}
}

/*
 * Seed the whole config shadow from the device. clarett_hw_init() zeroes the shadow; the seed
 * gives the command-3 enable bytes (72/73) their real values, which their packed bits need for a
 * safe read-modify-write. The first GET after programming the DMA address can come back empty
 * (echo word 0), so retry briefly. Guard on BOTH the echo word AND the response size: a refusing
 * device returns the header with size=0 and no payload, and copying that would seed stale bytes.
 */
static int clarett_seed_shadow(struct clarett *c)
{
	const u8 *r = c->resp_buf;
	u32 echo;
	u16 size;
	int err, attempt;

	for (attempt = 0; attempt < 3; attempt++) {
		err = clarett_get_data(c, 0, CLARETT_CONFIG_SIZE);
		if (err)
			return err;

		dma_rmb();	/* order the DMAed response before we read resp_buf */
		echo = clarett_get_le32(r + FCP_RESP_ECHO_OFF);
		size = r[FCP_RESP_SIZE_OFF] | r[FCP_RESP_SIZE_OFF + 1] << 8;
		if (echo == (CMD_EXEC_FLAG | FCP_GET_DATA) && size >= CLARETT_CONFIG_SIZE) {
			memcpy(c->shadow, r + FCP_RESP_DATA_OFF, CLARETT_CONFIG_SIZE);
			/* Only 24/28/112 are known to read back live; mark just those known. The rest of
			 * the seed — preamp Mode/Air in particular — is not device-reported, so it stays
			 * unknown and its first write goes through. */
			set_bit(24,  c->shadow_known);
			set_bit(28,  c->shadow_known);
			set_bit(112, c->shadow_known);
			return 0;
		}
	}
	return -EIO;
}

/*
 * Make the two monitor outputs follow the monitor section's Mute/Dim. Without these command-3
 * enable bits, writing the global Mute (24) / Dim (28) flips the master flag but no output obeys
 * it. RMW from the seeded shadow so the other outputs' enable bits are kept;
 * idempotent — clarett_write_bits() no-ops if the bits are already set.
 */
static int clarett_enable_monitor_hw_controls(struct clarett *c)
{
	int err;

	err = clarett_write_bits(c, HWEN_MUTE_OFFSET, HWEN_MONITOR_MUTE_MASK,
				 HWEN_MONITOR_MUTE_MASK, HWEN_ACTIVATE);
	if (err)
		return err;
	return clarett_write_bits(c, HWEN_DIM_OFFSET, HWEN_MONITOR_DIM_MASK,
				  HWEN_MONITOR_DIM_MASK, HWEN_ACTIVATE);
}

/*
 * stream_probe diagnostic layout: two identical rings (block 0 = TX, block 1 = RX). Each ring is a
 * zero-terminated descriptor table followed by its sample fragments; *ring is one ring's byte span,
 * so block 1 begins at offset *ring. Same math in start and report so they agree.
 */
static void clarett_ring_layout(struct clarett *c, size_t *tbl, size_t *smp, size_t *ring)
{
	*tbl = clarett_tbl_bytes();
	*smp = clarett_buf_bytes(c);
	*ring = clarett_ring_bytes(c);
}

/* 1 s after engine-start, log whether DMA advanced: period IRQs, pointer regs, capture-buffer writes. */
static void clarett_stream_report(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, stream_report.work);
	size_t tbl, smp, ring, i;
	const u8 *rx_smp;
	bool rx_data = false;
	u32 c1_hits = 0, c2_hits = 0, c2_min = 0xffffffff, c2_max = 0;

	/*
	 * The streaming signal is the block cause register: while streaming, 0x300 reads
	 * 0x80000000 | an advancing period counter. Poll 0x200/0x300 here in a ~40 ms burst; bit31-set
	 * reads with an advancing low counter == periods firing.
	 */
	for (i = 0; i < 2000; i++) {
		u32 c1 = readl(c->bar0 + STREAM_BLK0);		/* 0x200 cause (read-to-clear) */
		u32 c2 = readl(c->bar0 + STREAM_BLK1);		/* 0x300 cause (read-to-clear) */

		if (c1 & 0x80000000)
			c1_hits++;
		if (c2 & 0x80000000) {
			u32 ctr = c2 & 0x7fffffff;

			c2_hits++;
			if (ctr < c2_min)
				c2_min = ctr;
			if (ctr > c2_max)
				c2_max = ctr;
		}
		udelay(20);
	}
	if (c2_min == 0xffffffff)
		c2_min = 0;

	clarett_ring_layout(c, &tbl, &smp, &ring);
	rx_smp = (const u8 *)c->stream_buf + ring + tbl;	/* block-1 (capture) sample area */
	for (i = 0; i < smp; i++) {
		if (rx_smp[i] != 0xAA) {	/* any byte the device overwrote (incl. zeros) */
			rx_data = true;
			break;
		}
	}

	dev_info(&c->pci->dev,
		 "engine probe @1s: vec1=%d vec2=%d IRQs; ptr0=0x%x ptr1=0x%x; "
		 "cause-poll 0x200:hits=%u 0x300:hits=%u ctr=0x%x..0x%x; capture-buf=%s\n",
		 atomic_read(&c->period_irqs[1]), atomic_read(&c->period_irqs[2]),
		 readl(c->bar0 + STREAM_BLK0 + STREAM_OFF_PTR),
		 readl(c->bar0 + STREAM_BLK1 + STREAM_OFF_PTR),
		 c1_hits, c2_hits, c2_min, c2_max,
		 rx_data ? "WRITTEN (marker gone)" : "untouched (marker intact)");
}

/*
 * Persistent servicing engine. The data-plane DMA engine is flow-controlled: it raises a period on the
 * 0x300 cause register and waits for the host to ACK by reading it (read-to-clear). Without that the
 * engine does a few descriptors and stalls; serviced, it clocks for the stream's lifetime. This kthread
 * polls the cause block for the whole stream and drives the PCM path (clarett_pcm_tick) from it.
 */
/*
 * Log one servicer telemetry line at info if the window was BAD, else at debug. The healthy case is
 * the common case and it repeats every 2 s for as long as a stream is open — which, with enable_pcm
 * on, is "as long as the machine is up" (PipeWire adopts the card and holds a PCM). See the
 * default-quiet rationale at the window log below.
 */
#define clarett_svc_log(c, bad, fmt, ...)					\
	do {									\
		if (bad)							\
			dev_info(&(c)->pci->dev, fmt, ##__VA_ARGS__);		\
		else								\
			dev_dbg(&(c)->pci->dev, fmt, ##__VA_ARGS__);		\
	} while (0)

static int clarett_stream_service(void *data)
{
	struct clarett *c = data;
	void __iomem *bar = c->bar0;
	unsigned long next_log = jiffies + msecs_to_jiffies(2000);
	u32 wraps = 0, bad_reads = 0;
	u32 bad_or = 0;		/* cumulative OR of every rejected 0x300 sample (badbits) */
	u32 overruns = 0;	/* events flagged CLARETT_CTR_OVERRUN (device saw a missed ack) */
	/* Previous window's values of the cumulative counters, so a window can tell whether IT was bad. */
	u32 prev_bad_reads = 0, prev_overruns = 0;
	/* Run-wide worst case, for the one-line summary at stop (the windows reset their own maxima). */
	u64 run_gap_max_us = 0, run_read_us_max = 0;
	u32 run_late = 0, run_step_max = 0;
	bool seen = false;
	bool gone = false;	/* set when the device leaves the bus: park the loop, never return early (kthread_stop UAF) */
	/*
	 * Tick-lateness telemetry. The period counters cannot show
	 * this: they accumulate the HARDWARE ctr delta, so a late servicer catches up on the next tick and
	 * the totals stay perfectly smooth while the audio glitches. What matters is the wall-clock gap
	 * between period events — nominal is CLARETT_CTR_FRAMES*step/rate, which is NOT a constant: dyn_period
	 * ties the IRQ cadence to the negotiated ALSA period, so it spans 0.33 ms (cadence 1) to tens of ms,
	 * and the rate varies too. clarett_tick_late_us() derives the threshold from both — do not reintroduce
	 * a fixed one. A gap far over nominal means the TX refill landed late and the engine read ring content the app had not been
	 * copied into yet. step_max is the same signal without a clock: the counter delta per event is
	 * normally one period's worth, so a doubled step IS a missed poll. Reset each log window.
	 */
	ktime_t last_ev = ktime_get();
	u64 gap_max_us = 0;
	u32 gap_late = 0, step_max = 0;
	/*
	 * readmax = longest a single MMIO read-block took (us). Tells a stall's mechanism: if readmax ~=
	 * gapmax, the time is spent INSIDE a readl() (the TB link stalling a read completion, or the whole
	 * host frozen); if readmax stays small while gapmax spikes, the thread was descheduled BETWEEN reads.
	 */
	u64 read_us_max = 0;

	while (!kthread_should_stop()) {
		u32 c2;

		if (gone) {		/* bus gone (below): idle here, never self-exit, until kthread_stop() reaps us */
			usleep_range(1000, 2000);
			continue;
		}

		/*
		 * Gate ACKing on stream_run. The engine is armed (and prefetches a few descriptors) by
		 * clarett_engine_arm, but stays paused until stream_run is set — reading
		 * 0x300 (read-to-clear) IS the period ACK that releases it, so withholding the read keeps
		 * the engine quiescent between prepare and START. The probe path sets stream_run at start.
		 */
		if (!READ_ONCE(c->stream_run)) {
			usleep_range(300, 600);
			continue;
		}

		/*
		 * Cause-register ownership. When no command is in flight the servicer sweeps all five blocks
		 * (0x100, 0x300, 0x200, 0x400, 0x500): 0x100 asserts bit31 per period (read-to-clear) and must
		 * be serviced. But 0x100 (mailbox DONE), 0x400 (mailbox command phase) and 0x500 (IRQ summary)
		 * belong to the MAILBOX while a command is in flight: the servicer runs at ~6.6 kHz, and
		 * read-clearing 0x400 mid-command corrupts the command-phase handshake, so the command times
		 * out. During a command the servicer therefore reads ONLY the stream period blocks it owns
		 * (0x300/0x200); the mailbox sweep reciprocally skips those two while streaming
		 * (clarett_mailbox.c). Every mailbox command sets cmd_inflight, so the guard covers them all.
		 */
		bool busy = atomic_read(&c->cmd_inflight);
		ktime_t rt0 = ktime_get();		/* time the read block: is a stall inside a readl()? */

		if (!busy)
			readl(bar + REG_IRQ0_CAUSE);	/* 0x100 per-period cause (bit31) + mailbox DONE */
		c2 = readl(bar + STREAM_BLK1);		/* 0x300 read-to-clear = period event + counter (owned) */
		readl(bar + STREAM_BLK0);		/* 0x200 TX cause (owned) */
		if (!busy) {
			readl(bar + REG_NOTIFY_CAUSE);	/* 0x400 command-phase/notify — mailbox's during a command */
			readl(bar + 0x500);		/* 0x500 IRQ summary */
		}
		{
			u64 rus = ktime_us_delta(ktime_get(), rt0);

			if (rus > read_us_max)
				read_us_max = rus;
		}
		/*
		 * An all-ones read is a FAILED PCIe transaction, never data. bit31 is set in ~0, so it would
		 * otherwise present as a period event carrying a garbage counter, and the frame advance
		 * derived from it would wreck pcm_frames.
		 *
		 * Two cases. The device really left the bus (cable pulled, unit switched off) — stop. Or the
		 * link is TRANSIENTLY unreachable while the device is still attached (windows of tens of ms
		 * in which every MMIO read returns ~0 have been seen on some hosts). Drop the sample and let
		 * the next good read's modular difference recover the whole advance.
		 */
		if (c2 == 0xffffffff) {
			if (pci_dev_is_disconnected(c->pci)) {
				/*
				 * The device left the bus. Do NOT return from the threadfn here: this kthread
				 * is the target of kthread_stop() in clarett_engine_stop(), and a kthread that
				 * exits on its own BEFORE kthread_stop() runs makes kthread_stop() dereference
				 * the already-reaped task — a use-after-free that oopses in kthread_stop() from
				 * the pciehp remove thread. Park instead (the
				 * `gone` check at the loop top idles without touching the dead device) and let
				 * kthread_stop() set kthread_should_stop() to end the loop and reap us cleanly.
				 */
				gone = true;
				continue;
			}
			if (!bad_reads++)
				dev_warn(&c->pci->dev,
					 "0x300 read returned ~0 with the device still attached: the link is "
					 "transiently unreachable (PCIe power management?). Dropping the sample.\n");
			usleep_range(100, 200);
			continue;
		}

		if (c2 & CLARETT_CTR_EVENT) {
			u32 ctr = c2 & CLARETT_CTR_MASK;
			u32 step;

			/* First events of a session: raw counter values (the frame advance is ctr-delta
			 * driven). Debug: eight lines per arm is noise on a working driver. */
			if (atomic_read(&c->stream_periods) < 8)
				dev_dbg(&c->pci->dev, "stream-ev[%d]: 0x300=0x%08x\n",
					atomic_read(&c->stream_periods), c2);

			/*
			 * Frames captured since the last event = ctr delta * CLARETT_CTR_FRAMES, where the delta
			 * is the MODULAR difference (CLARETT_CTR_MOD). A counter wrap and a late poll are the same
			 * arithmetic, so this recovers the frames a delayed tick has to make up instead of
			 * throwing them away — losing them would put pcm_frames permanently behind the engine,
			 * so the TX guard would no longer cover where the engine reads.
			 *
			 * Reject only samples carrying bits outside CLARETT_CTR_KNOWN — in practice the all-ones
			 * reads of a dead link. CLARETT_CTR_OVERRUN samples carry a valid counter and are kept
			 * (see clarett.h). Dropping is safe whatever an unknown bit means: the next accepted
			 * sample's modular difference recovers the advance across the gap.
			 */
			if (c2 & ~CLARETT_CTR_KNOWN) {
				bad_reads++;
				bad_or |= c2;
				if (bad_reads <= 8)
					dev_info(&c->pci->dev,
						 "stream-badread[%u]: 0x300=0x%08x (unknown bits 0x%08x)\n",
						 bad_reads, c2, c2 & ~CLARETT_CTR_KNOWN);
				usleep_range(100, 200);
				continue;
			}
			if (c2 & CLARETT_CTR_OVERRUN)
				overruns++;
			step = (ctr - c->stream_ctr) & (CLARETT_CTR_MOD - 1);
			if (!step) {
				/* An exact multiple of the modulus: the advance is genuinely unknowable. */
				step = c->stream_ctr_step;
				if (seen)
					wraps++;
			}
			if (step)
				c->stream_ctr_step = step;
			c->stream_ctr = ctr;
			seen = true;

			{
				ktime_t now = ktime_get();
				u64 gap = ktime_to_us(ktime_sub(now, last_ev));

				last_ev = now;
				if (gap > gap_max_us)
					gap_max_us = gap;
				if (gap > clarett_tick_late_us(c))
					gap_late++;
			}
			if (step > step_max)
				step_max = step;

			if (tx_trace > 0 && (atomic_read(&c->stream_periods) % tx_trace) == 0) {
				u32 txptr = readl(bar + STREAM_BLK0 + STREAM_OFF_PTR); /* 0x218 engine TX read pos */
				u32 rxptr = readl(bar + STREAM_BLK1 + STREAM_OFF_PTR); /* 0x318 engine RX write pos */

				dev_info(&c->pci->dev,
					 "tx-trace[%d]: ctr=0x%x step=%u frames=%u | TXptr=0x%08x RXptr=0x%08x | fill=pcm_frames=%llu\n",
					 atomic_read(&c->stream_periods), ctr, step,
					 step * CLARETT_CTR_FRAMES, txptr, rxptr,
					 (unsigned long long)c->pcm_frames);
			}

			atomic_inc(&c->stream_periods);
			clarett_pcm_tick(c, step * CLARETT_CTR_FRAMES);	/* advance by real captured frames (no-op if idle) */
		}
		/* Between period events: copy playback audio the app wrote since the tick (.ack). */
		clarett_pcm_tx_refill(c);
		if (time_after(jiffies, next_log)) {
			/*
			 * DEFAULT-QUIET: a stream is open for as long as PipeWire holds the card, so an
			 * unconditional line would log every 2 s indefinitely. Print at info only when the
			 * window saw something actually wrong — a late tick (how a platform stall presents), a
			 * device-side period overrun, or a rejected sample — and leave the healthy case to
			 * dynamic debug. To get every window back without also enabling the ~24 Hz
			 * mailbox trace, match this statement by its format:
			 *   echo 'format "stream-svc:" +p' >/sys/kernel/debug/dynamic_debug/control
			 */
			bool bad = gap_late || bad_reads != prev_bad_reads ||
				   overruns != prev_overruns;

			clarett_svc_log(c, bad,
				 "stream-svc: periods=%d ctr=0x%x wraps=%u gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u\n",
				 atomic_read(&c->stream_periods), c->stream_ctr, wraps,
				 gap_max_us, read_us_max, gap_late, step_max, bad_reads, bad_or, overruns);

			if (gap_max_us > run_gap_max_us)
				run_gap_max_us = gap_max_us;
			if (read_us_max > run_read_us_max)
				run_read_us_max = read_us_max;
			if (step_max > run_step_max)
				run_step_max = step_max;
			run_late += gap_late;
			prev_bad_reads = bad_reads;
			prev_overruns = overruns;

			gap_max_us = 0;
			read_us_max = 0;
			gap_late = 0;
			step_max = 0;
			next_log = jiffies + msecs_to_jiffies(2000);
		}
		usleep_range(100, 200);
	}
	/* Fold the final (partial) window into the run-wide worst case before summarising. */
	if (gap_max_us > run_gap_max_us)
		run_gap_max_us = gap_max_us;
	if (read_us_max > run_read_us_max)
		run_read_us_max = read_us_max;
	if (step_max > run_step_max)
		run_step_max = step_max;
	run_late += gap_late;

	/*
	 * One summary per stream, same default-quiet rule as the window line: at info only if the run
	 * had anything wrong with it, so a clean session leaves the log untouched from arm to teardown.
	 * PTR tells whether the engine walked the descriptor table at all.
	 */
	clarett_svc_log(c, run_late || bad_reads || overruns,
		 "stream-svc: stopped (periods=%d ctr=0x%x wraps=%u ptr0=0x%x ptr1=0x%x; "
		 "run gapmax=%lluus readmax=%lluus late=%u stepmax=0x%x badreads=%u badbits=0x%08x overrun=%u)\n",
		 atomic_read(&c->stream_periods), c->stream_ctr, wraps,
		 readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
		 run_gap_max_us, run_read_us_max, run_late, run_step_max,
		 bad_reads, bad_or, overruns);
	return 0;
}

/*
 * Program the data-plane engine over caller-provided descriptor-table bases.
 * r0 = block-0 (TX/playback) table base, r1 = block-1 (RX/capture) table base; pass 0 to skip a block.
 * Leaves the engine armed but paused: it prefetches a few descriptors and waits for the servicer to
 * ACK 0x300 (gated by stream_run).
 */
static void clarett_engine_program(struct clarett *c, dma_addr_t r0, dma_addr_t r1)
{
	void __iomem *bar = c->bar0;

	/*
	 * The register-only arm sequence: ack the cause block (0x100=0xf), 0x108, the global enable
	 * (0x20c=1) BEFORE the geometry/base writes, per-block geometry and bases, 0x10c, then 0x110=7.
	 * DMA only starts on the 0x110=7 arm, which stays last, so there is no null-base fault; 0x300 then
	 * starts ticking. No FCP commands here: the mailbox side (SET_CLOCK, CONFIG_PUSH, sync queries) is
	 * clarett_stream_handshake(), run from PCM prepare before this. A null base arg skips that block.
	 */
	writel(0xf, bar + REG_IRQ0_CAUSE);				/* 0x100 = 0xf: ack cause block */
	writel(0x10, bar + REG_STREAM_IRQ_CFG);				/* 0x108 */
	writel(1, bar + STREAM_BLK0 + STREAM_OFF_CTRL);			/* 0x20c global enable (before geometry) */

	if (r0) {
		writel(c->model->playback_channels, bar + STREAM_BLK0 + STREAM_OFF_CHANS); /* 0x204 */
		writel(clarett_period_bytes(c->model->playback_channels),
		       bar + STREAM_BLK0 + STREAM_OFF_SIZE);				  /* 0x208 period */
		writel(upper_32_bits(r0),
		       bar + STREAM_BLK0 + STREAM_OFF_BASE_HI);			  /* 0x214 */
		writel(lower_32_bits(r0), bar + STREAM_BLK0 + STREAM_OFF_BASE_LO); /* 0x210 (low last) */
	}
	if (r1) {
		writel(c->model->capture_channels, bar + STREAM_BLK1 + STREAM_OFF_CHANS);  /* 0x304 */
		writel(clarett_period_bytes(c->model->capture_channels),
		       bar + STREAM_BLK1 + STREAM_OFF_SIZE);				  /* 0x308 period */
		writel(upper_32_bits(r1),
		       bar + STREAM_BLK1 + STREAM_OFF_BASE_HI);			  /* 0x314 */
		writel(lower_32_bits(r1), bar + STREAM_BLK1 + STREAM_OFF_BASE_LO); /* 0x310 */
	}

	writel(0x1e70700, bar + REG_STREAM_IRQ_CFG2);			/* 0x10c */
	writel(0x7, bar + REG_STREAM_IRQ_ARM);				/* 0x110 arm (0x0 is stream-stop) */

	c->stream_on = true;
}

/* Arm the engine. Sleeps — process context only, which is where every caller already is. */
void clarett_engine_arm(struct clarett *c, dma_addr_t r0, dma_addr_t r1)
{
	void __iomem *bar = c->bar0;

	clarett_engine_program(c, r0, r1);

	/*
	 * Post-arm state: whether the bases latched and where the per-block pointers start. Debug, not
	 * info: this path runs every time an app opens the card.
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

/* Start the persistent 0x300 servicer kthread. The caller flips stream_run to release ACKing. */
void clarett_engine_run(struct clarett *c)
{
	/*
	 * Never start a second servicer. The pointer is the ONLY handle kthread_stop() has, so overwriting
	 * it orphans a SCHED_FIFO thread that polls the BAR forever and faults on module text after rmmod.
	 * clarett_pcm_prepare() claims the arm under pcm_lock so this cannot be reached, but a silent
	 * thread leak is far too expensive to leave guarded by one caller's discipline.
	 */
	if (WARN_ON_ONCE(c->stream_svc))
		return;

	atomic_set(&c->stream_periods, 0);
	c->stream_ctr = 0;
	c->stream_ctr_step = 0;
	c->stream_svc = kthread_run(clarett_stream_service, c, "clarett-svc");
	if (IS_ERR(c->stream_svc)) {
		dev_warn(&c->pci->dev, "stream servicer failed to start: %ld\n", PTR_ERR(c->stream_svc));
		c->stream_svc = NULL;
		return;
	}
	/*
	 * Real-time priority for the servicer: it ACKs 0x300 and refills the TX ring every period, and if
	 * userspace preempts it the TX fill falls behind and the engine replays stale audio — audible skips
	 * despite no XRUN. It
	 * sleeps ~150 us between polls (work is microseconds), so it yields constantly and cannot starve the
	 * system; SCHED_FIFO just guarantees it runs promptly when a period is due. Low RT priority so it
	 * sits above normal tasks but below any other real-time thread.
	 */
	sched_set_fifo_low(c->stream_svc);
}

/*
 * Data-plane engine-start diagnostic (opt-in via stream_probe). Arms the engine over a minimal descriptor
 * table — 0x210/0x214 point at a zero-terminated array of 8-byte bus addresses, each naming one
 * stream_frag fragment of a coherent buffer — and watches whether it runs (vec1/vec2 IRQs, advancing
 * pointer, the device writing the capture buffer). NOT a PCM implementation.
 */
int clarett_engine_start(struct clarett *c)
{
	void __iomem *bar = c->bar0;
	size_t tbl, smp, ring;
	__le64 *tx_tbl, *rx_tbl;
	dma_addr_t tx_smp, rx_smp, r0, r1;
	unsigned int i;

	clarett_ring_layout(c, &tbl, &smp, &ring);
	c->stream_size = 2 * ring;
	c->stream_buf = dmam_alloc_coherent(&c->pci->dev, c->stream_size,
					    &c->stream_dma, GFP_KERNEL);
	if (!c->stream_buf)
		return -ENOMEM;

	/* Block 0 (vec1) = playback/TX, block 1 (vec2) = capture/RX. */
	tx_tbl = (__le64 *)c->stream_buf;
	rx_tbl = (__le64 *)((u8 *)c->stream_buf + ring);
	tx_smp = c->stream_dma + tbl;
	rx_smp = c->stream_dma + ring + tbl;

	/* Fill descriptors with fragment bus addresses; entry [NDESC] stays 0 = ring terminator. */
	for (i = 0; i < CLARETT_STREAM_NDESC; i++) {
		tx_tbl[i] = cpu_to_le64(tx_smp + (dma_addr_t)i * c->model->stream_frag);
		rx_tbl[i] = cpu_to_le64(rx_smp + (dma_addr_t)i * c->model->stream_frag);
	}

	/*
	 * Bit 0 on the LAST entry is the engine's end-of-list / ring-wrap marker; without it the
	 * playback engine's per-descriptor status writeback bursts to base 0. Fragment addresses are
	 * even, so bit 0 is free to carry the flag.
	 */
	tx_tbl[CLARETT_STREAM_NDESC - 1] |= cpu_to_le64(1);
	rx_tbl[CLARETT_STREAM_NDESC - 1] |= cpu_to_le64(1);

	/*
	 * Mark the RX (capture) sample area with 0xAA so the report can tell "device wrote silence (zeros)"
	 * from "device never wrote" — with nothing plugged in, a working capture writes near-zero samples
	 * that a plain non-zero scan would miss.
	 */
	memset((u8 *)c->stream_buf + ring + tbl, 0xAA, smp);

	r0 = c->stream_dma;		/* block-0 descriptor-table base */
	r1 = c->stream_dma + ring;	/* block-1 descriptor-table base */

	/* Arm the engine (register sequence only; no handshake on this diagnostic path). */
	clarett_engine_arm(c, r0, r1);
	WRITE_ONCE(c->stream_run, true);	/* probe streams immediately — no PCM trigger to gate it */
	clarett_engine_run(c);

	dev_info(&c->pci->dev,
		 "engine probe: started; %u-desc tables @ %pad / %pad, ptr0=0x%x ptr1=0x%x\n",
		 CLARETT_STREAM_NDESC, &r0, &r1,
		 readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_PTR));
	/* Diagnostic: did the base latch, and what does the device see at descriptor[0]? */
	dev_info(&c->pci->dev,
		 "engine regs: blk0 base=%08x:%08x ctrl=%08x ptr=%08x | blk1 base=%08x:%08x ctrl=%08x ptr=%08x | "
		 "tx_desc[0]=%016llx rx_desc[0]=%016llx\n",
		 readl(bar + STREAM_BLK0 + STREAM_OFF_BASE_HI),
		 readl(bar + STREAM_BLK0 + STREAM_OFF_BASE_LO),
		 readl(bar + STREAM_BLK0 + STREAM_OFF_CTRL),
		 readl(bar + STREAM_BLK0 + STREAM_OFF_PTR),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_BASE_HI),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_BASE_LO),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_CTRL),
		 readl(bar + STREAM_BLK1 + STREAM_OFF_PTR),
		 le64_to_cpu(tx_tbl[0]), le64_to_cpu(rx_tbl[0]));
	schedule_delayed_work(&c->stream_report, msecs_to_jiffies(1000));
	return 0;
}

/* Halt the engine before the ring buffer is freed (a bad/continued DMA would fault the IOMMU). */
void clarett_engine_stop(struct clarett *c)
{
	struct task_struct *svc;

	/*
	 * CLAIM THE TEARDOWN UNDER pcm_lock. Both directions can close simultaneously, and
	 * clarett_pcm_detach() makes its "last one out" test AFTER dropping pcm_lock, so two callers can
	 * both observe both substreams NULL and both arrive here. If both read the same stream_svc and
	 * both called kthread_stop(), the second would stop a task that has already exited and been
	 * reaped, blocking forever — an unkillable closing process, and a module that cannot be unloaded.
	 *
	 * Taking stream_on and the servicer handle together under the lock makes exactly one caller
	 * proceed; the loser returns at the stream_on test. This is the teardown mirror of the arm claim
	 * in clarett_pcm_prepare().
	 *
	 * kthread_stop() MUST run outside pcm_lock: the servicer calls clarett_pcm_tick(), which takes
	 * pcm_lock, so stopping it while holding the lock trades this hang for a deadlock.
	 */
	mutex_lock(&c->pcm_lock);
	if (!c->stream_on) {
		mutex_unlock(&c->pcm_lock);
		return;
	}
	c->stream_on = false;
	WRITE_ONCE(c->stream_run, false);		/* stop the servicer ACKing 0x300 */
	svc = c->stream_svc;
	c->stream_svc = NULL;				/* take the handle: only this caller may stop it */
	mutex_unlock(&c->pcm_lock);

	if (svc)
		kthread_stop(svc);			/* stop acking 0x300 before the engine is torn down */
	cancel_delayed_work_sync(&c->stream_report);
	writel(0, c->bar0 + STREAM_BLK0 + STREAM_OFF_CTRL);	/* disable ring 0 */
	writel(0, c->bar0 + STREAM_BLK1 + STREAM_OFF_CTRL);	/* disable ring 1 */
	writel(0, c->bar0 + REG_STREAM_IRQ_ARM);
	writel(0, c->bar0 + REG_STREAM_IRQ_CFG);
	readl(c->bar0 + STREAM_BLK0 + STREAM_OFF_CTRL);		/* flush posted writes */
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
static void clarett_quiesce_dma(struct pci_dev *pci, void __iomem *bar0)
{
	pci_clear_master(pci);
	if (!bar0)
		return;
	writel(0, bar0 + REG_DMA_ADDR_LO);
	writel(0, bar0 + REG_DMA_ADDR_HI);
	readl(bar0 + REG_DMA_ADDR_LO);	/* flush posted writes before the buffer is freed */
}

/*
 * MSI handler. One Linux IRQ per MSI vector, dispatched by vector index (dev_id).
 * The device signals control-plane events on vec0. While a mailbox command is in flight a vec0
 * MSI is its completion, and the ISR reads the mailbox cause (0x100); otherwise it checks the
 * notification cause (0x400). vec1/vec2 are the stream period vectors; the servicer polls the
 * period blocks, so here they are only counted (for the stream_probe diagnostic). MSI is
 * edge-triggered, so not clearing a cause register cannot storm. Every vector returns
 * IRQ_HANDLED so the core doesn't disable the MSI as spurious.
 */
static irqreturn_t clarett_irq(int irq, void *dev_id)
{
	struct clarett_irqctx *ic = dev_id;
	struct clarett *c = ic->c;

	/*
	 * MIDI RX is register PIO (REG_MIDI_DATA); which MSI vector signals it is not known, so drain on
	 * every vector — that is vector-agnostic AND stops an undrained RX FIFO from livelocking whichever
	 * line it lands on. A no-op until the rawmidi device exists and whenever the FIFO is empty.
	 */
	if (READ_ONCE(c->rmidi))
		clarett_midi_irq(c);

	if (ic->idx == CLARETT_VEC_EVENT) {
		bool inflight = atomic_read(&c->cmd_inflight);

		/* Default mailbox cycle: a vec0 MSI during a command IS the completion signal. Read
		 * the mailbox cause here (the sweep's first, MSI-paced 0x100 read) and hand the rest
		 * of the sweep to the waiting clarett_fcp. An in-command MSI without DONE (e.g. a
		 * notification) is left to the waiter's timeout/poll fallback. */
		if (inflight) {
			u32 cause = readl(c->bar0 + REG_IRQ0_CAUSE);	/* read-to-clear */

			if (cause & IRQ_DONE_BIT) {
				c->mbox_cause = cause;
				complete(&c->mbox_done);
			}
			return IRQ_HANDLED;
		}

		{
		u32 cause = readl(c->bar0 + REG_NOTIFY_CAUSE);	/* 0x400, read-to-clear */
		u32 ev = cause & NOTIFY_MONITOR_MASK;

		/* vec0 also fires on mailbox-DONE, and 0x400 reads its idle level 0x3 (== NOTIFY_MON_PRIMARY)
		 * at completion time (see the REG_NOTIFY_CAUSE note in clarett.h). Skipping the notify path
		 * while our own command is in flight suppresses that self-reflection. ctl_ready gates the
		 * relay: the handlers are hooked BEFORE the controls exist (for MSI-paced completion), and a
		 * pre-controls event must not notify.
		 *
		 * stream_on gate: while the engine streams, vec0 ALSO fires on every audio period, and 0x400
		 * reads its idle 0x3 each time — so this path would schedule a relay ~per period (coalesced to
		 * one wake per notify_ms). The relay is a wildcard (the FCP notify word is not exposed), so
		 * fcp-server answers each wake by re-reading every notifiable control, a GET_DATA apiece. The
		 * gate saves that mailbox load; notify_while_streaming turns it off. While it is on,
		 * front-panel changes reach userspace mid-stream only through monitor_poll, which covers the
		 * monitor region and nothing else. */
		if (ev && READ_ONCE(c->ctl_ready) &&
		    (!READ_ONCE(c->stream_on) || READ_ONCE(notify_while_streaming))) {
			atomic_or(ev, &c->notify_bits);
			schedule_work(&c->notify_work);
		}
		}
	} else if (ic->idx == 1 || ic->idx == 2) {	/* data-plane period IRQs (probe) */
		readl(c->bar0 + (ic->idx == 1 ? STREAM_BLK0 : STREAM_BLK1));   /* read-to-clear/observe */
		atomic_inc(&c->period_irqs[ic->idx]);
	}
	return IRQ_HANDLED;
}

/*
 * Debounced flash-persist worker. clarett_write_u8 schedules this CLARETT_SAVE_DELAY_MS after a
 * control change (cancel+reschedule, so a burst coalesces into one save); it issues the single
 * DATA_CMD{PERSIST} that writes the device's live config to its own NVRAM. This is the device-owns-
 * the-state model (the upstream scarlett2 policy): settings survive a power cycle without the host
 * having to re-apply them. Debouncing keeps a slider drag from hammering the flash.
 */
static void clarett_save_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, save_work.work);
	int err = clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);

	if (err)
		dev_warn_ratelimited(&c->pci->dev, "config persist (DATA_CMD{%u}) failed: %d\n",
				     FCP_ACTIVATE_PERSIST, err);
}

static void clarett_notify_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, notify_work);
	u32 ev = atomic_xchg(&c->notify_bits, 0);

	if (!ev)
		return;

	/* Controls live in userspace (fcp-server), so a device notification is simply relayed: it
	 * re-reads whatever it owns. The driver keeps no control state to refresh, and re-reading the
	 * monitor region here would only add mailbox traffic competing with fcp-server's own reads. */
	clarett_hwdep_notify(c, ev);

	dev_dbg(&c->pci->dev, "async notification handled: 0x%x\n", ev);
}

/*
 * Monitor-region change poll: keeps the front-panel knob live while streaming.
 *
 * The 0x400 notification relay is suppressed for the duration of a stream (the stream_on gate in
 * clarett_irq), and with PipeWire holding a PCM open more or less permanently, that is most of the
 * time. So read the monitor region at the meter rate and relay only when the bytes actually CHANGE.
 * Cost is one GET_DATA per tick beside the GET_METER heartbeat; a steady state with nobody touching
 * the unit relays nothing at all. The region (24, len 92) covers the monitor mute/dim flags, the
 * master volume pair at 32/33, the HW-enable bits and the knob.
 *
 * The poll runs whether or not audio is streaming: the relay it feeds is only needed while streaming
 * (outside a stream the 0x400 relay is live and does that job), but clarett_hw_gain_follow() hangs
 * off the same change detection and has to track the knob at all times.
 */
static bool monitor_poll = true;
module_param(monitor_poll, bool, 0644);
MODULE_PARM_DESC(monitor_poll,
		 "Poll the monitor config region and act when it changes: relay a notification while "
		 "streaming, so the front-panel knob keeps tracking (the 0x400 relay is gated off for the "
		 "duration of a stream), and drive hw_gain_follow. Default on; with 0 the knob does not "
		 "update in userspace for as long as any PCM is open.");

/*
 * Keep the SW gain of every output under HARDWARE control equal to the front-panel knob.
 *
 * The device applies the knob in its own signal path and never writes it back: turning it moves byte
 * 112 only, while the per-output gains (out_gains[].offset) stay wherever software last left them.
 * Two visible consequences, both of which the USB siblings avoid because the in-kernel scarlett2 driver synthesises the link: the GUI's per-output
 * fader does not follow the knob, and toggling an output HW -> SW makes its volume JUMP to whatever
 * stale software value was stored.
 *
 * Fix it at the source rather than in the presentation layer — write the knob's value into the SW
 * gain of each HW-controlled output, so the device's own config says what is actually being heard.
 * Both symptoms then fall out for free and no userspace change is needed: fcp-server re-reads the
 * byte and the fader tracks, and a HW -> SW toggle is silent because the stored value already
 * matches. Writing an output's SW gain while it is under HW control is inaudible by definition (the
 * knob owns the level), so this only takes effect at the moment control returns to software.
 *
 * NOT persisted (clarett_write_u8_nosave): this is a mirror, not user intent, and committing the
 * flash on every movement of the knob would wear the NVRAM. Writes are also change-gated, so a
 * stationary knob costs nothing at all.
 *
 * `cfg` is the monitor region as fetched by clarett_monitor_poll — offset MONITOR_CFG_OFFSET, at least
 * MONITOR_CFG_LEN long, which spans the gains, the HW-enable bits and the knob alike.
 */
static bool hw_gain_follow = true;
module_param(hw_gain_follow, bool, 0644);
MODULE_PARM_DESC(hw_gain_follow,
		 "Track the front-panel volume knob into the SW gain of every output set to HW, so the "
		 "GUI fader follows it and a HW->SW toggle does not jump (default on; matches the USB "
		 "models' scarlett2 behaviour). 0 leaves the stored SW gains untouched.");

/* Byte `off` as fetched into the monitor-region buffer, or -1 if it falls outside that window. */
static int clarett_cfg_byte(const u8 *cfg, u32 off)
{
	if (off < MONITOR_CFG_OFFSET || off >= MONITOR_CFG_OFFSET + MONITOR_CFG_LEN)
		return -1;
	return cfg[off - MONITOR_CFG_OFFSET];
}

static void clarett_hw_gain_follow(struct clarett *c, const u8 *cfg)
{
	int mon = clarett_cfg_byte(cfg, MONITOR_VOLUME_OFFSET);
	int i;

	if (mon < 0)
		return;

	for (i = 0; i < c->model->n_out_gains; i++) {
		u32 gain_off = c->model->out_gains[i].offset;
		u32 hwen_off = HWEN_GAIN_OFFSET + (i / 2) * 4;
		u8 hwen_bit = 1u << (i % 2);
		int hwen = clarett_cfg_byte(cfg, hwen_off);
		int gain = clarett_cfg_byte(cfg, gain_off);
		int err;

		if (hwen < 0 || gain < 0)	/* a model whose offsets escape the polled window */
			continue;
		/* Only outputs whose enable-hardware-gain bit is set; the rest are software's. */
		if (!(hwen & hwen_bit))
			continue;
		if (gain == mon)
			continue;

		err = clarett_write_u8_nosave(c, gain_off, mon, OUT_GAIN_ACTIVATE);
		if (err) {
			dev_warn_ratelimited(&c->pci->dev,
					     "hw-gain follow: writing output %d gain @%u failed: %d\n",
					     i, gain_off, err);
			return;
		}
		dev_dbg(&c->pci->dev, "hw-gain follow: output %d gain @%u -> 0x%02x\n",
			i, gain_off, mon);
	}
}

/*
 * Meter-source follow (8PreX). fcp-server's "Meter Source" enum writes only the selector byte @184,
 * but the physical meter bridge is routed by the per-source channel-index tables @136/146/156. Nothing
 * derives those from 184 on its own — so without this the selector LED
 * moves while the meters keep displaying whichever bank the tables were last set to. On a change we
 * write the selected source's three per-band tables and commit with activate 8, which commits them
 * together with the selector byte fcp-server already staged. No-op on models with no meter-source
 * control or for an unrecognised source value. Called from the hwdep CMD path on a SET_DATA @184.
 */
void clarett_meter_source_follow(struct clarett *c, u8 source)
{
	const struct clarett_model *m = c->model;
	const struct clarett_meter_source *ms = NULL;
	int i, err;

	for (i = 0; i < m->n_meter_sources; i++)
		if (m->meter_sources[i].value == source) {
			ms = &m->meter_sources[i];
			break;
		}
	if (!ms)
		return;

	err = clarett_set_data(c, METER_TABLE_L_OFFSET, METER_TABLE_LEN, ms->tbl[0]);
	if (!err)
		err = clarett_set_data(c, METER_TABLE_M_OFFSET, METER_TABLE_LEN, ms->tbl[1]);
	if (!err)
		err = clarett_set_data(c, METER_TABLE_H_OFFSET, METER_TABLE_LEN, ms->tbl[2]);
	if (!err)
		err = clarett_data_cmd(c, METER_SOURCE_ACTIVATE);
	if (err)
		dev_warn_ratelimited(&c->pci->dev,
				     "meter-source follow (%s) failed: %d\n", ms->name, err);
	else
		dev_dbg(&c->pci->dev, "meter-source follow: %s (0x%02x)\n", ms->name, source);
}

static void clarett_monitor_poll(struct clarett *c)
{
	u8 buf[MONITOR_CFG_MAX_LEN];
	u32 len = c->model->monitor_cfg_len ? : MONITOR_CFG_LEN;
	u8 req[8];		/* GET_DATA {u32 offset, u32 len} */
	bool changed, first;
	int err;

	if (WARN_ON_ONCE(len > sizeof(buf)))
		len = sizeof(buf);

	clarett_put_le32(req, MONITOR_CFG_OFFSET);
	clarett_put_le32(req + 4, len);

	/* clarett_fcp_cmd (not clarett_get_data) so the payload is copied out under mbox_lock —
	 * reading c->resp_buf here would race the next command. */
	err = clarett_fcp_cmd(c, FCP_GET_DATA, req, sizeof(req), buf, len);
	if (err) {
		dev_dbg(&c->pci->dev, "monitor poll: GET_DATA failed: %d\n", err);
		return;
	}

	first = !c->mon_snap_valid;
	changed = !first && memcmp(c->mon_snap, buf, len);
	memcpy(c->mon_snap, buf, len);
	c->mon_snap_valid = true;

	if (!first && !changed)
		return;

	if (changed) {
		dev_dbg(&c->pci->dev, "monitor poll: region changed\n");
		/* Only while streaming: outside one the 0x400 relay is live and notifies for us. */
		if (READ_ONCE(c->stream_on))
			clarett_hwdep_notify(c, NOTIFY_MON_PRIMARY);
	}

	/* Also on the FIRST poll: the stored SW gains of HW-controlled outputs are whatever the last
	 * session left behind, so without an initial sync they stay stale until the knob is next
	 * touched — and a HW->SW toggle before that would jump, which is the case this exists to fix. */
	if (hw_gain_follow)
		clarett_hw_gain_follow(c, buf);
}

static void clarett_meter_work(struct work_struct *work)
{
	struct clarett *c = container_of(work, struct clarett, meter_work.work);
	int delay = meter_poll_ms > 0 ? meter_poll_ms : CLARETT_METER_POLL_MS;
	int n = c->hwdep_n_meter_slots;

	if (meter_poll_ms > 0) {
		if (n > 0 && c->hwdep_meter_levels) {
			/*
			 * The meter control exists (fcp-server set the map): make the heartbeat's poll ALSO
			 * refresh the shared cache and stamp hwdep_meter_polled, so the GUI-facing .get serves
			 * that cache instead of issuing its own GET_METER per read — one meter poll rate, not
			 * one device command per GUI read.
			 */
			u8 req[8];		/* {pad:u16=0, num_meters:u16, magic:u32=1} */

			clarett_put_le16(req, 0);
			clarett_put_le16(req + 2, n);
			clarett_put_le32(req + 4, 1);
			mutex_lock(&c->hwdep_lock);
			if (!clarett_fcp_cmd(c, FCP_GET_METER, req, sizeof(req),
					     (u8 *)c->hwdep_meter_levels, n * sizeof(__le32)))
				c->hwdep_meter_polled = jiffies ? jiffies : 1;
			mutex_unlock(&c->hwdep_lock);
		} else {
			/* No meter control yet — the plain heartbeat. */
			static const u8 meter_req[8] = { 0x00, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00 };

			clarett_fcp(c, FCP_GET_METER, meter_req, sizeof(meter_req));
		}

		/* Runs streaming or not — hw_gain_follow has to track the knob at all times. */
		if (monitor_poll && READ_ONCE(c->ctl_ready))
			clarett_monitor_poll(c);

		schedule_delayed_work(&c->meter_work, msecs_to_jiffies(delay));
	}
}

/* Allocate MSI vectors. Done early for the config-space side effect: pci_alloc_irq_vectors()
 * programs the device's MSI capability (address/data, MME) and sets the enable bit (plus
 * INTx-disable), so MSI is on BEFORE the first BAR access, as the device sees on a normal
 * attach. Handlers hook later (clarett_setup_irq); unhandled-but-enabled MSI in between is
 * harmless (edge-triggered, and the mailbox falls back to polling). */
static void clarett_enable_msi(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int nvec;

	/* Request up to CLARETT_NUM_VECTORS but accept as few as 1: some platforms cannot grant all 4,
	 * and failing outright would leave no interrupts at all. With fewer vectors the device funnels
	 * all causes to the lower vector(s); clarett_irq reads the cause regs regardless, so
	 * notifications still work. */
	nvec = pci_alloc_irq_vectors(pci, 1, CLARETT_NUM_VECTORS, PCI_IRQ_MSI);
	if (nvec < 0) {
		dev_warn(&pci->dev,
			 "MSI alloc failed (%d); async notifications disabled\n", nvec);
		return;
	}
	c->n_vec = nvec;
	/* Record the achieved count. A short count is a degraded configuration worth a warn; the
	 * expected 4/4 is debug. */
	if (nvec < CLARETT_NUM_VECTORS)
		dev_warn(&pci->dev, "MSI: got %d/%d vectors (causes funnel to the allocated ones)\n",
			 nvec, CLARETT_NUM_VECTORS);
	else
		dev_dbg(&pci->dev, "MSI: got %d/%d vectors\n", nvec, CLARETT_NUM_VECTORS);
}

/* Hook the notification handlers onto the already-allocated vectors. Best-effort: on
 * failure the driver still works (control plane, polled mailbox) but without async
 * notifications — and the vectors stay allocated so the device keeps seeing MSI enabled. */
static void clarett_setup_irq(struct clarett *c)
{
	struct pci_dev *pci = c->pci;
	int i, err;

	if (!c->n_vec)	/* MSI alloc failed */
		return;

	for (i = 0; i < c->n_vec; i++) {
		c->irq_ctx[i].c = c;
		c->irq_ctx[i].idx = i;
		err = request_irq(pci_irq_vector(pci, i), clarett_irq, 0,
				  KBUILD_MODNAME, &c->irq_ctx[i]);
		if (err) {
			dev_warn(&pci->dev,
				 "request_irq vec %d failed (%d); notifications disabled\n",
				 i, err);
			while (--i >= 0)
				free_irq(pci_irq_vector(pci, i), &c->irq_ctx[i]);
			return;
		}
	}
	c->irq_ready = true;
}

static void clarett_teardown_irq(struct clarett *c)
{
	int i;

	if (c->irq_ready)
		for (i = 0; i < c->n_vec; i++)
			free_irq(pci_irq_vector(c->pci, i), &c->irq_ctx[i]);
	if (c->n_vec)
		pci_free_irq_vectors(c->pci);
	c->irq_ready = false;
	c->n_vec = 0;
}

/*
 * card->private_free: runs inside snd_card_free AFTER userspace is disconnected and the
 * last file handle has closed — after the final possible mailbox transaction — but before
 * c (card private data) is freed, and with the MSI completion path still hooked, so those
 * last transactions run the normal MSI-paced cycle rather than a 100 ms degraded wait.
 * Must tolerate a partially initialized card (probe error
 * paths reach here through the same snd_card_free).
 */
static void clarett_card_free(struct snd_card *card)
{
	struct clarett *c = card->private_data;

	WRITE_ONCE(c->ctl_ready, false);	/* stop the ISR queueing new notify work */
	cancel_delayed_work_sync(&c->meter_work);
	/* Flush a pending debounced persist so a change made within the last CLARETT_SAVE_DELAY_MS
	 * still reaches NVRAM. Done here — after the meter worker is stopped (it shares the mailbox
	 * resp_buf) but while the MSI completion path is still hooked — so the DATA_CMD completes
	 * normally. cancel_delayed_work_sync returns true only if a save was actually queued. */
	if (cancel_delayed_work_sync(&c->save_work))
		clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);
	cancel_work_sync(&c->notify_work);
	clarett_midi_stop(c);			/* cancel the MIDI TX drain before the rawmidi is freed */
	clarett_engine_stop(c);			/* halt streaming DMA before devres frees the ring */
	if (c->bar0)
		writel(0, c->bar0 + REG_IRQ0_ENABLE);	/* mask causes before freeing handlers */
	clarett_teardown_irq(c);		/* syncs any in-flight ISR */
	/* An ISR already past its ctl_ready check when the flag flipped may have queued one
	 * more notify after the cancel above; re-cancel now that free_irq has synced them all
	 * (a straggler that already started runs its GET on the poll fallback — harmless). */
	cancel_work_sync(&c->notify_work);
	/* Last, because notify_work is what re-arms it: the debounced hwdep relay wake. c is freed
	 * the moment this returns to snd_card_free, and a live timer into freed memory would panic
	 * the host on a surprise removal (device powered off). */
	clarett_hwdep_free(c);
}

/*
 * /proc/asound/cardN/clarett — the stable per-model identity for userspace. The whole Thunderbolt
 * line shares PCI id 1cb5:0002, so a device-map consumer (fcp-server) cannot key a model-specific
 * map on the PCI id; it keys on `slug` here instead. `model` is the human name; `slug` is the
 * machine key (never mangled, unlike card->id). Kept minimal and greppable on purpose.
 */
static void clarett_proc_read(struct snd_info_entry *entry, struct snd_info_buffer *buf)
{
	struct clarett *c = entry->private_data;

	snd_iprintf(buf, "model: %s\n", c->model->name);
	snd_iprintf(buf, "slug: %s\n", c->model->slug);
	snd_iprintf(buf, "rate: %u\n", READ_ONCE(c->cur_rate));
}

static int clarett_probe(struct pci_dev *pci, const struct pci_device_id *ent)
{
	struct snd_card *card;
	struct clarett *c;
	bool pcm_ok = false;			/* PCM fully registered (not just snd_pcm_new'd) */
	void __iomem *bar0;
	int err, seeded;

	err = snd_card_new(&pci->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, sizeof(*c), &card);
	if (err < 0)
		return err;

	c = card->private_data;
	c->card = card;
	c->pci = pci;
	/* Shared PCI id across the line → the match can't pick the model, and there is no override.
	 * The id_table's entry is a placeholder only: clarett_detect_model() replaces it below, or probe
	 * fails. Nothing that depends on the model may be sized before that point. */
	c->model = (const struct clarett_model *)ent->driver_data;
	mutex_init(&c->mbox_lock);
	mutex_init(&c->hwdep_lock);
	mutex_init(&c->pcm_lock);
	init_waitqueue_head(&c->hwdep_notify_wait);
	init_completion(&c->mbox_done);
	INIT_WORK(&c->notify_work, clarett_notify_work);
	INIT_DELAYED_WORK(&c->save_work, clarett_save_work);
	INIT_DELAYED_WORK(&c->meter_work, clarett_meter_work);
	atomic_set(&c->notify_bits, 0);
	atomic_set(&c->cmd_inflight, 0);
	INIT_DELAYED_WORK(&c->stream_report, clarett_stream_report);
	atomic_set(&c->period_irqs[1], 0);
	atomic_set(&c->period_irqs[2], 0);
	/* Teardown lives in private_free so snd_card_free sequences it after the last
	 * userspace handle closes but before c is freed (works are all INIT'd above). */
	card->private_free = clarett_card_free;

	err = pcim_enable_device(pci);
	if (err)
		goto err_free;

	err = pcim_iomap_regions(pci, BIT(CLARETT_BAR), KBUILD_MODNAME);
	if (err)
		goto err_free;
	c->bar0 = pcim_iomap_table(pci)[CLARETT_BAR];

	pci_set_master(pci);

	if (dma_bits < 28 || dma_bits > 64)
		dma_bits = 32;
	err = dma_set_mask_and_coherent(&pci->dev, DMA_BIT_MASK(dma_bits));
	if (err)
		goto err_free;

	c->resp_size = PAGE_SIZE;
	c->resp_buf = dmam_alloc_coherent(&pci->dev, c->resp_size,
					  &c->resp_dma, GFP_KERNEL);
	if (!c->resp_buf) {
		err = -ENOMEM;
		goto err_free;
	}
	/* Log the response buffer's bus address (0x414 carries the high word). */
	dev_dbg(&pci->dev, "resp buffer dma addr %pad (0x414 high word 0x%x, dma_bits=%d)\n",
		&c->resp_dma, upper_32_bits(c->resp_dma), dma_bits);

	/* MSI is enabled in config space before the first BAR access, so the device never sees
	 * a session start from a host without an interrupt path. Handlers hook immediately too:
	 * mailbox completion is MSI-paced from the first command (the notify path stays gated on
	 * ctl_ready). */
	clarett_enable_msi(c);
	clarett_setup_irq(c);

	if (settle_ms)
		msleep(settle_ms);	/* cold attach: do not touch the device before it can answer */
	clarett_hw_init(c);

	/*
	 * Establish the session. The driver never arms: a device that has been set up once restores its
	 * session from flash, so reads, input metering and control writes all work with no host bring-up. Wait for that
	 * flash-persisted session to answer, detect the model from it, and leave the device's own routing
	 * alone.
	 *
	 * A cold Thunderbolt attach can race device readiness (command #0's response may not land), so poll
	 * clarett_detect_model quietly — a warn per attempt would be noise — until it answers or the
	 * wait_ready_ms budget expires.
	 */
	{
		bool collapsed = false;
		const struct clarett_model *det = NULL;
		unsigned long deadline = jiffies + msecs_to_jiffies(wait_ready_ms);
		int tries = 0;

		/*
		 * Readiness retry, with the emphasis on QUIET rather than on frequency: recovering a device
		 * caught mid-wake needs a long stretch left completely alone AND a fresh pre-mailbox init
		 * after it (see CLARETT_READY_RETRY_MS). So each retry waits out the full interval
		 * untouched, then re-inits and asks once. Re-running the init also clears a mailbox wedged
		 * by a failed first command, which is why a reload recovers where waiting does not.
		 *
		 * A warm device answers the first attempt in ~90 us and never re-inits.
		 */
		for (;;) {
			tries++;
			det = clarett_detect_model(c, &collapsed, true);
			if (det || !collapsed || time_after(jiffies, deadline))
				break;
			msleep(CLARETT_READY_RETRY_MS);
			clarett_hw_init(c);	/* replay the init; the device may be awake now */
		}
		if (tries > 1)
			dev_dbg(&pci->dev, "readiness: %d attempts, session %s\n",
				tries, det ? "answered" : "still refusing");
		/*
		 * One non-quiet pass on ANY failure, to log the detail before deciding: a refusal's
		 * status/size, or the raw geometry pair of an unmatched device (the poll above runs
		 * quiet, and it breaks out of the loop immediately on an unmatched-but-valid reply —
		 * which would otherwise leave nothing logged at all).
		 */
		if (!det)
			det = clarett_detect_model(c, &collapsed, false);

		if (collapsed) {
			/*
			 * Never became ready. Do NOT register a placeholder, which would pass a not-ready
			 * device off as a working card. Fail the probe loudly so it gets attention.
			 */
			dev_err(&pci->dev,
				"device did not become ready within %u ms over %d attempts (mailbox %s) — refusing "
				"to register. A unit still waking from power-up cannot answer, and each command "
				"renews that state; replug or reload to retry once it has settled.\n",
				wait_ready_ms, tries,
				c->mbox_wedged ? "wedged: no response, or one echoing another command's seq"
					       : "answering, but refusing the request");
			err = -ENODEV;
			goto err_free;
		}

		if (!det) {
			/*
			 * The device answered cleanly but with a geometry no clarett_model claims. There is
			 * no override to fall back on, and standing in a wrong model would size the DMA
			 * rings, fragment strides and routing tables for different hardware — so refuse.
			 * The warn just above carries the raw pair to add to the table.
			 */
			dev_err(&pci->dev,
				"unknown Clarett model (stream geometry matches none in the table) — refusing to register. "
				"Add a clarett_model entry for the playback/capture pair logged above.\n");
			err = -ENODEV;
			goto err_free;
		}
		c->model = det;
		/* Nothing arms the device, so its flash-persisted routing stands untouched. */
	}

	/* RX fragment slot stride: default = page-safe pow2; 0 = contiguous; >0 = audio + padding. */
	{
		u32 frag = clarett_frag_bytes(c->model->capture_channels);

		c->rx_slot = rx_frag_pad < 0 ? roundup_pow_of_two(frag)
			   : rx_frag_pad == 0 ? frag
					      : frag + rx_frag_pad;
	}
	/* TX fragment slot stride, mirror of rx_slot (a contiguous TX ring garbles playback on
	 * models whose fragment is not a power of two). Default page-safe pow2. */
	{
		u32 frag = clarett_tx_frag_bytes(c->model->playback_channels);

		c->tx_slot = tx_frag_pad < 0 ? roundup_pow_of_two(frag)
			   : tx_frag_pad == 0 ? frag
					      : frag + tx_frag_pad;
	}

	/* Start the GET_METER heartbeat now, so it is running for the rest of probe too, as in a
	 * normal session, when the monitor-enable writes below go out. */
	if (meter_poll_ms > 0)
		schedule_delayed_work(&c->meter_work,
				      msecs_to_jiffies(meter_poll_ms > 0 ? meter_poll_ms : CLARETT_METER_POLL_MS));

	/* Seed the shadow from the device so the enable-byte RMW below is safe. Best-effort: if it
	 * fails we skip the enable writes. */
	seeded = clarett_seed_shadow(c);
	if (seeded)
		dev_warn(&pci->dev,
			 "config shadow seed failed (%d); leaving hardware mute/dim enables untouched\n",
			 seeded);
	else if (seed_dump) {
		/* One-shot full [0,256) seeded-shadow dump (gated on seed_dump so normal loads stay
		 * quiet); diff two dumps to locate a setting. */
		int off;

		for (off = 0; off < CLARETT_CONFIG_SIZE; off += 16)
			dev_info(&pci->dev, "seeded shadow [%3d]=%*ph\n",
				 off, 16, c->shadow + off);
	}

	/* Diagnostic: log the FCP status codes for valid and malformed commands. Requires
	 * meter_poll_ms=0 so the heartbeat doesn't share resp_buf/seq (see param desc). */
	if (error_probe) {
		if (meter_poll_ms > 0)
			dev_warn(&pci->dev,
				 "error_probe needs meter_poll_ms=0 (meter worker races the shared response buffer); results unreliable\n");
		clarett_error_probe(c);
	}

	/* Make Mute/Dim actually affect Monitor Out 1-2 by setting the per-output enable bits: the
	 * master flag alone does nothing until an output opts in. This is a hardware-side write, so it
	 * is needed whoever owns the controls — fcp-server drives the same Mute/Dim bytes. Needs the
	 * seeded shadow for a correct read-modify-write, so only attempt it when seeding succeeded. */
	if (!seeded && monitor_enables) {
		err = clarett_enable_monitor_hw_controls(c);
		if (err)
			dev_warn(&pci->dev,
				 "could not enable monitor hardware mute/dim (%d)\n", err);
	}

	/*
	 * Seed the published sample rate from the device, ONCE. The device keeps its rate across driver
	 * reloads, so without this /proc/asound/cardN/clarett would claim the default until something
	 * streams. Userspace (fcp-server, picking the per-rate meter layout) then reads the rate for free
	 * instead of polling the mailbox for it.
	 */
	{
		u8 r[4];

		if (!clarett_fcp_cmd(c, FCP_SYNC_RATE, NULL, 0, r, sizeof(r)))
			WRITE_ONCE(c->cur_rate, clarett_get_le32(r));
		if (!READ_ONCE(c->cur_rate))
			WRITE_ONCE(c->cur_rate, CLARETT_DEFAULT_RATE);
	}

	/* Controls live in userspace: expose the FCP hwdep for fcp-server. */
	err = clarett_hwdep_init(c);
	if (err)
		dev_warn(&pci->dev, "FCP hwdep create failed (%d)\n", err);
	err = 0;

	/* The one exception to "controls live in userspace": the clock source is not a config byte, so
	 * fcp-server cannot map it, and SET_CLOCK needs the sample rate that only this driver knows. */
	err = clarett_add_clock_control(c);
	if (err)
		dev_warn(&pci->dev, "Clock Source control create failed (%d)\n", err);
	err = 0;

	WRITE_ONCE(c->ctl_ready, true);	/* controls exist; the ISR notify path may fire */

	/* PCM. Owns the engine, so it excludes stream_probe. Every model uses the per-direction
	 * descriptor path (geometry derived from channel counts), so no per-model gate is needed. */
	if (enable_pcm) {
		err = clarett_create_pcm(c);
		if (err)
			dev_warn(&pci->dev, "PCM create failed (%d); continuing without PCM\n", err);
		else
			pcm_ok = true;	/* c->pcm is set before the buffer alloc that can still fail */
		err = 0;
	}

	/* DIN MIDI (rawmidi over the 0x58c register UART). Line-wide, so not model-gated; no-op if
	 * enable_midi is off. Non-fatal — a failure just leaves the card without MIDI. */
	err = clarett_create_midi(c);
	if (err)
		dev_warn(&pci->dev, "MIDI create failed (%d); continuing without MIDI\n", err);
	err = 0;

	/* Opt-in data-plane diagnostic: start the engine and report what it does. Best-effort; needs
	 * the IRQ handlers (above) hooked first so vec1/vec2 period IRQs are counted. */
	if (stream_probe && c->model->stream_frag && !enable_pcm && c->irq_ready) {
		err = clarett_engine_start(c);
		if (err)
			dev_warn(&pci->dev, "engine-start probe failed (%d)\n", err);
		err = 0;
	}

	strscpy(card->driver, "Clarett", sizeof(card->driver));
	/* Mirror snd-usb-audio's naming: brand-free product name in the shortname
	 * (== api.alsa.card.name), manufacturer only in the longname. The USB
	 * Claretts show "Clarett 8Pre USB" on the /proc/asound/cards bracket line
	 * because snd-usb-audio takes the shortname from the device's iProduct
	 * string; we have no such descriptor, so we synthesise the same shape. */
	snprintf(card->shortname, sizeof(card->shortname), "%s", c->model->name);
	snprintf(card->longname, sizeof(card->longname),
		 "Focusrite %s at %s, fw app 0x%08x", c->model->name, pci_name(pci),
		 c->fw_app);

	/* Expose the stable per-model slug at /proc/asound/cardN/clarett (see clarett_proc_read).
	 * Best-effort: the entry's lifetime is the card's; a failure only costs userspace its model
	 * auto-detect, not function, so it is not fatal to probe. */
	if (snd_card_ro_proc_new(card, "clarett", c, clarett_proc_read))
		dev_warn(&pci->dev, "could not create /proc/asound/.../clarett model entry\n");

	err = snd_card_register(card);
	if (err)
		goto err_free;

	pci_set_drvdata(pci, card);
	/*
	 * The probe summary — and, on a healthy load, the ONLY thing this driver prints. The detail
	 * (register dumps, the DMA response address, the MSI count, per-subsystem lines, the
	 * servicer telemetry) sits at dev_dbg. Bring any of it back at runtime with dynamic debug, e.g. `echo 'module snd_clarett +p' >/sys/kernel/debug/dynamic_debug/control`
	 * for all of it, or match one statement by format string to skip the ~24 Hz mailbox trace.
	 */
	{
		char feat[64];
		int n = 0;

		if (c->hwdep_ready)
			n += scnprintf(feat + n, sizeof(feat) - n, "%sFCP hwdep", n ? ", " : "");
		if (pcm_ok)
			n += scnprintf(feat + n, sizeof(feat) - n, "%sPCM %u/%uch", n ? ", " : "",
				       c->model->playback_channels, c->model->capture_channels);
		if (READ_ONCE(c->rmidi))
			n += scnprintf(feat + n, sizeof(feat) - n, "%sMIDI", n ? ", " : "");
		if (!n)
			scnprintf(feat, sizeof(feat), "no interfaces registered");

		/* No "(auto-detected)" tag: detection is the only way c->model can be set, so saying so
		 * would be noise. A model named here is one the device claimed. */
		dev_info(&pci->dev,
			 "%s: serial %08x%08x fw app 0x%08x fpga 0x%08x; %s\n",
			 c->model->name, c->serial_hi, c->serial_lo,
			 c->fw_app, c->fw_fpga, feat);
	}
	return 0;

err_free:
	bar0 = c->bar0;			/* may still be NULL; c is freed by snd_card_free */
	snd_card_free(card);		/* clarett_card_free() runs the full teardown */
	clarett_quiesce_dma(pci, bar0);
	return err;
}

static void clarett_remove(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct clarett *c = card->private_data;
	void __iomem *bar0 = c->bar0;

	/*
	 * Stop the stream servicer FIRST, before any of the card teardown.
	 *
	 * snd_card_free() frees the PCM devices — and with them each substream's runtime and
	 * runtime->dma_area — before it calls card->private_free (clarett_card_free). A servicer still
	 * ticking across that window would dereference a freed capture buffer in clarett_pcm_tick(). Unloading the module normally hides this because
	 * userspace has already closed the PCM, but a SURPRISE REMOVAL — the unit powered off mid-
	 * stream — leaves the engine armed and the servicer running straight into the free.
	 *
	 * c->pcm_lock does not help: it serialises the tick against our own hw_free, not against ALSA
	 * tearing the substream down underneath us.
	 */
	WRITE_ONCE(c->ctl_ready, false);	/* no new notify work from the ISR */
	cancel_delayed_work_sync(&c->meter_work);
	clarett_engine_stop(c);

	/* Blocks: disconnect → wait for the last userspace handle to close (the final
	 * mailbox transactions run the normal MSI-paced cycle) → clarett_card_free()
	 * teardown → frees c. Only then is the device quiesced. */
	/* Disconnect first and kick the relay waiters: snd_card_free() waits for the last handle to
	 * close, and fcp-server parked in read()/poll() on hwdep_notify_wait would never be woken by
	 * the disconnect alone — the wait re-checks card->shutdown, which snd_card_disconnect sets. */
	snd_card_disconnect(card);
	wake_up_interruptible(&c->hwdep_notify_wait);
	snd_card_free(card);
	/* Devres releases in REVERSE order after remove returns: the response buffer is
	 * freed (and IOMMU-unmapped) BEFORE pcim's disable clears bus master — so without
	 * an explicit quiesce the device holds a live registration to freed memory. */
	clarett_quiesce_dma(pci, bar0);
	/* BAR mapping, DMA buffer and device enable are devres-managed */
}

/* remove() never runs on reboot/kexec; without this the device enters the next kernel
 * still bus-mastering with the old kernel's response buffer registered. */
static void clarett_shutdown(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct clarett *c;

	if (!card)
		return;
	c = card->private_data;
	cancel_delayed_work_sync(&c->meter_work);
	clarett_hwdep_free(c);		/* no-op unless the hwdep path armed the relay */
	/* Persist a just-made change before the reboot tears the device down (mailbox still up). */
	if (cancel_delayed_work_sync(&c->save_work))
		clarett_data_cmd(c, FCP_ACTIVATE_PERSIST);
	clarett_engine_stop(c);
	writel(0, c->bar0 + REG_IRQ0_ENABLE);
	clarett_quiesce_dma(pci, c->bar0);
}

/* --- per-model descriptors ---------------------------------------------- */

/* "Line NN (descr)" format matching the USB models; the 8PreX has no USB sibling, so only the
 * monitor pair carries a descr (outputs 3-10 are generic line outs per its XML). */
static const struct clarett_out_gain clarett_8prex_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03", 36 }, { "Line 04", 37 }, { "Line 05", 40 }, { "Line 06", 41 },
	{ "Line 07", 44 }, { "Line 08", 45 }, { "Line 09", 48 }, { "Line 10", 49 },
};

static const char * const clarett_mode_mli[] = { "Mic", "Line", "Inst" };
static const char * const clarett_mode_ml[]  = { "Mic", "Line" };

/* Hardware-meter sources for the 8PreX (activate 8). Selecting one writes its three per-band
 * channel-index tables (@136/146/156) then SET_DATA{184}. */
static const struct clarett_meter_source clarett_8prex_meter_sources[] = {
	{ "Analogue", 1, {
		{  0,  1,  2,  3,  4,  5,  6,  7, 26, 27 },
		{  0,  1,  2,  3,  4,  5,  6,  7, 18, 19 },
		{  0,  1,  2,  3,  4,  5,  6,  7, 14, 15 } } },
	{ "S/PDIF", 2, {
		{  8,  9, 255, 255, 255, 255, 255, 255, 26, 27 },
		{  8,  9, 255, 255, 255, 255, 255, 255, 18, 19 },
		{  8,  9, 255, 255, 255, 255, 255, 255, 14, 15 } } },
	{ "ADAT 1", 4, {
		{ 10, 11, 12, 13, 14, 15, 16, 17, 26, 27 },
		{ 10, 11, 12, 13, 255, 255, 255, 255, 18, 19 },
		{ 10, 11, 255, 255, 255, 255, 255, 255, 14, 15 } } },
	{ "ADAT 2", 8, {
		{ 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 },
		{ 14, 15, 16, 17, 255, 255, 255, 255, 18, 19 },
		{ 12, 13, 255, 255, 255, 255, 255, 255, 14, 15 } } },
};

/* Analogue 1-2 add Inst; 3-8 are Mic/Line. Device byte == text index (identity). */
static const struct clarett_preamp clarett_8prex_preamps[] = {
	{ clarett_mode_mli, NULL, 3 }, { clarett_mode_mli, NULL, 3 },
	{ clarett_mode_ml,  NULL, 2 }, { clarett_mode_ml,  NULL, 2 },
	{ clarett_mode_ml,  NULL, 2 }, { clarett_mode_ml,  NULL, 2 },
	{ clarett_mode_ml,  NULL, 2 }, { clarett_mode_ml,  NULL, 2 },
};

/*
 * Per-channel stream-routing CONFIG_PUSH ids. The id space is model-independent, with per-category
 * reserved blocks —
 *   Analogue N -> 0x0d + (N-1)   (block reserves 8: 0x0d..0x14)
 *   S/PDIF   N -> 0x15 + (N-1)   (0x15..0x16)
 *   ADAT     N -> 0x17 + (N-1)   (block reserves 16: 0x17..0x26 — why loopback is 0x27 even on the
 *                                 8-ADAT 2Pre/4Pre, which use only 0x17..0x1e)
 *   Loopback N -> 0x27 + (N-1)
 *   Playback N -> 0x2b + (N-1)   (TX)
 * The 8PreX just fills more of each block. Order: TX = Playback 1..28; RX = the record-outputs order
 * (Analogue 1-8, S/PDIF 1-2, Loopback 1-2, then ADAT 1-16). Without these the stream-config handshake
 * pushes nothing, the device streams at a narrower default width, and a 28ch playback ring is consumed
 * as ~12ch — the stereo pair smears across outputs.
 */
static const u8 clarett_8prex_stream_tx[] = {
	0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
};
static const u8 clarett_8prex_stream_rx[] = {
	0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,	/* Analogue 1-8 */
	0x15, 0x16,					/* S/PDIF 1-2 */
	0x27, 0x28,					/* Loopback 1-2 (mid-block, matching record-outputs) */
	0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,	/* ADAT 1-8 */
	0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,	/* ADAT 9-16 */
};

/*
 * Selectable clock sources for the "Clock Source" control, Internal first. Values are the SET_CLOCK
 * enums (clarett.h): Internal 24, S/PDIF 3, ADAT 0, shared by the whole line. The 8PreX's second ADAT
 * port and wordclock input are unverified (see CLARETT_CLOCK_ADAT2), but they are real connectors, so
 * they are offered.
 */
static const struct clarett_clock_src clarett_clock_srcs[] = {
	{ "Internal", CLARETT_CLOCK_INTERNAL },
	{ "S/PDIF",   CLARETT_CLOCK_SPDIF },
	{ "ADAT",     CLARETT_CLOCK_ADAT },
};

static const struct clarett_clock_src clarett_8prex_clock_srcs[] = {
	{ "Internal",  CLARETT_CLOCK_INTERNAL },
	{ "S/PDIF",    CLARETT_CLOCK_SPDIF },
	{ "ADAT 1",    CLARETT_CLOCK_ADAT },
	{ "ADAT 2",    CLARETT_CLOCK_ADAT2 },
	{ "Wordclock", CLARETT_CLOCK_WORDCLOCK },
};

static const struct clarett_model clarett_8prex = {
	.name = "Clarett 8PreX",
	.slug = "clarett-8prex",
	.out_gains = clarett_8prex_gains,
	.n_out_gains = ARRAY_SIZE(clarett_8prex_gains),
	.n_analogue = 8,
	.analogue = clarett_8prex_preamps,
	.in_prefix = "Line In",			/* match the USB models' input naming */
	.mode_label = "Mode",			/* but keep "Mode": Mic/Line/Inst is richer than "Level" */
	.has_spdif_source = true,
	.meter_sources = clarett_8prex_meter_sources,
	.n_meter_sources = ARRAY_SIZE(clarett_8prex_meter_sources),
	.capture_channels = STREAM_CHANS,
	.playback_channels = STREAM_CHANS,
	.rx_live_mid = 20,			/* two ADAT ports: ch20-27 gone at double speed */
	.clock_srcs = clarett_8prex_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_8prex_clock_srcs),
	.rx_live_high = 16,			/* + ch16-19 gone at quad */
	.max_rate = 192000,			/* double + quad speed verified for capture (correct pitch at 96k
						 * and 192k, full width); playback verified at single speed. */
	.stream_frag = STREAM_SIZE_VAL,
	.stream_tx_ids = clarett_8prex_stream_tx,
	.n_stream_tx_ids = ARRAY_SIZE(clarett_8prex_stream_tx),
	.stream_rx_ids = clarett_8prex_stream_rx,
	.n_stream_rx_ids = ARRAY_SIZE(clarett_8prex_stream_rx),
};

/*
 * Clarett 2Pre (Thunderbolt). Shares the 8PreX's config offsets and commands; the first 4 of the
 * 8PreX output gains; 2 combo-jack preamps with the Line/Inst encoding (Line=1, Inst=2 — Mic is
 * auto-detected by the jack, not a software mode; see clarett_mode_li). Streams 4 playback / 14 record,
 * which is also how it is detected (clarett_detect_model). Asymmetric TX 4ch / RX 14ch: periods
 * 0x40 / 0xe0, descriptor fragments 0x100 / 0x380 (clarett_frag_bytes).
 */
/* Names mirror scarlett2's Clarett 2Pre USB line_out_descrs (Monitor L/R, Headphones L/R);
 * the "Line NN" number is 1-based output index, matching scarlett2's "Line %02d (%s)". */
static const struct clarett_out_gain clarett_2pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03 (Headphones L)", 36 }, { "Line 04 (Headphones R)", 37 },
};

/*
 * Combo-jack input mode (shared by 2Pre / 4Pre / 8Pre). These models use a single combined XLR/TRS jack
 * per input, so the connector auto-selects Mic (XLR inserted) vs the 1/4" path — Mic is NOT a software
 * option; the mode control only chooses Line vs Inst for the 1/4" path. So the enum is {Line=1, Inst=2}
 * with no Mic(0). (The 8PreX, by contrast, has SEPARATE XLR and 1/4" ports, so software must pick
 * Mic/Line/Inst explicitly — see clarett_mode_mli.)
 */
static const char * const clarett_mode_li[] = { "Line", "Inst" };
static const u8 clarett_mode_li_vals[] = { 1, 2 };

static const struct clarett_preamp clarett_2pre_preamps[] = {
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
};

/* Per-channel stream-routing CONFIG_PUSH ids: 4 TX (playback) + 14 RX (record) channels, pushed at
 * PCM prepare. */
static const u8 clarett_2pre_stream_tx[] = { 0x2b, 0x2c, 0x2d, 0x2e };
static const u8 clarett_2pre_stream_rx[] = {
	0x0d, 0x0e, 0x15, 0x16, 0x27, 0x28, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
};

static const struct clarett_model clarett_2pre = {
	.name = "Clarett 2Pre",
	.slug = "clarett-2pre",
	.out_gains = clarett_2pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_2pre_gains),
	.n_analogue = 2,
	.analogue = clarett_2pre_preamps,
	.in_prefix = "Line In",			/* match scarlett2 Clarett 2Pre USB */
	.mode_label = "Level",
	.capture_channels = 14,			/* record-outputs pin count (12 record + 2 loopback) */
	.playback_channels = 4,			/* playback pin count */
	.rx_live_mid = 10,			/* ADAT 5-8 -> ch10-13 gone at double speed */
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	.rx_live_high = 8,			/* + ADAT 3-4 -> ch8-9 gone at quad */
	.max_rate = 192000,			/* double + quad speed verified (correct pitch at 96k and 192k,
						 * full width). */
	.stream_frag = 0,			/* stream_probe unused on the 2Pre; PCM uses
						 * clarett_frag_bytes() per direction */
	.stream_tx_ids = clarett_2pre_stream_tx,
	.n_stream_tx_ids = ARRAY_SIZE(clarett_2pre_stream_tx),
	.stream_rx_ids = clarett_2pre_stream_rx,
	.n_stream_rx_ids = ARRAY_SIZE(clarett_2pre_stream_rx),
};

/*
 * Clarett 4Pre (Thunderbolt). Detected by its (8,20) geometry (clarett_detect_model).
 *
 *   channel counts 8 playback / 20 record (18 record + 2 loopback).
 *   stream-routing CONFIG_PUSH ids: 8 TX after GET_7.2, 20 RX after GET_7.3, in wire order.
 *   inputs: only Analogue 1-2 have a mode (Line=1/Inst=2; Mic is auto-detected by the combo
 *           XLR/TRS jack, not a software option — see clarett_mode_li), at mode@166/167 cmd 6;
 *           Analogue 1-4 each have Air at air@174..177 cmd 7; Analogue 5-8 have no preamp controls.
 *           So n_analogue=4 (the Air-capable inputs); 3-4 are air-only (n_modes=0).
 *   output gains (cmd 1, 8-bit): Monitor 1-2 @ 32/33, Line 3-4 @ 36/37, Headphone 2 L/R @ 40/41.
 *           S/PDIF outputs carry no gain. Six gains total (no 44/45 pair on this model).
 */
/* Names mirror scarlett2's Clarett 4Pre USB line_out_descrs (Monitor L/R, Headphones 1 L/R,
 * Headphones 2 L/R). */
static const struct clarett_out_gain clarett_4pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03 (Headphones 1 L)", 36 }, { "Line 04 (Headphones 1 R)", 37 },
	{ "Line 05 (Headphones 2 L)", 40 }, { "Line 06 (Headphones 2 R)", 41 },
};

/* Analogue 1-2 Line/Inst (combo jack auto-detects Mic); 3-4 air-only (n_modes=0). */
static const struct clarett_preamp clarett_4pre_preamps[] = {
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
	{ NULL, NULL, 0 },
	{ NULL, NULL, 0 },
};

/* Per-channel stream-routing CONFIG_PUSH ids: 8 TX (playback) + 20 RX (record), pushed at PCM
 * prepare. */
static const u8 clarett_4pre_stream_tx[] = { 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32 };
static const u8 clarett_4pre_stream_rx[] = {
	0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
	0x27, 0x28, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
};

static const struct clarett_model clarett_4pre = {
	.name = "Clarett 4Pre",
	.slug = "clarett-4pre",
	.out_gains = clarett_4pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_4pre_gains),
	.n_analogue = 4,
	.analogue = clarett_4pre_preamps,
	.in_prefix = "Line In",			/* match scarlett2 Clarett 4Pre USB */
	.mode_label = "Level",
	.has_spdif_source = true,
	.capture_channels = 20,			/* record-outputs pin count */
	.playback_channels = 8,			/* playback pin count */
	.rx_live_mid = 16,			/* ADAT 5-8 -> ch16-19 gone at double speed */
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	.rx_live_high = 14,			/* + ADAT 3-4 -> ch14-15 gone at quad */
	.max_rate = 192000,			/* double + quad speed verified for capture (correct pitch at 96k
						 * and 192k, full width); playback verified at single speed. */
	.stream_frag = 0,			/* PCM uses clarett_frag_bytes() per direction (asymmetric) */
	.stream_tx_ids = clarett_4pre_stream_tx,
	.n_stream_tx_ids = ARRAY_SIZE(clarett_4pre_stream_tx),
	.stream_rx_ids = clarett_4pre_stream_rx,
	.n_stream_rx_ids = ARRAY_SIZE(clarett_4pre_stream_rx),
};

/*
 * Clarett 8Pre (Thunderbolt) — a DISTINCT model from the 8PreX (do not confuse).
 *
 * Differences from the 8PreX — these are physical: the 8Pre uses combo XLR/TRS jacks (the jack
 * auto-detects Mic when an XLR is inserted), whereas the 8PreX has SEPARATE XLR + 1/4" ports per input
 * (so its mode must be software-selected). Hence:
 *   - inputs: all 8 carry Air @ 174..181 (cmd 7), but only Analogue 1-2 have a mode (Line=1/Inst=2 for
 *     the 1/4" path; Mic is jack-auto, not a software option) @ 166/167 (cmd 6); Analogue 3-8 are
 *     air-only. (The 8PreX, with discrete ports, exposes software Mic/Line[/Inst] on all 8.)
 *   - streams: 20 playback / 20 record (the 8PreX is 28/28 — the 8Pre has a single ADAT bank).
 *   - outputs: same 10-gain offsets as the 8PreX (Monitor @ 32/33, then 36..49), but named to
 *     mirror scarlett2's Clarett 8Pre USB line_out_descrs (Monitor L/R, Line 03-06 unlabelled,
 *     Headphones 1/2), so it gets its own gain table rather than reusing the 8PreX's.
 */
static const struct clarett_preamp clarett_8pre_preamps[] = {	/* 1-2 Line/Inst, 3-8 air-only */
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
	{ clarett_mode_li, clarett_mode_li_vals, 2 },
	{ NULL, NULL, 0 }, { NULL, NULL, 0 },
	{ NULL, NULL, 0 }, { NULL, NULL, 0 },
	{ NULL, NULL, 0 }, { NULL, NULL, 0 },
};

/* scarlett2 Clarett 8Pre USB line_out_descrs: Monitor L/R, four unlabelled line outs, Headphones 1/2. */
static const struct clarett_out_gain clarett_8pre_gains[] = {
	{ "Line 01 (Monitor L)", 32 }, { "Line 02 (Monitor R)", 33 },
	{ "Line 03", 36 }, { "Line 04", 37 }, { "Line 05", 40 }, { "Line 06", 41 },
	{ "Line 07 (Headphones 1 L)", 44 }, { "Line 08 (Headphones 1 R)", 45 },
	{ "Line 09 (Headphones 2 L)", 48 }, { "Line 10 (Headphones 2 R)", 49 },
};

/*
 * Per-channel stream-routing CONFIG_PUSH ids, from the same model-independent source-id enumeration as
 * the 8PreX (see clarett_8prex_stream_tx). The 8Pre's physical input layout is identical
 * to the 4Pre (Analogue 1-8, S/PDIF 1-2, ADAT 1-8, Loopback mid-block), so its RX ids come out equal to
 * the 4Pre's; TX is Playback 1-20 -> 0x2b..0x3e.
 */
static const u8 clarett_8pre_stream_tx[] = {
	0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34,
	0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e,
};
static const u8 clarett_8pre_stream_rx[] = {
	0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,	/* Analogue 1-8 */
	0x15, 0x16,					/* S/PDIF 1-2 */
	0x27, 0x28,					/* Loopback 1-2 (mid-block, matching record-outputs) */
	0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,	/* ADAT 1-8 */
};

static const struct clarett_model clarett_8pre = {
	.name = "Clarett 8Pre",
	.slug = "clarett-8pre",
	.out_gains = clarett_8pre_gains,
	.n_out_gains = ARRAY_SIZE(clarett_8pre_gains),
	.n_analogue = 8,
	.analogue = clarett_8pre_preamps,
	.in_prefix = "Line In",			/* match scarlett2 Clarett 8Pre USB */
	.mode_label = "Level",
	.has_spdif_source = true,
	.capture_channels = 20,			/* 18 record + 2 loopback */
	.playback_channels = 20,		/* Playback 1-20 */
	.rx_live_mid = 16,			/* ADAT 5-8 -> ch16-19 gone at double speed */
	.clock_srcs = clarett_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(clarett_clock_srcs),
	.rx_live_high = 14,			/* + ADAT 3-4 -> ch14-15 gone at quad */
	.max_rate = 192000,			/* double + quad speed verified for capture (correct pitch at 96k
						 * and 192k, full width). */
	.stream_frag = 0,
	.stream_tx_ids = clarett_8pre_stream_tx,
	.n_stream_tx_ids = ARRAY_SIZE(clarett_8pre_stream_tx),
	.stream_rx_ids = clarett_8pre_stream_rx,
	.n_stream_rx_ids = ARRAY_SIZE(clarett_8pre_stream_rx),
};

/*
 * Focusrite Red 8Line — the first non-Clarett model, and one whose entry is deliberately CONTROL-PLANE
 * EMPTY. It shares PCI id 1cb5:0002 with every Clarett, so the id_table already matches it; this entry
 * gives clarett_detect_model() its geometry: playback=64, capture=60 (58 record + 2 loopback).
 *
 * WHAT IS DELIBERATELY LEFT NULL/ZERO, AND WHY. The Red's control plane is NOT a wider Clarett:
 *  - out_gains: the Red's output gains are 16-bit values on a 2-byte stride (gain @24, 26, 28, ...,
 *    bits="16", min-gain -112.0 dB), where every Clarett uses a 7-bit 1 dB/step attenuation byte.
 *    clarett_hw_gain_follow() writes a BYTE at out_gains[].offset, so populating this table from the
 *    Red's offsets would write half a gain field. NULL correctly disables that mirror outright.
 *  - the preamp/naming fields (n_analogue, analogue, in_prefix, mode_label, has_spdif_source): dead
 *    since the in-kernel control layer was removed, and the Red's preamps carry phantom, phase,
 *    stereo-link and separate mic/line/inst gains that this struct cannot describe anyway.
 *  - meter_sources: the Red's front-panel meter bridge is not mapped.
 *  - stream_tx_ids/rx_ids: the per-channel CONFIG_PUSH ids are unknown for this model. Zero skips
 *    the burst.
 * The control plane therefore reaches userspace only through the FCP hwdep, where fcp-server's
 * red-8line map pair describes it.
 *
 * max_rate is 0 (44.1/48 kHz only) ON PURPOSE: the higher rates are unverified on this model. Raise
 * it only after a pitch check on hardware, per the field comment on max_rate.
 */
static const struct clarett_clock_src red_8line_clock_srcs[] = {
	{ "Internal",  CLARETT_CLOCK_INTERNAL },
	{ "Wordclock", CLARETT_CLOCK_WORDCLOCK },
	{ "ADAT 1",    CLARETT_CLOCK_ADAT },
	{ "ADAT 2",    CLARETT_CLOCK_ADAT2 },
	{ "S/PDIF",    CLARETT_CLOCK_SPDIF },
	{ "Dante",     CLARETT_CLOCK_DANTE },
	{ "Loop Sync", CLARETT_CLOCK_LOOPSYNC },
};

static const struct clarett_model red_8line = {
	.name = "Red 8Line",
	.slug = "red-8line",
	.capture_channels = 60,			/* 58 record + 2 loopback */
	.playback_channels = 64,		/* Playback 1-64 */
	/*
	 * S/MUX: a channel gone at one speed stays gone at every higher speed, as on the Clarett
	 * models. The Red differs in KIND, though: rather than simply losing channels it
	 * RE-PINS survivors (a slot carrying ADAT 5 at single speed carries ADAT 9 at double and Dante 1 at
	 * quad), and only the tail actually goes away. Both dead sets are still contiguous tails, which is
	 * what these two counts require. UNTESTED — and unreachable while max_rate is 0.
	 */
	.rx_live_mid = 52,			/* Dante 25-32 gone at double speed */
	.rx_live_high = 32,			/* + Dante 5-24 gone at quad speed */
	.max_rate = 0,				/* single speed only until higher rates are verified */
	.clock_srcs = red_8line_clock_srcs,
	.n_clock_srcs = ARRAY_SIZE(red_8line_clock_srcs),
	.stream_frag = 0,
	/*
	 * The three front-panel knob groups (monitor, headphones 1, headphones 2) keep their gains at
	 * 112/116/120 and their mute/dim at 124/126/128, so the watched region runs to 129 — the
	 * Clarett default stops at 115 and would miss every one of them but the monitor gain.
	 */
	.monitor_cfg_len = 130 - MONITOR_CFG_OFFSET,
};

static const struct pci_device_id clarett_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_FOCUSRITE, PCI_DEVICE_CLARETT),
	  .driver_data = (kernel_ulong_t)&clarett_2pre },
	{ }
};
MODULE_DEVICE_TABLE(pci, clarett_ids);

static struct pci_driver clarett_driver = {
	.name = KBUILD_MODNAME,
	/*
	 * Probe can wait tens of seconds for a cold device to answer (see wait_ready_ms). Asynchronous
	 * so that wait runs on its own worker instead of stalling the PCI hotplug path behind it.
	 */
	.driver = { .probe_type = PROBE_PREFER_ASYNCHRONOUS },
	.id_table = clarett_ids,
	.probe = clarett_probe,
	.remove = clarett_remove,
	.shutdown = clarett_shutdown,
};
module_pci_driver(clarett_driver);

/*
 * Defined on the compiler command line by the Makefile, which reads it out of dkms.conf so
 * the version in `modinfo` is the version of the package that shipped the module. The
 * fallback is deliberately not a plausible number: it marks a build that came in through
 * some other path (a bare kbuild invocation), where claiming a real version would send a
 * bug report chasing the wrong tree.
 */
#ifndef CLARETT_VERSION
#define CLARETT_VERSION "0.0.0-unknown"
#endif

MODULE_DESCRIPTION("Focusrite Clarett/Red (Thunderbolt) ALSA driver");
MODULE_AUTHOR("Miles Ramage <miles.ramage@yahoo.com>");
MODULE_VERSION(CLARETT_VERSION);
MODULE_LICENSE("GPL");
