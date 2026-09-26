/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Focusrite Clarett/Red (Thunderbolt) ALSA driver — shared definitions.
 *
 * Register map, FCP framing and control offsets/commands come from clean-room
 * reverse engineering of the device's host interface.
 */
#ifndef CLARETT_H
#define CLARETT_H

#include <linux/types.h>
#include <linux/bitmap.h>		/* DECLARE_BITMAP, set_bit/test_bit — shadow_known */
#include <linux/mutex.h>
#include <linux/spinlock.h>	/* spinlock_t — MIDI RX drain serialisation */
#include <linux/pci.h>
#include <linux/atomic.h>
#include <linux/wait.h>		/* wait_queue_head_t — hwdep notification relay */
#include <linux/workqueue.h>
#include <linux/lcm.h>		/* lcm() — descriptor fragment alignment */
#include <linux/string.h>	/* memcpy/memset */
#include <linux/log2.h>		/* roundup_pow_of_two() — page-safe fragment slots */
#include <linux/math64.h>	/* div_u64() — period-relative tick-late threshold */
#include <linux/minmax.h>	/* max_t() — same */
#include <sound/core.h>

struct snd_kcontrol;
struct snd_pcm;
struct snd_pcm_substream;
struct snd_rawmidi;
struct snd_rawmidi_substream;

/* --- BAR0 register map --------------------------------------------------- */
#define CLARETT_BAR              0
#define REG_CAPS                 0x000
#define REG_SERIAL_LO            0x010
#define REG_SERIAL_HI            0x014
#define REG_IRQ0_CAUSE           0x100   /* read-to-clear; bit DONE = mailbox complete */
#define REG_IRQ0_ENABLE          0x104   /* initialised to 0xf000003f                 */
#define REG_NOTIFY_CAUSE         0x400   /* read-to-clear; carries the notify mask     */
#define REG_DOORBELL             0x408   /* write 1 = submit, 2 = ack/clear prior      */
#define REG_DMA_ADDR_LO          0x410   /* GET-response DMA buffer bus address (low 32)  */
#define REG_DMA_ADDR_HI          0x414   /* DMA buffer bus address (high 32)              */
#define REG_INFO                 0x8000  /* read-only fw-info header (fw versions, ...) */
#define REG_MBOX                 0x8020  /* FCP request mailbox                         */

/*
 * DIN MIDI UART (rawmidi) — register PIO, NOT the FCP mailbox / audio DMA. REG_MIDI_DATA is
 * bidirectional: a TX write packs up to 3 MIDI
 * bytes with a byte-count in the top byte; an RX read returns one byte with bit24 (MIDI_RX_VALID) set, or
 * 0 when the RX FIFO is empty. RX is interrupt-driven — the shared IRQ summary REG_MIDI_STATUS low byte
 * carries a MIDI-RX-pending code (0x0a); the driver drains REG_MIDI_DATA, then writes
 * MIDI_IRQ_ACK_VAL to REG_MIDI_ACK to clear it. See clarett_midi.c.
 */
#define REG_MIDI_STATUS          0x500   /* IRQ summary (low byte 0x0a = MIDI RX pending); also read by servicer */
#define REG_MIDI_ACK             0x504   /* write MIDI_IRQ_ACK_VAL to clear the MIDI RX interrupt */
#define REG_MIDI_DATA            0x58c   /* TX: (count<<24)|(b2<<16)|(b1<<8)|b0 ; RX: (valid<<24)|byte */
#define MIDI_RX_VALID            0x01000000u  /* bit24 of a REG_MIDI_DATA read: a byte is present */
#define MIDI_RX_BYTE_MASK        0x000000ffu
#define MIDI_IRQ_ACK_VAL         0x8          /* -> REG_MIDI_ACK to clear the MIDI RX interrupt */
#define MIDI_TX_COUNT_SHIFT      24           /* TX packed word: byte count (1..3) in bits 24-31 */

/*
 * Data-plane streaming registers. Two structurally identical ring blocks; block 0 (0x200) → MSI vec1,
 * block 1 (0x300) → vec2.
 */
#define REG_STREAM_IRQ_CFG       0x108   /* stream-start writes 0x10        */
#define REG_STREAM_IRQ_CFG2      0x10c   /* stream-start writes 0x1e70700   */
#define REG_STREAM_IRQ_ARM       0x110   /* stream-start writes 0x7 then 0x0 */
#define STREAM_BLK0              0x200   /* ring block 0 (vec1); +0x00 = cause (read-to-clear) */
#define STREAM_BLK1              0x300   /* ring block 1 (vec2) */
#define   STREAM_OFF_CHANS       0x04    /* channel count = 0x1c (28)       */
#define   STREAM_OFF_SIZE        0x08    /* IRQ period in bytes (0x1c0)     */
#define   STREAM_OFF_CTRL        0x0c    /* enable bit    = 1               */
#define   STREAM_OFF_BASE_LO     0x10    /* ring base bus address low 32    */
#define   STREAM_OFF_BASE_HI     0x14    /* ring base bus address high 32   */
#define   STREAM_OFF_PTR         0x18    /* DMA position (read-only)        */
#define STREAM_CHANS             0x1c    /* 8PreX: 28 PCM channels/direction (populates clarett_8prex) */
#define STREAM_SIZE_VAL          0x1c0   /* 8PreX: bytes the engine DMAs per descriptor (0x208 reg)    */
/*
 * Descriptor ring. 0x210/0x214 (and 0x310/0x314) point at a table of bare
 * 8-byte little-endian guest-physical addresses, zero-terminated; each entry is one DMA fragment of
 * clarett_model.stream_frag bytes holding capture_channels-wide S32_LE (24-bit MSB-justified)
 * interleaved frames (frame stride = channels * 4). The probe lays CLARETT_STREAM_NDESC valid entries
 * (+ a zero terminator) per ring over one contiguous coherent buffer.
 *
 * STREAM_CHANS / STREAM_SIZE_VAL are the 8PreX values that populate clarett_8prex; all runtime stream
 * geometry is derived per-model from c->model via clarett_buf_bytes() &c. (defined below struct clarett).
 */
#define CLARETT_STREAM_NDESC     256            /* descriptors per ring (model-independent) */

/* --- PCM (data plane) --------------------------------------------------- */
#define CLARETT_PCM_RATE         48000          /* default rate, all models (see clocking enum) */

/* FCP mailbox header layout, relative to REG_MBOX */
#define MBOX_CMD                 0x00    /* bit31 = execute flag | opcode               */
#define MBOX_SIZESEQ             0x04    /* size (low 16) | seq (high 16)               */
#define MBOX_ERROR               0x08
#define MBOX_PAD                 0x0c
#define MBOX_DATA                0x10

#define CMD_EXEC_FLAG            0x80000000u
#define IRQ_DONE_BIT             0x20000000u   /* mailbox-complete cause bit @ REG_IRQ0_CAUSE */
#define DOORBELL_SUBMIT          1
#define DOORBELL_ACK             2

/*
 * MSI: the device delivers ALL control-plane interrupts on vector 0 — both mailbox-done and
 * front-panel notifications. The cause registers (not the MSI vector index) distinguish them:
 * 0x100 = mailbox done, 0x400 = notification. vec1/vec2 carry the stream period events.
 */
#define CLARETT_NUM_VECTORS      4
#define CLARETT_VEC_EVENT        0       /* the device signals control events on vec0 */

/*
 * REG_NOTIFY_CAUSE (0x400) is NOT an async event queue — it is a 2-bit command-phase/status
 * register holding {0,1,2,3}: idle/ready = 0x3, dipping to 0x0 while a mailbox command is accepted
 * and passing 0x1->0x2 mid-command, back to 0x3 at completion. There is no per-bit follow-up read.
 *
 * Consequence for the ISR: vec0 fires on mailbox-DONE too, and at completion 0x400 reads its idle
 * 0x3 (== NOTIFY_MON_PRIMARY), so a completion MSI can be misread as a monitor event; the
 * cmd_inflight guard suppresses that self-reflection.
 */
#define NOTIFY_MON_PRIMARY       0x00000003u  /* bit0|bit1 — raised on every monitor (mute/dim) event */
#define NOTIFY_MON_AUX           0x00200000u  /* bit21 — co-occurs intermittently */
#define NOTIFY_MONITOR_MASK      (NOTIFY_MON_PRIMARY | NOTIFY_MON_AUX)

/* Monitoring config region re-read on a notification. */
#define MONITOR_CFG_OFFSET       24
#define MONITOR_CFG_LEN          92      /* default: 24..115, the Clarett's gains, HW bits and knob */
#define MONITOR_CFG_MAX_LEN      245     /* largest per-model region (clarett_model.monitor_cfg_len) */
#define MONITOR_VOLUME_OFFSET    112     /* the front-panel knob's level; read-only reflection */
#define MONITOR_ACTIVATE         2       /* DATA_CMD code shared by the monitor controls:
                                          * mute@24 / dim@28 are 1-bit fields that toggle 0/1
                                          * and commit with activate=2. */

/*
 * DATA_CMD{5} = persist the app config to flash, issued standalone with no preceding SET_DATA. A plain
 * control commit (DATA_CMD{activate}) applies the change live but RAM-only; this persists it across a
 * power cycle. Issued on a debounce (CLARETT_SAVE_DELAY_MS) rather than per change, to spare the flash.
 */
#define FCP_ACTIVATE_PERSIST     5

/*
 * Per-output "follow the monitor section" hardware-enable bits (command 3).
 * The global Mute (offset 24) / Dim (offset 28) only affect an output whose enable bit is set —
 * the master flag alone does nothing. The driver force-enables the two monitor outputs at probe
 * so global Mute/Dim actually act on Monitor Out 1-2 (matching the USB unit's behaviour).
 * These bytes pack one bit per output, so they must be read-modify-written from a shadow seeded
 * from the device (clarett_seed_shadow), never blindly overwritten.
 *   byte 72: enable-hardware-mute (Monitor Out 1 = bit0, Monitor Out 2 = bit1)
 *   byte 73: enable-hardware-dim  (Monitor Out 1 = bit2, Monitor Out 2 = bit3)
 */
#define HWEN_ACTIVATE            3
#define HWEN_GAIN_OFFSET         52    /* enable-hardware-gain (SW/HW) base; byte 52+(out/2)*4, bit out%2 */
#define OUT_GAIN_ACTIVATE        1     /* DATA_CMD code committing an output gain byte (out_gains[].offset) */
#define HWEN_MUTE_OFFSET         72
#define HWEN_DIM_OFFSET          73
#define HWEN_MONITOR_MUTE_MASK   0x03    /* Monitor Out 1-2 mute enables */
#define HWEN_MONITOR_DIM_MASK    0x0c    /* Monitor Out 1-2 dim enables  */

/* S/PDIF source select: 2-bit fields, DATA_CMD activate 4. @132 picks the S/PDIF *input* to capture
 * (scarlett2's "S/PDIF Source Capture Enum"); @124 picks the S/PDIF *output* connector.
 * Enum None=0 / Optical=1 / RCA=2. */
#define SPDIF_SOURCE_OFFSET      132
#define SPDIF_SOURCE_ACTIVATE    4
#define SPDIF_OUTPUT_OFFSET      124
#define SPDIF_OUTPUT_ACTIVATE    4

/* Hardware-meter source select (@184, DATA_CMD activate 8) + the per-band channel index tables written
 * alongside it (@136 / @146 / @156 for single / double / quad speed, 10 bytes each). Enum is a bitmask
 * value: Analogue=1 / S/PDIF=2 / ADAT1=4 / ADAT2=8. */
#define METER_SOURCE_OFFSET      184
#define METER_SOURCE_ACTIVATE    8
#define METER_TABLE_L_OFFSET     136
#define METER_TABLE_M_OFFSET     146
#define METER_TABLE_H_OFFSET     156
#define METER_TABLE_LEN          10

/* FCP "big" opcodes (low bits of cmd); == scarlett2 USB values */
#define FCP_GET_DATA             0x800000
#define FCP_SET_DATA             0x800001
#define FCP_DATA_CMD             0x800002

/*
 * GET_METER (0x001001): read the level meters. The driver also issues it periodically as a host
 * heartbeat, as Focusrite Control does while connected, with the 8-byte payload {0x00300000, 0x00000001}.
 * See clarett_meter_work().
 */
#define FCP_GET_METER            0x001001
#define CLARETT_METER_POLL_MS    40

/*
 * MUX_READ: read back the routing table. Request {u8 offset, u8 pad, u8 count, u8 band}; reply is an
 * array of u32 entries (src << 12 | dst), capped at 28 per reply whatever `count` asks for.
 */
#define FCP_MUX_READ             0x003001
#define CLARETT_MUX_READ_MAX     28
/*
 * Minimum spacing between GET_METER device polls from the meter control's .get. The mixer GUI reads the
 * control at its UI refresh rate (30-60 Hz); a device command per read floods the mailbox and disrupts
 * streaming (skips + command timeouts, since control and stream contend on this Thunderbolt device). The
 * .get serves the cached levels between polls, so the GUI still sees a live meter with far fewer device
 * commands. Kept just above the heartbeat interval so the heartbeat's own refresh (below) keeps the cache
 * fresh and the .get never has to poll on its own — one meter poll rate, not two.
 */
#define CLARETT_METER_CACHE_MS   50

/* Debounced flash persist: after a control change, schedule a single DATA_CMD{PERSIST} this many ms
 * later (cancel+reschedule on each change) so a burst coalesces into one NVRAM write. Matches the
 * upstream scarlett2 driver's 2 s save debounce. See clarett_save_work() / FCP_ACTIVATE_PERSIST. */
#define CLARETT_SAVE_DELAY_MS    2000

/* SET_CLOCK: payload {u32 sample_rate, u32 clock_source}. */
#define FCP_SET_CLOCK            0x006003
/*
 * Clock-source enum values. Internal, ADAT and S/PDIF are the same on every model. S/PDIF is 3 on the
 * 2Pre too: 4 there locks to any external source rather than to S/PDIF specifically. Verify a clock
 * source by Sync Status, not by the audio: S/PDIF and ADAT keep arriving on their capture channels
 * whatever the clock source says, even while unlocked.
 */
#define CLARETT_CLOCK_ADAT       0	/* "ADAT 1" on the 8PreX */
/*
 * 8PreX only, and unverified: on that model Sync Status reports a lock if EITHER ADAT receiver has one,
 * so it cannot confirm which port each value selects.
 */
#define CLARETT_CLOCK_ADAT2      1	/* 8PreX only, unverified */
#define CLARETT_CLOCK_WORDCLOCK  2	/* 8PreX only, untested */
#define CLARETT_CLOCK_SPDIF      3	/* all models */
/* Red range only, both unverified. Encodings are per-model: the Red's 4 is unrelated to the 2Pre's. */
#define CLARETT_CLOCK_DANTE      4	/* Red only, unverified */
#define CLARETT_CLOCK_LOOPSYNC   5	/* Red only, unverified */
#define CLARETT_CLOCK_INTERNAL   24
#define CLARETT_DEFAULT_RATE     48000

/*
 * CLOCK/SYNC category (0x006xxx, fcp-server's FCP_OPCODE_CATEGORY_SYNC) — QUERIES, not commands:
 *
 *   0x006004   sync lock status (fcp-server's SYNC_READ)
 *   0x006005   current rate
 *   0x006000   caps/bitmask (undecoded)
 *   0x006001/2/3  rates
 *
 * The stream handshake issues SYNC_READ and SYNC_RATE; they enable nothing, but their answers are worth
 * logging (a device reporting unlocked would explain a dead engine).
 *
 * FCP_SYNC_RATE is a LIVE rate readback that PERSISTS while nothing is streaming and across a driver
 * reload, so probe seeds cur_rate from it and /proc/asound/cardN/clarett is truthful before the first
 * stream.
 *
 * FCP_SYNC_READ is NOT a clean 0/1 lock flag: it returns 1 or 3 depending on model and stream state, so
 * it looks like a bitfield whose upper bit is undecoded. fcp-server collapses it with !!, which keeps
 * the exposed "Sync Status" sane.
 */
#define FCP_SYNC_READ            0x006004   /* lock status bitfield */
#define FCP_SYNC_RATE            0x006005   /* u32 rate, live */

/*
 * Session and identity opcodes, not fully decoded: CONFIG_PUSH registers config items by id, and
 * GET_6x/GET_7x/READ_SEG are version/identity queries. The device restores its own session from flash,
 * so the host issues only what the stream path needs: clarett_stream_handshake() issues CONFIG_PUSH and
 * the GET_7.x queries at every arm, and clarett_detect_model() uses GET_7.1's channel-count answer as the
 * model identity.
 */
#define FCP_READ_SEG             0x800005
#define FCP_INIT_2               0x000002
#define FCP_CONFIG_PUSH          0x005000
#define FCP_GET_60               0x006000
#define FCP_GET_61               0x006001
#define FCP_GET_62               0x006002
#define FCP_GET_70               0x007000
#define FCP_GET_71               0x007001
#define FCP_GET_72               0x007002
#define FCP_GET_73               0x007003

/*
 * Further opcodes, named for documentation; the driver issues none of them itself. SET_MIX and SET_MUX
 * are what a routing or mixer edit from fcp-server issues.
 *   0x000001 subsystem enable {u16 id}; 0x001000/0x002000/0x003000/0x004000 subsystem-count
 *   queries; 0x002002 SET_MIX {u16 mix, u16 coeff[30]}; 0x003002 SET_MUX; 0x004001/0x004005
 *   subsystem-4 setup; 0x005000 CONFIG_PUSH {u16 id}.
 */
#define FCP_INIT_1               0x000001
#define FCP_SET_MIX              0x002002
#define FCP_SET_MUX              0x003002

/*
 * 0x000001 is also the CAPABILITY READ: {u16 category} -> one byte, non-zero = that opcode category
 * is live on this session. fcp-server calls it first and refuses the device unless INIT (0x000) and
 * DATA (0x800) both answer non-zero, so it is the authoritative "is the session really up?" test.
 * The driver itself does not run it: probe waits for the identity query to answer instead, which is
 * the same evidence one command earlier.
 */
#define FCP_CAP_READ             FCP_INIT_1
#define FCP_CAT_INIT             0x000
#define FCP_CAT_DATA             0x800

/*
 * Per-model descriptor. One const instance per supported model, detected at probe and pinned as
 * clarett.model. Every value that differs between models lives here; the mailbox/engine *code* stays
 * model-agnostic. Encodings are per-model — never assume a value carries across models.
 *
 * Every model shares PCI id 1cb5:0002, so the id_table cannot distinguish them; the model is detected
 * from the device's reported stream geometry (clarett_detect_model).
 */
struct clarett_out_gain {
	const char *name;	/* ALSA control name prefix, e.g. "Monitor 1"  */
	u8 offset;		/* config-space byte offset (SET_DATA target)  */
};

/*
 * Per-preamp input mode enum. mode_values is the per-model device encoding:
 * NEVER assume text index == device byte (8PreX Mic/Line/Inst = 0/1/2, but the
 * 2Pre's Line/Inst = 1/2 with no Mic). NULL = identity (index == device byte).
 */
struct clarett_preamp {
	const char * const *mode_texts;
	const u8 *mode_values;
	int n_modes;
};

/*
 * One selectable clock source: the enum value SET_CLOCK carries, and the label the "Clock Source" ALSA
 * control shows. Per model, Internal first (the default, and what a user without digital inputs wants).
 */
struct clarett_clock_src {
	const char *name;
	u8 value;
};

struct clarett_model {
	/*
	 * Human-readable model name ("Clarett 8PreX"), also the card shortname. The rules in
	 * wireplumber/51-clarett-naming.conf match on it verbatim, so a new or renamed model needs
	 * its rule there too.
	 */
	const char *name;
	/*
	 * Stable machine-readable model slug ("clarett-8prex"), exposed at /proc/asound/cardN/clarett.
	 * The whole Thunderbolt line shares PCI id 1cb5:0002, so the PCI id cannot select a per-model
	 * control map; userspace (fcp-server) keys its model-specific maps on this slug instead. Unlike
	 * card->id it is never mangled for uniqueness, so it is a reliable contract. Keep it stable.
	 */
	const char *slug;

	/* control plane */
	const struct clarett_out_gain *out_gains;
	int n_out_gains;
	int n_analogue;				/* preamp count (air + mode controls) */
	const struct clarett_preamp *analogue;	/* [n_analogue] */
	/* Input-control naming. All models use in_prefix "Line In" (matching the scarlett2 names for
	 * the USB siblings). mode_label is "Level" for the USB models (their Line/Inst switch) but
	 * "Mode" for the 8PreX, whose Mic/Line/Inst mode is richer than scarlett2's Line/Inst "Level".
	 * Output-gain names carry the full "Line NN (descr)" string per model in out_gains[].name. */
	const char *in_prefix;			/* "Line In" (all models) */
	const char *mode_label;			/* "Level" (USB models) or "Mode" (8PreX) */
	/* "S/PDIF Source Capture Enum" (None/Optical/RCA @ SPDIF_SOURCE_OFFSET). Present where the
	 * device has a selectable S/PDIF input — 4Pre/8Pre/8PreX. The 2Pre has optical only (one
	 * option), so it gets no control, matching scarlett2 (which omits it for the 2Pre). */
	bool has_spdif_source;
	/* Hardware-meter source selector (8PreX only; others have one or no source). */
	const struct clarett_meter_source *meter_sources;
	int n_meter_sources;

	/* data plane / PCM geometry */
	u8 capture_channels;			/* block-1 RX stream width */
	u8 playback_channels;			/* block-0 TX stream width */
	u32 max_rate;				/* highest verified sample rate; 0 = single speed (48k) only.
						 * The stream WIDTH is rate-independent (the frame stride never
						 * shrinks), so raising this just advertises the higher SET_CLOCK rates.
						 * Raise it per model only after a pitch check on hardware (the max_rate
						 * module param overrides it for testing). */
	/*
	 * ADAT S/MUX: the frame stays capture_channels wide at every rate, but the device stops WRITING the
	 * ADAT channels that S/MUX removes (8 -> 4 -> 2 per port at single/double/quad speed). Those slots
	 * are not silence: the engine keeps writing a sparse residue into them (one non-zero sample every
	 * 32 frames, roughly -25 dBFS). So the dead tail is blanked per period, on the frames handed to ALSA;
	 * clarett_set_rx_live() latches the split at prepare and clarett_rx_drain() does the blanking. These
	 * are the counts of leading capture channels the device still writes at double and quad speed; the
	 * dead remainder is a contiguous tail on every model. 0 = all channels live (no ADAT, or unknown).
	 */
	u8 rx_live_mid;				/* capture channels written at 88.2/96 kHz */
	u8 rx_live_high;			/* capture channels written at 176.4/192 kHz */
	const struct clarett_clock_src *clock_srcs;	/* selectable clock sources, Internal first */
	u8 n_clock_srcs;
	u32 stream_frag;			/* engine-start diagnostic only (uniform per-descriptor DMA bytes);
						 * the PCM path derives per-direction fragments from channel counts */
	/*
	 * Per-channel stream-routing CONFIG_PUSH ids, issued at PCM prepare (the device resets stream
	 * routing when idle): one CONFIG_PUSH{u16 id} per stream channel, tx[] after GET_7.2 and rx[] after
	 * GET_7.3. NULL/0 = skip the burst (ids unknown for that model).
	 */
	const u8 *stream_tx_ids;
	const u8 *stream_rx_ids;
	u8 n_stream_tx_ids;
	u8 n_stream_rx_ids;
	/*
	 * Bytes from MONITOR_CFG_OFFSET that clarett_monitor_poll watches for front-panel changes. Must
	 * cover every config byte a front-panel control can move. 0 = MONITOR_CFG_LEN; at most
	 * MONITOR_CFG_MAX_LEN.
	 */
	u8 monitor_cfg_len;
};

/*
 * GET-response DMA layout. The device DMAs the response into resp_buf as a 16-byte FCP header
 * followed by the requested bytes:
 *   resp[0..3]  = echoed cmd (CMD_EXEC_FLAG | opcode) — guard on this
 *   resp[4..5]  = size: # of payload bytes the device actually returned
 *   resp[6..7]  = echoed request seq
 *   resp[8..11] = FCP error word: 0 = OK; 0x3 = the device refusing the session
 *   resp[16+i]  = config[offset + i]  for a GET_DATA{offset, len}
 * The header is zeroed before each command, so an echo of 0 means nothing landed. The echo alone
 * is not sufficient: a refused command can land a header with size=0 and no payload, so a reader
 * must ALSO require size > 0 before consuming resp[16+].
 */
#define FCP_RESP_ECHO_OFF        0
#define FCP_RESP_SIZE_OFF        4
#define FCP_RESP_SEQ_OFF         6      /* echoed request seq in the DMAed response header */
#define FCP_RESP_STATUS_OFF      8      /* FCP error word; see layout comment above */
#define FCP_RESP_DATA_OFF        16
#define FCP_RESP_ERR_OK          0x00

#define CLARETT_MBOX_TIMEOUT_MS  100
/*
 * Interval between readiness attempts. A device caught mid-wake does not latch the pre-mailbox init,
 * and nothing done afterwards over the mailbox recovers it; what does is a long stretch left completely
 * alone followed by a fresh init. So each retry waits this long untouched, replays the init and asks
 * once. Neither half works alone: re-asking without re-initialising fails at any spacing, and
 * re-initialising every few seconds fails too.
 */
#define CLARETT_READY_RETRY_MS		30000u
#define CLARETT_MAX_PAYLOAD      64      /* clarett_set_data single-write cap (small configs) */
#define CLARETT_MBOX_DATA_MAX    1024    /* mailbox data region past MBOX_DATA; SET_MUX = 412 */
#define CLARETT_CONFIG_SIZE      256     /* shadow of the device config/app space       */

/* A hardware-meter source option: its device value and the three per-band channel-index tables the
 * host writes (@136/146/156) when selecting it, alongside SET_DATA{184}=value + DATA_CMD{8}. The
 * selector control is fcp-server's; clarett_meter_source_follow() writes the tables. */
struct clarett_meter_source {
	const char *name;
	u8 value;			/* Analogue=1 / S/PDIF=2 / ADAT1=4 / ADAT2=8 */
	u8 tbl[3][10];			/* meters-l, meters-m, meters-h (per sample-rate band) */
};

#define CLARETT_N_METERS         48    /* GET_METER returns 48 u32 levels (num_meters=0x30)  */
#define CLARETT_METER_MAX        4095  /* meter level range 0..4095 (matches scarlett2)       */
/* Most channels the Level Meter control can expose: an INTEGER control's value array. */
#define CLARETT_METER_MAX_CHANNELS \
	((int)ARRAY_SIZE(((struct snd_ctl_elem_value *)NULL)->value.integer.value))

struct clarett;

/* request_irq dev_id: identifies the card and which MSI vector fired */
struct clarett_irqctx {
	struct clarett *c;
	unsigned int idx;
};

/* --- per-card state ----------------------------------------------------- */
struct clarett {
	struct pci_dev *pci;
	struct snd_card *card;
	void __iomem *bar0;
	const struct clarett_model *model;	/* selected at probe; see struct clarett_model */

	struct mutex mbox_lock;		/* serialises FCP transactions */
	u16 seq;

	void *resp_buf;			/* coherent GET-response DMA buffer */
	dma_addr_t resp_dma;
	size_t resp_size;

	u32 serial_lo, serial_hi, fw_app, fw_fpga;

	/* MSI / async notifications (vec0). In the default mailbox cycle the ISR IS the
	 * completion path: while cmd_inflight it reads the 0x100 mailbox cause (the sweep's
	 * first read, MSI-paced) and completes mbox_done; clarett_fcp then finishes the sweep.
	 * Under legacy_mbox_cycle=1 the ISR never touches 0x100. */
	bool irq_ready;
	bool ctl_ready;				/* controls registered; notify path may snd_ctl_notify */
	int n_vec;				/* MSI vectors actually allocated (<= CLARETT_NUM_VECTORS) */
	struct clarett_irqctx irq_ctx[CLARETT_NUM_VECTORS];
	struct work_struct notify_work;
	struct delayed_work save_work;		/* debounced DATA_CMD{PERSIST}; see CLARETT_SAVE_DELAY_MS */
	atomic_t notify_bits;
	/*
	 * Last-seen monitor config region, for the change-detecting poll that keeps the front-panel
	 * knob live while streaming (clarett_monitor_poll; the 0x400 relay is gated off by stream_on).
	 * Touched only from the meter worker, so no lock of its own.
	 */
	u8 mon_snap[MONITOR_CFG_MAX_LEN];
	bool mon_snap_valid;
	struct completion mbox_done;		/* completed by the vec0 ISR on mailbox DONE */
	u32 mbox_cause;				/* 0x100 value the ISR consumed with DONE set */
	/*
	 * Is the mailbox wedged? Set when the last command either produced no response DMA at all, or
	 * produced one echoing a sequence number that is not the one we sent. Both are the same fault:
	 * a command whose response never landed has its trailing ack withheld — as it must be, since
	 * acking an unlanded response makes the device refuse the session — leaving the device
	 * holding that command unretired and answering it in place of every later one, which is exactly
	 * what a stale echoed seq means. clarett_fcp() cannot report this through its return value
	 * without turning a response-less-but-successful SET into a failure, so the readiness poll reads
	 * it here when deciding how to report a device that never became ready.
	 */
	bool mbox_wedged;
	/*
	 * Set while a mailbox command is in flight (clarett_fcp submit->complete). vec0 fires on
	 * mailbox-DONE as well as front-panel notifications, and 0x400 reads its idle level (bit0|bit1
	 * = 0x3) at completion, so a completion MSI can be misread as a monitor event. This guard makes
	 * the ISR skip 0x400 while our own command is in flight, suppressing that self-reflection.
	 * See clarett_irq() and the REG_NOTIFY_CAUSE note.
	 */
	atomic_t cmd_inflight;

	/* Periodic GET_METER heartbeat. See FCP_GET_METER / clarett_meter_work(). */
	struct delayed_work meter_work;

	/*
	 * FCP hwdep level meter. fcp-server creates the "Level Meter"
	 * control via FCP_IOCTL_SET_METER_MAP: hwdep_meter_map[i] indexes the device's raw meter array
	 * (or -1 = no source) for output channel i; hwdep_meter_levels is the GET_METER scratch buffer
	 * (hwdep_n_meter_slots u32s). hwdep_meter_labels_tlv carries the channel-name TLV set by
	 * FCP_IOCTL_SET_METER_LABELS. All devm-allocated (freed at detach). Mirrors sound/usb/fcp.c. */
	struct mutex hwdep_lock;		/* serialises the hwdep meter ioctls vs the control callbacks */
	/*
	 * hwdep notification relay. A device notification (0x400 cause,
	 * detected in clarett_irq -> clarett_notify_work) sets hwdep_notify_event and wakes any
	 * fcp-server blocked in read()/poll(). Lock-free: atomic_or to accumulate, atomic_xchg to drain.
	 */
	wait_queue_head_t hwdep_notify_wait;
	atomic_t hwdep_notify_event;
	struct delayed_work hwdep_notify_dwork;	/* coalesces relay wakes (0x400 heartbeat ~13.4 Hz) */
	bool hwdep_ready;			/* dwork INIT'd: gates cancel (probe-error paths never got here) */
	struct snd_kcontrol *hwdep_meter_ctl;
	s16 *hwdep_meter_map;
	__le32 *hwdep_meter_levels;	/* GET_METER scratch + cache (rate-limited; see clarett_hwdep_meter_get) */
	unsigned long hwdep_meter_polled;	/* jiffies of the last GET_METER; 0 = never */
	int hwdep_meter_channels;	/* map_size: channels the control exposes */
	int hwdep_n_meter_slots;	/* device raw meter count */
	unsigned int *hwdep_meter_labels_tlv;
	unsigned int hwdep_meter_labels_tlv_size;

	/* Data-plane engine state, shared by the PCM path and the stream_probe diagnostic. */
	bool stream_on;
	u32 rx_slot;			/* RX descriptor fragment SLOT stride in bytes (>= audio bytes/fragment):
					 * a page-safe power of two by default, so no fragment straddles a page;
					 * = audio bytes when contiguous (rx_frag_pad=0). */
	u32 tx_slot;			/* TX descriptor fragment SLOT stride, mirror of rx_slot: a straddling TX
					 * fragment is mis-framed by the device. = audio bytes when contiguous
					 * (tx_frag_pad=0); page-safe pow2 default. */
	u32 cur_rate;			/* sample rate last programmed with SET_CLOCK, published at
					 * /proc/asound/cardN/clarett. Seeded at probe from the device so it
					 * is truthful before anything streams; lets userspace read the rate
					 * with NO device traffic (fcp-server needs it to pick the per-rate
					 * meter layout, and polling the mailbox for it would be gratuitous). */
	u32 rx_live_bytes;		/* leading bytes of each capture frame the device fills at the negotiated
					 * rate, and the S/MUX-removed tail after them. Latched at prepare from
					 * clarett_model.rx_live_{mid,high}; the drain blanks the tail because the
					 * engine still leaves a sparse residue there. 0 dead = full width. */
	u32 rx_dead_bytes;
	u32 irq_descs;			/* effective RX IRQ cadence (descriptors between markers); 0 = default 16.
					 * dyn_period derives it from the negotiated ALSA period (clarett_irq_descs). */
	u32 lock_period;		/* dyn_period: frame count both directions share this session (0 = none).
					 * The first configured direction pins it; the other is constrained to match. */
	void *stream_buf;		/* coherent streaming ring buffer */
	dma_addr_t stream_dma;
	size_t stream_size;
	atomic_t period_irqs[CLARETT_NUM_VECTORS];   /* per-vector IRQ counts */
	struct delayed_work stream_report;	/* logs pointer/IRQ progress after start */
	struct task_struct *stream_svc;		/* polls/acks 0x300 to keep the engine clocked */
	atomic_t stream_periods;		/* running period count (servicer -> hw pointer) */
	u32 stream_ctr;				/* last 0x300 period counter (servicer-private) */
	u32 stream_ctr_step;			/* last positive ctr delta (reused across counter wraps) */
	bool stream_run;			/* servicer ACKs 0x300 only while set (PCM trigger gate) */

	/*
	 * PCM capture (data plane). The descriptor table maps the ALSA-managed DMA buffer into
	 * hardware fragments; the servicer kthread advances pcm_frames per 0x300 tick and calls
	 * snd_pcm_period_elapsed (clarett_pcm_tick). Capture = ring block 1 (0x300) only.
	 */
	struct snd_pcm *pcm;
	struct snd_pcm_substream *pcm_sub;	/* live capture substream (NULL when idle) */
	struct snd_pcm_substream *pcm_play_sub;	/* live playback substream (NULL when idle) */
	/*
	 * The hardware rings live in ONE contiguous coherent buffer (c->stream_buf) — split allocations
	 * (separate table / ALSA buffer / TX ring) do not clock. Block 0 (TX; silence when no playback
	 * stream is attached, since the engine needs both directions armed) comes first, block 1 (capture)
	 * second. Samples are copied between the rings and the ALSA buffers each period (clarett_pcm_tick).
	 */
	struct mutex pcm_lock;			/* guards the tick's ring<->ALSA copies vs hw_free teardown */
	bool pcm_running;			/* capture trigger START..STOP: gate period delivery */
	bool play_running;			/* playback trigger START..STOP: gate period delivery */
	atomic_t tx_dirty;			/* app wrote playback frames since the last fill (.ack) */
	u64 pcm_frames;			/* engine frame clock since arm (shared by both directions) */
	/*
	 * Where each direction joined that shared clock (its frame 0). The engine free-runs from the arm,
	 * but ALSA zeroes hw_ptr at every prepare() — so a direction that attaches to an already-armed
	 * engine, or re-prepares after an xrun, must report its position RELATIVE to this base or the
	 * first .pointer call hands the core a huge hw_ptr jump and it xruns instantly. base % ring is
	 * also the rotation between ALSA buffer offsets and hardware ring offsets in the tick's copies.
	 */
	u64 pcm_base;				/* capture: value of pcm_frames when it attached */
	u64 play_base;				/* playback: value of pcm_frames when it attached */
	u64 pcm_last_period;			/* last capture period index reported via period_elapsed */
	u64 play_last_period;			/* last playback period index reported via period_elapsed */

	/*
	 * DIN MIDI (rawmidi over the REG_MIDI_DATA register UART).
	 * RX is drained from the ISR (clarett_midi_irq) and pushed to midi_in when the input is triggered;
	 * TX is drained from midi_out into REG_MIDI_DATA by midi_tx_work. The *_up flags are the rawmidi
	 * trigger gates. rmidi is set LAST at create and doubles as the ISR's "MIDI live" guard.
	 */
	struct snd_rawmidi *rmidi;
	struct snd_rawmidi_substream *midi_in;	/* RX substream (set at input open) */
	struct snd_rawmidi_substream *midi_out;	/* TX substream (set at output open) */
	bool midi_in_up;			/* input trigger gate: push RX bytes to ALSA */
	bool midi_out_up;			/* output trigger gate: TX work may run */
	struct work_struct midi_tx_work;	/* drains rawmidi output -> REG_MIDI_DATA */
	/*
	 * Serialises the RX FIFO drain. clarett_midi_irq() runs from clarett_irq for EVERY MSI vector, and
	 * while streaming the period vectors (vec1/vec2) fire alongside vec0 on other CPUs — two concurrent
	 * drainers of the single-byte 0x58c FIFO otherwise interleave the byte order and corrupt multi-byte
	 * MIDI. Hardirq-only, so a plain spinlock suffices.
	 */
	spinlock_t midi_rx_lock;

	/*
	 * Write-through shadow of the config space, seeded from the device at probe, so the
	 * probe-time enable-bit writes can read-modify-write correctly.
	 */
	u8 shadow[CLARETT_CONFIG_SIZE];
	/*
	 * Per-byte "the shadow is known to match hardware" flags. A shadow byte is
	 * only authoritative once we've written it (write-through) or seeded it from
	 * the device. A skip-if-unchanged optimisation is sound ONLY for known bytes:
	 * for bytes the device does not report back (preamp Mode@166/Air@174) the
	 * shadow stays 0, and skipping "set to 0" would silently drop a real change.
	 */
	DECLARE_BITMAP(shadow_known, CLARETT_CONFIG_SIZE);
};

static inline void clarett_put_le32(u8 *p, u32 v)
{
	p[0] = v;
	p[1] = v >> 8;
	p[2] = v >> 16;
	p[3] = v >> 24;
}

static inline u32 clarett_get_le32(const u8 *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | (u32)p[3] << 24;
}

static inline u16 clarett_get_le16(const u8 *p)
{
	return p[0] | p[1] << 8;
}

static inline void clarett_put_le16(u8 *p, u16 v)
{
	p[0] = v;
	p[1] = v >> 8;
}

/*
 * Runtime stream geometry, derived per-model from c->model. The hardware rings live in one contiguous
 * coherent buffer of 2 * clarett_ring_bytes(): block 0 (TX) then block 1 (RX), each a descriptor table
 * (clarett_tbl_bytes, model-independent) followed by CLARETT_STREAM_NDESC sample fragments. Used only by
 * the stream_probe diagnostic, where TX and RX share one stream_frag; the PCM path uses the per-direction
 * clarett_pcm_* geometry below.
 */
static inline size_t clarett_tbl_bytes(void)
{
	return ALIGN((CLARETT_STREAM_NDESC + 1) * sizeof(__le64), 64);
}

static inline size_t clarett_buf_bytes(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * c->model->stream_frag;
}

static inline size_t clarett_ring_bytes(const struct clarett *c)
{
	return clarett_tbl_bytes() + clarett_buf_bytes(c);
}

/*
 * Hardware IRQ period in bytes for a stream of `channels` (the 0x208/0x308 SIZE register value): one period
 * is 4 interleaved S32_LE frames (8PreX 28ch -> 0x1c0, 2Pre TX 4ch -> 0x40 / RX 14ch -> 0xe0, Red 8Line
 * 64ch -> 0x400). This is the IRQ granularity, decoupled from the descriptor fragment (4 periods per 8PreX fragment).
 */
static inline u32 clarett_period_bytes(u8 channels)
{
	return (u32)channels * 4 * 4;
}

/*
 * PCM descriptor-table geometry (per-direction). Every entry is a bare 8-byte LE bus address; the
 * fragment is exactly CLARETT_FRAG_FRAMES interleaved frames = channels*4*16 bytes, with no alignment
 * rounding (2Pre TX 4ch->0x100, RX 14ch->0x380, 8PreX 28ch->0x700). The RX ring carries a periodic IRQ
 * flag (bit1) every clarett_irq_descs() descriptors — THIS is what raises the counted 0x300 period; a
 * ring flagged only at the end never advances the counter. The LAST entry adds the wrap flag (bit0):
 * TX 0x01, RX 0x03 (wrap|IRQ). No zero terminator.
 */
#define CLARETT_DESC_ALIGN	0x100	/* pad the table so the sample area starts 0x100-aligned (harmless) */
#define CLARETT_DESC_WRAP_TX	0x01	/* last-entry flag, block 0 (TX): bit0 = end-of-list/wrap */
#define CLARETT_DESC_WRAP_RX	0x03	/* last-entry flag, block 1 (RX): bit0 wrap | bit1 IRQ */
#define CLARETT_DESC_IRQ	0x02	/* periodic per-period IRQ marker on RX descriptors (bit1) */

/*
 * A descriptor covers exactly CLARETT_FRAG_FRAMES frames. CLARETT_IRQ_DESCS is the default number of
 * descriptors the RX engine consumes between period IRQs: 16 = 256 frames = 5.33 ms at 48k. The count is
 * the host's choice, as long as RX descriptors carry the flag at that cadence.
 */
#define CLARETT_FRAG_FRAMES	16
#define CLARETT_IRQ_DESCS	16

static inline u32 clarett_frag_bytes(u8 channels)
{
	return (u32)channels * 4 * CLARETT_FRAG_FRAMES;
}
/*
 * The playback engine reads at most CLARETT_TX_FRAG_MAX_BYTES per descriptor, then moves to the next one.
 * Every Clarett TX fragment fits (8PreX 0x700), so they keep 16 frames; the Red 8Line's 64 channels make
 * 0x1000, and with 16-frame fragments the engine played the first 8 frames of each and skipped the rest,
 * walking the ring at twice the rate it was filled (a digital-loopback ramp shows exactly that). So a wide
 * TX stream halves its fragment until it fits. Capture is not capped: the Red's 0xf00 RX fragment is read
 * whole.
 */
#define CLARETT_TX_FRAG_MAX_BYTES	0x800

static inline u32 clarett_tx_frag_frames(u8 channels)
{
	u32 frames = CLARETT_FRAG_FRAMES;

	while (frames > 1 && (u32)channels * 4 * frames > CLARETT_TX_FRAG_MAX_BYTES)
		frames /= 2;
	return frames;
}
static inline u32 clarett_tx_frag_bytes(u8 channels)
{
	return (u32)channels * 4 * clarett_tx_frag_frames(channels);
}
/*
 * Effective RX IRQ cadence (descriptors between periodic IRQ markers). CLARETT_IRQ_DESCS (16) is the
 * default; the dyn_period path (clarett_pcm.c) overrides c->irq_descs per-stream from the negotiated ALSA
 * period so a DAW can pick a smaller buffer. A zero field reads as the default, so it is safe before probe
 * sets it. Must divide CLARETT_STREAM_NDESC so the markers (and the wrap on the last entry) place evenly.
 */
static inline u32 clarett_irq_descs(const struct clarett *c)
{
	return c->irq_descs ? c->irq_descs : CLARETT_IRQ_DESCS;
}
/* Frames advanced per 0x300 period IRQ (one IRQ-flagged descriptor consumed = irq_descs frags).
 * Used only as the ALSA period granularity; the actual capture advance is ctr-delta driven (below). */
static inline u32 clarett_irq_period_frames(const struct clarett *c)
{
	return clarett_irq_descs(c) * CLARETT_FRAG_FRAMES;
}
/*
 * Frames per 0x300 counter unit: one unit == 16 frames (one descriptor fragment). The capture path
 * advances by (measured ctr delta) * this, self-calibrating to the real hardware period regardless of
 * the per-model step or the IRQ-marker spacing.
 */
#define CLARETT_CTR_FRAMES	16
/*
 * Modulus of the 0x300 period counter: it wraps at 0x100. Knowing it is what lets a wrap and a LATE
 * POLL be the same arithmetic — a modular difference — so the frames a delayed tick
 * has to make up are recovered instead of discarded (see clarett_stream_service). Recovery is exact
 * for gaps up to a full modulus (16 periods, ~85 ms); beyond that the advance genuinely aliases.
 */
#define CLARETT_CTR_MOD		0x100
/*
 * Layout of the 0x300 cause word.
 *
 * BIT30 == PERIOD OVERRUN: the device sets it on an event raised while the PREVIOUS period had not yet
 * been acknowledged, i.e. exactly the events the host observes as two periods merged (stepmax 0x2).
 *
 * The counter in an overrun sample is fully valid, and the only bits ever seen are 0xc00000ff, so
 * bit30 is orthogonal to the counter and the sample must be consumed, not dropped. Masking with
 * ~CLARETT_CTR_KNOWN (rather than a bare range test on 0x7fffffff, which keeps bit30 and turns a
 * valid 0x1a into an out-of-range 0x4000001a) still rejects the all-ones reads of a dead link.
 *
 * PRACTICAL CONSEQUENCE: cadence 4 (64-frame period, 1.33 ms) is the lowest setting that runs with zero
 * coalescing and zero overruns. Cadence 1 (16 frames) works, but the engine flags ~0.6 overruns/s.
 * Treat 64 frames as the practical floor for low-latency work; 16 is the hardware floor.
 */
#define CLARETT_CTR_EVENT	0x80000000u	/* bit31: a period event is pending */
#define CLARETT_CTR_OVERRUN	0x40000000u	/* bit30: raised before the previous period was acked */
#define CLARETT_CTR_MASK	(CLARETT_CTR_MOD - 1)	/* the counter itself */
#define CLARETT_CTR_KNOWN	(CLARETT_CTR_EVENT | CLARETT_CTR_OVERRUN | CLARETT_CTR_MASK)
/*
 * A period-event gap over clarett_tick_late_us() counts as a LATE tick in the servicer's telemetry.
 *
 * It must scale with the period: dyn_period makes the nominal period span 16 frames (0.33 ms at 48k) to
 * thousands, and the rate varies 44.1-192 kHz, so any fixed threshold is wrong at one end or the other.
 * It is derived from the live counter step and cur_rate. The multiplier is deliberately LOW: a platform
 * stall of ~40-60 ms must still register at a 1024-frame (21 ms) period, and 3/2 clears normal jitter
 * (~2% of nominal) by a wide margin. The floor keeps sub-millisecond cadences from tripping on ordinary
 * RT jitter; it never applies once nominal reaches 1.33 ms.
 */
#define CLARETT_TICK_LATE_NUM		3
#define CLARETT_TICK_LATE_DEN		2
#define CLARETT_TICK_LATE_FLOOR_US	2000
static inline u64 clarett_tick_late_us(const struct clarett *c)
{
	/* Both fall back to a 48 kHz, step-0xd default, so it stays sane before the first counter
	 * delta has been measured. cur_rate is seeded at probe from
	 * FCP_SYNC_RATE and re-published at each stream handshake, so it is live here. */
	u32 rate = READ_ONCE(c->cur_rate) ? READ_ONCE(c->cur_rate) : CLARETT_DEFAULT_RATE;
	u32 step = c->stream_ctr_step ? c->stream_ctr_step : 0xd;
	u64 nominal = div_u64((u64)step * CLARETT_CTR_FRAMES * USEC_PER_SEC, rate);

	return max_t(u64, nominal * CLARETT_TICK_LATE_NUM / CLARETT_TICK_LATE_DEN,
		     CLARETT_TICK_LATE_FLOOR_US);
}
/* PCM descriptor table size: NDESC bare 8-byte entries, padded to keep the following sample area 0x100-aligned.
 * No +1 terminator slot — the wrap flag on the last entry is the terminator. */
static inline size_t clarett_pcm_tbl_bytes(void)
{
	return ALIGN((size_t)CLARETT_STREAM_NDESC * sizeof(__le64), CLARETT_DESC_ALIGN);
}
/*
 * TX, like RX, has two byte sizes once the fragments are slot-padded (c->tx_slot > audio bytes):
 *   _samples = LOGICAL audio (contiguous frames) — the ALSA playback buffer and the per-period frame math.
 *   _dev     = DEVICE sample area = NDESC slots of c->tx_slot each — what is allocated and what the
 *              descriptors stride over (gaps between fragments when padded). Equal when unpadded.
 */
static inline size_t clarett_pcm_tx_samples(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * clarett_tx_frag_bytes(c->model->playback_channels);
}
static inline size_t clarett_pcm_tx_dev_bytes(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * c->tx_slot;
}
/*
 * RX has TWO byte sizes once the fragments are slot-padded (c->rx_slot > audio bytes):
 *   _samples = the LOGICAL audio (contiguous frames) — the ALSA buffer and the per-period frame math.
 *   _dev     = the DEVICE sample area = NDESC slots of c->rx_slot each — what is allocated and what the
 *              descriptors stride over (with gaps between fragments when padded). Equal when unpadded.
 */
static inline size_t clarett_pcm_rx_samples(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * clarett_frag_bytes(c->model->capture_channels);
}
static inline size_t clarett_pcm_rx_dev_bytes(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * c->rx_slot;
}
/* One ring per direction = table + samples. The contiguous buffer is [TX ring][RX ring]; r1 = r0 + tx ring. */
static inline size_t clarett_pcm_tx_ring(const struct clarett *c)
{
	return clarett_pcm_tbl_bytes() + clarett_pcm_tx_dev_bytes(c);	/* device area (slotted) for allocation */
}
static inline size_t clarett_pcm_rx_ring(const struct clarett *c)
{
	return clarett_pcm_tbl_bytes() + clarett_pcm_rx_dev_bytes(c);	/* device area (slotted) for allocation */
}

/*
 * Stream geometry accessors. total = both rings; rx_off = byte offset of the RX SAMPLE area (the
 * engine's capture write target, and the source of the per-period copy) within the contiguous buffer;
 * rx_area = its size, which is also the ALSA buffer size; r1 = the block-1 descriptor table base.
 */
static inline size_t clarett_stream_tx_off(const struct clarett *c)
{
	return clarett_pcm_tbl_bytes();			/* samples sit past the TX descriptor table */
}
static inline size_t clarett_stream_tx_area_bytes(const struct clarett *c)
{
	return clarett_pcm_tx_samples(c);
}
static inline size_t clarett_stream_rx_off(const struct clarett *c)
{
	/* Past the TX ring and the RX table, PAGE-ALIGNED so each RX fragment slot (a power of two,
	 * <= PAGE) is page-contained; a fragment straddling a page drifts the capture channels. */
	return ALIGN(clarett_pcm_tx_ring(c) + clarett_pcm_tbl_bytes(), PAGE_SIZE);
}
static inline size_t clarett_stream_rx_area_bytes(const struct clarett *c)
{
	return clarett_pcm_rx_samples(c);
}
static inline size_t clarett_stream_r1_off(const struct clarett *c)
{
	return clarett_pcm_tx_ring(c);			/* base of block 1: its descriptor table */
}
static inline size_t clarett_stream_total_bytes(const struct clarett *c)
{
	return clarett_stream_rx_off(c) + clarett_pcm_rx_dev_bytes(c);	/* RX samples last; page-aligned */
}

/* mailbox.c */
int clarett_fcp(struct clarett *c, u32 opcode, const u8 *data, u16 len);
int clarett_fcp_cmd(struct clarett *c, u32 opcode, const u8 *req, u16 req_len,
		    u8 *resp, u16 resp_len);

/* hwdep.c */
int clarett_hwdep_init(struct clarett *c);	/* create the FCP hwdep (fcp-server transport) */
void clarett_hwdep_notify(struct clarett *c, u32 ev);	/* relay a device notification to fcp-server */
void clarett_hwdep_free(struct clarett *c);	/* stop the relay before c is freed (UAF guard) */
int clarett_get_data(struct clarett *c, u32 offset, u32 len);
int clarett_set_data(struct clarett *c, u32 offset, u32 len, const u8 *val);
int clarett_data_cmd(struct clarett *c, u32 activate);
int clarett_write_u8(struct clarett *c, u32 offset, u8 val, u32 activate);
int clarett_write_u8_nosave(struct clarett *c, u32 offset, u8 val, u32 activate);
/* Schedule the debounced NVRAM commit (DATA_CMD{PERSIST}) so a control change survives a power
 * cycle. Called from the in-kernel write path and from the hwdep relay when fcp-server commits a
 * config change; gated on ctl_ready. See the definition. */
void clarett_schedule_persist(struct clarett *c);
/* Write the selected meter source's per-band channel tables (@136/146/156) and commit; see the
 * definition. Called from the hwdep CMD path when fcp-server writes the selector byte @184. */
void clarett_meter_source_follow(struct clarett *c, u8 source);

/* BAR0 access wrappers. */
void clarett_wl(struct clarett *c, u32 off, u32 val);
u32 clarett_rl(struct clarett *c, u32 off);

int clarett_write_bits(struct clarett *c, u32 offset, u8 mask, u8 val, u32 activate);

/* main.c — data-plane engine (shared with pcm.c) */
void clarett_engine_arm(struct clarett *c, dma_addr_t r0, dma_addr_t r1);
void clarett_engine_run(struct clarett *c);
void clarett_engine_stop(struct clarett *c);
int clarett_engine_start(struct clarett *c);

/* pcm.c */
int clarett_create_pcm(struct clarett *c);
/* The "Clock Source" enum control. Driver-owned rather than fcp-server's, because the source lives only
 * in the SET_CLOCK payload alongside the rate — see the comment on the control in clarett_pcm.c. */
int clarett_add_clock_control(struct clarett *c);
void clarett_pcm_tick(struct clarett *c, u32 add_frames);
void clarett_pcm_tx_refill(struct clarett *c);	/* servicer: refill TX if the app wrote since the tick */

/* midi.c */
int clarett_create_midi(struct clarett *c);	/* register the DIN MIDI rawmidi (no-op if enable_midi off) */
void clarett_midi_irq(struct clarett *c);	/* drain the RX FIFO from the ISR */
void clarett_midi_stop(struct clarett *c);	/* cancel TX work before teardown (safe if no MIDI) */

#endif /* CLARETT_H */
