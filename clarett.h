/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Focusrite Clarett (Thunderbolt) ALSA driver — shared definitions.
 *
 * Register map and FCP framing are from clean-room reverse-engineering of the
 * device (confirmed against MMIO traces). Control offsets/commands likewise.
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

/* --- BAR0 register map (confirmed) -------------------------------------- */
#define CLARETT_BAR              0
#define REG_CAPS                 0x000
#define REG_SERIAL_LO            0x010
#define REG_SERIAL_HI            0x014
#define REG_IRQ0_CAUSE           0x100   /* read-to-clear; bit DONE = mailbox complete */
#define REG_IRQ0_ENABLE          0x104   /* observed init value 0xf000003f            */
#define REG_NOTIFY_CAUSE         0x400   /* read-to-clear; carries the notify mask     */
#define REG_DOORBELL             0x408   /* write 1 = submit, 2 = ack/clear prior      */
#define REG_DMA_ADDR_LO          0x410   /* GET-response DMA buffer bus address (low 32)  */
#define REG_DMA_ADDR_HI          0x414   /* DMA buffer bus address (high 32) — confirmed  */
#define REG_INFO                 0x8000  /* read-only fw-info header (fw versions, ...) */
#define REG_MBOX                 0x8020  /* FCP request mailbox                         */

/*
 * DIN MIDI UART (rawmidi) — register PIO, NOT the FCP mailbox / audio DMA (reverse-engineered from
 * MMIO captures). REG_MIDI_DATA is bidirectional: a TX write packs up to 3 MIDI
 * bytes with a byte-count in the top byte; an RX read returns one byte with bit24 (MIDI_RX_VALID) set, or
 * 0 when the RX FIFO is empty. RX is interrupt-driven — the shared IRQ summary REG_MIDI_STATUS low byte
 * carries a MIDI-RX-pending code (observed 0x0a); the driver drains REG_MIDI_DATA, then writes
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
 * Data-plane streaming registers (recovered from a streaming capture).
 * Two structurally identical ring blocks; block 0 (0x200) → MSI vec1, block 1 (0x300) → vec2.
 * `clarett_engine_start()` replays the captured stream-start sequence with our own ring buffer.
 */
#define REG_STREAM_IRQ_CFG       0x108   /* stream-start writes 0x10        */
#define REG_STREAM_IRQ_CFG2      0x10c   /* stream-start writes 0x1e70700   */
#define REG_STREAM_IRQ_ARM       0x110   /* stream-start writes 0x7 then 0x0 */
#define STREAM_BLK0              0x200   /* ring block 0 (vec1); +0x00 = cause (read-to-clear) */
#define STREAM_BLK1              0x300   /* ring block 1 (vec2) */
#define   STREAM_OFF_CHANS       0x04    /* channel count = 0x1c (28)       */
#define   STREAM_OFF_SIZE        0x08    /* size/period   = 0x1c0 [HYP]     */
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
#define CLARETT_PCM_RATE         48000          /* default rate, both models (see clocking enum) */
/*
 * Frames the engine advances per 0x300 period event = clarett_irq_period_frames() (one IRQ-flagged
 * descriptor consumed). CALIBRATE on hardware: if the reported rate/pitch is off, the true
 * frames-per-event differs from CLARETT_IRQ_DESCS*CLARETT_FRAG_FRAMES — count 0x300 events/second at a
 * known 48 kHz and adjust. The plumbing is correct regardless of the exact value.
 */

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
 * MSI: bare-metal /proc/interrupts shows the device delivers ALL control-plane
 * interrupts on vector 0 — both mailbox-done and front-panel notifications. The
 * cause registers (not the MSI vector index) distinguish them: 0x100 = mailbox
 * done (polled), 0x400 = notification mask. vec1/vec2/vec3 never fire here; they
 * are the data-plane (period IRQ) suspects.
 */
#define CLARETT_NUM_VECTORS      4
#define CLARETT_VEC_EVENT        0       /* the device signals control events on vec0 */

/*
 * REG_NOTIFY_CAUSE (0x400) is NOT an async event queue — it is a 2-bit command-phase/status
 * register. Correlating every FC boot capture (tools scratch: notify_correlate) shows it only ever
 * holds {0,1,2,3}: idle/ready = 0x3, then it dips 0x3->0x0 while a mailbox command is accepted and
 * blips 0x1->0x2 mid-command, returning to 0x3 at completion. FC POLLS it as flat status and
 * branches on nothing — it performs NO per-bit follow-up read (the Apollo-style "each notification
 * bit points to a descriptor block you must read" model was tested here and refuted). One capture
 * (4pre, at stream time) is the sole place bit3 (0x8) ever appears.
 *
 * Consequence for our ISR: vec0 fires on mailbox-DONE too, and at completion 0x400 reads its idle
 * 0x3 (== NOTIFY_MON_PRIMARY), so a completion MSI can be misread as a monitor event; the cmd_inflight
 * guard suppresses that self-reflection. But hardware (response-logging on the 2Pre) shows
 * the guard is only a minor cleanup: the dominant "notification retried indefinitely" storm is the
 * DEVICE genuinely re-asserting 0x3 in us-scale bursts (8 in 234us, far faster than our ~30ms command
 * rate, inflight=0) because our GET returns empty (size=0) and never satisfies it — where FC's returns
 * real config and it goes quiet. So this is a dormant-backend symptom, not a driver bug. (The earlier
 * 0x00200000/0x00400000 pair was an unverified notify-mask guess that never matched.)
 */
#define NOTIFY_MON_PRIMARY       0x00000003u  /* bit0|bit1 — raised on every monitor (mute/dim) event */
#define NOTIFY_MON_AUX           0x00200000u  /* bit21 — co-occurs intermittently */
#define NOTIFY_MONITOR_MASK      (NOTIFY_MON_PRIMARY | NOTIFY_MON_AUX)

/* Monitoring config region re-read on a notification. */
#define MONITOR_CFG_OFFSET       24
#define MONITOR_CFG_LEN          92
#define MONITOR_VOLUME_OFFSET    112     /* the front-panel knob's level; read-only reflection */
#define MONITOR_ACTIVATE         2       /* DATA_CMD code shared by the monitor controls.
                                          * Trace-confirmed: mute@24 / dim@28 are 1-bit fields that
                                          * toggle 0/1 and commit with activate=2. */

/*
 * DATA_CMD{5} = flash / persist app config (TRACE-confirmed: a monitor mute/dim
 * change emits a standalone DATA_CMD{5} on a debounce, with no preceding SET_DATA). A plain control
 * commit (DATA_CMD{activate}) applies the change live but RAM-only; this persists it across a power
 * cycle. The driver deliberately does NOT auto-issue it — persisting on every mixer tweak would wear
 * the device flash (the vendor app debounces). To add a deliberate "save" action, call
 * clarett_data_cmd(c, FCP_ACTIVATE_PERSIST).
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

/* S/PDIF source select (XML <spdif-mode>): 2-bit fields, DATA_CMD activate 4. <input> @132 picks the
 * S/PDIF *input* to capture (matches scarlett2's "S/PDIF Source Capture Enum"); <output> @124 picks the
 * S/PDIF *output* connector. Enum None=0 / Optical=1 / RCA=2. Activate 4 [TRACE-CONFIRMED]. */
#define SPDIF_SOURCE_OFFSET      132
#define SPDIF_SOURCE_ACTIVATE    4
#define SPDIF_OUTPUT_OFFSET      124
#define SPDIF_OUTPUT_ACTIVATE    4

/* Hardware-meter source select (XML <meter-source> @184, DATA_CMD activate 8) + the per-band channel
 * index tables written alongside it (<hardware-meters> meters-l@136 / meters-m@146 / meters-h@156,
 * 10 bytes each). Enum is a bitmask value: Analogue=1 / S/PDIF=2 / ADAT1=4 / ADAT2=8. [TRACE-CONFIRMED] */
#define METER_SOURCE_OFFSET      184
#define METER_SOURCE_ACTIVATE    8
#define METER_TABLE_L_OFFSET     136
#define METER_TABLE_M_OFFSET     146
#define METER_TABLE_H_OFFSET     156
#define METER_TABLE_LEN          10

/* FCP "big" opcodes (low bits of cmd) — confirmed; == scarlett2 USB values */
#define FCP_GET_DATA             0x800000
#define FCP_SET_DATA             0x800001
#define FCP_DATA_CMD             0x800002

/*
 * GET_METER (0x001001): Focusrite Control polls this continuously (~24 Hz) the entire time it is
 * connected — the bulk of the trace "noise". It is not just a GUI meter read: it is the device's
 * required host heartbeat. With NO periodic poll the device accepts control writes (done=1, fcperr=0)
 * but never applies them to hardware (front-panel LEDs/preamp do not move), and the stream engine
 * stalls after one ring pass. Replaying FC's exact 8-byte payload {0x00300000, 0x00000001} as a
 * periodic heartbeat is what makes control changes physically manifest. See clarett_meter_work().
 */
#define FCP_GET_METER            0x001001
#define CLARETT_METER_POLL_MS    40

/*
 * MUX_READ: read back the routing table. Request {u8 offset, u8 pad, u8 count, u8 band}; reply is an
 * array of u32 entries (src << 12 | dst), capped at 28 per reply. Used at probe to tell
 * an already-configured device (routing present — do not clobber) from an unconfigured one.
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
 * upstream scarlett2 driver's 2 s save debounce, and FC's own traced behaviour (a monitor change
 * emits a standalone DATA_CMD{5} on a debounce). See clarett_save_work() / FCP_ACTIVATE_PERSIST. */
#define CLARETT_SAVE_DELAY_MS    2000

/* SET_CLOCK (TRACE-CONFIRMED): payload {u32 sample_rate, u32 clock_source}. */
#define FCP_SET_CLOCK            0x006003
/*
 * Clock-source enum values. Internal, ADAT and S/PDIF are the same on every model — including the 2Pre,
 * whose [XML] claims S/PDIF is 4. That claim was tested and does NOT match the hardware. Feeding one
 * optical port from an 8PreX and reading Sync Status per value, with an invalid value (7) as the negative
 * control and a real source proven present by the captured audio each time:
 *
 *   value | S/PDIF on the wire | ADAT on the wire | conclusion
 *      0  |        -           |      Locked      | ADAT
 *      3  |     Locked         |     Unlocked     | S/PDIF — tracks that source and only that source
 *      4  |     Locked         |      Locked      | NOT source-specific; locks to whatever is present
 *      7  |    Unlocked        |     Unlocked     | rejected, so Sync really does discriminate
 *
 * So 3 is the S/PDIF selector line-wide and 4 is something looser on the 2Pre (any external / optical),
 * not a per-model S/PDIF encoding. Kept as a documented observation rather than a define, because nothing
 * in the driver selects it. Note the audio path is NOT a probe here: S/PDIF and ADAT keep arriving on
 * their capture channels whatever the clock source says, even while Sync reads Unlocked — the router does
 * not care. Sync Status is the only signal that distinguishes these values.
 */
#define CLARETT_CLOCK_ADAT       0	/* "ADAT 1" on the 8PreX */
/*
 * 8PreX-only, and NOT verifiable by the method above: on that model Sync Status does not reliably track
 * the selected source. Feeding one ADAT port from an 8Pre and stepping the value, the invalid control (7)
 * read Locked in 2 of 3 trials, and value 1 locked with either port fed while value 0 locked only with
 * port 2 fed — mutually inconsistent, so no port mapping can be claimed. Likely the 8PreX reports a lock
 * if EITHER ADAT receiver has locked, independently of the SET_CLOCK selection. Whether the XML's
 * "ADAT 1"/"ADAT 2" labels match the physical ports is therefore OPEN; the values below are XML-derived.
 */
#define CLARETT_CLOCK_ADAT2      1	/* 8PreX only, UNVERIFIED */
#define CLARETT_CLOCK_WORDCLOCK  2	/* 8PreX only, untested (needs a BNC wordclock source) */
#define CLARETT_CLOCK_SPDIF      3	/* all models; hardware-verified on the 2Pre and 8Pre */
/*
 * Red-range only, both [XML]-derived and UNVERIFIED. Note value 4 collides with the loose
 * "any external / optical" behaviour measured on a 2Pre at that value (see the table above): these
 * encodings are per-model, so the Red's 4 meaning Dante neither confirms nor contradicts that
 * observation. Nothing selects them by default — the Red's clock_srcs list is what exposes them.
 */
#define CLARETT_CLOCK_DANTE      4	/* Red only, UNVERIFIED */
#define CLARETT_CLOCK_LOOPSYNC   5	/* Red only, UNVERIFIED */
#define CLARETT_CLOCK_INTERNAL   24
#define CLARETT_DEFAULT_RATE     48000

/*
 * CLOCK/SYNC category (0x006xxx) — these are QUERIES, not commands `[HW — 4Pre]`.
 *
 * They were named FCP_STREAM_ENABLE/FCP_STREAM_COMMIT from watching the vendor issue them in-session
 * immediately before arming the engine, and that inference was WRONG: the category number is the
 * sync category (fcp-server: FCP_OPCODE_CATEGORY_SYNC = 0x006, SYNC_READ = 0x006004), and reading
 * them back on a live 4Pre returns state, not acknowledgement:
 *
 *   0x006004 -> 1        sync lock status (0 = unlocked, 1 = locked)   [== fcp-server SYNC_READ]
 *   0x006002 -> 48000    current rate (the rate we had just set)
 *   0x006005 -> 48000    rate
 *   0x006000 -> 0x30018  caps/bitmask (undecoded)
 *   0x006001 -> 44100    rate
 *   0x006003 -> 44100    rate
 *
 * So the vendor was POLLING whether its clock had locked, not enabling a stream — which also explains
 * its 3-second stall before streaming with zero MMIO writes in it. Consequence for us: the stream
 * handshake has NO enabling function beyond SET_CLOCK and the CONFIG_PUSH burst; issuing these three
 * is inert. They are kept (and still issued) only to keep our command stream byte-identical to the
 * vendor's, and because their responses are worth reading — a device reporting unlocked would explain
 * a dead engine. Ours reports LOCKED at 48000, so the data-plane stall is not a clock problem.
 */
/*
 * FCP_SYNC_RATE is a LIVE rate readback, confirmed on all four models (2Pre, 4Pre, 8Pre, 8PreX): it
 * answers the rate the device is actually running at, and — the property that makes it useful —
 * PERSISTS while nothing is streaming and across a driver reload. Probe seeds cur_rate from it so
 * /proc/asound/cardN/clarett is truthful before the first stream.
 *
 * FCP_SYNC_READ is NOT the clean 0/1 lock flag its name suggests: it returns 1 or 3 depending on model
 * and stream state (a 2Pre and 4Pre read 3 while streaming at 48 kHz where an 8Pre and 8PreX read 1),
 * so it looks like a bitfield whose upper bit is undecoded. fcp-server collapses it with !!, which is
 * why the exposed "Sync Status" is still sane. Suspected cause of that control being unreliable as a
 * clock-source probe on the 8PreX.
 */
#define FCP_SYNC_READ            0x006004   /* lock status bitfield; was misnamed FCP_STREAM_ENABLE */
#define FCP_SYNC_RATE            0x006005   /* u32 rate, live; was misnamed FCP_STREAM_COMMIT */
/* Back-compat aliases: the old names appear in comments/specs written before the decode. */
#define FCP_STREAM_ENABLE        FCP_SYNC_READ
#define FCP_STREAM_COMMIT        FCP_SYNC_RATE

/*
 * Firmware init-handshake opcodes, observed at device attach from the vendor app and not fully
 * decoded: CONFIG_PUSH registers config items by id (arms the config space so SET_DATA writes
 * actually reach hardware), and GET_6x/GET_7x/READ_SEG are version/identity queries.
 *
 * Probe no longer replays any of this — the device restores its own session from flash, so the
 * bring-up is a no-op on a configured unit. What survives is the subset the stream path still
 * needs: clarett_stream_handshake() re-issues CONFIG_PUSH and the GET_7.x queries at every arm,
 * and clarett_detect_model() uses GET_7.1's channel-count answer as the model identity.
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
 * Device bring-up opcodes seen in the vendor attach capture. Not fully decoded, and the driver does
 * not replay them: every unit self-arms from flash, so the host has no bring-up to do. SET_MIX and
 * SET_MUX are live opcodes — they are what a routing or mixer edit issues. Named for documentation.
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
 * the same evidence one command earlier. A capability-dump bench tool dumps every category, and is
 * what distinguishes a device that never came up from one whose session collapsed (a collapsed
 * session denies DATA while a DATA-category read is still answering — the self-contradiction is
 * the tell).
 */
#define FCP_CAP_READ             FCP_INIT_1
#define FCP_CAT_INIT             0x000
#define FCP_CAT_DATA             0x800

/*
 * Per-model descriptor (multi-model support). One const instance per supported
 * Clarett Thunderbolt variant, selected at probe and pinned as clarett.model.
 * Every value that differs between variants lives here; the mailbox/engine/mixer
 * *code* stays model-agnostic. Encodings are per-model (clean-room rule) — never
 * assume a value carries across models.
 *
 * NOTE: all Clarett Thunderbolt units reportedly share PCI id 1cb5:0002, so the
 * id_table cannot distinguish models; driver_data carries the default (2Pre)
 * and runtime disambiguation (fw-info / routing-count query) is a later step.
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
	const char *name;			/* human-readable model name ("Clarett 8PreX") */
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
	u32 max_rate;				/* highest HARDWARE-CONFIRMED sample rate; 0 = single speed (48k)
						 * only. The stream WIDTH is rate-independent (the frame stride never
						 * shrinks), so raising this just advertises the higher SET_CLOCK rates.
						 * Gate per model: bump only after a hardware pitch-check confirms the data
						 * plane (see the max_rate module param, which overrides this for testing). */
	/*
	 * ADAT S/MUX: the frame stays capture_channels wide at every rate, but the device stops WRITING the
	 * ADAT channels that S/MUX removes (8 -> 4 -> 2 per port at single/double/quad speed). Those slots
	 * are not silence, and blanking the ring once does not make them silent: the engine keeps writing a
	 * sparse residue into them — one non-zero sample every 32 frames, an impulse train at roughly
	 * -25 dBFS — and only into the channels dropped at the immediately preceding speed tier (channels
	 * dropped a full tier earlier stay exactly zero). So the dead tail has to be blanked per period, on
	 * the frames handed to ALSA; clarett_set_rx_live() latches the split at prepare and
	 * clarett_rx_drain() does the blanking. These are the counts of leading capture channels the device
	 * still writes at double and quad speed; the dead remainder is a contiguous tail on every model.
	 * 0 = all channels live (no ADAT, or unknown). Derived from the [XML] <record-outputs> pin-m/pin-h
	 * overrides, where "0x0" means the slot is gone at that speed and above.
	 */
	u8 rx_live_mid;				/* capture channels written at 88.2/96 kHz */
	u8 rx_live_high;			/* capture channels written at 176.4/192 kHz */
	const struct clarett_clock_src *clock_srcs;	/* selectable clock sources, Internal first */
	u8 n_clock_srcs;
	u32 stream_frag;			/* legacy engine-start probe only (uniform per-descriptor DMA bytes);
						 * the PCM path derives per-direction fragments from channel counts */
	/*
	 * Per-channel stream-routing CONFIG_PUSH ids, re-issued in-session at PCM prepare (the device resets
	 * stream routing when idle; the probe-time push goes stale). Captured from the VM rate-change handshake:
	 * one CONFIG_PUSH{u16 id} per stream channel. tx[] after GET_7.2, rx[] after
	 * GET_7.3, matching the wire order. NULL/0 = skip the burst (8PreX ids not yet captured).
	 */
	const u8 *stream_tx_ids;
	const u8 *stream_rx_ids;
	u8 n_stream_tx_ids;
	u8 n_stream_rx_ids;
};

/*
 * GET-response DMA layout (confirmed on hardware). The device DMAs the response
 * into resp_buf as a 16-byte FCP header followed by the requested bytes:
 *   resp[0..3]  = echoed cmd (CMD_EXEC_FLAG | opcode) — guard on this
 *   resp[4..5]  = size: # of payload bytes the device actually returned
 *   resp[16+i]  = config[offset + i]  for a GET_DATA{offset, len}
 * A failed/absent DMA leaves the echo word 0, so checking it avoids consuming a
 * stale buffer (seen on the first GET at load, which DMAs all zeroes). But the echo
 * word alone is NOT sufficient: our device answers GET_DATA with the header present
 * yet size=0 and NO payload — the config backend refuses our session (see below).
 * So a reader must ALSO require size > 0 before consuming resp[16+]; otherwise it
 * copies stale buffer bytes.
 *
 * resp[8..11] is the FCP ERROR word: 0 = OK. A working session's responses carry 0
 * with real payload sizes (pmemsave of FC's live buffer). Our sessions
 * get 0x3 on every response — a refusal code, NOT "success" (the
 * earlier reading, calibrated only on walled responses, had this backwards).
 */
#define FCP_RESP_ECHO_OFF        0
#define FCP_RESP_SIZE_OFF        4
#define FCP_RESP_SEQ_OFF         6      /* echoed request seq in the DMAed response header */
#define FCP_RESP_STATUS_OFF      8      /* FCP error word; see layout comment above */
#define FCP_RESP_DATA_OFF        16
#define FCP_RESP_ERR_OK          0x00   /* working-session responses */
#define FCP_RESP_ERR_WALLED      0x03   /* the refusal every command gets on our sessions */

#define CLARETT_MBOX_TIMEOUT_MS  100
/*
 * Interval between readiness attempts.
 *
 * What a cold attach needs is not a longer wait before asking, nor a longer silence between asks — it
 * is the PRE-MAILBOX INIT itself replayed once the device is awake. Measured on an 8Pre: with hw_init
 * done once at ~1 s, mailbox attempts at 0, 25, 50 and 75 s ALL fail; but a first attempt whose hw_init
 * runs at 20 s succeeds at 20 s in, and so does any later bind or module reload — every one of which
 * re-runs hw_init. A device caught mid-wake evidently does not latch those writes, and nothing done
 * afterwards over the mailbox recovers it.
 *
 * BOTH ingredients are required, and each alone is measured useless: re-asking over the mailbox without
 * replaying the init fails at 50 ms, 25 s and 180 s spacing alike, and replaying the init every 5 s
 * fails across 13 attempts. The two successes both had a LONG quiet followed by a fresh init — 20 s and
 * 30 s — so the retry does both: leave the device completely alone for this interval, then replay the
 * init and ask once.
 */
#define CLARETT_READY_RETRY_MS		30000u
#define CLARETT_MAX_PAYLOAD      64      /* clarett_set_data single-write cap (small configs) */
#define CLARETT_MBOX_DATA_MAX    1024    /* mailbox data region past MBOX_DATA; SET_MUX = 412 */
#define CLARETT_CONFIG_SIZE      256     /* shadow of the device config/app space       */
#define CLARETT_APPSPACE_SIZE    8392    /* full persistent config/appspace: the arm's bulk
					  * GET_DATA reads span exactly [0, 8392) and its
					  * writebacks fall inside that range */

/* A hardware-meter source option: its device value and the three per-band channel-index tables the
 * host writes (@136/146/156) when selecting it, alongside SET_DATA{184}=value + DATA_CMD{8}. Kept as
 * per-model RE data (referenced by the model table); the control that consumed it is now fcp-server's. */
struct clarett_meter_source {
	const char *name;
	u8 value;			/* Analogue=1 / S/PDIF=2 / ADAT1=4 / ADAT2=8 */
	u8 tbl[3][10];			/* meters-l, meters-m, meters-h (per sample-rate band) */
};

#define CLARETT_N_METERS         48    /* GET_METER returns 48 u32 levels (num_meters=0x30)  */
#define CLARETT_METER_MAX        4095  /* meter level range 0..4095 (matches scarlett2)       */

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

	/* MSI / async notifications (vec0). With the vendor mailbox cycle (default) the ISR
	 * IS the completion path: while cmd_inflight it reads the 0x100 mailbox cause (the
	 * vendor sweep's first read, MSI-paced) and completes mbox_done; clarett_fcp then
	 * finishes the sweep. legacy_mbox_cycle=1 restores the pure polled mailbox, where
	 * the ISR deliberately never touches 0x100. */
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
	u8 mon_snap[MONITOR_CFG_LEN];
	bool mon_snap_valid;
	struct completion mbox_done;		/* completed by the vec0 ISR on mailbox DONE */
	u32 mbox_cause;				/* 0x100 value the ISR consumed with DONE set */
	/*
	 * Is the mailbox wedged? Set when the last command either produced no response DMA at all, or
	 * produced one echoing a sequence number that is not the one we sent. Both are the same fault:
	 * a command whose response never landed has its trailing ack withheld — as it must be, since
	 * acking an unlanded response is what caused the original session wall — leaving the device
	 * holding that command unretired and answering it in place of every later one, which is exactly
	 * what a stale echoed seq means. clarett_fcp() cannot report this through its return value
	 * without turning a response-less-but-successful SET into a failure, so the readiness poll reads
	 * it here to decide whether to reset the mailbox and retry.
	 */
	bool mbox_wedged;
	/*
	 * Set while a mailbox command is in flight (clarett_fcp submit->complete). vec0 fires on
	 * mailbox-DONE as well as front-panel notifications, and 0x400 reads its idle level (bit0|bit1
	 * = 0x3) at completion, so a completion MSI can be misread as a monitor event. This guard makes
	 * the ISR skip 0x400 while our own command is in flight, suppressing that self-reflection.
	 * NOTE (hardware-confirmed): this is only a MINOR contributor. On a walled device the
	 * dominant "notification retried indefinitely" storm is the DEVICE genuinely re-asserting 0x3 in
	 * us-scale bursts because our GET returns empty (size=0) and never satisfies it — the guard cannot
	 * stop that (the device fires in the idle gaps where inflight=0). See clarett_irq() and the
	 * REG_NOTIFY_CAUSE note.
	 */
	atomic_t cmd_inflight;

	/* Periodic GET_METER heartbeat — the device requires it to apply control writes to
	 * hardware (and to sustain streaming). See FCP_GET_METER / clarett_meter_work(). */
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
	struct delayed_work hwdep_notify_dwork;	/* coalesces relay wakes (idle 0x400 storms ~30 Hz) */
	bool hwdep_ready;			/* dwork INIT'd: gates cancel (probe-error paths never got here) */
	struct snd_kcontrol *hwdep_meter_ctl;
	s16 *hwdep_meter_map;
	__le32 *hwdep_meter_levels;	/* GET_METER scratch + cache (rate-limited; see clarett_hwdep_meter_get) */
	unsigned long hwdep_meter_polled;	/* jiffies of the last GET_METER; 0 = never */
	int hwdep_meter_channels;	/* map_size: channels the control exposes */
	int hwdep_n_meter_slots;	/* device raw meter count */
	unsigned int *hwdep_meter_labels_tlv;
	unsigned int hwdep_meter_labels_tlv_size;

	/*
	 * Data-plane engine-start probe (opt-in via the stream_probe module param). Not a PCM
	 * implementation — it programs the ring registers with this buffer and watches whether
	 * the engine runs (vec1/vec2 period IRQs + DMA pointer advancing). See clarett_engine_start().
	 */
	bool stream_on;
	u32 rx_slot;			/* RX descriptor fragment SLOT stride in bytes (>= audio bytes/fragment).
					 * = audio bytes when contiguous (rx_frag_pad=0); larger to break buffer
					 * contiguity (scatter-gather experiment for the page-drift glitch). */
	u32 tx_slot;			/* TX descriptor fragment SLOT stride, mirror of rx_slot (the working
					 * RX path is non-contiguous; the contiguous TX ring folded 28ch->4 on the
					 * 8PreX). = audio bytes when contiguous (tx_frag_pad=0); page-safe pow2 default. */
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
	 * The hardware rings live in ONE contiguous coherent buffer (c->stream_buf), the exact layout the
	 * engine-start probe proved clocks — split allocations (separate table / ALSA buffer / TX ring) do
	 * NOT clock. Block 0 (silent dummy TX, full-duplex requirement) occupies the first half, block 1
	 * (capture) the second. Captured samples are memcpy'd from the block-1 RX area into the ALSA buffer
	 * each period (clarett_pcm_tick). FC always arms both blocks even for record-only.
	 */
	struct mutex pcm_lock;			/* guards the tick's ring<->ALSA copies vs hw_free teardown */
	bool pcm_running;			/* capture trigger START..STOP: gate period delivery */
	bool play_running;			/* playback trigger START..STOP: gate period delivery */
	u64 pcm_frames;				/* engine frame clock since arm (shared by both directions) */
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
	 * Shadow of the config space backing mixer "get". Updated write-through on
	 * every put; the monitor bytes (24/28/112) are additionally refreshed from
	 * the DMAed GET response on a front-panel notification (clarett_notify_work),
	 * so those reflect live hardware state. Other bytes remain write-through.
	 */
	u8 shadow[CLARETT_CONFIG_SIZE];
	/*
	 * Per-byte "the shadow is known to match hardware" flags. A shadow byte is
	 * only authoritative once we've written it (write-through) or read it from a
	 * trusted live source (the 24/28/112 monitor refresh). The put handler's
	 * skip-if-unchanged optimisation is sound ONLY for known bytes: for a control
	 * the device does not report back (preamp Mode@166/Air@174), the seed leaves
	 * the shadow at 0, and skipping "set to 0" would silently drop a real change.
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
 * (clarett_tbl_bytes, model-independent) followed by CLARETT_STREAM_NDESC sample fragments. NOTE: TX and
 * RX share one stream_frag here (true on the 8PreX, where both directions are 28ch); per-direction
 * (asymmetric) geometry for narrower models is deferred until captured (step 5).
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
 * is 4 interleaved S32_LE frames. Holds for both models (8PreX 28ch -> 0x1c0, 2Pre TX 4ch -> 0x40 / RX 14ch
 * -> 0xe0). This is the IRQ granularity, decoupled from the descriptor fragment (4 periods per 8PreX fragment).
 */
static inline u32 clarett_period_bytes(u8 channels)
{
	return (u32)channels * 4 * 4;
}

/*
 * PCM descriptor-table geometry (per-direction), built to match the LIVE 2Pre vendor tables read out by
 * pmemsave. Every entry is a bare 8-byte LE bus address; the fragment is
 * exactly CLARETT_FRAG_FRAMES interleaved frames = channels*4*16 bytes, packed with NO 0x100 rounding
 * (2Pre TX 4ch->0x100, RX 14ch->0x380, 8PreX 28ch->0x700 — the vendor RX stride 0x380 is only 0x80-aligned,
 * disproving the earlier lcm(0x100,...) rule that doubled 14ch to 0x700). The RX ring carries a periodic
 * IRQ flag (bit1) every CLARETT_IRQ_DESCS descriptors — THIS is what raises the counted 0x300 period; a
 * ring flagged only at the end never advances the counter (the ctr=0 wall). The LAST entry adds the wrap
 * flag (bit0): TX 0x01, RX 0x03 (wrap|IRQ). No zero terminator.
 */
#define CLARETT_DESC_ALIGN	0x100	/* pad the table so the sample area starts 0x100-aligned (harmless) */
#define CLARETT_DESC_WRAP_TX	0x01	/* last-entry flag, block 0 (TX): bit0 = end-of-list/wrap */
#define CLARETT_DESC_WRAP_RX	0x03	/* last-entry flag, block 1 (RX): bit0 wrap | bit1 IRQ */
#define CLARETT_DESC_IRQ	0x02	/* periodic per-period IRQ marker on RX descriptors (bit1) */

/*
 * A descriptor covers exactly CLARETT_FRAG_FRAMES frames (the vendor's fragment is channels*4*16 on both
 * 2Pre directions and the 8PreX, verified by RAM dump — no alignment rounding). CLARETT_IRQ_DESCS is how
 * many descriptors the RX engine consumes between period IRQs; the vendor's 2Pre RX flags roughly every 14
 * (a fractional ~228-frame period). We pick a clean 16 (= 256 frames = 5.33 ms at 48k) since we own our
 * buffer/period; the exact count is our choice as long as RX descriptors carry the flag at this cadence.
 */
#define CLARETT_FRAG_FRAMES	16
#define CLARETT_IRQ_DESCS	16

static inline u32 clarett_frag_bytes(u8 channels)
{
	return (u32)channels * 4 * CLARETT_FRAG_FRAMES;
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
 * Frames per 0x300 counter unit. Hardware-derived: the vendor steps +0xc/period == 192
 * frames == 4 ms at 48k, so one unit == 16 frames; our 2Pre steps +0xd (~208 frames/event ~= 48 kHz).
 * The capture path advances by (measured ctr delta) * this, self-calibrating to the real hardware period
 * regardless of the per-model step or our IRQ-marker spacing. Sanity cap so a glitched read can't
 * over-advance the ring: a real delta is ~12-13, never dozens.
 */
#define CLARETT_CTR_FRAMES	16
/*
 * Modulus of the 0x300 period counter. MEASURED on the 2Pre from the servicer's own
 * 2-second telemetry: the counter steps 0x10 per period and the sampled value advances by exactly
 * (events * 0x10) mod 0x100 across every window, wrapping every 16 events. Knowing it is what lets a
 * wrap and a LATE POLL be the same arithmetic — a modular difference — so the frames a delayed tick
 * has to make up are recovered instead of discarded (see clarett_stream_service). Recovery is exact
 * for gaps up to a full modulus (16 periods, ~85 ms); beyond that the advance genuinely aliases.
 */
#define CLARETT_CTR_MOD		0x100
/*
 * Layout of the 0x300 cause word.
 *
 * BIT30 == PERIOD OVERRUN: the device sets it on an event raised while the PREVIOUS period had not yet
 * been acknowledged. Established on hardware (2Pre) by a dyn_period cadence sweep, and it is
 * about as clean as a black-box result gets:
 *
 *   cadence   period    events/s   stepmax      bit30 in 60 s
 *     1       16 fr       3000     0x1-0x2      36, and 36 again on a repeat run
 *     4       64 fr        750     0x4          0
 *    16      256 fr        187     0x10         0
 *    64     1024 fr         47     0x40         0
 *
 * A cliff, not a slope — a constant per-event rate predicts ~9 at cadence 4, and e^-9 says zero is not
 * that. Note stepmax EQUALS the cadence at 4/16/64 (one period per observation, no coalescing) and
 * alternates 0x1/0x2 at cadence 1. Cross-referencing every 2-second window of both cadence-1 runs:
 * stepmax=0x1 => bit30 delta 0, stepmax=0x2 => bit30 delta >= 1, in 59 of 59 windows with no exceptions.
 * So the flag marks exactly the events the host observed as two periods merged.
 *
 * (An earlier reading of a single run as "front-loads then settles, so it is a startup transient" was
 * WRONG — the repeat accrues throughout, and the quiet windows are simply the stepmax=0x1 ones. There is
 * no time dependence, only coalescing dependence.)
 *
 * The counter in an overrun sample is fully valid — not merely in range but CORRECT: across seven
 * consecutive logged pairs, elapsed * rate / CLARETT_CTR_FRAMES mod CLARETT_CTR_MOD predicted the next
 * counter exactly, over gaps from 16.6 ms to 566 ms. The cumulative OR of every such sample is exactly
 * 0xc00000ff, never another bit. So bit30 is orthogonal to the counter and the sample must be consumed,
 * not dropped.
 *
 * Masking with ~CLARETT_CTR_KNOWN (rather than a bare range test on 0x7fffffff, which keeps bit30 and so
 * turns a valid 0x1a into an out-of-range 0x4000001a) still rejects the all-ones reads of a stalled link,
 * which is what the range test was written for.
 *
 * PRACTICAL CONSEQUENCE: cadence 4 (64-frame period, 1.33 ms) is the lowest setting that runs with zero
 * coalescing and zero overruns, gapmax only ~5% over nominal. Cadence 1 works, but the engine flags
 * ~0.6 overruns/s. Treat 64 frames as the practical floor for low-latency work; 16 is the hardware floor.
 */
#define CLARETT_CTR_EVENT	0x80000000u	/* bit31: a period event is pending */
#define CLARETT_CTR_OVERRUN	0x40000000u	/* bit30: raised before the previous period was acked */
#define CLARETT_CTR_MASK	(CLARETT_CTR_MOD - 1)	/* the counter itself */
#define CLARETT_CTR_KNOWN	(CLARETT_CTR_EVENT | CLARETT_CTR_OVERRUN | CLARETT_CTR_MASK)
/*
 * A period-event gap over clarett_tick_late_us() counts as a LATE tick in the servicer's telemetry.
 *
 * This WAS a fixed 16 ms, calibrated when the period was always ~5.3 ms (step 0xd). It cannot be a
 * constant now: dyn_period derives the IRQ cadence from the negotiated ALSA period, so nominal spans
 * 16 frames (0.33 ms at 48k, cadence 1) to thousands, and the rate itself varies 44.1-192 kHz. A fixed
 * threshold is wrong in BOTH directions — measured on a 1024-frame period (21.33 ms nominal),
 * every healthy tick exceeded 16 ms, so late == the period count in every window and the documented
 * `late=[1-9]` stall grep fired continuously; at cadence 1 the same 16 ms is 48 periods of lateness and
 * would flag nothing at all.
 *
 * So derive it from the live counter step (already self-calibrating) and cur_rate. The multiplier is
 * deliberately LOW: the platform freeze this exists to catch is ~42-48 ms, and 3x — what the old constant
 * was relative to a 5.3 ms period — is 64 ms at a 1024-frame period, i.e. above the blackout. 3/2 clears
 * the measured jitter (~2% of nominal) by a wide margin and stays under one blackout at every period size.
 * The floor keeps sub-millisecond cadences from tripping on ordinary RT jitter; 2 ms is ~6 periods at
 * cadence 1 and still far below anything audible-but-recoverable, and it never applies once nominal
 * reaches 1.33 ms.
 */
#define CLARETT_TICK_LATE_NUM		3
#define CLARETT_TICK_LATE_DEN		2
#define CLARETT_TICK_LATE_FLOOR_US	2000
static inline u64 clarett_tick_late_us(const struct clarett *c)
{
	/* Both fall back to what this threshold was originally calibrated against, so it stays sane
	 * before the first counter delta has been measured. cur_rate is seeded at probe from
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
	return (size_t)CLARETT_STREAM_NDESC * clarett_frag_bytes(c->model->playback_channels);
}
static inline size_t clarett_pcm_tx_dev_bytes(const struct clarett *c)
{
	return (size_t)CLARETT_STREAM_NDESC * c->tx_slot;
}
/*
 * RX has TWO byte sizes once the scatter-gather experiment pads the fragments (c->rx_slot > audio bytes):
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
	 * <= PAGE) is page-contained — the fix for the 8-bytes-per-page capture drift. */
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

/* midi.c */
int clarett_create_midi(struct clarett *c);	/* register the DIN MIDI rawmidi (no-op if enable_midi off) */
void clarett_midi_irq(struct clarett *c);	/* drain the RX FIFO from the ISR */
void clarett_midi_stop(struct clarett *c);	/* cancel TX work before teardown (safe if no MIDI) */

#endif /* CLARETT_H */
