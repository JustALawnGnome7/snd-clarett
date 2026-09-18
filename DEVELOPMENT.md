# snd-clarett — internals

Maintainer / contributor documentation. For build and usage, see [README.md](README.md).

This is a clean-room driver, built only from Focusrite's device XML descriptors, black-box MMIO
observation, and the public `scarlett2`/FCP documentation. No vendor driver code was disassembled
or copied. Encodings are per-model; never assume a value carries across models.

## Overview

The driver follows the in-kernel FCP model used by the 4th-gen Scarlett (`sound/usb/fcp.c`): **a
minimal kernel driver that exposes a hwdep interface**, with Geoffrey Bennett's userspace
`fcp-server` implementing the mixer, routing and preamp controls. There is no in-kernel mixer.
What the kernel does own is the part userspace cannot: the PCI transport, the PCM data plane, DIN
MIDI, the sample clock (the `Clock Source` control), and model detection.

| File | Role |
|---|---|
| `clarett_main.c` | PCI probe, model table and detection, IRQs, notification relay, monitor poll |
| `clarett_mailbox.c` | FCP command transport over BAR0 |
| `clarett_hwdep.c` | The FCP hwdep ABI consumed by `fcp-server` |
| `clarett_pcm.c` | Capture/playback PCM on the descriptor-ring engine, `Clock Source` |
| `clarett_midi.c` | DIN MIDI rawmidi |
| `clarett_fcp_uapi.h` | The hwdep ioctl ABI (uapi, `Linux-syscall-note`) |

## Transport

The device has one 64 KB MMIO BAR, which carries the whole register interface; audio moves by
bus-master DMA. An FCP command is written into the mailbox at BAR0 `0x8020` (`cmd` with bit 31 as
the execute flag, then `size | seq << 16`, `error`, and the payload at `+0x10` — the scarlett2
header layout), and the doorbell at `0x408` is rung with `1`. Completion is polled on the DONE bit
(`0x20000000`) of cause register `0x100`.

GET responses do not come back through the BAR: the device DMAs them into a host buffer whose
address is programmed at `0x410` (low) / `0x414` (high). The response carries a 16-byte echo of the
FCP header, then the requested bytes.

**The trailing ack (`0x408 = 2`) must wait for that response to land**, not merely for DONE. The
DMA arrives asynchronously after DONE; acknowledging before it lands makes the device refuse the
session (`err=3` on every later command). `clarett_resp_wait()` gates the ack on the response
header, which is zeroed before each submit so a stale one cannot satisfy it.

Mailbox completion is polled rather than interrupt-driven, so the completion path never races the
read-to-clear cause register against the ISR. MSI carries the notification (`0x400`) and stream
period (`0x300`) events.

## The hwdep ABI

`PVERSION`, `CMD`, `INIT`, `SET_METER_MAP`/`SET_METER_LABELS`, and the notification
`read()`/`poll()` relay:

- `CMD` maps straight onto the mailbox — the FCP wire packet *is* the mailbox packet.
- `INIT` runs `INIT_1`/`INIT_2` with the opcodes `fcp-server` passes and returns the firmware-info
  block in `step2[]`. `step0` is zero-filled (its USB `STEP0` class request has no mailbox
  equivalent; `fcp-server` ignores it), and `c->seq` is *not* reset — the in-kernel `GET_METER`
  heartbeat shares it, and the device only echoes seq.
- `SET_METER_MAP`/`SET_METER_LABELS` let `fcp-server` create and drive `Level Meter`: it installs a
  channel → raw-slot map (the control's `.get` polls `GET_METER` and projects through it) plus an
  optional `FCP_CHANNEL_LABELS` TLV. The map's size is fixed once the control exists; a map of a
  different size is rejected until the driver is reloaded.
- **The notification relay delivers a wildcard.** The USB FCP device carries a precise notification
  bitmask; this device only signals *that* something may have changed, and the `0x400` signal is in
  fact a periodic heartbeat at ~13.4 Hz rather than a change event. So the relay delivers an
  all-categories event (`~0`) and `fcp-server` re-reads every notifiable control. Wakes are
  **rate-limited** to one per `notify_ms` — deliberately a rate limit (one interval after the
  *first* event of a burst), not a debounce, which against a source that never goes idle would
  never fire.
- **The relay is suppressed while a stream runs** (default), because each wake costs a re-read of
  every notifiable control. `notify_while_streaming=1` keeps it running. While it is suppressed the
  monitor section still tracks: `monitor_poll` reads the monitor region (`GET_DATA(24, 92)`) at the
  meter rate and relays **only when the bytes change**.

The device also needs a host heartbeat: the driver issues `GET_METER` every `meter_poll_ms`, as
Focusrite Control does, and discards the response (`fcp-server` polls the meter itself).

### Device maps

The device does not self-describe — `DEVMAP_INFO` returns size 0 — so `fcp-server`'s
device-provided map path yields nothing. The maps are authored instead (`fcp-server-data/` in the
clarett-sre project), keyed on the model slug the driver publishes (see below). They currently need
local `fcp-server` patches, listed in that directory's README.

### Things the driver does to the device's config

- **At probe it force-enables hardware Mute/Dim for Monitor Out 1-2** (bytes 72/73, command 3) so the
  global `Mute`/`Dim` act on those outputs — the master flag (offset 24/28) does nothing unless an
  output opts in via its enable bit. `monitor_enables=0` skips this. The remaining per-output
  mute/dim enables are not exposed. There is no per-output mute on this hardware.
- **It keeps the software gain of a hardware-controlled output in step with the knob**
  (`hw_gain_follow`). An output whose volume is set to HW ignores its stored software gain, and the
  device never mirrors the knob back into it — so the GUI fader would not follow, and switching the
  output back to SW would jump to a stale value. On every monitor-region change the driver writes the
  knob position (byte 112) into the software gain of each HW-controlled output. These writes are
  change-gated and are not persisted to NVRAM.
- **It persists changes, debounced.** Each control commits live (`DATA_CMD{activate}`), and
  `CLARETT_SAVE_DELAY_MS` (2 s) after the last change a single `DATA_CMD{FCP_ACTIVATE_PERSIST}`
  writes the config to NVRAM, coalescing a burst such as a volume drag into one flash write. A
  pending save is flushed at unload.
- A small write-through shadow of the config space (`clarett_set_data()`) exists so the probe-time
  monitor-enable write can read-modify-write correctly. It is seeded at probe with
  `GET_DATA(24, 92)`; nothing reads it for display.

## Probe and model detection

**The driver never arms the device.** A unit that has been set up once — by Focusrite Control, as
every unit in the field has — restores its session from flash at power-up, so config reads,
metering and control writes all work with no host bring-up. Probe leaves the device's routing and
mixer untouched.

Probe sequence:

1. **Settle.** `settle_ms` (default 3 s) leaves a freshly attached device completely untouched. A
   first mailbox command issued too early (~140 ms after enumeration has been seen to fail) can
   wedge the mailbox in a way nothing but re-initialisation recovers, so the driver does not ask
   early.
2. **Detect.** Every model shares PCI id `1cb5:0002` and an identical pre-mailbox surface —
   registers, config space, the firmware-info header and even the serial are the same across the
   line — but the session reports the model's stream geometry: `GET_7.1{band 0}` answers
   `{u16 playback_channels, u16 capture_channels}`, a pair unique per model.
3. **Retry if not ready.** If the session does not answer, probe waits `CLARETT_READY_RETRY_MS`
   (30 s) untouched, replays the pre-mailbox init and asks once more, within a total budget of
   `wait_ready_ms`. A warm device answers the first attempt. Probe is asynchronous, so the wait does
   not stall the PCI hotplug worker.

**Detection is the only path — there is no override.** If the device never answers, or answers
with a geometry no `clarett_model` claims, probe **fails with `-ENODEV` and registers no card**,
logging why (and, for an unknown geometry, the raw `playback=/capture=` pair). Channel counts, DMA
ring and descriptor geometry, fragment strides and the meter layout are all derived from the model,
so a wrong guess would stream the wrong width into wrongly strided rings. Supporting new hardware is
a `clarett_model` entry keyed on the logged pair.

| Model | Playback / capture channels | Slug |
|---|---|---|
| Clarett 2Pre | 4 / 14 | `clarett-2pre` |
| Clarett 4Pre | 8 / 20 | `clarett-4pre` |
| Clarett 8Pre | 20 / 20 | `clarett-8pre` |
| Clarett 8PreX | 28 / 28 | `clarett-8prex` |
| Red 8Line | 64 / 60 | `red-8line` |

The detected model is published at `/proc/asound/card<N>/clarett`:

```
model: Clarett 8PreX
slug: clarett-8prex
```

The slug is the key `fcp-server` selects its per-model map by. Unlike `card->id` it is never mangled
for uniqueness, so it is a stable contract. The model name is also the card's shortname
(`api.alsa.card.name`), which `wireplumber/51-clarett-naming.conf` matches on — a new or renamed
model needs its rule there too.

The Thunderbolt DROM names the model as well, but is not used: whether a Thunderbolt 2 unit is
enumerated as a kernel-managed Thunderbolt device at all depends on the host firmware, so it cannot
be relied on.

## PCM data plane

Capture and playback share one descriptor-ring engine, driven by a SCHED_FIFO servicer thread that
handles the `0x300` period events (`clarett_pcm.c`). Whichever direction prepares first arms the
engine; the other joins the running clock. Each tick drains the RX ring into the capture buffer and
refills the TX ring ahead of the device's read position, keeping `CLARETT_TX_GUARD_FRAMES` clear of
it.

- **Descriptor fragments are page-safe.** A fragment is `channels × 4 × 16` bytes; each sits in its
  own power-of-two slot so none straddles a 4 KB page, which the device's per-fragment DMA
  mis-frames. On the 2Pre and 4Pre the fragment is already a power of two; the 8Pre and 8PreX need
  the padding.
- **Buffer size is bounded per stream by the period**: at most `CLARETT_MAX_PERIODS` (4) periods,
  with a floor of `CLARETT_MIN_BUFFER_FRAMES` (128), and always a power of two, so it divides the
  4096-frame hardware ring. `max_buffer` adds an optional hard ceiling.
- **Sample rates** 44.1-192 kHz on the Clarett models, 44.1/48 kHz on the Red 8Line until its
  higher rates are verified. The stream width does not change with rate. At double and quad speed
  the ADAT channels S/MUX removes still arrive in the stream with junk in them, so the servicer
  blanks that dead tail of the capture frame every period.
- **Clock source** is the `Clock Source` ALSA control, backed by the per-card `clock_source[]`
  parameter. It is sent with the sample rate in `SET_CLOCK` at every arm. Changing it while idle
  applies immediately; while streaming it waits for the next arm. `Sync Status` (from `fcp-server`)
  shows the lock.
- **There is no default route**: playback is silent until a PCM source is wired to an output in the
  router.
- Surprise removal is handled: the servicer and meter poll stop before the card is disconnected, a
  dead link (all-ones reads) ends the servicer, and the mailbox fails fast on a disconnected device.

On some hosts low-latency streaming glitches periodically because of a Thunderbolt-triggered
firmware SMI that stalls the whole machine for ~40-60 ms. That is a platform fault, not the driver;
the servicer telemetry below shows it as `readmax`/`gapmax` in the tens of milliseconds.

## Kernel log and diagnostics

A healthy load prints **one** line and then stays quiet, including while streaming:

```
snd_clarett 0000:0a:00.0: Clarett 2Pre: serial 000012345678abcd fw app 0x04061973 fpga 0x18101966; FCP hwdep, PCM 4/14ch, MIDI
```

The serial and firmware words are the same constants on every unit and model seen, so they cannot
identify a unit.

- **info** — the probe summary above, and a `stream-svc:` line for any 2-second window (or any
  stream) in which something went wrong: a late servicer tick, a device-side period overrun, or an
  unusable `0x300` sample. Silence during playback is the healthy state.
- **warn** — degraded but working: a short MSI vector count, a subsystem that failed to register, a
  config-shadow seed failure, a transiently unreachable link.
- **err** — the device never became ready, so no card was registered.
- **debug** — everything else: register dumps at probe and arm, descriptor-ring geometry, the
  stream handshake, the per-command mailbox trace, and the healthy-case servicer telemetry.

To bring the detail back at runtime, with no reload:

```sh
# everything (includes the per-command mailbox trace — very noisy)
echo 'module snd_clarett +p' | sudo tee /sys/kernel/debug/dynamic_debug/control

# just the servicer telemetry: match the one statement by its format string
echo 'format "stream-svc:" +p' | sudo tee /sys/kernel/debug/dynamic_debug/control

# or scope it by file / function
echo 'file clarett_main.c +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
echo 'func clarett_stream_service +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
```

`-p` in place of `+p` turns them off again. To catch probe-time lines, pass it at load instead:
`sudo make load ARGS="dyndbg='+p'"`.

Reading the servicer line: `gapmax` is the longest interval between ticks and tracks the nominal
period, so divide by it before calling anything a stall. Judge a stream by `late`, `overrun` and
`badreads`, which are zero when healthy. `readmax` is the slowest single register read.
`cat /proc/asound/card<N>/pcm*/sub*/status` shows which process owns a running stream
(`owner_pid`) — the card allows one playback and one capture stream, so a second client gets
*busy*.

## Module parameters

`modinfo snd-clarett` lists all of them. The ones that matter operationally:

- `enable_pcm` (default on) — register the PCM devices; `0` for a control-only card.
- `enable_midi` (default on) — register the DIN MIDI rawmidi.
- `clock_source` (per card, runtime-writable) — the sample clock source; the same value as the
  `Clock Source` control.
- `max_rate` — override the highest advertised sample rate for every model
  (`48000`/`96000`/`192000`). `0` (default) uses each model's verified limit. When raising it on an
  unverified model, check pitch with a known tone before trusting a rate.
- `max_buffer` — optional hard ceiling on the ALSA buffer, in frames (`0`, the default, leaves it to
  the per-period rule).
- `notify_ms` (default 50) — rate limit for the notification relay.
- `notify_while_streaming` (default off, runtime-writable) — keep relaying notifications while a
  stream runs.
- `monitor_poll` (default on) — the change-detecting monitor-region poll.
- `hw_gain_follow` (default on) — mirror the knob into the software gain of HW-controlled outputs.
- `monitor_enables` (default on) — the probe-time Monitor Out 1-2 mute/dim enables.
- `settle_ms` (default 3000), `wait_ready_ms` (default 100000, runtime-writable) — the probe
  settle window and readiness budget described above. `settle_ms=0` skips the wait when the device
  is known to have been up and untouched.
- `resp_timeout_ms` (default 100) — how long one command's response DMA may take to land.

The rest (`stream_probe`, `error_probe`, `seed_dump`, `resp_trace`, `tx_trace`, the fragment-padding
and DMA-mask levers, and others) are diagnostics; their `MODULE_PARM_DESC` strings say what each
does.

## Settings persistence

The device owns its settings: they live in its NVRAM and survive power cycles, reboots and moving
to another host, so the driver does not re-apply them. This mirrors the in-kernel scarlett2 policy.

- **`alsactl` restore must be disabled for this card** (see README.md), or it overwrites the
  device's stored state on every load with an older snapshot.
- **Preamp Mode/Air can read wrong right after a cold boot.** The preamp bytes (Mode `166+i`, Air
  `174+i`) read back correctly within a powered session and across a module reload. But at power-up
  the device restores Mode/Air to the hardware from NVRAM while bringing the host-readable config up
  at a fixed default that does not mirror it, so the display can disagree with the front-panel LEDs
  until a control is touched. The hardware is correct; there is no read of the true preamp state on a
  fresh boot. `seed_dump=1` dumps the config shadow at probe for anyone digging further.
