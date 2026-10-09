# Testing the Red 16Line, 4Pre and 8Pre

The driver has model entries for the **Focusrite Red 16Line**, **Red 4Pre** and **Red 8Pre**, and
`fcp-server` has device maps for them, but nobody has run any of them on the real hardware yet.
Everything in them comes from the models' device descriptors, checked against a Red 8Line, which
does work. If you have one of these units, this guide walks through the checks that would confirm
it, and what to send back.

The Red range comes in two pairs that share a design:

| Model | Thunderbolt | Its twin | Differs from the twin by |
|---|---|---|---|
| Red 16Line | 3 | Red 8Pre | — |
| Red 8Pre | 2 | Red 16Line | preamps on inputs 1-8 (not 1-2) |
| Red 4Pre | 2 | Red 8Line (works) | preamps on inputs 1-4 (not 1-2) |

The 16Line and 8Pre are the 8Line with eight more line inputs and outputs. The 8Line's results
suggest most of this will simply work. These are the parts that could still be wrong:

- **Detection.** Each twin pair reports the same channel counts, so the driver tells them apart by
  the unit's own Thunderbolt controller: Thunderbolt 2 means a Pre model, Thunderbolt 3 a Line
  model (check 1).
- **The meter layout.** Which meter belongs to which router destination is predicted, not
  measured (check 5).
- **What the 8Line does not have:** line inputs and outputs 9-16 (16Line, 8Pre), and preamps
  beyond inputs 1-2 (4Pre, 8Pre).

Some commands below contain a word in angle brackets, such as `<card>`. Type the value for your
model in its place, without the brackets; each command says what to use. Your model's values:

| Model | Name the system shows | Card ID (`<card>`) | Slug (`<slug>`) |
|---|---|---|---|
| Red 16Line | `Red 16Line` | `R16Line` | `red-16line` |
| Red 4Pre | `Red 4Pre` | `R4Pre` | `red-4pre` |
| Red 8Pre | `Red 8Pre` | `R8Pre` | `red-8pre` |

## 1. Install

The README covers each step in more detail.

1. **Get the interface onto the PCI bus.**
   - The **16Line** is Thunderbolt 3 and connects directly.
   - The **4Pre** and **8Pre** are Thunderbolt 2. A modern host needs Apple's Thunderbolt 3 to
     Thunderbolt 2 adapter, and on a host whose Thunderbolt 4 is built into the CPU, a dock between
     the host and the adapter. See the README's *Thunderbolt 2 setup*.

   Authorize it if your system asks, then confirm it is there:

   ```sh
   boltctl list
   # If your Red is listed but not authorized, authorize it by its uuid: copy the long
   # "uuid:" value boltctl printed for it, for example
   #   boltctl authorize 8076f600-0000-1200-ffff-ffffffffffff
   lspci -d 1cb5:0002            # must list it
   ```

2. **Build and load the driver** (from this repository):

   ```sh
   make
   sudo make load
   ```

3. **Install `fcp-server` and the device maps** from the `snd_clarett` branch of
   [fcp-support](https://github.com/JustALawnGnome7/fcp-support/tree/snd_clarett):

   ```sh
   git clone -b snd_clarett https://github.com/JustALawnGnome7/fcp-support.git
   cd fcp-support && make && sudo make install
   ```

4. **Build and install alsa-scarlett-gui** from the `snd_clarett` branch of
   [alsa-scarlett-gui](https://github.com/JustALawnGnome7/alsa-scarlett-gui/tree/snd_clarett)
   (its `docs/INSTALL.md` lists the build dependencies):

   ```sh
   git clone -b snd_clarett https://github.com/JustALawnGnome7/alsa-scarlett-gui.git
   cd alsa-scarlett-gui/src && make -j$(nproc) && sudo make install
   ```

If a step fails, stop and report it with the error output.

## 2. Checks

Do them in order; each one depends on the one before. For each, note **pass** or **fail**, and
for a failure, what you saw.

### Check 1: detection

```sh
sudo dmesg | grep snd_clarett
cat /proc/asound/cards
```

**Expect** two lines for your model, such as for an 8Pre:

```
snd_clarett ...: Red 8Pre: identified by its Thunderbolt 2 controller 8086:xxxx (0x000 = 0x........)
snd_clarett ...: Red 8Pre: serial ... FCP hwdep, PCM 64/64ch
```

(`PCM 64/60ch` on a 4Pre), and in `/proc/asound/cards` a line with your model's card ID and the
name the system shows, as in the table at the top.

Things to **send exactly as printed**, because each one settles a question:

- the `identified by its Thunderbolt ...` line, whatever it says — the controller ID and the
  `0x000` value are not yet known for any of these models;
- a `... assumed: its geometry twin cannot be ruled out ...` warning, if you get one instead —
  the driver did not recognise your unit's Thunderbolt controller, and **guessed**;
- an `unrecognized stream geometry (playback=... capture=...)` line, if you get one — the unit
  reported channel counts the driver does not expect.

If the driver names the **wrong model** (a 4Pre as a Red 8Line, say), also send the output of
`lspci -tvnn`.

Note your Red's **card number**: the number at the start of its line in `/proc/asound/cards`
(for ` 3 [R8Pre ]: Clarett - Red 8Pre`, it is `3`). In the commands below, type that number
where `N` appears:

```sh
alsactl info N | grep components     # for card 3: alsactl info 3 | grep components
```

**Expect** `Clarett:` followed by your model's slug from the table at the top, for example
`Clarett:red-8pre`.

### Check 2: the mixer controls

```sh
systemctl status fcp-server@N
journalctl -u fcp-server@N | grep -E "Loaded|rror"
```

**Expect** the service running, with `Loaded FCP ALSA map from .../fcp-alsa-map-<slug>.json` in
its log, where `<slug>` is your model's slug (`fcp-alsa-map-red-8pre.json` on an 8Pre). In the two
commands above, `N` is the card number again (for card 3: `systemctl status fcp-server@3`).

Then open **alsa-scarlett-gui**: the main window should show your model with its controls,
including a preamp section for each of inputs 1-2 (16Line), 1-4 (4Pre) or 1-8 (8Pre).
The Routing window should list every analogue input as a source and every analogue output among
the destinations. The Pre models number their line outputs from 3 (3-10 on the 4Pre, 3-18 on the
8Pre), as on the unit.

If `fcp-server` exits, **send the whole `journalctl -u fcp-server@N` output.**

### Check 3: recording

PipeWire holds the card while it is active, so release it first:

```sh
pactl list cards short
```

This lists every sound card PipeWire knows. Each line's second column is a **card name** such as
`alsa_card.pci-0000_07_00.0`. To find your Red's, run:

```sh
lspci -d 1cb5:0002
```

Its first column is your Red's address, such as `07:00.0`. Your Red's card name is the one in
`pactl list cards short` ending with that address, with the `:` replaced by `_`
(`alsa_card.pci-0000_07_00.0`). Type that name where `<card name>` appears:

```sh
pactl set-card-profile <card name> off
# for example: pactl set-card-profile alsa_card.pci-0000_07_00.0 off
```

Then record. Replace `<card>` with your model's card ID — `R16Line`, `R4Pre` or `R8Pre` — and on
a **Red 4Pre** change `-c 64` to `-c 60` (it records 60 channels):

```sh
arecord -D hw:CARD=<card>,0 -c 64 -f S32_LE -r 48000 -d 10 /tmp/red-48k.wav
# for example, on a Red 8Pre:
#   arecord -D hw:CARD=R8Pre,0 -c 64 -f S32_LE -r 48000 -d 10 /tmp/red-48k.wav
# on a Red 4Pre:
#   arecord -D hw:CARD=R4Pre,0 -c 60 -f S32_LE -r 48000 -d 10 /tmp/red-48k.wav
```

**Expect** it to run for 10 seconds with no errors. Then check that a signal on an input really
arrives: in the Routing window, see which PCM channel an input you are feeding is routed to,
record again, and look at that channel in any audio editor.

Repeat at **96000** and **192000** Hz: run the same `arecord` command with `-r 48000` changed to
`-r 96000`, then to `-r 192000`. At those rates the interface carries fewer channels, but
the recording keeps its full width; the unused channels should read silence.

Set the profile back afterwards, with the same card name as before:
`pactl set-card-profile <card name> pro-audio`.

### Check 4: playback

In the Routing window, route **PCM 1** and **PCM 2** to **Monitor Output 1** and **2** (there is
no default routing, so playback is silent until you do), then:

Replace `<card>` with your model's card ID — `R16Line`, `R4Pre` or `R8Pre` (all three play 64
channels, so `-c 64` stays as it is):

```sh
speaker-test -D hw:CARD=<card>,0 -c 64 -F S32_LE -r 48000 -t sine -s 1
# for example, on a Red 16Line:
#   speaker-test -D hw:CARD=R16Line,0 -c 64 -F S32_LE -r 48000 -t sine -s 1
```

**Expect** a sine tone from the monitor outputs (`-s 1` plays only channel 1, so the left
side). Release PipeWire first, as in check 3.

### Check 5: meters — the most important check

This is the one prediction that could be wrong without anything else failing.

1. In the Routing window, route a signal to **Monitor Output 1 only**: playback (PCM 1, while
   running the `speaker-test` above), or an input you are feeding.
2. Look at the level meters in the Routing window.

**Expect** the meter on **Monitor Output 1** to move with the signal. On a 16Line or 8Pre, if the
meter on **Headphone 2 (L)** moves instead — four places further along — every output meter is
four places off. That is a known possibility with a one-line fix. **Whatever your model, report
which meter moved**, and if it was somewhere unexpected, where.

Then do the same with a **Line Output** and with **Mixer Input 01** (route a signal into the
mixer).

### Check 6: what the 8Line does not have

**16Line and 8Pre — line inputs and outputs 9-16:**

- Feed a signal into **Line 9** (or any of 9-16), route it to a PCM channel, and confirm it
  records, as in check 3.
- Route a signal to one of the highest line outputs, and confirm its **level**, **mute** and
  **dim** controls in the main window change what comes out of that output.

**4Pre and 8Pre — the extra preamps (inputs 3-4, or 3-8):**

- **Phantom power, Air, high-pass filter and phase** on each: each should show its LED or take
  effect on the unit.
- **Mode**: these inputs offer Mic and Line only (inputs 1-2 also have Inst).
- **Gain**: the fader should change the level in both modes. Note the range you hear, if you
  can — it is only measured on inputs 1-2.
- **Stereo link** on inputs 3-4 (and 5-6, 7-8): linking a pair should make a gain change on one
  input move the other by the same amount, as it does on inputs 1-2.

### Check 7: the rest

Each of these works on the 8Line; just note anything that does not:

- **Preamps** on inputs 1-2: phantom power, Air, high-pass filter, phase, gain, stereo link and
  the Mic/Line/Inst mode.
- **Monitor and headphone knobs** on the front panel: the GUI's faders should follow them.
- **Clock Source** in the main window should list Internal, Wordclock, ADAT 1, ADAT 2, S/PDIF,
  Dante and Loop Sync, and **Sync Status** should read Locked on Internal.
- **Meter Source** should list your model's banks; choosing one should change what the
  front-panel meters show.

## 3. Reporting

Open an issue on this repository with:

- the model as printed on the unit,
- the `snd_clarett` lines from `dmesg` (check 1),
- **pass/fail for each check**, with what you saw for any failure, and
- for check 5, **which meter moved**.

Thank you — a few minutes of this turns a model from "expected to work" into supported.
