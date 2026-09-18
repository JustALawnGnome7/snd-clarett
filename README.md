# snd-clarett — Focusrite Clarett (Thunderbolt) ALSA driver

Out-of-tree ALSA driver for the Focusrite **Clarett** Thunderbolt audio interfaces —
**Clarett 2Pre**, **4Pre**, **8Pre**, and **8PreX** — as a single module, with the model
auto-detected at probe.

**Status: working.** Mixer control plane (through `fcp-server`), PCM capture and playback,
and DIN MIDI, all confirmed on real hardware. Built clean-room; for how it works and the
reverse-engineering history, see **[DEVELOPMENT.md](DEVELOPMENT.md)**.

> Control changes take physical effect: preamp Mode/Air, monitor Mute/Dim, and the SW/HW
> gain selector all move the hardware, and settings persist in the device's own NVRAM.

## Requirements

- Kernel headers for your running kernel (this is an out-of-tree module).
- The interface **visible on the host PCI bus** (`lspci -d 1cb5:0002`). The Clarett line is
  Thunderbolt 2, which needs some firmware/boot setup to reach a modern host — see
  *Thunderbolt 2 setup* below.
- Secure Boot **off**, or the module signed with an enrolled MOK (it ships unsigned).
- For the mixer/routing controls: **`fcp-server`** with the Clarett device maps (from the
  clarett-sre project's `fcp-server-data/`), and **`alsa-scarlett-gui`** for the full
  mixer/router. **At the moment both need patched builds** — the required changes are not yet
  in Geoffrey Bennett's upstream `fcp-server` / `alsa-scarlett-gui`, so use the forks with the
  Clarett patches applied. The kernel module provides only the transport; the controls live in
  userspace.

## Thunderbolt 2 setup (getting the device onto the PCI bus)

The driver binds a PCI device, so the Clarett must be visible on the host's PCI bus before it
can work at all:

```sh
lspci -d 1cb5:0002        # the interface has to show up here first
```

The whole Clarett line is **Thunderbolt 2**, which takes a little setup to reach a modern
(Thunderbolt 3+) host:

- **Try `boltctl` first; only disable Thunderbolt security if the device never appears.**
  Whether a TB2 Clarett is enumerated by the bolt daemon turns out to be **host-firmware
  dependent**, so start with security left on:

  ```sh
  boltctl list                  # is the interface there?
  boltctl authorize <uuid>      # if so, approve it, then re-check lspci
  ```

  On an **HP EliteBook 840 G5** the Clarett Thunderbolt units are listed by `boltctl` and reach
  the PCI bus as soon as they are authorized, with the firmware's Thunderbolt Security Level left
  at *PCIe and DisplayPort - User Authorization* — no need to drop to *No Security*, and no need
  to clear *Require BIOS PW to change Thunderbolt Security Level*. That machine's *Thunderbolt
  PCIe Hot plug Mode* was set to *Native + Lower Power Mode* (its *Legacy Mode* alternative is
  untested); selecting the native option also disables Thunderbolt S4 boot, which the firmware
  says on screen. Other vendors appear to spell this setting *Thunderbolt BIOS Assist Mode*.

  On an **ASRock X570 Creator** the same units never appear in `boltctl` at all, so they cannot be
  user-approved and the security level has to be set to none/legacy before they reach the bus.
  If your board behaves that way, that is the fallback — not the starting point.

- **Bridge TB2 to the host with an Apple Thunderbolt 2 → Thunderbolt 3 adapter.** It is an
  active cable that appears to the host as a PCI bridge, with the Clarett sitting behind it.

  **Some** boards' firmware does not reserve enough PCI bus numbers for a second device behind
  that bridge, so the interface fails to enumerate even though the bridge itself shows up. Only
  if `lspci` lists the bridge but not `1cb5:0002`, add these kernel boot parameters (through
  GRUB, or whatever bootloader you use) and reboot:

  ```
  pci=assign-busses,realloc,hpbussize=0x10
  ```

  Needed on an **ASRock X570 Creator**; **not** needed on an **HP EliteBook 840 G5**, where the
  Clarett enumerates with stock kernel parameters. Try it stock first — this is a workaround for
  firmware that under-allocates, not a requirement of the device.

## Build & install

```sh
make                        # builds snd-clarett.ko against the running kernel
sudo make load              # loads it (auto-binds PCI 1cb5:0002)
```

Build against another tree with `make KDIR=/path/to/kernel`, and pass module parameters with
`sudo make load ARGS='enable_pcm=0'`.

To install it permanently instead, so it loads by name and survives across sessions:

```sh
sudo make modules_install   # installs under /lib/modules/$(uname -r) and runs depmod
sudo modprobe snd-clarett
```

**Do not use a bare `insmod snd-clarett.ko`.** `insmod` loads exactly the file you name and does
not resolve dependencies, and this module links against `snd-pcm`, `snd-hwdep` and `snd-rawmidi`.
The first two are normally already loaded by whatever drives your onboard audio, but
`snd-rawmidi` only appears once something needs MIDI — so on a machine with no MIDI device
present, `insmod` fails with:

```
snd_clarett: Unknown symbol snd_rawmidi_receive (err -2)
```

That is a missing dependency, not a broken build. `make load` pulls the three in first;
`modprobe` resolves them itself once the module is installed.

### Keeping it installed across kernel upgrades

`make modules_install` puts the module under the kernel it was built against and nowhere
else, so the next kernel update leaves you with no driver until you rebuild by hand. For
anything other than a quick test, install it through a system that rebuilds it for you.

**DKMS** — works the same on Fedora, Debian/Ubuntu and Arch, and can sign the module for
Secure Boot:

```sh
sudo dnf install dkms        # or: apt install dkms / pacman -S dkms
sudo make dkms-install       # registers the source and builds for the running kernel
sudo modprobe snd-clarett
```

It rebuilds automatically on every kernel install from then on. Check with
`dkms status snd-clarett`, and undo the whole thing with `sudo make dkms-uninstall`.

**RPM (Fedora)** — the Fedora-native route is an akmod, which ships its source and lets
`akmods.service` rebuild the module at boot after a kernel upgrade:

```sh
sudo dnf install akmods rpm-build kernel-devel
make rpm-akmod               # builds; prints the dnf command, does not install
```

Read the install transaction before confirming it. `akmods` carries a rich dependency on
`kernel-devel`, which can resolve to `kernel-core` without `kernel-modules` and leave you
a kernel that boots with no graphics and no network — check that every package removed at
the old version has a counterpart installed at the new one.

`make rpm-kmod` builds a plain binary module for one kernel instead (`KVER=` to pick it,
default the running one); it does not rebuild itself, so it suits a pinned kernel or a
build host. Both targets call `make dist` for the source tarball and stage the spec into
`%_topdir` first, which is required — kmodtool re-invokes `rpmbuild` against
`%{_specdir}/%{name}.spec` and fails if the spec is not there.

Do not install the akmod and DKMS at the same time: they install to different paths that
are both in depmod's search path, and which module loads is undefined. `modinfo -n
snd-clarett` names the one that won.

`packaging/` also holds `snd-clarett-dkms.spec`, which wraps the DKMS route above in an
RPM; build instructions are in its header.

Either route makes the module load on its own when the interface appears, through the
PCI id alias — no `modprobe` and no udev rule needed once it is installed. Both also
install the ALSA card configuration; see *Device lists in applications* below.

### Secure Boot

The module is unsigned, so on a machine with Secure Boot enabled the kernel will refuse to
load it (`Key was rejected by service`). Either turn Secure Boot off in your firmware, or
enrol a Machine Owner Key and sign with it. DKMS is the easier path for the second: recent
versions generate and sign with a MOK automatically, leaving you to enrol it once with
`sudo mokutil --import /var/lib/dkms/mok.pub` and confirm at the next reboot.

## Using it

### Controls — mixer, routing, preamps, metering

The module exposes a minimal FCP *hwdep* transport; the actual controls are created by
**`fcp-server`** in userspace (the same model the mainline 4th-gen Scarlett driver uses):

```sh
sudo fcp-server <card>      # <card> = the ALSA card number or id
```

Then use `alsamixer -c <card>`, or **alsa-scarlett-gui** for the full mixer and router.
`fcp-server` can also be auto-started per interface through its udev/systemd integration.

### Model selection

The whole Clarett Thunderbolt line shares one PCI id, so the model is detected from the
device itself — it reports its own stream geometry, which is unique per model. There is
nothing to configure:

```sh
sudo make load
```

The driver logs which model it found:

```
snd_clarett 0000:0a:00.0: Clarett 2Pre: serial 0000000012345678 fw app 0x00010007 fpga 0x00000104; FCP hwdep, PCM 4/14ch, MIDI
```

There is deliberately no way to override this. Channel counts, DMA ring geometry, routing
and mixer tables, and the meter layout are all sized from the model, so a wrong one is not a
mislabel — it is a card that streams the wrong width into wrongly strided buffers. If the
driver cannot identify the device it refuses to register rather than guess, and says so in
the kernel log.

### PCM (recording and playback)

Capture and playback are available as standard ALSA PCM devices. **There is no default
route** — playback is silent until you wire a PCM source to a physical output in the router
(alsa-scarlett-gui). That is a mixer-config step, not a bug.

### Device lists in applications

Applications that build their device list from ALSA's name hints — JUCE-based ones among
them — show the interface as *"Clarett 8PreX, Clarett 8PreX; Front output / input"* (the ALSA
device `front:CARD=<id>,DEV=0`). That entry comes from `alsa/Clarett.conf`, which alsa-lib
must find in `/usr/share/alsa/cards/` — it looks nowhere else. The DKMS and RPM routes install
it; after `make load` or `make modules_install`, install it once with:

```sh
sudo make alsa-install      # sudo make alsa-uninstall removes it
```

Without it the card still works as `hw:<card>,0`, but it is missing from those applications'
lists altogether.

It opens the hardware directly, so it is exclusive in the same way `hw:` is: while PipeWire
holds the card, the application gets *"Device or resource busy"*. Either run the application
through PipeWire (`pw-jack <app>` with its JACK backend), or release the card from PipeWire
while it runs — find the card's name with `pactl list cards short`, then:

```sh
pactl set-card-profile <card name> off         # pro-audio gives it back to PipeWire
```

### Device names in PipeWire and the desktop

Every model shares one PCI ID, which `pci.ids` names just "Clarett", and WirePlumber prefers
that name over the card's own. So without help every unit shows up as *"Clarett
Multichannel"*. `wireplumber/51-clarett-naming.conf` renames each card after the model the
driver detected ("Clarett 2Pre", "Red 8Line", …), which also tells two units apart. The DKMS
and RPM routes install it; otherwise:

```sh
sudo make wireplumber-install    # sudo make wireplumber-uninstall removes it
systemctl --user restart wireplumber
```

### MIDI

The interface's DIN MIDI ports appear as a standard ALSA rawmidi device.

## Settings persistence

The device **owns its settings** — they live in its own NVRAM and survive power cycles,
reboots, and moving to another host. Every control change you make — mixer, routing, preamp,
output gain, S/PDIF source — is committed to that NVRAM by the driver on a short debounce, so it
sticks across a power cycle without the host having to re-apply it.

Because the device already restores its own state, **disable `alsactl` restore for this
card** so it doesn't overwrite that state on every load:

```sh
sudo systemctl mask alsa-state alsa-restore
sudo systemctl stop alsa-state alsa-restore
sudo rm /var/lib/alsa/asound.state          # only if no other card needs it
```

## Known limitations

- **Low-latency streaming can glitch on some platforms** — periodic audible skips traced to a
  Thunderbolt-triggered firmware SMI, not the driver (the DMA rides through it; the click is
  the audio server re-preparing). A BIOS/firmware issue.
- **Preamp Mode/Air can read wrong right after a cold boot** — the hardware is correct (the
  LEDs are right); only the on-screen value can lag until you touch a control, after which it
  tracks correctly.
- **No per-output mute** — the hardware has none (only master Mute/Dim, reaching the outputs
  that opt in).
- **Rarely, no card appears right after a cold Thunderbolt attach** — if the driver loads before
  the interface has finished coming up, it waits briefly and, if the device still isn't
  responding, refuses to register rather than come up half-working (it logs *"device did not
  become ready"*). Just reload the module (`sudo make unload && sudo make load`) — the
  device settles within a moment.

## How it works / contributing

The design, the FCP protocol, model detection, the module parameters, and the
reverse-engineering history are documented in **[DEVELOPMENT.md](DEVELOPMENT.md)**.

## License

**GPL-2.0-only.** The full text is in [LICENSE](LICENSE); every source file carries an
`SPDX-License-Identifier` saying which terms apply to it.

The one exception is `clarett_fcp_uapi.h`, the header describing the hwdep ioctl interface,
which is `GPL-2.0 WITH Linux-syscall-note` — the same terms the kernel puts on its own uapi
headers, so that a userspace program using this interface is not made a derived work by
including it. That exception's text is in
[LICENSES/Linux-syscall-note.txt](LICENSES/Linux-syscall-note.txt).

This driver was produced by clean-room reverse engineering. It is not affiliated with,
endorsed by, or supported by Focusrite, and no vendor driver code was disassembled or
copied in building it.
