# snd-clarett — Focusrite Clarett/Red (Thunderbolt) ALSA driver

Out-of-tree ALSA driver for Focusrite's Thunderbolt audio interfaces — the **Clarett** and
**Red** ranges — as a single module, with the model detected at probe. Supported today: the
**Clarett 2Pre**, **4Pre**, **8Pre** and **8PreX**, and the **Red 8Line** (single-speed sample
rates for now; see *Known limitations*). The other Red models are expected to share the same interface, so
adding one is mostly a matter of a model entry here and a device map for `fcp-server`.

**Status: working.** Mixer control plane (through `fcp-server`), PCM capture and playback
at 44.1–192 kHz, and DIN MIDI, all confirmed on real hardware. Control changes take physical
effect — preamp Mode/Air, monitor Mute/Dim and the SW/HW gain selector all move the hardware —
and settings persist in the device's own NVRAM.

This is a **clean-room reverse-engineered** driver: it was built from black-box observation
of the device's host interface, Focusrite's device descriptors and the public `scarlett2`/FCP
documentation, and no vendor driver code was disassembled or copied. The research behind it —
protocol specification, tools and evidence trail — lives in the
**[clarett-sre](https://github.com/JustALawnGnome7/clarett-sre)** repository.

## The three pieces

The driver is one of three components, following the same split as the mainline 4th-gen
Scarlett driver: the kernel module provides the PCI transport, audio streaming and MIDI, and
the mixer, routing, preamp and metering controls are created in userspace.

| Component | What it does | Source |
|---|---|---|
| **snd-clarett** (this repository) | Kernel driver: FCP hwdep transport, PCM, MIDI | here |
| **fcp-server** | Creates the ALSA mixer controls over the hwdep | [fcp-support, `snd_clarett` branch](https://github.com/JustALawnGnome7/fcp-support/tree/snd_clarett) |
| **alsa-scarlett-gui** | Full mixer, router and level meters | [alsa-scarlett-gui, `snd_clarett` branch](https://github.com/JustALawnGnome7/alsa-scarlett-gui/tree/snd_clarett) |

Use those branches: the Clarett support in `fcp-server` and `alsa-scarlett-gui` is not yet in
Geoffrey Bennett's upstream projects. The `fcp-support` branch also carries the per-model
Clarett and Red **device maps** that `fcp-server` needs, and installs them with the rest of it.

Without the userspace pieces the driver still provides audio and MIDI; only the mixer controls
are missing.

## Requirements

- Kernel headers for your running kernel (this is an out-of-tree module).
- The interface **visible on the host PCI bus** (`lspci -d 1cb5:0002`). The Clarett line is
  Thunderbolt 2, which needs some setup to reach a modern host — see *Thunderbolt 2 setup*
  below.
- Secure Boot **off**, or the module signed with an enrolled key (see *Secure Boot*).
- For the mixer controls: `fcp-server` and `alsa-scarlett-gui`, as above.

## Thunderbolt 2 setup (getting the device onto the PCI bus)

The driver binds a PCI device, so the Clarett must be visible on the host's PCI bus before the
driver can work at all:

```sh
lspci -d 1cb5:0002        # the interface has to show up here first
```

- **Connect it through an Apple Thunderbolt 2 → Thunderbolt 3 adapter.** On a host with
  Thunderbolt 4 built into the CPU (recent Intel laptops), the adapter may not work when
  plugged into the host directly; in this case, put a Thunderbolt dock with its own controller
  between the host and the adapter.
- **Authorize it with `boltctl` first; only lower Thunderbolt security if it never appears.**
  Whether a Thunderbolt 2 unit is listed depends on the host firmware:

  ```sh
  boltctl list                  # is the interface there?
  boltctl authorize <uuid>      # if so, approve it, then re-check lspci
  ```

  Some boards never list these units at all; on those, setting the firmware's Thunderbolt
  security level to none/legacy is the fallback. Also check that `lspci` shows the device after
  authorizing — being authorized is not the same as being on the bus.
- **If `lspci` shows the adapter's bridge but not `1cb5:0002`**, the firmware may not have
  reserved enough PCI bus numbers behind it. Add these kernel boot parameters and reboot:

  ```
  pci=assign-busses,realloc,hpbussize=0x10
  ```

  Try without them first — most hosts do not need them.

## Build & install

```sh
make                        # builds snd-clarett.ko against the running kernel
sudo make load              # loads it (auto-binds PCI 1cb5:0002)
```

Build against another tree with `make KDIR=/path/to/kernel`, and pass module parameters with
`sudo make load ARGS='enable_pcm=0'`.

To install it permanently instead, so it loads by name:

```sh
sudo make modules_install   # installs under /lib/modules/$(uname -r) and runs depmod
sudo modprobe snd-clarett
```

**Do not use a bare `insmod snd-clarett.ko`.** `insmod` does not resolve dependencies, and this
module needs `snd-pcm`, `snd-hwdep` and `snd-rawmidi` loaded first. On a machine with no other
MIDI device, `snd-rawmidi` is usually absent and `insmod` fails with
`Unknown symbol snd_rawmidi_receive`. `make load` loads the three first; `modprobe` resolves
them itself once the module is installed.

### Keeping it installed across kernel upgrades

`make modules_install` puts the module under the running kernel only, so the next kernel
update leaves you without it. For anything beyond a quick test, install it through a system
that rebuilds it for you.

**DKMS** — works the same on Fedora, Debian/Ubuntu and Arch:

```sh
sudo dnf install dkms        # or: apt install dkms / pacman -S dkms
sudo make dkms-install       # registers the source and builds for the running kernel
sudo modprobe snd-clarett
```

It rebuilds automatically on every kernel install from then on. Check with
`dkms status snd-clarett`, and undo it with `sudo make dkms-uninstall`.

**RPM (Fedora)** — the Fedora-native route is an akmod, which `akmods.service` rebuilds at boot
after a kernel upgrade:

```sh
sudo dnf install akmods rpm-build kernel-devel
make rpm-akmod               # builds the packages and prints the dnf command to install them
```

Read the install transaction before confirming it: `akmods` can pull in a new `kernel-core`
without the matching `kernel-modules`, which boots without graphics or network. Every kernel
package removed at the old version should have a counterpart installed at the new one.

`make rpm-kmod` builds a plain binary module for one kernel instead (`KVER=` to pick it,
default the running one); it does not rebuild itself. `packaging/snd-clarett-dkms.spec` wraps
the DKMS route in an RPM; build instructions are in its header.

Do not install the akmod and DKMS versions together — both land in depmod's search path and
which one loads is undefined (`modinfo -n snd-clarett` names the winner).

Every packaged route loads the module automatically when the interface appears, and installs
the ALSA and WirePlumber configuration described under *Using it*.

### Secure Boot

With Secure Boot enabled the kernel refuses an unsigned module (`Key was rejected by
service`). Either turn Secure Boot off, or enrol the key the build system signs with:

- **DKMS** signs with its own key; enrol it once with
  `sudo mokutil --import /var/lib/dkms/mok.pub` and confirm at the next reboot.
- **akmod** signs with a key generated on first use; enrol it with
  `sudo mokutil --import /etc/pki/akmods/certs/public_key.der`.

Some laptops ship with Secure Boot set to trust Windows only, in which case Linux's own
bootloader is rejected too; look for a firmware option that enables the Microsoft third-party
UEFI CA.

## Using it

### Controls — mixer, routing, preamps, metering

With `fcp-server` from the fork installed, its udev rule starts an instance for each
interface automatically (`fcp-server@<card>`). To run one by hand instead:

```sh
sudo fcp-server <card-number>
```

Then use **alsa-scarlett-gui** for the full mixer, router and meters, or
`alsamixer -c <card>` for the plain controls.

### Model detection

Every model shares one PCI id, so the model is detected from the device itself: it reports its
own stream geometry, which is unique per model. There is nothing to configure, and the driver
logs what it found:

```
snd_clarett 0000:0a:00.0: Clarett 2Pre: serial 000012345678abcd fw app 0x04061973 fpga 0x18101966; FCP hwdep, PCM 4/14ch, MIDI
```

The serial and firmware fields read the same on every unit, so they do not identify one.

There is deliberately no way to override detection: channel counts, DMA geometry and the meter
layout are all sized from the model, so a wrong guess would stream the wrong width. If the
driver cannot identify the device it refuses to register and says so in the kernel log.

### PCM (recording and playback)

Capture and playback are standard ALSA PCM devices. **There is no default route** — playback
is silent until you wire a PCM source to a physical output in the router (alsa-scarlett-gui).

### Device lists in applications

Applications that build their device list from ALSA's name hints — JUCE-based ones among
them — show the interface as *"Clarett 8PreX, Clarett 8PreX; Front output / input"* (the ALSA
device `front:CARD=<id>,DEV=0`). That entry comes from `alsa/Clarett.conf`, which alsa-lib
reads only from `/usr/share/alsa/cards/`. The packaged routes install it; after `make load` or
`make modules_install`, install it once with:

```sh
sudo make alsa-install      # sudo make alsa-uninstall removes it
```

Without it the card still works as `hw:<card>,0`, but those applications do not list it.

The device supports one playback and one capture stream, so while PipeWire holds the card an
application opening it directly gets *"Device or resource busy"*. Either run the application
through PipeWire (`pw-jack <app>`, using its JACK backend), or release the card from PipeWire
while it runs — find the card's name with `pactl list cards short`, then:

```sh
pactl set-card-profile <card name> off         # set it back to pro-audio afterwards
```

### Device names in PipeWire and the desktop

Because every model shares one PCI id, which `pci.ids` names just "Clarett", every unit would
otherwise show up in the desktop as *"Clarett Multichannel"*.
`wireplumber/51-clarett-naming.conf` names each card after the model the driver detected
("Clarett 2Pre", "Red 8Line", …), which also tells two units apart. The packaged routes install
it; otherwise:

```sh
sudo make wireplumber-install    # sudo make wireplumber-uninstall removes it
systemctl --user restart wireplumber
```

### MIDI

The interface's DIN MIDI ports appear as a standard ALSA rawmidi device.

## Settings persistence

The device **owns its settings**: they live in its own NVRAM and survive power cycles, reboots
and moving to another host. The driver commits every control change to that NVRAM a couple of
seconds after the last change.

Because the device restores its own state, **disable `alsactl` restore for this card**, or it
overwrites that state with an older snapshot on every load. If no other card on the system
relies on it:

```sh
sudo systemctl mask alsa-state alsa-restore
sudo systemctl stop alsa-state alsa-restore
sudo rm /var/lib/alsa/asound.state
```

## Known limitations

- **Red 8Line: 44.1/48 kHz only, and no level meters.** The higher sample rates are not enabled
  until they are verified on hardware, and the Red's meter layout is not mapped yet.
- **Periodic audible glitches on some hosts** come from the host firmware stalling the whole
  machine for tens of milliseconds while a Thunderbolt device streams, not from the driver. The
  kernel log shows them as `stream-svc:` lines with a large `readmax`.
- **Preamp Mode/Air can read wrong right after a cold boot.** The hardware is correct (the
  front-panel LEDs are right); only the on-screen value lags until you touch a control.
- **No per-output mute** — the hardware has none, only the master Mute/Dim.
- **Rarely, no card appears right after a cold Thunderbolt attach.** The driver waits for the
  interface to come up and refuses to register if it never answers (it logs *"device did not
  become ready"*). Reload the module once the interface has settled
  (`sudo make unload && sudo make load`).

## How it works / contributing

The design, the FCP transport, model detection, diagnostics and the module parameters are
documented in **[DEVELOPMENT.md](DEVELOPMENT.md)**. The protocol specification and the
reverse-engineering record are in
[clarett-sre](https://github.com/JustALawnGnome7/clarett-sre).

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
