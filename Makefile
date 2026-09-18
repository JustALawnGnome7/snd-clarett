# Out-of-tree build for the Clarett Thunderbolt driver.
#   make                  # build snd-clarett.ko against the running kernel
#   make KDIR=...         # build against a specific kernel tree
#   make modules_install  # install + depmod, so `modprobe snd-clarett` works (needs root)
#   make load             # load in place, pulling in the ALSA modules it needs
#   make dkms-install     # register with DKMS so it rebuilds on every kernel upgrade
#   make alsa-install     # install alsa/Clarett.conf, so ALSA lists the card (needs root)
#   make dist             # source tarball, the input to the RPM packaging
#   make rpm-akmod        # build the akmod RPM (rebuilds itself for each new kernel)
#   make rpm-kmod         # build a binary kmod RPM for one kernel (KVER=...)
#   make clean

# Named once, for both halves of this file: the kbuild half turns it into the object
# list, the packaging half into the tarball/DKMS payload. Adding a source file here is
# the only edit either needs.
CLARETT_SRCS := clarett_main.c clarett_mailbox.c clarett_pcm.c clarett_hwdep.c clarett_midi.c
CLARETT_HDRS := clarett.h clarett_fcp_uapi.h

# Kbuild includes this file with `obj` set (along with src, M and KERNELRELEASE); a direct
# `make` has none of them.
#
# Testing KERNELRELEASE instead — the usual idiom, and what this used to do — is wrong here.
# DKMS rewrites the leading `make` of its MAKE[0] into `make -jN KERNELRELEASE=<kver>` and
# invokes the Makefile DIRECTLY, so the KERNELRELEASE test sends it into the kbuild half,
# which defines variables and declares no targets:
#
#     make[1]: *** No targets.  Stop.
#
# `obj` tells the two apart, because only kbuild sets it. Verified by printing all four in
# both invocations.
ifneq ($(obj),)

obj-m := snd-clarett.o
snd-clarett-objs := $(CLARETT_SRCS:.c=.o)

# CLARETT_VERSION is handed down by the outer half below, which reads it out of
# dkms.conf. Anything that invokes kbuild directly (rather than through this file's
# `all` target) won't set it, and a module that quietly reports a version it did not
# come from is worse than one that admits it doesn't know — hence the explicit
# unknown marker rather than a plausible-looking default.
CLARETT_VERSION ?= 0.0.0-unknown
ccflags-y += -DCLARETT_VERSION=\"$(CLARETT_VERSION)\"

else

KDIR ?= /lib/modules/$(shell uname -r)/build

# dkms.conf owns the version (see the comment there). Parse rather than duplicate, so
# there is exactly one place to bump and no way for the compiled-in MODULE_VERSION to
# disagree with the package that shipped it.
PKGNAME := snd-clarett
CLARETT_VERSION := $(shell sed -n 's/^PACKAGE_VERSION="\(.*\)"$$/\1/p' $(CURDIR)/dkms.conf)

# LICENSES/ (holding the Linux-syscall-note exception that clarett_fcp_uapi.h's SPDX tag refers
# to) and alsa/ are directories, so anything consuming DISTFILES has to recurse — hence `cp -r`
# below.
DISTFILES := $(CLARETT_SRCS) $(CLARETT_HDRS) Makefile dkms.conf \
             LICENSE LICENSES README.md DEVELOPMENT.md alsa
TARBALL   := $(PKGNAME)-$(CLARETT_VERSION).tar.gz
DKMS_SRC  := /usr/src/$(PKGNAME)-$(CLARETT_VERSION)

# alsa-lib loads cards/<driver>.conf only from its own compiled-in data directory, so this
# deliberately follows no PREFIX: a copy under /usr/local/share/alsa is never read. Override
# it for a distribution that relocates alsa-lib's data; DESTDIR stages as usual.
ALSA_CARDS_DIR ?= /usr/share/alsa/cards
ALSA_CONF      := alsa/Clarett.conf

# Ask rpm where its build tree is rather than assuming ~/rpmbuild: %_topdir is
# configurable, and a wrong guess would stage the spec somewhere %akmod_install
# will not find it.
RPM_TOPDIR := $(shell rpm --eval '%{_topdir}' 2>/dev/null)
SPECFILE   := packaging/$(PKGNAME)-kmod.spec
KVER       ?= $(shell uname -r)

# insmod does NOT resolve dependencies — it loads exactly the file named — so loading the .ko
# in place needs the modules it links against resident first. snd-pcm and snd-hwdep are usually
# already up (any onboard HD-Audio card pulls them in), but snd-rawmidi is only loaded once
# something needs MIDI, so a machine with no MIDI device fails with "Unknown symbol
# snd_rawmidi_*". Read the list out of the built module rather than keeping a second copy here,
# so it stays right when the module gains or drops a dependency.
#
# NOTE the -a on the modprobe below is REQUIRED, not decorative: modprobe's synopsis is
# `modprobe [modulename] [module parameters...]`, so without it only the FIRST name is treated
# as a module and the rest are passed to it as parameters. That fails silently — unknown module
# parameters are ignored with a warning rather than rejected — so it exits 0 having loaded one
# module, and the missing one only surfaces later as "Unknown symbol" from insmod.
MODDEPS = $(shell modinfo -F depends $(CURDIR)/snd-clarett.ko 2>/dev/null | tr ',' ' ')

.PHONY: all clean modules_install load unload version dist alsa-install alsa-uninstall \
        dkms-install dkms-uninstall rpm-akmod rpm-kmod

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) CLARETT_VERSION=$(CLARETT_VERSION) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f $(PKGNAME)-*.tar.gz

version:
	@echo $(CLARETT_VERSION)

modules_install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a

# Deliberately NOT dependent on `all`: this is run under sudo, and building as root would leave
# root-owned objects behind and break the next unprivileged `make`. Build first, then load.
load:
	@test -f $(CURDIR)/snd-clarett.ko || \
		{ echo "snd-clarett.ko not built — run 'make' first (as yourself, not root)"; exit 1; }
	$(if $(strip $(MODDEPS)),modprobe -a $(MODDEPS))
	insmod $(CURDIR)/snd-clarett.ko $(ARGS)

unload:
	rmmod snd_clarett

# The card config is userspace data, but it lives with the driver rather than in the top-level
# install because alsa-lib keys it on the name the driver registers the card under
# (card->driver) — nothing fcp-server or the maps know about. DKMS and both RPMs install it
# too; this target is for the `make load` / `modules_install` routes. Needs root.
alsa-install:
	install -D -m 644 $(ALSA_CONF) $(DESTDIR)$(ALSA_CARDS_DIR)/$(notdir $(ALSA_CONF))

alsa-uninstall:
	rm -f $(DESTDIR)$(ALSA_CARDS_DIR)/$(notdir $(ALSA_CONF))

# Source tarball named the way rpmbuild expects (%{name}-%{version}/ prefix inside), so
# packaging/snd-clarett-kmod.spec can consume it unmodified. Only the files needed to
# build and understand the module go in — no build artefacts, no capture logs.
dist:
	@test -n "$(CLARETT_VERSION)" || { echo "no PACKAGE_VERSION in dkms.conf"; exit 1; }
	tar --transform 's,^,$(PKGNAME)-$(CLARETT_VERSION)/,' \
	    --owner=0 --group=0 --numeric-owner --sort=name \
	    -czf $(TARBALL) $(DISTFILES)
	@echo "wrote $(TARBALL)"

# DKMS wants the source under /usr/src/<name>-<version>/ and finds it by that exact name,
# so the copy and the registration both key off CLARETT_VERSION. Needs root.
dkms-install:
	@command -v dkms >/dev/null || \
		{ echo "dkms is not installed (Fedora: dnf install dkms)"; exit 1; }
	install -d $(DKMS_SRC)
	cp -r -t $(DKMS_SRC) $(DISTFILES)
	@# `dkms add` fails rather than no-ops when the version is already registered, and
	@# --force is not portable across dkms 2.x/3.x — so ask first.
	@dkms status -m $(PKGNAME) -v $(CLARETT_VERSION) | grep -q . || \
		dkms add -m $(PKGNAME) -v $(CLARETT_VERSION)
	dkms build -m $(PKGNAME) -v $(CLARETT_VERSION)
	dkms install -m $(PKGNAME) -v $(CLARETT_VERSION) --force
	@# dkms installs modules and nothing else, so the card config goes in separately.
	$(MAKE) --no-print-directory alsa-install
	@echo "installed: modprobe snd-clarett now works, and rebuilds on kernel upgrade"

dkms-uninstall:
	-dkms remove -m $(PKGNAME) -v $(CLARETT_VERSION) --all
	rm -rf $(DKMS_SRC)
	$(MAKE) --no-print-directory alsa-uninstall

# --- RPM packaging (Fedora / RPM Fusion kmodtool) ----------------------------
#
# Both targets BUILD ONLY and print the dnf command instead of running it. The
# asymmetry with dkms-install is deliberate: akmods carries a
# `(kernel-devel-matched if kernel-core)` rich dependency, so its install
# transaction can drag in kernel-core with no kernel-modules — a kernel that
# boots to 800x600 with no network. Reading that transaction (every package
# removed at the old version must have a counterpart installed at the new one)
# is the operator's job and cannot be delegated to a Makefile.
#
# Neither target takes the bare `rpmbuild -bb` path, and that is not an
# oversight: with neither buildforkernels nor kernels defined, kmodtool builds
# for "the current kernels", which needs --repo and a
# buildsys-build-<repo>-kerneldevpkgs helper — RPM Fusion build-farm
# infrastructure that an ordinary Fedora machine does not have.

# Staging, shared by both. The spec is copied into %{_specdir} rather than built
# where it lives because kmodtool's %akmod_install re-invokes `rpmbuild -bs`
# against %{_specdir}/%{name}.spec and fails outright if it is not there.
define rpm_prepare
	@command -v rpmbuild >/dev/null || \
		{ echo "rpmbuild is not installed (Fedora: dnf install rpm-build)"; exit 1; }
	@test -n "$(RPM_TOPDIR)" || { echo "rpm did not report a %_topdir"; exit 1; }
	@# dkms.conf owns the version; the spec has to carry a copy because rpm cannot
	@# read it out. A mismatch makes rpmbuild fail on a missing Source0 instead of
	@# on the actual cause, so check it here where the message can say so.
	@spec_ver=$$(sed -n 's/^Version:[[:space:]]*//p' $(SPECFILE)); \
	 test "$$spec_ver" = "$(CLARETT_VERSION)" || { \
	   echo "$(SPECFILE) has Version: $$spec_ver, dkms.conf has $(CLARETT_VERSION)"; \
	   echo "dkms.conf is the source of truth — update the spec to match."; exit 1; }
	install -d $(RPM_TOPDIR)/SOURCES $(RPM_TOPDIR)/SPECS
	install -m 644 $(TARBALL) $(RPM_TOPDIR)/SOURCES/
	install -m 644 $(SPECFILE) $(RPM_TOPDIR)/SPECS/
endef

# $(1) = shell glob(s) matching the RPMs the build should have produced.
define rpm_report
	@set -- $$(ls -1 $(1) 2>/dev/null); \
	 test $$# -gt 0 || { echo "no RPMs matched — see the rpmbuild output above"; exit 1; }; \
	 echo; echo "the packages to install:"; for f; do echo "  $$f"; done; \
	 dkms status -m $(PKGNAME) 2>/dev/null | grep -q . && { \
	   echo; \
	   echo "WARNING: $(PKGNAME) is also registered with DKMS. The two routes install"; \
	   echo "  to different paths that are both in depmod's search path (akmod:"; \
	   echo "  extra/$(PKGNAME)/, dkms: extra/), and which one loads is undefined."; \
	   echo "  Run 'make dkms-uninstall' first; 'modinfo -n $(PKGNAME)' names the winner."; \
	 }; \
	 echo; \
	 echo "install with (read the transaction before confirming: if it touches any"; \
	 echo "kernel package, every package removed at the old version must have a"; \
	 echo "counterpart installed at the new one, or you get a kernel that boots"; \
	 echo "with no graphics and no network):"; \
	 echo; echo "  sudo dnf install $$*"; echo
endef

# The end-user package: ships the source RPM, and akmods.service rebuilds it at
# boot after any kernel upgrade. Compiles nothing here, so it needs no kernel
# tree and is not tied to the running kernel.
#
# kmodtool also emits an empty kmod-snd-clarett metapackage that just requires
# the akmod; it exists so a repo can track the newest kernel, and it is left out
# of the install line below because it adds nothing to a local file install.
rpm-akmod: dist
	$(rpm_prepare)
	rpmbuild -bb --define 'buildforkernels akmod' $(RPM_TOPDIR)/SPECS/$(notdir $(SPECFILE))
	$(call rpm_report,\
	  $(RPM_TOPDIR)/RPMS/*/akmod-$(PKGNAME)-$(CLARETT_VERSION)-*.rpm \
	  $(RPM_TOPDIR)/RPMS/*/$(PKGNAME)-kmod-common-$(CLARETT_VERSION)-*.rpm)

# A binary module for one kernel, compiled now. Defaults to the running kernel;
# override with KVER=<uname -r> to build for another (its kernel-devel must be
# installed).
rpm-kmod: dist
	$(rpm_prepare)
	rpmbuild -bb --define "kernels $(KVER)" $(RPM_TOPDIR)/SPECS/$(notdir $(SPECFILE))
	$(call rpm_report,\
	  $(RPM_TOPDIR)/RPMS/*/kmod-$(PKGNAME)-$(KVER)-*.rpm \
	  $(RPM_TOPDIR)/RPMS/*/$(PKGNAME)-kmod-common-$(CLARETT_VERSION)-*.rpm)

endif
