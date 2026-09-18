# RPM packaging for snd-clarett, the Fedora/RPM-Fusion way: kmodtool expands into a
# kmod-snd-clarett-<kernel> subpackage per kernel built for, plus an akmod-snd-clarett
# that ships the source RPM and lets the akmods service rebuild it locally whenever a
# new kernel is installed.
#
# The spec has to be installed in %%{_specdir} before building, not built in place: the
# akmod is produced by kmodtool's %%akmod_install re-invoking `rpmbuild -bs` against
# %%{_specdir}/%%{name}.spec, and it fails there if the file is somewhere else.
#
# The Makefile does all of that, and is the recipe to use:
#
#   make -C .. rpm-akmod            # the akmod — rebuilds itself for each new kernel
#   make -C .. rpm-kmod             # a binary kmod for the running kernel
#   make -C .. rpm-kmod KVER=<ver>  # ... or for another one
#
# It builds the tarball, stages both it and this spec into %%{_topdir}, and prints the
# resulting packages rather than installing them. By hand it is:
#
#   make -C .. dist
#   cp ../snd-clarett-*.tar.gz ~/rpmbuild/SOURCES/
#   cp snd-clarett-kmod.spec   ~/rpmbuild/SPECS/
#   rpmbuild -bb --define 'buildforkernels akmod' ~/rpmbuild/SPECS/snd-clarett-kmod.spec
#   rpmbuild -bb --define "kernels $(uname -r)"   ~/rpmbuild/SPECS/snd-clarett-kmod.spec
#
# Note there is deliberately no plain `rpmbuild -bb` recipe here. Without either define,
# kmodtool takes its "build for the current kernels" path, which requires --repo and the
# buildsys-build-<repo>-kerneldevpkgs helper — RPM Fusion build-farm infrastructure that
# is not present on an ordinary Fedora machine. Name the kernel instead.
#
# Keep Version in step with dkms.conf's PACKAGE_VERSION — that file is the source of
# truth, and the module compiles its value in as MODULE_VERSION.

%global kmod_name snd-clarett

# An akmod build compiles nothing — it only repackages the source RPM — so the debugsource
# package comes out empty and rpmbuild treats that as an error. Kernel-module debuginfo is
# of little use out of tree anyway (a kernel debug build is what you actually want), so
# turn it off for both build modes rather than making it conditional.
%global debug_package %{nil}

Name:           %{kmod_name}-kmod
Version:        0.1.1
Release:        1%{?dist}
Summary:        Focusrite Clarett/Red (Thunderbolt) ALSA kernel module

License:        GPL-2.0-only
URL:            https://github.com/JustALawnGnome7/snd-clarett
Source0:        %{kmod_name}-%{version}.tar.gz

BuildRequires:  kmodtool
# Only needed when actually compiling a module. An akmod-only build ships source, so it
# neither has nor needs a kernel tree; %%{kernels} being set is how kmodtool signals that
# a real per-kernel build is happening.
%{!?kernels:BuildRequires: gcc, make, kernel-devel}

# kmodtool writes the per-kernel subpackage stanzas and the akmod stanza into the spec.
%{expand:%(kmodtool --target %{_target_cpu} --kmodname %{name} %{?buildforkernels:--%{buildforkernels}} %{?kernels:--for-kernels "%{?kernels}"} 2>/dev/null)}

%description
An ALSA driver for Focusrite's Clarett and Red Thunderbolt audio interfaces. One module
covers the Clarett 2Pre, 4Pre, 8Pre and 8PreX and the Red 8Line, and detects the model at
probe from the stream geometry the device reports.

The module provides PCM capture and playback, DIN MIDI, and an FCP hwdep transport; the
mixer, routing and preamp controls are created in userspace by fcp-server over that
transport, the same split the mainline 4th-generation Scarlett driver uses.

This driver was produced by clean-room reverse engineering and is not affiliated with or
endorsed by Focusrite.

# Not optional decoration: kmodtool puts `Requires: %%{name}-common` on every kmod and akmod
# subpackage it generates, so without this the packages build but will not install.
%package -n %{name}-common
Summary:          Documentation and ALSA/WirePlumber configuration shared by every %{kmod_name} package
# alsa-lib owns the directory the card configuration goes into.
Requires:         alsa-lib

%description -n %{name}-common
Documentation for the %{kmod_name} kernel module, and the ALSA card configuration that gives
each interface a standard front device — without it, applications that list devices from
ALSA's name hints do not show the card at all — and the WirePlumber rules that name each
card by model in PipeWire and the desktop. Shared by the per-kernel kmod packages and by the
akmod, and installed as a dependency of those; there is no reason to install it on its own.

%files -n %{name}-common
%license %{kmod_name}-%{version}/LICENSE
%license %{kmod_name}-%{version}/LICENSES/Linux-syscall-note.txt
%doc %{kmod_name}-%{version}/README.md
%doc %{kmod_name}-%{version}/DEVELOPMENT.md
%{_datadir}/alsa/cards/Clarett.conf
%{_datadir}/wireplumber/wireplumber.conf.d/51-clarett-naming.conf

%prep
%{?kmodtool_check}
kmodtool --target %{_target_cpu} --kmodname %{name} %{?buildforkernels:--%{buildforkernels}} %{?kernels:--for-kernels "%{?kernels}"} 2>/dev/null

# -c -T -a 0: unpack into a fresh directory without an implicit top-level %%setup, because
# one source tree has to be copied per kernel being built for.
%setup -q -c -T -a 0

for kernel_version in %{?kernel_versions}; do
    cp -a %{kmod_name}-%{version} _kmod_build_${kernel_version%%%%___*}
done

%build
# %%{kernel_versions} entries are "<uname -r>___<path to the build tree>". Empty for an
# akmod-only build, so this loop correctly does nothing there.
for kernel_version in %{?kernel_versions}; do
    make %{?_smp_mflags} -C _kmod_build_${kernel_version%%%%___*} \
         KDIR="${kernel_version##*___}"
done

%install
# Unconditional, and outside the loop below: an akmod build compiles nothing and runs no
# per-kernel iteration, but its -common package still has to carry the configuration files.
install -D -m 644 %{kmod_name}-%{version}/alsa/Clarett.conf \
    %{buildroot}%{_datadir}/alsa/cards/Clarett.conf
install -D -m 644 %{kmod_name}-%{version}/wireplumber/51-clarett-naming.conf \
    %{buildroot}%{_datadir}/wireplumber/wireplumber.conf.d/51-clarett-naming.conf

for kernel_version in %{?kernel_versions}; do
    install -D -m 755 _kmod_build_${kernel_version%%%%___*}/%{kmod_name}.ko \
        %{buildroot}%{kmodinstdir_prefix}${kernel_version%%%%___*}%{kmodinstdir_postfix}/%{kmod_name}.ko
done
# Supplied by kmodtool; packages the source RPM under /usr/src/akmods for the akmods
# service to rebuild against future kernels.
%{?akmod_install}
