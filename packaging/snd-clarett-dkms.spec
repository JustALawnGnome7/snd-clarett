# RPM packaging for snd-clarett via DKMS — the portable alternative to the kmod/akmod
# spec next to this one.
#
# Which to use: akmod is the Fedora-native route and needs no compiler configuration from
# the user, but it is RPM-specific. DKMS is one package that works the same on Fedora,
# Debian/Ubuntu and Arch, and its Fedora package can auto-sign modules for Secure Boot
# with a locally enrolled MOK. Shipping both costs one extra spec file and lets a user
# pick whichever their system already uses.
#
# Build:
#   make -C .. dist && mv ../snd-clarett-*.tar.gz ~/rpmbuild/SOURCES/
#   rpmbuild -bb snd-clarett-dkms.spec
#
# Keep Version in step with dkms.conf's PACKAGE_VERSION — that file is the source of
# truth, and the module compiles its value in as MODULE_VERSION.

%global module_name snd-clarett

# The payload is source code; there is nothing compiled here to extract symbols from.
%global debug_package %{nil}

Name:           %{module_name}-dkms
Version:        0.1.1
Release:        1%{?dist}
Summary:        Focusrite Clarett/Red (Thunderbolt) ALSA kernel module (DKMS)

License:        GPL-2.0-only
URL:            https://github.com/JustALawnGnome7/snd-clarett
Source0:        %{module_name}-%{version}.tar.gz

BuildArch:      noarch

Requires:       dkms
# DKMS builds on the target machine rather than here, so the toolchain is a runtime
# dependency of this package, not a build-time one.
Requires:       gcc
Requires:       make
Requires:       kernel-devel

%description
An ALSA driver for Focusrite's Clarett and Red Thunderbolt audio interfaces. One module
covers the Clarett 2Pre, 4Pre, 8Pre and 8PreX and the Red 8Line, and detects the model at
probe from the stream geometry the device reports.

The module provides PCM capture and playback, DIN MIDI, and an FCP hwdep transport; the
mixer, routing and preamp controls are created in userspace by fcp-server over that
transport, the same split the mainline 4th-generation Scarlett driver uses.

This package installs the source and registers it with DKMS, which rebuilds the module
for each kernel as it is installed.

This driver was produced by clean-room reverse engineering and is not affiliated with or
endorsed by Focusrite.

%prep
%autosetup -n %{module_name}-%{version}

%build
# Nothing to do: DKMS compiles on the target machine, against its running kernel.

%install
mkdir -p %{buildroot}%{_usrsrc}/%{module_name}-%{version}
cp -a . %{buildroot}%{_usrsrc}/%{module_name}-%{version}/
# DKMS builds and installs modules and nothing else, so the WirePlumber naming drop-in is
# installed by the package in its own right.
install -D -m 644 wireplumber/51-clarett-naming.conf \
    %{buildroot}%{_datadir}/wireplumber/wireplumber.conf.d/51-clarett-naming.conf

%post
# --rpm_safe_upgrade keeps an upgrade from tearing down the module the outgoing package's
# %%preun is about to remove, which would otherwise leave the machine with neither version.
dkms add -m %{module_name} -v %{version} --rpm_safe_upgrade || :
dkms build -m %{module_name} -v %{version} || :
dkms install -m %{module_name} -v %{version} --force || :

%preun
dkms remove -m %{module_name} -v %{version} --all --rpm_safe_upgrade || :

%files
%license LICENSE LICENSES/Linux-syscall-note.txt
%doc README.md DEVELOPMENT.md
%{_usrsrc}/%{module_name}-%{version}
%{_datadir}/wireplumber/wireplumber.conf.d/51-clarett-naming.conf
