/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * FCP (Focusrite Control Protocol) hwdep user-space ABI.
 *
 * Vendored verbatim from the mainline kernel's include/uapi/sound/fcp.h
 * (Copyright (c) 2024-2025 Geoffrey D. Bennett <g at b4.vu>) so this
 * out-of-tree driver can present the same hwdep interface as sound/usb/fcp.c
 * — the interface Geoffrey Bennett's user-space `fcp-server` drives. Keep in
 * sync with the mainline header; do not diverge (interop depends on it).
 */
#ifndef __CLARETT_FCP_UAPI_H
#define __CLARETT_FCP_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define FCP_HWDEP_MAJOR 2
#define FCP_HWDEP_MINOR 0
#define FCP_HWDEP_SUBMINOR 0

#define FCP_HWDEP_VERSION \
	((FCP_HWDEP_MAJOR << 16) | \
	 (FCP_HWDEP_MINOR << 8) | \
	  FCP_HWDEP_SUBMINOR)

#define FCP_HWDEP_VERSION_MAJOR(v) (((v) >> 16) & 0xFF)
#define FCP_HWDEP_VERSION_MINOR(v) (((v) >> 8) & 0xFF)
#define FCP_HWDEP_VERSION_SUBMINOR(v) ((v) & 0xFF)

/* Get protocol version */
#define FCP_IOCTL_PVERSION _IOR('S', 0x60, int)

/* Start the protocol. Step 0 and step 2 responses are variable length and
 * placed in resp[] one after the other. */
struct fcp_init {
	__u16 step0_resp_size;
	__u16 step2_resp_size;
	__u32 init1_opcode;
	__u32 init2_opcode;
	__u8  resp[];
} __attribute__((packed));
#define FCP_IOCTL_INIT _IOWR('S', 0x64, struct fcp_init)

/* Perform a command. The request data is placed in data[] and the response
 * data will overwrite it. */
struct fcp_cmd {
	__u32 opcode;
	__u16 req_size;
	__u16 resp_size;
	__u8  data[];
} __attribute__((packed));
#define FCP_IOCTL_CMD _IOWR('S', 0x65, struct fcp_cmd)

/* Set the meter map */
struct fcp_meter_map {
	__u16 map_size;
	__u16 meter_slots;
	__s16 map[];
} __attribute__((packed));
#define FCP_IOCTL_SET_METER_MAP _IOW('S', 0x66, struct fcp_meter_map)

/* Set the meter labels */
struct fcp_meter_labels {
	__u16 labels_size;
	char  labels[];
} __attribute__((packed));
#define FCP_IOCTL_SET_METER_LABELS _IOW('S', 0x67, struct fcp_meter_labels)

#endif /* __CLARETT_FCP_UAPI_H */
