/* $Id: Virtio.h $ */
/** @file
 * VirtioModern.h - Virtio Declarations
 * Based on Virtio.h
 */

/*
 * Copyright (C) 2009-2019 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef VBOX_INCLUDED_SRC_VirtIOModern_VirtioModern_h
#define VBOX_INCLUDED_SRC_VirtIOModern_VirtioModern_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

#include <iprt/ctype.h>

#define DEVICE_PCI_VENDOR_ID                0x1AF4
#define DEVICE_PCI_BASE_ID                  0x1040
#define DEVICE_PCI_SUBSYSTEM_VENDOR_ID      0x1AF4
#define DEVICE_PCI_SUBSYSTEM_BASE_ID        1

#define VIRTIO_QUEUE_MAX 1024
/* common configuration offsets */
#define VIRTIO_PCI_COMMON_DFSELECT	0
#define VIRTIO_PCI_COMMON_DF		4
#define VIRTIO_PCI_COMMON_GFSELECT	8
#define VIRTIO_PCI_COMMON_GF		12
#define VIRTIO_PCI_COMMON_MSIX		16
#define VIRTIO_PCI_COMMON_NUMQ		18
#define VIRTIO_PCI_COMMON_STATUS	20
#define VIRTIO_PCI_COMMON_CFGGENERATION	21
#define VIRTIO_PCI_COMMON_Q_SELECT	22
#define VIRTIO_PCI_COMMON_Q_SIZE	24
#define VIRTIO_PCI_COMMON_Q_MSIX	26
#define VIRTIO_PCI_COMMON_Q_ENABLE	28
#define VIRTIO_PCI_COMMON_Q_NOFF	30
#define VIRTIO_PCI_COMMON_Q_DESCLO	32
#define VIRTIO_PCI_COMMON_Q_DESCHI	36
#define VIRTIO_PCI_COMMON_Q_AVAILLO	40
#define VIRTIO_PCI_COMMON_Q_AVAILHI	44
#define VIRTIO_PCI_COMMON_Q_USEDLO	48
#define VIRTIO_PCI_COMMON_Q_USEDHI	52

/* Common configuration */
#define VIRTIO_PCI_CAP_COMMON_CFG 1
/* Notifications */
#define VIRTIO_PCI_CAP_NOTIFY_CFG 2
/* ISR Status */
#define VIRTIO_PCI_CAP_ISR_CFG 3
/* Device specific configuration */
#define VIRTIO_PCI_CAP_DEVICE_CFG 4
/* PCI configuration access */
#define VIRTIO_PCI_CAP_PCI_CFG 5

#define CAP_BASE 0x40

struct virtio_pci_cap {
    uint8_t cap_vndr; /* Generic PCI field: PCI_CAP_ID_VNDR */
    uint8_t cap_next; /* Generic PCI field: next ptr. */
    uint8_t cap_len; /* Generic PCI field: capability length */
    uint8_t cfg_type; /* Identifies the structure. */
    uint8_t bar; /* Where to find it. */
    uint8_t padding[3]; /* Pad to full dword. */
    uint32_t offset; /* Offset within bar. */
    uint32_t length; /* Length of the structure, in bytes. */
};

typedef struct VPCIQueue {
    uint16_t size;
    bool enabled;
    uint32_t desc[2];
    uint32_t avail[2];
    uint32_t used[2];
} VPCIQueue;

/**
 * The core (/common) state of the VirtIO PCI device
 *
 */
typedef struct VirtioPCIState_st
{
    PDMCRITSECT            cs;      /**< Critical section - what is it protecting? */
    /* Read-only part, never changes after initialization. */
    char                   szInstance[8];         /**< Instance name, e.g. VNet#1. */

#if HC_ARCH_BITS != 64
    uint32_t               padding1;
#endif

    /** Status LUN: Base interface. */
    PDMIBASE               IBase;

    PPDMDEVINSR3           pDevInsR3;                   /**< Device instance - R3. */
    PPDMDEVINSR0           pDevInsR0;                   /**< Device instance - R0. */
    PPDMDEVINSRC           pDevInsRC;                   /**< Device instance - RC. */

#if HC_ARCH_BITS == 64
    uint32_t               padding2;
#endif

    /** TODO */
    PDMPCIDEV              pciDevice;

#if HC_ARCH_BITS != 64
    uint32_t               padding3;
#endif

    uint32_t device_feature_select;
    uint32_t device_feature;
    uint32_t driver_feature_select;
    uint32_t driver_feature;
    uint16_t msix_config;
    uint16_t num_queues;
    uint8_t device_status;
    uint8_t config_generation;

    uint16_t queue_select;
    uint16_t queue_msix_vector;
    uint16_t queue_notify_off;
    VPCIQueue vqs[VIRTIO_QUEUE_MAX];
} VirtioPCIState;

DECLCALLBACK(int) virtioModernMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                  RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType);

DECLCALLBACK(void) virtioPCIConfigure(PDMPCIDEV& pci, uint16_t uDeviceId, uint16_t uClass);
void virtioPCISetCapabilityList(PPDMPCIDEV pci, uint8_t cap_base);

int virtioPCIConstruct(PPDMDEVINS pDevIns, VirtioPCIState *pState, int iInstance, const char *pcszNameFmt, 
                  uint16_t uDeviceId, uint16_t uClass, uint32_t nQueues);
int   virtioPCIDestruct(VirtioPCIState* pState);


DECLCALLBACK(int) virtioModernCommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
DECLCALLBACK(int) virtioModernCommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
DECLCALLBACK(int) virtioModernISRWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
DECLCALLBACK(int) virtioModernISRRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
DECLCALLBACK(int) virtioModernDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
DECLCALLBACK(int) virtioModernDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
DECLCALLBACK(int) virtioModernNotifyWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
DECLCALLBACK(int) virtioModernNotifyRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);

#endif /* !VBOX_INCLUDED_SRC_VirtIOModern_VirtioModern_h */
