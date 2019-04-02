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

#ifndef VBOX_INCLUDED_SRC_VirtIO_Virtio_h
#define VBOX_INCLUDED_SRC_VirtIO_Virtio_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

#include <iprt/ctype.h>


/** @name Saved state versions.
 * The saved state version is changed if either common or any of specific
 * parts are changed. That is, it is perfectly possible that the version
 * of saved vnet state will increase as a result of change in vblk structure
 * for example.
 */
#define VIRTIO_SAVEDSTATE_VERSION_3_1_BETA1 1
#define VIRTIO_SAVEDSTATE_VERSION           2
/** @} */

#define DEVICE_PCI_VENDOR_ID                0x1AF4
#define DEVICE_PCI_BASE_ID                  0x1040
#define DEVICE_PCI_SUBSYSTEM_VENDOR_ID      0x1AF4
#define DEVICE_PCI_SUBSYSTEM_BASE_ID        1

#define VIRTIO_MAX_NQUEUES                  3

#define VPCI_HOST_FEATURES                  0x0
#define VPCI_GUEST_FEATURES                 0x4
#define VPCI_QUEUE_PFN                      0x8
#define VPCI_QUEUE_NUM                      0xC
#define VPCI_QUEUE_SEL                      0xE
#define VPCI_QUEUE_NOTIFY                   0x10
#define VPCI_STATUS                         0x12
#define VPCI_ISR                            0x13
#define VPCI_CONFIG                         0x14

#define VPCI_ISR_QUEUE                      0x1
#define VPCI_ISR_CONFIG                     0x3

#define VPCI_STATUS_ACK                     0x01
#define VPCI_STATUS_DRV                     0x02
#define VPCI_STATUS_DRV_OK                  0x04
#define VPCI_STATUS_FEATURES_OK             0x08
#define VPCI_STATUS_NEEDS_RESET             0x40
#define VPCI_STATUS_FAILED                  0x80

#define VPCI_F_NOTIFY_ON_EMPTY              0x01000000
#define VPCI_F_ANY_LAYOUT                   0x08000000
#define VPCI_F_RING_INDIRECT_DESC           0x10000000
#define VPCI_F_RING_EVENT_IDX               0x20000000
#define VPCI_F_BAD_FEATURE                  0x40000000

#define VIRTIO_F_RING_INDIRECT_DESC         0x1C
#define VIRTIO_F_RING_EVENT_IDX             0x1D
#define VIRTIO_F_VERSION_1                  0x20
#define VRINGDESC_MAX_SIZE                  (2 * 1024 * 1024)
#define VRINGDESC_F_NEXT                    0x01
#define VRINGDESC_F_WRITE                   0x02
#define VRINGDESC_F_INDIRECT                0x04

#define MAX_NUM_QUEUES 1024

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

typedef struct VPICQueue {
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
typedef struct VPCIState_st
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
    VPCIQueue vqs[MAX_NUM_QUEUES];
} VPCISTATE;
/** Pointer to the core (/common) state of a VirtIO PCI device. */
typedef VPCISTATE *PVPCISTATE;

/** @name VirtIO port I/O callbacks.
 * @{ */
typedef struct VPCIIOCALLBACKS
{
     DECLCALLBACKMEMBER(uint32_t, pfnGetHostFeatures)(void *pvState);
     DECLCALLBACKMEMBER(void,     pfnSetHostFeatures)(void *pvState, uint32_t fFeatures);
     DECLCALLBACKMEMBER(int,      pfnGetConfig)(void *pvState, uint32_t offCfg, uint32_t cb, void *pvData);
     DECLCALLBACKMEMBER(int,      pfnSetConfig)(void *pvState, uint32_t offCfg, uint32_t cb, void *pvData);
     DECLCALLBACKMEMBER(int,      pfnReset)(void *pvState);
     DECLCALLBACKMEMBER(void,     pfnReady)(void *pvState);
} VPCIIOCALLBACKS;
/** Pointer to a const VirtIO port I/O callback structure. */
typedef const VPCIIOCALLBACKS *PCVPCIIOCALLBACKS;
/** @} */

void vpciSetCapabilityList(PPDMPCIDEV pci, uint8_t cap_base);


int   vpciConstruct(PPDMDEVINS pDevIns, VPCISTATE *pState, int iInstance, const char *pcszNameFmt,
                    uint16_t uDeviceId, uint16_t uClass, uint32_t nQueues);
int   vpciDestruct(VPCISTATE* pState);


PDMBOTHCBDECL(int) virtioModernCommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernCommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernISRWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernISRRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernNotifyWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb);
PDMBOTHCBDECL(int) virtioModernNotifyRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb);

DECLCALLBACK(int) virtioModernMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                         RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType);



#endif /* !VBOX_INCLUDED_SRC_VirtIO_Virtio_h */
