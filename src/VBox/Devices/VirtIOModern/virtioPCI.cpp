/* $Id: VirtioModern.cpp $ */
/** @file
 * Virtio - Virtio Common Functions (VRing, VQueue, Virtio PCI)
 * Based on Virtio.cpp
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


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_VIRTIO

#include <iprt/param.h>
#include <iprt/uuid.h>
#include <VBox/vmm/pdmdev.h>
#include "VirtioModern.h"

#ifndef VBOX_DEVICE_STRUCT_TESTCASE

/**
 * Set and populate the PCI capability list
 * 
 * Adds pci capabilities according to VIRTIO standard
 * 
 * @param   pci      Reference to PCI device structure
 * @param   cap_base location of the first capability
 */
void vpciSetCapabilityList(PPDMPCIDEV pci, uint8_t cap_base) {
    PDMPciDevSetStatus(pci, VBOX_PCI_STATUS_CAP_LIST);
    PDMPciDevSetCapabilityList(pci, cap_base);
    struct virtio_pci_cap tmp_cap = { 
        VBOX_PCI_CAP_ID_VNDR,       //cap_vndr
        cap_base+0x10,              //cap_next
        sizeof(virtio_pci_cap),     //cap_len
        VIRTIO_PCI_CAP_COMMON_CFG,  //cfg_type
        2,                          //bar
        {0,0,0},                    //padding
        RT_H2LE_U32(0x00000000),    //offset
        RT_H2LE_U32(0x00000800),    //length
    };
    memcpy(&pci->abConfig[0x40], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = cap_base+0x20;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_ISR_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00001000);
    tmp_cap.length = RT_H2LE_U32(0x00000800);
    memcpy(&pci->abConfig[0x50], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = cap_base+0x30;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_DEVICE_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00002000);
    tmp_cap.length = RT_H2LE_U32(0x00001000);
    memcpy(&pci->abConfig[0x60], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = 0x0;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_NOTIFY_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00003000);
    tmp_cap.length = RT_H2LE_U32(0x00001000);
    memcpy(&pci->abConfig[0x70], &tmp_cap, sizeof(tmp_cap));
}

/**
 * Set PCI configuration space registers.
 *
 * @param   pci          Reference to PCI device structure.
 * @param   uDeviceId    VirtiO Device Id
 * @param   uClass       Class of PCI device (network, etc)
 * @thread  EMT
 */
static DECLCALLBACK(void) vpciConfigure(PDMPCIDEV& pci,
                                        uint16_t uDeviceId,
                                        uint16_t uClass)
{
    /* Configure PCI Device, assume 32-bit mode ******************************/
    PDMPciDevSetVendorId(&pci, DEVICE_PCI_VENDOR_ID);
    PDMPciDevSetDeviceId(&pci, DEVICE_PCI_BASE_ID + uDeviceId);
    PDMPciDevSetSubSystemVendorId(&pci, DEVICE_PCI_SUBSYSTEM_VENDOR_ID);
    PDMPciDevSetSubSystemId(&pci, DEVICE_PCI_SUBSYSTEM_BASE_ID + uDeviceId);
    PDMPciDevSetWord(&pci, VBOX_PCI_CLASS_DEVICE, uClass);

    PDMPciDevSetRevisionId(&pci, 0x01);
    PDMPciDevSetClassProg(&pci, 0x00);
    /* Interrupt Pin: INTA# */
    PDMPciDevSetInterruptPin(&pci, 0x01);

    PDMPciDevSetBaseAddress(&pci, 2, false, true, true, 0x00000000);
}

PDMBOTHCBDECL(int) virtioModernISRWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernISRRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernNotifyWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernNotifyRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}

DECLCALLBACK(int) virtioModernMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType) {
    VPCISTATE *pThis = PDMINS_2_DATA(pDevIns, VPCISTATE *);
    
    int rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x0000, 0x1000, NULL, 0,
                                   virtioModernCommonCfgWrite, virtioModernCommonCfgRead, "VirtioPCICommonCfg");
    Assert(rc == VINF_SUCCESS);

    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x1000, 0x1000, NULL, 0,
                                   virtioModernISRWrite, virtioModernISRRead, "VirtioPCIISR");
    Assert(rc == VINF_SUCCESS);

    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x2000, 0x1000, NULL, 0,
                                   virtioModernDeviceCfgWrite, virtioModernDeviceCfgRead, "VirtioPCIDeviceCfg");
    Assert(rc == VINF_SUCCESS);
    
    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x3000, 0x1000, NULL, 0,
                                   virtioModernNotifyWrite, virtioModernNotifyRead, "VirtioPCINotify");
    Assert(rc == VINF_SUCCESS);
    RT_NOREF(enmType, cb, iRegion, pPciDev, pThis);
    return rc;
}

/// @todo header
int vpciConstruct(PPDMDEVINS pDevIns, VPCISTATE *pState,
                  int iInstance, const char *pcszNameFmt,
                  uint16_t uDeviceId, uint16_t uClass,
                  uint32_t nQueues)
{
    /* Init handles and log related stuff. */
    RTStrPrintf(pState->szInstance, sizeof(pState->szInstance),
                pcszNameFmt, iInstance);

    pState->pDevInsR3    = pDevIns;
    pState->pDevInsR0    = PDMDEVINS_2_R0PTR(pDevIns);
    pState->pDevInsRC    = PDMDEVINS_2_RCPTR(pDevIns);

    /* Initialize critical section. */
    int rc = PDMDevHlpCritSectInit(pDevIns, &pState->cs, RT_SRC_POS, "%s", pState->szInstance);
    if (RT_FAILURE(rc))
        return rc;

    /* Set PCI config registers */
    vpciConfigure(pState->pciDevice, uDeviceId, uClass);
    vpciSetCapabilityList(&pState->pciDevice, CAP_BASE);
    /* Register PCI device */
    rc = PDMDevHlpPCIRegister(pDevIns, &pState->pciDevice);
    if (RT_FAILURE(rc))
        return rc;
    /* Register required MMIO Regions */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 2, 0x0000000000004000, PCI_ADDRESS_SPACE_MEM_PREFETCH, virtioModernMap);
    if (RT_FAILURE(rc))
        return rc;

    /* Status driver */
    PPDMIBASE pBase;
    rc = PDMDevHlpDriverAttach(pDevIns, PDM_STATUS_LUN, &pState->IBase, &pBase, "Status Port");
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the status LUN"));

    pState->num_queues = nQueues;

    return rc;
}

/**
 * Destruct PCI-related part of device.
 *
 * We need to free non-VM resources only.
 *
 * @returns VBox status code.
 * @param   pState      The device state structure.
 */
int vpciDestruct(VPCISTATE* pState)
{
    Log(("%s Destroying PCI instance\n", pState->szInstance));

    if (PDMCritSectIsInitialized(&pState->cs))
        PDMR3CritSectDelete(&pState->cs);

    return VINF_SUCCESS;
}

PDMBOTHCBDECL(int) virtioModernCommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned size)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    uint8_t cmd = (uint8_t) GCPhysAddr;
    uint64_t write_data = *(uint64_t *)pv;
    Assert((size == 1) | (size == 2) | (size == 4));
    switch(cmd) {
        case VIRTIO_PCI_COMMON_DFSELECT: 
            pState->device_feature_select = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_GFSELECT: 
            pState->driver_feature_select = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_GF: 
            pState->driver_feature = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_MSIX: 
            Assert(0); break; //MSIX not supported. What to do?
        case VIRTIO_PCI_COMMON_STATUS: 
            pState->device_status = (uint8_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_SELECT: 
            pState->queue_select = (uint16_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_SIZE: 
            pState->vqs[pState->queue_select].size = (uint16_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_MSIX: 
            Assert(0); break; //MSIX not supported. What to do?
            break;
        case VIRTIO_PCI_COMMON_Q_ENABLE: 
            pState->vqs[pState->queue_select].enabled = write_data & 0x00000001;
            break;
        case VIRTIO_PCI_COMMON_Q_DESCLO: 
            pState->vqs[pState->queue_select].desc[0] = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_DESCHI: 
            pState->vqs[pState->queue_select].desc[1] = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_AVAILLO: 
            pState->vqs[pState->queue_select].avail[0] = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_AVAILHI: 
            pState->vqs[pState->queue_select].avail[1] = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_USEDLO: 
            pState->vqs[pState->queue_select].used[0] = (uint32_t) write_data;
            break;
        case VIRTIO_PCI_COMMON_Q_USEDHI: 
            pState->vqs[pState->queue_select].used[1] = (uint32_t) write_data;
            break;
    }

    RT_NOREF(pvUser);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernCommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned size)
{
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    uint8_t cmd = (uint8_t) GCPhysAddr;
    uint64_t read_data = *(uint64_t *)pv;
    Assert((size == 1) | (size == 2) | (size == 4));
    switch(cmd) {
        case VIRTIO_PCI_COMMON_DFSELECT: 
            read_data = pState->device_feature_select;
            break;
        case VIRTIO_PCI_COMMON_DF:
            read_data = pState->device_feature;
            break;
        case VIRTIO_PCI_COMMON_GFSELECT: 
            read_data = pState->driver_feature_select;
            break;
        case VIRTIO_PCI_COMMON_GF: 
            read_data = pState->driver_feature;
            break;
        case VIRTIO_PCI_COMMON_MSIX: 
            Assert(0); break; //MSIX not supported. What to do?
        case VIRTIO_PCI_COMMON_NUMQ:
            read_data = pState->num_queues;
            break;
        case VIRTIO_PCI_COMMON_STATUS: 
            read_data = pState->device_status;
            break;
        case VIRTIO_PCI_COMMON_CFGGENERATION: 
            read_data = pState->config_generation;
            break;
        case VIRTIO_PCI_COMMON_Q_SELECT: 
            read_data = pState->queue_select;
            break;
        case VIRTIO_PCI_COMMON_Q_SIZE: 
            read_data = pState->vqs[pState->queue_select].size;
            break;
        case VIRTIO_PCI_COMMON_Q_MSIX: 
            Assert(0); break; //MSIX not supported. What to do?
        case VIRTIO_PCI_COMMON_Q_ENABLE: 
            read_data = pState->vqs[pState->queue_select].enabled;
            break;
        case VIRTIO_PCI_COMMON_Q_NOFF:
            read_data = pState->queue_select;
            break; 
        case VIRTIO_PCI_COMMON_Q_DESCLO: 
            read_data = pState->vqs[pState->queue_select].desc[0];
            break;
        case VIRTIO_PCI_COMMON_Q_DESCHI: 
            read_data = pState->vqs[pState->queue_select].desc[1];
            break;
        case VIRTIO_PCI_COMMON_Q_AVAILLO: 
            read_data = pState->vqs[pState->queue_select].avail[0];
            break;
        case VIRTIO_PCI_COMMON_Q_AVAILHI: 
            read_data = pState->vqs[pState->queue_select].avail[1];
            break;
        case VIRTIO_PCI_COMMON_Q_USEDLO: 
            read_data = pState->vqs[pState->queue_select].used[0];
            break;
        case VIRTIO_PCI_COMMON_Q_USEDHI: 
            read_data = pState->vqs[pState->queue_select].used[1];
            break;
    }
    RT_NOREF(pState, pvUser, GCPhysAddr, pv);
    return read_data;
}

#endif /* VBOX_DEVICE_STRUCT_TESTCASE */

