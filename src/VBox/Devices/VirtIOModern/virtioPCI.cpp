/* $Id: virtioPCI.cpp $ */
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
 *   Header Files *
 *********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_VIRTIO

#include "virtioPCI.h"
#include "virtio.h"
#include <iprt/param.h>
#include <iprt/uuid.h>

#ifndef VBOX_DEVICE_STRUCT_TESTCASE

/**
 * Set and populate the PCI capability list
 *
 * Adds pci capabilities according to VIRTIO standard
 *
 * @param   pci      Reference to PCI device structure
 * @param   cap_base location of the first capability
 */
void virtioPCISetCapabilityList(PPDMPCIDEV pci, uint8_t cap_base) {
  PDMPciDevSetStatus(pci, VBOX_PCI_STATUS_CAP_LIST);
  PDMPciDevSetCapabilityList(pci, cap_base);
  struct virtio_pci_cap tmp_cap = {
      VBOX_PCI_CAP_ID_VNDR,      // cap_vndr
      cap_base + 0x10,           // cap_next
      sizeof(virtio_pci_cap),    // cap_len
      VIRTIO_PCI_CAP_COMMON_CFG, // cfg_type
      2,                         // bar
      {0, 0, 0},                 // padding
      RT_H2LE_U32(0x00000000),   // offset
      RT_H2LE_U32(0x00000800),   // length
  };
  memcpy(&pci->abConfig[0x40], &tmp_cap, sizeof(tmp_cap));

  tmp_cap.cap_next = cap_base + 0x20;
  tmp_cap.cfg_type = VIRTIO_PCI_CAP_ISR_CFG;
  tmp_cap.offset = RT_H2LE_U32(0x00001000);
  tmp_cap.length = RT_H2LE_U32(0x00000800);
  memcpy(&pci->abConfig[0x50], &tmp_cap, sizeof(tmp_cap));

  tmp_cap.cap_next = cap_base + 0x30;
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
DECLCALLBACK(void)
virtioPCIConfigure(PDMPCIDEV &pci, uint16_t uDeviceId, uint16_t uClass) {
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

DECLCALLBACK(int)
virtioPCIISRWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                  void const *pv, unsigned cb) {
  RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIISRRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                 void *pv, unsigned cb) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                        void const *pv, unsigned cb) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  VirtioDevice *vdev = pState->vdev;
  uint8_t addr = static_cast<uint8_t>(GCPhysAddr);
  uint64_t write_data = *(uint64_t *)pv;
  uint8_t *cfg = static_cast<uint8_t *>(vdev->config);

  if (addr + cb > vdev->config_len) {
    return 0;
  }

  switch (cb) {
  case 2:
    write_data = RT_LE2H_U16(write_data);
    break;
  case 4:
    write_data = RT_LE2H_U32(write_data);
    break;
  default:
    break;
  }

  memcpy(cfg + addr, &write_data, cb);

  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                       void *pv, unsigned cb) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  VirtioDevice *vdev = pState->vdev;
  uint8_t addr = static_cast<uint8_t>(GCPhysAddr);
  uint64_t *read_data = static_cast<uint64_t *>(pv);
  uint8_t *cfg = static_cast<uint8_t *>(vdev->config);

  if (addr + cb > vdev->config_len) {
    return 0;
  }

  memcpy(read_data, cfg + addr, cb);
  switch (cb) {
  case 2:
    *read_data = RT_H2LE_U16(*read_data);
    break;
  case 4:
    *read_data = RT_H2LE_U32(*read_data);
    break;
  default:
    break;
  }

  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCINotifyWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                     void const *pv, unsigned cb) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  RT_NOREF(pState, pvUser, GCPhysAddr, pv, cb);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCINotifyRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                    void *pv, unsigned cb) {
  RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
             RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType) {
  VirtioPCIState *pThis = PDMINS_2_DATA(pDevIns, VirtioPCIState *);

  int rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress + 0x0000, 0x1000, NULL,
                                 0, virtioPCICommonCfgWrite,
                                 virtioPCICommonCfgRead, "VirtioPCICommonCfg");
  Assert(rc == VINF_SUCCESS);

  rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress + 0x1000, 0x1000, NULL, 0,
                             virtioPCIISRWrite, virtioPCIISRRead,
                             "VirtioPCIISR");
  Assert(rc == VINF_SUCCESS);

  rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress + 0x2000, 0x1000, NULL, 0,
                             virtioPCIDeviceCfgWrite, virtioPCIDeviceCfgRead,
                             "VirtioPCIDeviceCfg");
  Assert(rc == VINF_SUCCESS);

  rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress + 0x3000, 0x1000, NULL, 0,
                             virtioPCINotifyWrite, virtioPCINotifyRead,
                             "VirtioPCINotify");
  Assert(rc == VINF_SUCCESS);
  RT_NOREF(enmType, cb, iRegion, pPciDev, pThis);
  return rc;
}

void virtioPCIReset(VirtioPCIState *pState) {
  VirtioDevice *vdev = pState->vdev;

  for (int i = 0; i < VIRTIO_QUEUE_MAX; i++) {
    pState->vqs[i].enabled = 0;
    pState->vqs[i].num = 0;
    pState->vqs[i].desc[0] = pState->vqs[i].desc[1] = 0;
    pState->vqs[i].avail[0] = pState->vqs[i].avail[1] = 0;
    pState->vqs[i].used[0] = pState->vqs[i].used[1] = 0;
  }
  vdev->status = 0x00;
}

/// @todo header
int virtioPCIConstruct(PPDMDEVINS pDevIns, VirtioPCIState *pState,
                       int iInstance, const char *pcszNameFmt,
                       uint16_t uDeviceId, uint16_t uClass, uint32_t nQueues) {
  /* Init handles and log related stuff. */
  RTStrPrintf(pState->szInstance, sizeof(pState->szInstance), pcszNameFmt,
              iInstance);

  pState->pDevInsR3 = pDevIns;
  pState->pDevInsR0 = PDMDEVINS_2_R0PTR(pDevIns);
  pState->pDevInsRC = PDMDEVINS_2_RCPTR(pDevIns);

  /* Initialize critical section. */
  int rc = PDMDevHlpCritSectInit(pDevIns, &pState->cs, RT_SRC_POS, "%s",
                                 pState->szInstance);
  if (RT_FAILURE(rc))
    return rc;

  /* Set PCI config registers */
  virtioPCIConfigure(pState->pciDevice, uDeviceId, uClass);
  virtioPCISetCapabilityList(&pState->pciDevice, CAP_BASE);
  /* Register PCI device */
  rc = PDMDevHlpPCIRegister(pDevIns, &pState->pciDevice);
  if (RT_FAILURE(rc))
    return rc;
  /* Register required MMIO Regions */
  rc = PDMDevHlpPCIIORegionRegister(pDevIns, 2, 0x0000000000004000,
                                    PCI_ADDRESS_SPACE_MEM_PREFETCH,
                                    virtioPCIMap);
  if (RT_FAILURE(rc))
    return rc;

  /* Status driver */
  PPDMIBASE pBase;
  pState->num_queues = nQueues;
  virtio_add_feature(&pState->vdev->host_features, VIRTIO_F_VERSION_1);
  rc = PDMDevHlpDriverAttach(pDevIns, PDM_STATUS_LUN, &pState->IBase, &pBase,
                             "Status Port");

  if (RT_FAILURE(rc))
    return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the status LUN"));

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
int virtioPCIDestruct(VirtioPCIState *pState) {
  Log(("%s Destroying PCI instance\n", pState->szInstance));

  if (PDMCritSectIsInitialized(&pState->cs))
    PDMR3CritSectDelete(&pState->cs);

  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCICommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                        void const *pv, unsigned size) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  VirtioDevice *vdev = pState->vdev;
  uint8_t cmd = (uint8_t)GCPhysAddr;
  uint64_t write_data = *(uint64_t *)pv;
  Assert((size == 1) | (size == 2) | (size == 4));
  switch (cmd) {
  case VIRTIO_PCI_COMMON_DFSELECT:
    pState->device_feature_select = write_data;
    break;
  case VIRTIO_PCI_COMMON_GFSELECT:
    pState->driver_feature_select = write_data;
    break;
  case VIRTIO_PCI_COMMON_GF:
    if (pState->driver_feature_select <= 1) {
      pState->driver_features[pState->driver_feature_select] = write_data;
      virtio_set_features(vdev, (((uint64_t)pState->driver_features[1]) << 32) |
                                    pState->driver_features[0]);
    }
    break;
  case VIRTIO_PCI_COMMON_MSIX:
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_STATUS:
    virtio_set_status(vdev, write_data & 0xFF);
    if (vdev->status == 0)
      virtioPCIReset(pState);
    break;
  case VIRTIO_PCI_COMMON_Q_SELECT:
    pState->queue_select = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_SIZE:
    pState->vqs[pState->queue_select].num = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX:
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_Q_ENABLE:
    virtio_queue_set_num(vdev, vdev->queue_select,
                         pState->vqs[vdev->queue_select].num);
    // TODO: update VRINGs
    pState->vqs[vdev->queue_select].enabled = 1;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCLO:
    pState->vqs[pState->queue_select].desc[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCHI:
    pState->vqs[pState->queue_select].desc[1] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILLO:
    pState->vqs[pState->queue_select].avail[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILHI:
    pState->vqs[pState->queue_select].avail[1] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_USEDLO:
    pState->vqs[pState->queue_select].used[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_USEDHI:
    pState->vqs[pState->queue_select].used[1] = write_data;
    break;
  }

  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCICommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                       void *pv, unsigned size) {
  VirtioPCIState *pState = PDMINS_2_DATA(pDevIns, VirtioPCIState *);
  VirtioDevice *vdev = pState->vdev;
  uint8_t cmd = (uint8_t)GCPhysAddr;
  uint64_t *read_data = (uint64_t *)pv;

  Assert((size == 1) | (size == 2) | (size == 4));
  switch (cmd) {
  case VIRTIO_PCI_COMMON_DFSELECT:
    *read_data = pState->device_feature_select;
    break;
  case VIRTIO_PCI_COMMON_DF:
    if (pState->device_feature_select <= 1)
      *read_data = vdev->host_features >> (32 * pState->device_feature_select);
    break;
  case VIRTIO_PCI_COMMON_GFSELECT:
    *read_data = pState->driver_feature_select;
    break;
  case VIRTIO_PCI_COMMON_GF:
    if (pState->driver_feature_select <= 1)
      *read_data = pState->driver_features[pState->driver_feature_select];
    break;
  case VIRTIO_PCI_COMMON_MSIX:
    *read_data = 0;
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_NUMQ:
    for (int i = 0; i < 1024; i++) {
      if (virtio_queue_get_num(vdev, i)) {
        *read_data = i + 1;
      }
    }
    break;
  case VIRTIO_PCI_COMMON_STATUS:
    *read_data = vdev->status;
    break;
  case VIRTIO_PCI_COMMON_CFGGENERATION:
    *read_data = vdev->generation;
    break;
  case VIRTIO_PCI_COMMON_Q_SELECT:
    *read_data = vdev->queue_select;
    break;
  case VIRTIO_PCI_COMMON_Q_SIZE:
    *read_data = virtio_queue_get_num(vdev, vdev->queue_select);
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX:
    *read_data = 0;
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_Q_ENABLE:
    *read_data = pState->vqs[vdev->queue_select].enabled;
    break;
  case VIRTIO_PCI_COMMON_Q_NOFF:
    *read_data = vdev->queue_select;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCLO:
    *read_data = pState->vqs[vdev->queue_select].desc[0];
    break;
  case VIRTIO_PCI_COMMON_Q_DESCHI:
    *read_data = pState->vqs[vdev->queue_select].desc[1];
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILLO:
    *read_data = pState->vqs[vdev->queue_select].avail[0];
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILHI:
    *read_data = pState->vqs[vdev->queue_select].avail[1];
    break;
  case VIRTIO_PCI_COMMON_Q_USEDLO:
    *read_data = pState->vqs[vdev->queue_select].used[0];
    break;
  case VIRTIO_PCI_COMMON_Q_USEDHI:
    *read_data = pState->vqs[vdev->queue_select].used[1];
    break;
  default:
    *read_data = 0;
  }
  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

#endif /* VBOX_DEVICE_STRUCT_TESTCASE */
