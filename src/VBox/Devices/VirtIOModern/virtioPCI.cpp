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
  tmp_cap.cap_len = sizeof(virtio_pci_notify_cap);

  struct virtio_pci_notify_cap notify = {
      tmp_cap,        // cap
      RT_H2LE_U32(4), // notify_off_multiplier
  };

  memcpy(&pci->abConfig[0x70], &notify, sizeof(notify));
}

/**
 * Set PCI configuration space registers.
 *
 * @param   pci          Reference to PCI device structure.
 * @param   uDeviceId    Virtio Device Id
 * @param   uClass       Class of PCI device (network, etc)
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
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint8_t *read_data = static_cast<uint8_t *>(pv);

  *read_data = ASMAtomicXchgU8(&vdev->isr, 0);
  PDMDevHlpPCISetIrq(vpciDev->CTX_SUFF(pDevIns), 0, PDM_IRQ_LEVEL_LOW);

  RT_NOREF(pvUser, GCPhysAddr, cb);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                        void const *pv, unsigned cb) {
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint8_t addr = static_cast<uint8_t>(GCPhysAddr);
  uint32_t write_data = *reinterpret_cast<uint32_t *>(const_cast<void *>(pv));

  switch (cb) {
  case 1:
    virtio_config_modern_writeb(vdev, addr, write_data);
    break;
  case 2:
    virtio_config_modern_writeb(vdev, addr, write_data);
    break;
  case 4:
    virtio_config_modern_writeb(vdev, addr, write_data);
    break;
  default:
    break;
  }

  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCIDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                       void *pv, unsigned cb) {
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint8_t addr = static_cast<uint8_t>(GCPhysAddr);
  uint64_t *read_data = static_cast<uint64_t *>(pv);
  *read_data &= 0x00000000;

  switch (cb) {
  case 1:
    *read_data = virtio_config_modern_readb(vdev, addr);
    break;
  case 2:
    *read_data = virtio_config_modern_readw(vdev, addr);
    break;
  case 4:
    *read_data = virtio_config_modern_readl(vdev, addr);
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
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint64_t addr = *(uint16_t *)pv;

  unsigned queue = addr;
  if (queue < VIRTIO_QUEUE_MAX)
    virtio_queue_notify(vdev, queue);

  RT_NOREF(pvUser, GCPhysAddr, cb);
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
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  vpciDev->mmioRegion = GCPhysAddress;
  
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
  RT_NOREF(enmType, cb, iRegion, pPciDev);
  return rc;
}

void virtioPCIUnmap(VirtioPCIDevice *vpciDev) {
  PDMDevHlpMMIODeregister(vpciDev->pDevInsR3, vpciDev->mmioRegion, 0x4000);
}

void virtioPCIReset(VirtioPCIDevice *vpciDev) {
  VirtioDevice *vdev = vpciDev->vdev;
  for (int i = 0; i < VIRTIO_QUEUE_MAX; i++) {
    vpciDev->vqs[i].enabled = 0;
    vpciDev->vqs[i].num = 0;
    vpciDev->vqs[i].desc[0] = vpciDev->vqs[i].desc[1] = 0;
    vpciDev->vqs[i].avail[0] = vpciDev->vqs[i].avail[1] = 0;
    vpciDev->vqs[i].used[0] = vpciDev->vqs[i].used[1] = 0;
  }
  vdev->status = 0x00;
  if (vdev)
    virtio_reset(vdev);
}

/**
 * Construct PCI-related part of device and configure PCI registers
 *
 * @returns VBox status code.
 * @param   pDevIns      Pointer to the PDM device instance
 * @param   vpciDev      Pointer to the VirtioPCIDevice
 * @param   iInstance    Device instance number
 * @param   pcszNameFmt  Device description
 * @param   uDeviceId    PCI device ID
 * @param   uClass       PCI device class
 * @param   nQueues      Number of VirtQueues this device uses.
 */
int virtioPCIConstruct(PPDMDEVINS pDevIns, VirtioPCIDevice *vpciDev,
                       uint16_t uDeviceId, uint16_t uClass, uint32_t nQueues) {
  vpciDev->pDevInsR3 = pDevIns;

  /* Set PCI config registers */
  virtioPCIConfigure(vpciDev->pciDevice, uDeviceId, uClass);
  virtioPCISetCapabilityList(&vpciDev->pciDevice, CAP_BASE);
  /* Register PCI device */
  int rc = PDMDevHlpPCIRegister(pDevIns, &vpciDev->pciDevice);
  if (RT_FAILURE(rc))
    return rc;
  /* Register required MMIO Regions */
  rc = PDMDevHlpPCIIORegionRegisterEx(
      pDevIns, &vpciDev->pciDevice, 2, 0x0000000000004000,
      PCI_ADDRESS_SPACE_MEM_PREFETCH, virtioPCIMap);

  if (RT_FAILURE(rc))
    return rc;

  vpciDev->num_queues = nQueues;
  virtio_add_feature(&vpciDev->vdev->host_features, VIRTIO_F_VERSION_1);

  if (RT_FAILURE(rc))
    return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the status LUN"));

  return rc;
}

DECLCALLBACK(int)
virtioPCICommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                        void const *pv, unsigned size) {
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint8_t cmd = (uint8_t)GCPhysAddr;
  uint64_t write_data = *(uint64_t *)pv;

  switch (size) {
  case 1:
    write_data = (uint8_t)write_data;
    break;
  case 2:
    write_data = (uint16_t)write_data;
    break;
  case 4:
    write_data = (uint32_t)write_data;
    break;
  default:
    return VWRN_INVALID_PARAMETER;
  }

  switch (cmd) {
  case VIRTIO_PCI_COMMON_DFSELECT:
    vpciDev->device_feature_select = write_data;
    break;
  case VIRTIO_PCI_COMMON_GFSELECT:
    vpciDev->driver_feature_select = write_data;
    break;
  case VIRTIO_PCI_COMMON_GF:
    if (vpciDev->driver_feature_select <= 1) {
      vpciDev->driver_features[vpciDev->driver_feature_select] = write_data;
      virtio_set_features(vdev,
                          (((uint64_t)vpciDev->driver_features[1]) << 32) |
                              vpciDev->driver_features[0]);
    }
    break;
  case VIRTIO_PCI_COMMON_MSIX:
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_STATUS:
    virtio_set_status(vdev, write_data & 0xFF);
    if (vdev->status == 0) {
      virtioPCIReset(vpciDev);
    }
    break;
  case VIRTIO_PCI_COMMON_Q_SELECT:
    if (write_data < VIRTIO_QUEUE_MAX) {
      vdev->queue_select = write_data;
    }
    break;
  case VIRTIO_PCI_COMMON_Q_SIZE:
    vpciDev->vqs[vdev->queue_select].num = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_MSIX:
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_Q_ENABLE:
    virtio_queue_set_num(vdev, vdev->queue_select,
                         vpciDev->vqs[vdev->queue_select].num);
    virtio_queue_set_rings(
        vdev, vdev->queue_select,
        ((uint64_t)vpciDev->vqs[vdev->queue_select].desc[1]) << 32 |
            vpciDev->vqs[vdev->queue_select].desc[0],
        ((uint64_t)vpciDev->vqs[vdev->queue_select].avail[1]) << 32 |
            vpciDev->vqs[vdev->queue_select].avail[0],
        ((uint64_t)vpciDev->vqs[vdev->queue_select].used[1]) << 32 |
            vpciDev->vqs[vdev->queue_select].used[0]);
    vpciDev->vqs[vdev->queue_select].enabled = 1;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCLO:
    vpciDev->vqs[vdev->queue_select].desc[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCHI:
    vpciDev->vqs[vdev->queue_select].desc[1] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILLO:
    vpciDev->vqs[vdev->queue_select].avail[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILHI:
    vpciDev->vqs[vdev->queue_select].avail[1] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_USEDLO:
    vpciDev->vqs[vdev->queue_select].used[0] = write_data;
    break;
  case VIRTIO_PCI_COMMON_Q_USEDHI:
    vpciDev->vqs[vdev->queue_select].used[1] = write_data;
    break;
  }

  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

DECLCALLBACK(int)
virtioPCICommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr,
                       void *pv, unsigned size) {
  VirtioPCIDevice *vpciDev = PDMINS_2_DATA(pDevIns, VirtioPCIDevice *);
  VirtioDevice *vdev = vpciDev->vdev;
  uint8_t cmd = (uint8_t)GCPhysAddr;
  uint64_t *read_data = (uint64_t *)pv;

  Assert((size == 1) | (size == 2) | (size == 4));
  switch (cmd) {
  case VIRTIO_PCI_COMMON_DFSELECT:
    *read_data = vpciDev->device_feature_select;
    break;
  case VIRTIO_PCI_COMMON_DF:
    if (vpciDev->device_feature_select <= 1)
      *read_data = vdev->host_features >> (32 * vpciDev->device_feature_select);
    break;
  case VIRTIO_PCI_COMMON_GFSELECT:
    *read_data = vpciDev->driver_feature_select;
    break;
  case VIRTIO_PCI_COMMON_GF:
    if (vpciDev->driver_feature_select <= 1)
      *read_data = vpciDev->driver_features[vpciDev->driver_feature_select];
    break;
  case VIRTIO_PCI_COMMON_MSIX:
    *read_data = 0;
    break; // MSIX not supported. What to do?
  case VIRTIO_PCI_COMMON_NUMQ:
    for (int i = 0; i < VIRTIO_QUEUE_MAX; i++) {
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
    *read_data = vpciDev->vqs[vdev->queue_select].enabled;
    break;
  case VIRTIO_PCI_COMMON_Q_NOFF:
    *read_data = vdev->queue_select;
    break;
  case VIRTIO_PCI_COMMON_Q_DESCLO:
    *read_data = vpciDev->vqs[vdev->queue_select].desc[0];
    break;
  case VIRTIO_PCI_COMMON_Q_DESCHI:
    *read_data = vpciDev->vqs[vdev->queue_select].desc[1];
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILLO:
    *read_data = vpciDev->vqs[vdev->queue_select].avail[0];
    break;
  case VIRTIO_PCI_COMMON_Q_AVAILHI:
    *read_data = vpciDev->vqs[vdev->queue_select].avail[1];
    break;
  case VIRTIO_PCI_COMMON_Q_USEDLO:
    *read_data = vpciDev->vqs[vdev->queue_select].used[0];
    break;
  case VIRTIO_PCI_COMMON_Q_USEDHI:
    *read_data = vpciDev->vqs[vdev->queue_select].used[1];
    break;
  default:
    *read_data = 0;
  }
  RT_NOREF(pvUser);
  return VINF_SUCCESS;
}

void virtioPCINotify(VirtioDevice *vdev) {
  PDMDevHlpPCISetIrq(vdev->pciDev->CTX_SUFF(pDevIns), 0, PDM_IRQ_LEVEL_HIGH);
}

/**
 * Read from a VirtioPCIDevices physical guest memory
 *
 * @param vpci  VirtioPCIDevice structure to read from
 * @param addr  physical guest address to read from
 * @param buf   buffer to read into
 * @param size  amount of bytes to read
 */
void virtioPCIPhysRead(VirtioPCIDevice *vpci, uint64_t addr, void *buf,
                       size_t size) {
  PDMDevHlpPhysRead(vpci->CTX_SUFF(pDevIns), addr, buf, size);
}

/**
 * Write to a VirtioPCIDevices physical guest memory
 *
 * @param vpci  VirtioPCIDevice structure to write to
 * @param addr  physical guest address to write to
 * @param buf   data to write
 * @param size  amount of bytes write
 */
void virtioPCIPhysWrite(VirtioPCIDevice *vpci, uint64_t addr, void *buf,
                        size_t size) {
  PDMDevHlpPhysWrite(vpci->CTX_SUFF(pDevIns), addr, buf, size);
}

#endif /* VBOX_DEVICE_STRUCT_TESTCASE */
