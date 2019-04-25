#ifndef VBOX_INCLUDED_SRC_VirtIOModern_virtio_h
#define VBOX_INCLUDED_SRC_VirtIOModern_virtio_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
#pragma once
#endif

#include "virtioPCI.h"

#include <array>
#include <iprt/sg.h>
#include <iprt/types.h>

#define VIRTIO_QUEUE_ALIGN 4096
#define VIRTIO_QUEUE_MAX 1024

/* Status byte for guest to report progress, and synchronize features. */
/* We have seen device and processed generic fields (VIRTIO_CONFIG_F_VIRTIO) */
#define VIRTIO_CONFIG_S_ACKNOWLEDGE 1
/* We have found a driver for the device. */
#define VIRTIO_CONFIG_S_DRIVER 2
/* Driver has used its parts of the config, and is happy */
#define VIRTIO_CONFIG_S_DRIVER_OK 4
/* Driver has finished configuring features */
#define VIRTIO_CONFIG_S_FEATURES_OK 8
/* Device entered invalid state, driver must reset it */
#define VIRTIO_CONFIG_S_NEEDS_RESET 0x40
/* We've given up on this device. */
#define VIRTIO_CONFIG_S_FAILED 0x80

#define VIRTIO_F_VERSION_1 32

#define VRING_DESC_F_NEXT 1
#define VRING_DESC_F_WRITE 2
#define VRING_DESC_F_INDIRECT 4
#define VIRTQUEUE_READ_DESC_ERROR -1
#define VIRTQUEUE_READ_DESC_DONE 0
#define VIRTQUEUE_READ_DESC_MORE 1

typedef struct VirtioPCIDevice VirtioPCIDevice;
typedef struct VirtioDevice VirtioDevice;
typedef struct VirtQueue VirtQueue;
typedef void (*VirtioHandleQueue)(VirtioDevice *, VirtQueue *);

typedef struct VirtQueueElement {
  uint32_t index;
  uint32_t out_num;
  uint32_t in_num;
  uint64_t in_addr[VIRTIO_QUEUE_MAX];
  uint64_t out_addr[VIRTIO_QUEUE_MAX];
  RTSGSEG in_sg[VIRTIO_QUEUE_MAX];
  RTSGSEG out_sg[VIRTIO_QUEUE_MAX];
  PGMPAGEMAPLOCK in_lock[VIRTIO_QUEUE_MAX];
  PGMPAGEMAPLOCK out_lock[VIRTIO_QUEUE_MAX];
} VirtQueueElement;

typedef struct VRingDesc {
  uint64_t addr;
  uint32_t len;
  uint16_t flags;
  uint16_t next;
} VRingDesc;

typedef struct VRingAvail {
  uint16_t flags;
  uint16_t idx;
  uint16_t ring[0];
} VRingAvail;

typedef struct VRingUsedElem {
  uint32_t id;
  uint32_t len;
} VRingUsedElem;

typedef struct VRingUsed {
  uint16_t flags;
  uint16_t idx;
  VRingUsedElem ring[0];
} VRingUsed;

typedef struct VRing {
  uint32_t num;
  uint32_t num_default;
  uint32_t align;
  uint64_t desc;
  uint64_t avail;
  uint64_t used;
} VRing;

typedef struct VirtQueue {
  VRing vring;
  uint16_t last_avail_idx;
  uint16_t shadow_avail_idx;
  uint16_t used_idx;
  uint16_t signalled_used;
  bool signalled_used_valid;
  bool notification;
  uint16_t queue_idx;
  uint32_t inuse;
  uint16_t vector;
  VirtioDevice *vdev;
  VirtioHandleQueue handle_queue;
} VirtQueue;

typedef struct VirtioDevice {
  VirtioPCIDevice *pciDev;
  const char *name;
  uint8_t status;
  uint8_t isr;
  uint16_t queue_select;
  uint64_t guest_features;
  uint64_t host_features;
  uint64_t backend_features;
  size_t config_len;
  void *config;
  uint32_t generation;
  std::array<VirtQueue, VIRTIO_QUEUE_MAX> vq;
  uint16_t device_id;
  bool vm_running;
  bool broken;
  uint8_t device_endian;
  void (*virtio_notify_bus)(VirtioDevice *pciDev);
  void (*reset)(VirtioDevice *vdev);
  void (*get_config)(VirtioDevice *vdev, uint8_t *config);
  void (*set_config)(VirtioDevice *vdev, uint8_t *config);
  void (*set_features)(VirtioDevice *vdev, uint64_t features);
  uint64_t (*get_features)(VirtioDevice *vdev, uint64_t features);
} VirtioDevice;

VirtQueue *virtio_add_queue(VirtioDevice *vdev, uint32_t queue_size,
                            VirtioHandleQueue handle_queue);
void virtio_del_queue(VirtioDevice *vdev, int n);
int virtio_queue_get_num(VirtioDevice *vdev, int n);
void virtio_queue_set_num(VirtioDevice *vdev, int n, int num);
int virtio_set_features(VirtioDevice *vdev, uint64_t val);
int virtio_set_status(VirtioDevice *vdev, uint8_t status);
void virtio_add_feature(uint64_t *features, unsigned int feature);
void virtio_queue_notify(VirtioDevice *vdev, int n);
void virtio_queue_set_rings(VirtioDevice *vdev, int n, uint64_t desc,
                            uint64_t avail, uint64_t used);
void virtio_notify(VirtioDevice *vdev, VirtQueue *vq);
void *virtqueue_pop(VirtQueue *vq, size_t sz);
void virtqueue_push(VirtQueue *vq, VirtQueueElement *vqe, unsigned int len);
void virtio_reset(VirtioDevice *vdev);
uint32_t virtio_config_modern_readb(VirtioDevice *vdev, uint32_t addr);
uint32_t virtio_config_modern_readw(VirtioDevice *vdev, uint32_t addr);
uint32_t virtio_config_modern_readl(VirtioDevice *vdev, uint32_t addr);
void virtio_config_modern_writeb(VirtioDevice *vdev, uint32_t addr,
                                 uint32_t data);
void virtio_config_modern_writew(VirtioDevice *vdev, uint32_t addr,
                                 uint32_t data);
void virtio_config_modern_writel(VirtioDevice *vdev, uint32_t addr,
                                 uint32_t data);
#endif // VBOX_INCLUDED_SRC_VirtIOModern_virtio_h