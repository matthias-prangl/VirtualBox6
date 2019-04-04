#ifndef VBOX_INCLUDED_SRC_VirtIOModern_virtio_h
#define VBOX_INCLUDED_SRC_VirtIOModern_virtio_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

#include <iprt/types.h>
#include <iprt/sg.h>

#define VIRTIO_QUEUE_ALIGN          4096
#define VIRTIO_QUEUE_MAX            1024

/* Status byte for guest to report progress, and synchronize features. */
/* We have seen device and processed generic fields (VIRTIO_CONFIG_F_VIRTIO) */
#define VIRTIO_CONFIG_S_ACKNOWLEDGE	1
/* We have found a driver for the device. */
#define VIRTIO_CONFIG_S_DRIVER		2
/* Driver has used its parts of the config, and is happy */
#define VIRTIO_CONFIG_S_DRIVER_OK	4
/* Driver has finished configuring features */
#define VIRTIO_CONFIG_S_FEATURES_OK	8
/* Device entered invalid state, driver must reset it */
#define VIRTIO_CONFIG_S_NEEDS_RESET	0x40
/* We've given up on this device. */
#define VIRTIO_CONFIG_S_FAILED		0x80

#define VIRTIO_F_VERSION_1		32

typedef struct VirtioPCIState VirtioPCIState;
typedef struct VirtioDevice VirtioDevice;
typedef struct VirtQueue VirtQueue;
typedef void (*VirtioHandleQueue)(VirtioDevice *, VirtQueue *);

typedef struct VirtQueueElement {
    uint32_t index;
    uint32_t out_num;
    uint32_t in_num;
    uint64_t *in_addr;
    uint64_t *out_addr;
    RTSGBUF *in_sg;
    RTSGBUF *out_sg;
} VirtQueueElement;


typedef struct VRingDesc
{
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} VRingDesc;

typedef struct VRingAvail
{
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[0];
} VRingAvail;

typedef struct VRingUsedElem
{
    uint32_t id;
    uint32_t len;
} VRingUsedElem;

typedef struct VRingUsed
{
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
    VirtioPCIState *pciDev;
    const char *name;
    uint8_t status;
    uint8_t isr;
    uint16_t queue_select;
    uint64_t guest_features;
    uint64_t host_features;
    uint64_t backend_features;
    size_t config_len;
    void* config;
    uint16_t config_vector;
    uint32_t generation;
    int nvectors;
    VirtQueue *vq;
    uint16_t device_id;
    bool vm_running;
    bool broken;
    uint8_t device_endian;
} VirtioDevice;

VirtQueue *virtio_add_queue(VirtioDevice *vdev, uint32_t queue_size, VirtioHandleQueue handle_queue);
int virtio_queue_get_num(VirtioDevice *vdev, int n);
void virtio_queue_set_num(VirtioDevice *vdev, int n, int num);
int virtio_set_features(VirtioDevice *vdev, uint64_t val);
int virtio_set_status(VirtioDevice *vdev, uint8_t status);
void virtio_add_feature(uint64_t *features, unsigned int feature);
#endif //VBOX_INCLUDED_SRC_VirtIOModern_virtio_h