#include <errno.h>
#include <iprt/assert.h>
#include "virtio.h"

VirtQueue *virtio_add_queue(VirtioDevice *vdev, uint32_t queue_size,
                            VirtioHandleQueue handle_queue) {

  auto it = vdev->vq.begin();
  for (; it != vdev->vq.end(); it++) {
    if (it->vring.num == 0) {
      break;
    }
  }

  it->vring.num = queue_size;
  it->vring.num_default = queue_size;
  it->vring.align = VIRTIO_QUEUE_ALIGN;
  it->handle_queue = handle_queue;
  return &(*it);
}

void virtio_notify(VirtioDevice *vdev, VirtQueue *vq) {
  vdev->virtio_notify_bus(vdev);
}

void virtio_queue_notify(VirtioDevice *vdev, int n) {
  VirtQueue *vq = &vdev->vq[n];

  if (vq->handle_queue) {
    vq->handle_queue(vdev, vq);
  }
}

void virtio_del_queue(VirtioDevice *vdev, int n) {
  Assert(n >= 0 && n < VIRTIO_QUEUE_MAX);
  vdev->vq[n].vring.num = 0;
  vdev->vq[n].vring.num_default = 0;
  vdev->vq[n].handle_queue = NULL;
}

int virtio_queue_get_num(VirtioDevice *vdev, int n) {
  return vdev->vq[n].vring.num;
}

void virtio_queue_set_rings(VirtioDevice *vdev, int n, uint64_t desc,
                            uint64_t avail, uint64_t used) {
  if (!vdev->vq[n].vring.num) {
    return;
  }
  vdev->vq[n].vring.desc = desc;
  vdev->vq[n].vring.avail = avail;
  vdev->vq[n].vring.used = used;
}

void virtio_add_feature(uint64_t *features, unsigned int feature) {
  *features |= (1ULL << feature);
}

void virtio_queue_set_num(VirtioDevice *vdev, int n, int num) {
  /* Don't allow guest to flip queue between existent and
   * nonexistent states, or to set it to an invalid size.
   */
  if (!!num != !!vdev->vq[n].vring.num || num > VIRTIO_QUEUE_MAX || num < 0) {
    return;
  }
  vdev->vq[n].vring.num = num;
}

static int virtio_set_features_nocheck(VirtioDevice *vdev, uint64_t val) {
  bool bad = (val & ~(vdev->host_features)) != 0;

  val &= vdev->host_features;
  vdev->guest_features = val;
  return bad ? -1 : 0;
}

int virtio_set_features(VirtioDevice *vdev, uint64_t val) {
  if (vdev->status & VIRTIO_CONFIG_S_FEATURES_OK)
    return -EINVAL;
  return virtio_set_features_nocheck(vdev, val);
}

int virtio_set_status(VirtioDevice *vdev, uint8_t status) {
  vdev->status = status;
  return 0;
}

/**
 * Get the availiable index of the VirtQueue
 *
 * @param vq    virtqueue whose availiable ring should be read
 * @return      index
 */
static uint16_t vring_avail_idx(VirtQueue *vq) {
  uint16_t idx = 0;
  virtioPCIPhysRead(vq->vdev->pciDev, vq->vring.avail + 2, &idx, sizeof(idx));
  vq->shadow_avail_idx = idx;
  return vq->shadow_avail_idx;
}

static int virtqueue_num_heads(VirtQueue *vq, unsigned int idx) {
  uint16_t num_heads = vring_avail_idx(vq) - idx;
  if (num_heads > vq->vring.num) {
    return -1;
  }
  return num_heads;
}

/**
 * Read the i-th availiable ring of the VirtQueue
 *
 * @param vq    virtqueue whose availiable ring should be read
 * @param i     availiable ring index
 * @return      availiable ring
 */
static uint16_t vring_avail_ring(VirtQueue *vq, int i) {
  uint16_t ring = 0;
  virtioPCIPhysRead(vq->vdev->pciDev,
                    vq->vring.avail + 4 + i * sizeof(uint16_t), &ring,
                    sizeof(ring));
  return ring;
}

/**
 * Read the i-th descriptor of the VirtQueue
 *
 * @param vq    virtqueue whose descriptor should be read
 * @param desc  VRingDesc structure
 * @param i     descriptor index to read
 */
static void vring_desc_read(VirtQueue *vq, VRingDesc *desc, int i) {
  /*  Descriptors are linearly stored in physical guest memory */
  virtioPCIPhysRead(vq->vdev->pciDev, vq->vring.desc + i * sizeof(VRingDesc),
                    desc, sizeof(VRingDesc));
}

/**
 * Read the next chained desctriptor of the virtqueue
 *
 * @param vq    virtqueue whose descriptor should be read
 * @param desc  VRingDesc structure
 * @param i     descriptor index to read
 */
static int virtqueue_read_next_desc(VirtQueue *vq, VRingDesc *desc,
                                    unsigned int max, unsigned int *next) {
  if (!(desc->flags & VRING_DESC_F_NEXT)) {
    return VIRTQUEUE_READ_DESC_DONE;
  }
  *next = desc->next;
  if (*next >= max) {
    return VIRTQUEUE_READ_DESC_ERROR;
  }
  vring_desc_read(vq, desc, *next);
  return VIRTQUEUE_READ_DESC_MORE;
}

static bool virtqueue_get_head(VirtQueue *vq, unsigned int idx,
                               unsigned int *head) {
  *head = vring_avail_ring(vq, idx % vq->vring.num);
  if (*head >= vq->vring.num) {
    return false;
  }
  return true;
}
