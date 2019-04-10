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