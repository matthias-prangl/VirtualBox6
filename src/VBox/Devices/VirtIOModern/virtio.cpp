#include "virtio.h"
#include "virtioPCI.h"
#include <errno.h>
#include <iprt/assert.h>

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

static void virtio_set_isr(VirtioDevice *vdev, int val) {
  uint8_t old = ASMAtomicReadU8(&vdev->isr);
  /* Do not write ISR if it does not change, so that its cacheline remains
   * shared in the common case where the guest does not read it.
   */
  if ((old & val) != val) {
    val |= old;
    ASMAtomicWriteU8(&vdev->isr, val);
  }
}

void virtio_notify(VirtioDevice *vdev, VirtQueue *vq) {
  virtio_set_isr(vdev, 0x01);
  // TODO: check if virtio should notify
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
  if (num_heads) {
    RT_UNTRUSTED_VALIDATED_FENCE();
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
  RT_UNTRUSTED_NONVOLATILE_COPY_FENCE();
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

static void vring_used_write(VirtQueue *vq, VRingUsedElem *uelem, int i) {
  virtioPCIPhysWrite(vq->vdev->pciDev,
                     vq->vring.used + 4 + i * sizeof(VRingUsedElem), uelem,
                     sizeof(VRingUsedElem));
}

static void virtqueue_unmap_sg(VirtQueue *vq, VirtQueueElement *vqe) {
  PPDMDEVINSR3 devins = vq->vdev->pciDev->pDevInsR3;
  for (unsigned int i = 0; i < vqe->out_num; i++) {
    PDMDevHlpPhysReleasePageMappingLock(devins, &vqe->out_lock[i]);
  }
  for (unsigned int i = 0; i < vqe->in_num; i++) {
    PDMDevHlpPhysReleasePageMappingLock(devins, &vqe->in_lock[i]);
  }
}

static void virtqueue_fill(VirtQueue *vq, VirtQueueElement *vqe,
                           unsigned int len, unsigned int idx) {
  VRingUsedElem uelem;
  virtqueue_unmap_sg(vq, vqe);

  if (vq->vdev->broken) {
    return;
  }
  if (!vq->vring.used) {
    return;
  }
  idx = (idx + vq->used_idx) % vq->vring.num;
  uelem.id = vqe->index;
  uelem.len = len;
  vring_used_write(vq, &uelem, idx);
}

static void virtqueue_map_sg(VirtQueue *vq, RTSGSEG *sg, uint64_t *addr,
                             size_t num_sg, PGMPAGEMAPLOCK *lock) {
  PPDMDEVINSR3 devins = vq->vdev->pciDev->pDevInsR3;
  for (int i = 0; i < num_sg; i++) {
    PDMDevHlpPhysGCPhys2CCPtr(devins, addr[i], 0, &sg[i].pvSeg, &lock[i]);
  }
}

static inline void vring_used_idx_set(VirtQueue *vq, uint16_t val) {
  virtioPCIPhysWrite(vq->vdev->pciDev, vq->vring.used + 2, &val,
                     sizeof(uint16_t));
  vq->used_idx = val;
}

static void virtqueue_flush(VirtQueue *vq, unsigned int count) {
  RT_UNTRUSTED_NONVOLATILE_COPY_FENCE();
  uint16_t old = vq->used_idx;
  uint16_t current = old + count;

  vring_used_idx_set(vq, current);
  vq->inuse -= count;
}

void virtqueue_push(VirtQueue *vq, VirtQueueElement *vqe, unsigned int len) {
  virtqueue_fill(vq, vqe, len, 0);
  virtqueue_flush(vq, 1);
}

void *virtqueue_pop(VirtQueue *vq, size_t sz) {
  unsigned int max = vq->vring.num;
  VRingDesc desc = {0};
  VirtQueueElement *vqe =
      (VirtQueueElement *)calloc(1, sizeof(VirtQueueElement));
  unsigned int i, head;
  int rc = 0;

  RT_UNTRUSTED_VALIDATED_FENCE();

  if (vq->inuse >= vq->vring.num) {
    // virtqueue size exceeded
    goto done;
  }

  if (!virtqueue_get_head(vq, vq->last_avail_idx++, &head)) {
    goto done;
  }

  i = head;

  vring_desc_read(vq, &desc, i);
  if (desc.flags & VRING_DESC_F_INDIRECT) {
    if (!desc.len || (desc.len % sizeof(VRingDesc))) {
      // invalid indirect buffer table size
      goto done;
    }
    max = desc.len / sizeof(VRingDesc);
    i = 0;
    vring_desc_read(vq, &desc, i);
  }

  do {
    RTSGSEG *sg;
    if (desc.flags & VRING_DESC_F_WRITE) {
      vqe->in_addr[vqe->in_num] = desc.addr;
      sg = &vqe->in_sg[vqe->in_num++];
    } else {
      vqe->out_addr[vqe->out_num] = desc.addr;
      sg = &vqe->out_sg[vqe->out_num++];
    }
    sg->cbSeg = desc.len;

    rc = virtqueue_read_next_desc(vq, &desc, max, &i);
  } while (rc == VIRTQUEUE_READ_DESC_MORE);

  virtqueue_map_sg(vq, vqe->in_sg, vqe->in_addr, vqe->in_num, vqe->in_lock);
  virtqueue_map_sg(vq, vqe->out_sg, vqe->out_addr, vqe->out_num, vqe->out_lock);
  vqe->index = head;
  vq->inuse++;

done:
  return vqe;
}