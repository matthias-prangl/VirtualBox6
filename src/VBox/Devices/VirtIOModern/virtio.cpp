#include "virtio.h"
#include <iprt/assert.h>
#include <errno.h>

VirtQueue *virtio_add_queue(VirtioDevice *vdev, uint32_t queue_size, 
                            VirtioHandleQueue handle_queue) {
    int i;
    for(i = 0; i < VIRTIO_QUEUE_MAX; i++)
        if(vdev->vq[i].vring.num == 0) 
            break;

    Assert(i != VIRTIO_QUEUE_MAX && queue_size <= VIRTIO_QUEUE_MAX);

    vdev->vq[i].vring.num = queue_size;
    vdev->vq[i].vring.num_default = queue_size;
    vdev->vq[i].vring.align = VIRTIO_QUEUE_ALIGN;
    vdev->vq[i].handle_queue = handle_queue;
    return &vdev->vq[i];
}

int virtio_queue_get_num(VirtioDevice *vdev, int n)
{
    return vdev->vq[n].vring.num;
}

void virtio_add_feature(uint64_t *features, unsigned int feature) {
    *features |= (1ULL << feature);
}

void virtio_queue_set_num(VirtioDevice *vdev, int n, int num)
{
    /* Don't allow guest to flip queue between existent and
     * nonexistent states, or to set it to an invalid size.
     */
    if (!!num != !!vdev->vq[n].vring.num ||
        num > VIRTIO_QUEUE_MAX || num < 0) {
        return;
    }
    vdev->vq[n].vring.num = num;
}

static int virtio_set_features_nocheck(VirtioDevice *vdev, uint64_t val)
{
    bool bad = (val & ~(vdev->host_features)) != 0;

    val &= vdev->host_features;
    vdev->guest_features = val;
    return bad ? -1 : 0;
}

int virtio_set_features(VirtioDevice *vdev, uint64_t val) {
    if(vdev->status & VIRTIO_CONFIG_S_FEATURES_OK)
        return -EINVAL;
    return virtio_set_features_nocheck(vdev, val);
}


int virtio_set_status(VirtioDevice *vdev, uint8_t status) {
    vdev->status = status;
    return 0;
}