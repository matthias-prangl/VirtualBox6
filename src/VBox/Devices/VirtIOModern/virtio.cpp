#include "virtio.h"
#include <iprt/assert.h>

VirtQueue *virtio_add_queue(VirtioDevice *vdev, uint32_t queue_size, 
                            VirtioHandleQueue handle_queue) {
    int i;
    for(i = 0; i < VIRTIO_QUEUE_MAX; i++)
        if(vdev->vq[i].vring.num == 0) 
            break;

    Assert(i != VIRTIO_QUEUE_MAX && queue_size <= VIRTIO_QUEUE_MAX);

    vdev->vq[i].vring.num = queue_size;
    vdev->vq[i].vring.num_default = queue_size;
    vdev->vq[i].vring.align = 4096;
    vdev->vq[i].handle_queue = handle_queue;
    return &vdev->vq[i];
}

int virtio_queue_get_num(VirtioDevice *vdev, int n)
{
    return vdev->vq[n].vring.num;
}