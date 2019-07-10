#include "virtioexample.h"
#include "../VirtIOModern/rtsgseg_helpers.h"
#include "../VirtIOModern/virtio.h"
#include "../VirtIOModern/virtioPCI.h"
#include <VBox/version.h>
#include <iprt/mem.h>
#include <iprt/uuid.h>

static void *rcv_bufs[1024];
static uint32_t rcv_count = 0;
void handle_q1(VirtioDevice *vdev, VirtQueue *vq) {
  VirtQueueElement *vqe =
      (VirtQueueElement *)virtqueue_pop(vq, sizeof(VirtQueueElement));
  if (vqe) {
    rcv_bufs[rcv_count] = RTMemAllocZ(vqe->out_sg[0].cbSeg);
    RTSGSEG_to_buf(vqe->out_sg, vqe->out_num, 0, rcv_bufs[rcv_count++],
                   vqe->out_sg->cbSeg);
    virtqueue_push(vq, vqe, 0);
    virtio_notify(vdev, vq);
    RTMemFree(vqe);
  }
  return;
}

void handle_q2(VirtioDevice *vdev, VirtQueue *vq) {
  VirtQueueElement *vqe =
      (VirtQueueElement *)virtqueue_pop(vq, sizeof(VirtQueueElement));
  if (vqe) {
    rcv_count = rcv_count - 1;
    size_t len = RTSGSEG_from_buf(vqe->in_sg, vqe->in_num, 0,
                                  rcv_bufs[rcv_count], vqe->in_sg->cbSeg);

    virtqueue_push(vq, vqe, static_cast<unsigned int>(len));
    virtio_notify(vdev, vq);
    RTMemFree(rcv_bufs[rcv_count]);
    RTMemFree(vqe);
  }
  return;
}

void virtioexample_get_config(VirtioDevice *vdev, uint8_t *config) {
  RT_NOREF(vdev, config);
  return;
}

void virtioexample_set_config(VirtioDevice *vdev, uint8_t *config) {
  RT_NOREF(vdev, config);
  return;
}

DECLCALLBACK(int)
virtioexampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg) {
  PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
  VirtioexampleState *pThis = PDMINS_2_DATA(pDevIns, VirtioexampleState *);
  VirtioPCIDevice *pciDev = &pThis->vpci;
  VirtioDevice *vdev = &pThis->vdev;
  vdev->pciDev = pciDev;
  pciDev->vdev = vdev;

  int rc =
      PDMDevHlpCritSectInit(pDevIns, &vdev->critsect, RT_SRC_POS, "vex");
  AssertRCReturn(rc, rc);
  rc = PDMDevHlpSetDeviceCritSect(pDevIns, &vdev->critsect);
  AssertRCReturn(rc, rc);
  rc = virtioPCIConstruct(pDevIns, pciDev,
                          VIRTIOEXAMPLE_ID, VIRTIOEXAMPLE_PCI_CLASS,
                          VIRTIOEXAMPLE_N_QUEUES);
  // Last call fails because there is no LUN, ignore for the time being
  if (rc != VINF_SUCCESS)
    rc = VINF_SUCCESS;
  vdev->config_len = sizeof(virtio_example_config);
  vdev->config = calloc(1, sizeof(virtio_example_config));
  if (!vdev->config) {
    rc = VERR_NO_MEMORY;
  }

  vdev->set_config = virtioexample_set_config;
  vdev->get_config = virtioexample_get_config;
  vdev->virtio_notify_bus = virtioPCINotify;
  vdev->config = &pThis->example_config;
  pThis->example_config.num_scanouts = 1;
  
  for (int i = 0; i < VIRTIO_QUEUE_MAX; i++) {
    vdev->vq[i].vdev = vdev;
  }

  pThis->vq1 = virtio_add_queue(&pThis->vdev, 256, &handle_q1);
  pThis->vq2 = virtio_add_queue(&pThis->vdev, 256, &handle_q2);

  RT_NOREF(pCfg, iInstance);
  return rc;
}

extern "C" DECLEXPORT(int)
    VBoxDevicesRegister(PPDMDEVREGCB pCallbacks, uint32_t u32Version) {
  AssertLogRelMsgReturn(
      u32Version >= VBOX_VERSION,
      ("VBox version %#x, expected %#x or higher\n", u32Version, VBOX_VERSION),
      VERR_VERSION_MISMATCH);
  AssertLogRelMsgReturn(pCallbacks->u32Version == PDM_DEVREG_CB_VERSION,
                        ("CB version %#x, expected %#x or higher\n",
                         pCallbacks->u32Version, PDM_DEVREG_CB_VERSION),
                        VERR_VERSION_MISMATCH);
  return pCallbacks->pfnRegister(pCallbacks, &g_virtioexample);
}
