#include "virtioexample.h"
#include "../VirtIOModern/virtioPCI.h"
#include <VBox/version.h>
#include <iprt/mem.h>
#include <iprt/uuid.h>

void handle_q1(VirtioDevice *vdev, VirtQueue *vq) {
  RT_NOREF(vq, vdev);
  return;
}

void handle_q2(VirtioDevice *vdev, VirtQueue *vq) {
  virtio_notify(vdev, vq);
  return;
}

void virtioexample_get_config(VirtioDevice *vdev, uint8_t *config) {
  RT_NOREF(vdev, config);
  return;
}

void virtioexample_set_config(VirtioDevice *vdev, const uint8_t *config) {
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
      PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
  AssertRCReturn(rc, rc);
  rc = virtioPCIConstruct(pDevIns, pciDev, iInstance, VIRTIOEXAMPLE_NAME_FMT,
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

  for (auto it = vdev->vq.begin(); it != vdev->vq.end(); it++) {
    it->vector = 0;
    it->vdev = vdev;
    it->queue_idx = it - vdev->vq.begin();
  }

  pThis->vq1 = virtio_add_queue(&pThis->vdev, 256, &handle_q1);
  pThis->vq2 = virtio_add_queue(&pThis->vdev, 16, &handle_q2);

  RT_NOREF(pCfg);
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
