#include <VBox/version.h>
#include <VBox/vmm/pdmdev.h>
#include <iprt/mem.h>
#include <iprt/uuid.h>
#include "virtioexample.h"
#include "../VirtIOModern/virtioPCI.h"

void handle_q1(VirtioDevice *vdev, VirtQueue *vq) {
  RT_NOREF(vq, vdev);
  return;
}

void handle_q2(VirtioDevice *vdev, VirtQueue *vq) {
  RT_NOREF(vq, vdev);
  return;
}

DECLCALLBACK(int)
virtioexampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg) {
  PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
  VirtioexampleState *pThis = PDMINS_2_DATA(pDevIns, VirtioexampleState *);
  VirtioPCIState *pciDev = &pThis->vpci;
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

  pThis->vdev.vq = (VirtQueue *)calloc(VIRTIO_QUEUE_MAX, sizeof(VirtQueue));
  Assert(pThis->vdev.vq != NULL);

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
