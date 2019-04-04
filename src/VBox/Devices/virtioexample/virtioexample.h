#ifndef DEVEXAMPLE
#define DEVEXAMPLE

#include <VBox/vmm/pdmdev.h>
#include "../VirtIOModern/virtio.h"
#include "../VirtIOModern/virtioPCI.h"

#define VIRTIOEXAMPLE_NAME_FMT "virtioexample%d"
#define VIRTIOEXAMPLE_PCI_CLASS 0x0300 // misc device
#define VIRTIOEXAMPLE_ID 16
#define VIRTIOEXAMPLE_N_QUEUES 2

typedef struct VirtioexampleState {
  VirtioPCIState vpci;
  VirtioDevice vdev;
  VirtQueue *vq1;
  VirtQueue *vq2;
  PPDMIBASE pDrvBase;
} VirtioexampleState;

DECLCALLBACK(int)
virtioexampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg);

const PDMDEVREG g_virtioexample = {
    PDM_DEVREG_VERSION,
    "virtioexample",            // device name
    "VBoxDDRC.rc",              // don't care; only if PDM_DEVREG_FLAGS_RC set
    "VBoxDDR0.r0",              // don't care; only if PDM_DEVREG_FLAGS_RC set
    "virtio example Device.\n", // device description
    PDM_DEVREG_FLAGS_DEFAULT_BITS,
    PDM_DEVREG_CLASS_MISC, // device class
    ~0U,                   // max num. instances
    sizeof(VirtioexampleState),
    virtioexampleConstruct, // pfnConstruct
    NULL,                   // pfnDestruct
    NULL,                   // pfnRelocate
    NULL,                   // pfnMemSetup.
    NULL,                   // pfnPowerOn
    NULL,                   // pfnReset
    NULL,                   // pfnSuspend
    NULL,                   // pfnResume
    NULL,                   // pfnAttach
    NULL,                   // pfnDetach
    NULL,                   // pfnQueryInterface
    NULL,                   // pfnInitComplete
    NULL,                   // pfnPowerOff
    NULL,                   // pfnSoftReset
    PDM_DEVREG_VERSION      // u32VersionEnd
};

#endif