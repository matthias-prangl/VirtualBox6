#ifndef DEVEXAMPLE
#define DEVEXAMPLE

#include <VBox/vmm/pdmdev.h>
#include "../VirtIOModern/VirtioModern.h"

#define DEV_EXAMPLE_NAME_FMT    "devExample%d"
#define DEV_EXAMPLE_PCI_CLASS   0x00ff //misc device
#define DEV_EXAMPLE_ID          20
#define DEV_EXAMPLE_N_QUEUES    2

typedef struct DEVEXAMPLESTATE {
    VPCISTATE VPCI;
    PVQUEUE pRxQueue;
    PVQUEUE pTxQueue;
} DEVEXAMPLESTATE;
typedef DEVEXAMPLESTATE *PDEVEXAMPLESTATE;

DECLCALLBACK(int) devExampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg);
DECLCALLBACK(void) devExampleRX(void *pvState, PVQUEUE pQueue);
DECLCALLBACK(void) devExampleTX(void *pvState, PVQUEUE pQueue);

const PDMDEVREG g_devExample = {
    PDM_DEVREG_VERSION,
    "devExample", //device name
    "VBoxDDRC.rc", //don't care; only if PDM_DEVREG_FLAGS_RC set
    "VBoxDDR0.r0", //don't care; only if PDM_DEVREG_FLAGS_RC set
    "Example Device.\n", //device description
    PDM_DEVREG_FLAGS_DEFAULT_BITS,
    PDM_DEVREG_CLASS_MISC, //device class
    ~0U, //max num. instances
    sizeof(DEVEXAMPLESTATE),    
    devExampleConstruct, //pfnConstruct
    NULL, //pfnDestruct
    NULL, //pfnRelocate
    NULL, //pfnMemSetup.
    NULL, //pfnPowerOn
    NULL, //pfnReset
    NULL, //pfnSuspend
    NULL, //pfnResume
    NULL, //pfnAttach
    NULL, //pfnDetach
    NULL, //pfnQueryInterface
    NULL, //pfnInitComplete
    NULL, //pfnPowerOff
    NULL, //pfnSoftReset
    PDM_DEVREG_VERSION //u32VersionEnd
};

#endif