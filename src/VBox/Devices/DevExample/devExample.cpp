#include <VBox/vmm/pdmdev.h>
#include <VBox/version.h>
#include <iprt/mem.h>
#include <iprt/uuid.h>
#include "devExample.h"
#include "devExampleIOPort.h"

DECLCALLBACK(void *) devExampleQueryInterface(struct PDMIBASE *pInterface, const char *pszIID) {
    PDEVEXAMPLESTATE pThis = RT_FROM_MEMBER(pInterface, DEVEXAMPLESTATE, VPCI.IBase);
    Assert(&pThis->VPCI.IBase == pInterface);
    return vpciQueryInterface(pInterface, pszIID);
}

DECLCALLBACK(void) devExampleRX(void *pvState, PVQUEUE pQueue) {
    RT_NOREF(pvState, pQueue);
}
DECLCALLBACK(void) devExampleTX(void *pvState, PVQUEUE pQueue) {
    RT_NOREF(pvState, pQueue);
}

DECLCALLBACK(int) devExampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg) {
    RT_NOREF(pCfg);
    PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
    PDEVEXAMPLESTATE pThis = PDMINS_2_DATA(pDevIns, PDEVEXAMPLESTATE);
    
    int rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    AssertRCReturn(rc, rc);

    pThis->VPCI.IBase.pfnQueryInterface = devExampleQueryInterface;
    rc = vpciConstruct(pDevIns, &pThis->VPCI, iInstance, 
                       DEV_EXAMPLE_NAME_FMT, DEV_EXAMPLE_ID,
                       DEV_EXAMPLE_PCI_CLASS, DEV_EXAMPLE_N_QUEUES);

    if(rc == VERR_PDM_NO_ATTACHED_DRIVER) {
        pThis->VPCI.nQueues = DEV_EXAMPLE_N_QUEUES;
        rc = VINF_SUCCESS;
    }

    pThis->pRxQueue = vpciAddQueue(&pThis->VPCI, 256, devExampleRX, "RX");
    pThis->pTxQueue = vpciAddQueue(&pThis->VPCI, 256, devExampleTX, "TX");

    vpciReset(&pThis->VPCI);
    pThis->VPCI.uStatus = 3;
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 0, VPCI_CONFIG, PCI_ADDRESS_SPACE_IO, devExampleMap);
    if(RT_FAILURE(rc)) {
        return rc; //... break point
    }



    return rc;
}

extern "C" DECLEXPORT(int) VBoxDevicesRegister(PPDMDEVREGCB pCallbacks, 
                                               uint32_t u32Version) {
	AssertLogRelMsgReturn(u32Version >= VBOX_VERSION, 
                          ("VBox version %#x, expected %#x or higher\n", 
                           u32Version,VBOX_VERSION), 
                          VERR_VERSION_MISMATCH);
	AssertLogRelMsgReturn(pCallbacks->u32Version == PDM_DEVREG_CB_VERSION, 
                          ("CB version %#x, expected %#x or higher\n", 
                           pCallbacks->u32Version,PDM_DEVREG_CB_VERSION),
                          VERR_VERSION_MISMATCH);
   	return pCallbacks->pfnRegister(pCallbacks,&g_devExample);
}
