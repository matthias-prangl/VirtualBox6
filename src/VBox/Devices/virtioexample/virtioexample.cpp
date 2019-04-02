#include <VBox/vmm/pdmdev.h>
#include <VBox/version.h>
#include <iprt/mem.h>
#include <iprt/uuid.h>
#include "virtioexample.h"

DECLCALLBACK(int) virtioexampleConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg) {
    RT_NOREF(pCfg);
    PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
    PvirtioexampleSTATE pThis = PDMINS_2_DATA(pDevIns, PvirtioexampleSTATE);
    
    int rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    AssertRCReturn(rc, rc);

    rc = virtioPCIConstruct(pDevIns, &pThis->VPCI, iInstance, 
                       VIRTIOEXAMPLE_NAME_FMT, VIRTIOEXAMPLE_ID,
                       VIRTIOEXAMPLE_PCI_CLASS, VIRTIOEXAMPLE_N_QUEUES);

    if(rc == VERR_PDM_NO_ATTACHED_DRIVER) {
        rc = VINF_SUCCESS;
    }

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
   	return pCallbacks->pfnRegister(pCallbacks, &g_virtioexample);
}
