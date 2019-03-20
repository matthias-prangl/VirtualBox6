#ifndef DEVEXAMPLEIOPORT
#define DEVEXAMPLEIOPORT
#include <VBox/vmm/pdmdev.h>
#include "../VirtIO/Virtio.h"
 
DECLCALLBACK(int) devExampleMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, 
                                uint32_t iRegion, RTGCPHYS GCPhysAddress, 
                                RTGCPHYS cb, PCIADDRESSSPACE enmType);

PDMBOTHCBDECL(int) devExampleIOPortIn(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t *pu32, unsigned cb);
PDMBOTHCBDECL(int) devExampleIOPortOut(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t u32, unsigned cb);

DECLCALLBACK(uint32_t) devExampleIOCb_GetHostFeatures(void *pvState);
DECLCALLBACK(uint32_t) devExampleIOCb_GetHostMinimalFeatures(void *pvState);
DECLCALLBACK(void) devExampleIOCb_SetHostFeatures(void *pvState, uint32_t fFeatures);
DECLCALLBACK(int) devExampleIOCb_GetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data);
DECLCALLBACK(int) devExampleIOCb_SetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data);
DECLCALLBACK(int) devExampleIOCb_Reset(void *pvState);
DECLCALLBACK(void) devExampleIOCb_Ready(void *pvState);



const VPCIIOCALLBACKS g_devExampleIOCbs = {
    devExampleIOCb_GetHostFeatures,
    devExampleIOCb_GetHostMinimalFeatures,
    devExampleIOCb_SetHostFeatures,
    devExampleIOCb_GetConfig,
    devExampleIOCb_SetConfig,
    devExampleIOCb_Reset,
    devExampleIOCb_Ready
};

#endif //DEVEXAMPLEIOPORT