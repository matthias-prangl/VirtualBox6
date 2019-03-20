#include "devExample.h"
#include "devExampleIOPort.h"

DECLCALLBACK(int) devExampleMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, 
                                uint32_t iRegion, RTGCPHYS GCPhysAddress, 
                                RTGCPHYS cb, PCIADDRESSSPACE enmType) {
    RT_NOREF(pPciDev, iRegion, enmType);
    PDEVEXAMPLESTATE pThis = PDMINS_2_DATA(pDevIns, PDEVEXAMPLESTATE);

    pThis->VPCI.IOPortBase = (RTIOPORT)GCPhysAddress;
    int rc = PDMDevHlpIOPortRegister(pDevIns, pThis->VPCI.IOPortBase, 
                                     cb, 0, devExampleIOPortOut, devExampleIOPortIn,
                                     NULL, NULL, "devExample");
    AssertRC(rc);
    return rc;
}

PDMBOTHCBDECL(int) devExampleIOPortIn(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t *pu32, unsigned cb) {
    return vpciIOPortIn(pDevIns, pvUser, port, pu32, cb, &g_devExampleIOCbs);
}
PDMBOTHCBDECL(int) devExampleIOPortOut(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT port, uint32_t u32, unsigned cb) {
    return vpciIOPortOut(pDevIns, pvUser, port, u32, cb, &g_devExampleIOCbs);
}

DECLCALLBACK(uint32_t) devExampleIOCb_GetHostFeatures(void *pvState) {
    RT_NOREF(pvState);
    return 0;
}
DECLCALLBACK(uint32_t) devExampleIOCb_GetHostMinimalFeatures(void *pvState) {
    RT_NOREF(pvState);
    return 0;
}
DECLCALLBACK(void) devExampleIOCb_SetHostFeatures(void *pvState, uint32_t fFeatures) {
    RT_NOREF(pvState, fFeatures);
    return;
}
DECLCALLBACK(int) devExampleIOCb_GetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data) {
    RT_NOREF(pvState, offCfg, cb, data);
    return 0;
}
DECLCALLBACK(int) devExampleIOCb_SetConfig(void *pvState, uint32_t offCfg, uint32_t cb, void *data) {
    RT_NOREF(pvState, offCfg, cb, data);
    return 0;
}
DECLCALLBACK(int) devExampleIOCb_Reset(void *pvState) {
    RT_NOREF(pvState);
    return 0;
}
DECLCALLBACK(void) devExampleIOCb_Ready(void *pvState) {
    RT_NOREF(pvState);
    return;
}