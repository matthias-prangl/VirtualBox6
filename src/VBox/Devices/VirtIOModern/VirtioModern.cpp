/* $Id: VirtioModern.cpp $ */
/** @file
 * Virtio - Virtio Common Functions (VRing, VQueue, Virtio PCI)
 * Based on Virtio.cpp
 */

/*
 * Copyright (C) 2009-2019 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_VIRTIO

#include <iprt/param.h>
#include <iprt/uuid.h>
#include <VBox/vmm/pdmdev.h>
#include "VirtioModern.h"

#define INSTANCE(pState) pState->szInstance
#define IFACE_TO_STATE(pIface, ifaceName) ((VPCISTATE *)((char*)(pIface) - RT_UOFFSETOF(VPCISTATE, ifaceName)))

#ifdef LOG_ENABLED
# define QUEUENAME(s, q) (q->pcszName)
#endif



#ifndef VBOX_DEVICE_STRUCT_TESTCASE

//RT_C_DECLS_BEGIN
//RT_C_DECLS_END


static void vqueueReset(PVQUEUE pQueue)
{
    pQueue->VRing.addrDescriptors = 0;
    pQueue->VRing.addrAvail       = 0;
    pQueue->VRing.addrUsed        = 0;
    pQueue->uNextAvailIndex       = 0;
    pQueue->uNextUsedIndex        = 0;
    pQueue->uPageNumber           = 0;
}

static void vqueueInit(PVQUEUE pQueue, uint32_t uPageNumber)
{
    pQueue->VRing.addrDescriptors = (uint64_t)uPageNumber << PAGE_SHIFT;
    pQueue->VRing.addrAvail       = pQueue->VRing.addrDescriptors
        + sizeof(VRINGDESC) * pQueue->VRing.uSize;
    pQueue->VRing.addrUsed        = RT_ALIGN(
        pQueue->VRing.addrAvail + RT_UOFFSETOF_DYN(VRINGAVAIL, auRing[pQueue->VRing.uSize]),
        PAGE_SIZE); /* The used ring must start from the next page. */
    pQueue->uNextAvailIndex       = 0;
    pQueue->uNextUsedIndex        = 0;
}

// void vqueueElemFree(PVQUEUEELEM pElem)
// {
// }

void vringReadDesc(PVPCISTATE pState, PVRING pVRing, uint32_t uIndex, PVRINGDESC pDesc)
{
    //Log(("%s vringReadDesc: ring=%p idx=%u\n", INSTANCE(pState), pVRing, uIndex));
    PDMDevHlpPhysRead(pState->CTX_SUFF(pDevIns),
                      pVRing->addrDescriptors + sizeof(VRINGDESC) * (uIndex % pVRing->uSize),
                      pDesc, sizeof(VRINGDESC));
}

uint16_t vringReadAvail(PVPCISTATE pState, PVRING pVRing, uint32_t uIndex)
{
    uint16_t tmp;

    PDMDevHlpPhysRead(pState->CTX_SUFF(pDevIns),
                      pVRing->addrAvail + RT_UOFFSETOF_DYN(VRINGAVAIL, auRing[uIndex % pVRing->uSize]),
                      &tmp, sizeof(tmp));
    return tmp;
}

uint16_t vringReadAvailFlags(PVPCISTATE pState, PVRING pVRing)
{
    uint16_t tmp;

    PDMDevHlpPhysRead(pState->CTX_SUFF(pDevIns),
                      pVRing->addrAvail + RT_UOFFSETOF(VRINGAVAIL, uFlags),
                      &tmp, sizeof(tmp));
    return tmp;
}

void vringSetNotification(PVPCISTATE pState, PVRING pVRing, bool fEnabled)
{
    uint16_t tmp;

    PDMDevHlpPhysRead(pState->CTX_SUFF(pDevIns),
                      pVRing->addrUsed + RT_UOFFSETOF(VRINGUSED, uFlags),
                      &tmp, sizeof(tmp));

    if (fEnabled)
        tmp &= ~ VRINGUSED_F_NO_NOTIFY;
    else
        tmp |= VRINGUSED_F_NO_NOTIFY;

    PDMDevHlpPCIPhysWrite(pState->CTX_SUFF(pDevIns),
                          pVRing->addrUsed + RT_UOFFSETOF(VRINGUSED, uFlags),
                          &tmp, sizeof(tmp));
}

bool vqueueSkip(PVPCISTATE pState, PVQUEUE pQueue)
{
    if (vqueueIsEmpty(pState, pQueue))
        return false;

    Log2(("%s vqueueSkip: %s avail_idx=%u\n", INSTANCE(pState),
          QUEUENAME(pState, pQueue), pQueue->uNextAvailIndex));
    pQueue->uNextAvailIndex++;
    return true;
}

bool vqueueGet(PVPCISTATE pState, PVQUEUE pQueue, PVQUEUEELEM pElem, bool fRemove)
{
    if (vqueueIsEmpty(pState, pQueue))
        return false;

    pElem->nIn = pElem->nOut = 0;

    Log2(("%s vqueueGet: %s avail_idx=%u\n", INSTANCE(pState),
          QUEUENAME(pState, pQueue), pQueue->uNextAvailIndex));

    VRINGDESC desc;
    uint16_t  idx = vringReadAvail(pState, &pQueue->VRing, pQueue->uNextAvailIndex);
    if (fRemove)
        pQueue->uNextAvailIndex++;
    pElem->uIndex = idx;
    do
    {
        VQUEUESEG *pSeg;

        /*
         * Malicious guests may try to trick us into writing beyond aSegsIn or
         * aSegsOut boundaries by linking several descriptors into a loop. We
         * cannot possibly get a sequence of linked descriptors exceeding the
         * total number of descriptors in the ring (see @bugref{8620}).
         */
        if (pElem->nIn + pElem->nOut >= VRING_MAX_SIZE)
        {
            static volatile uint32_t s_cMessages  = 0;
            static volatile uint32_t s_cThreshold = 1;
            if (ASMAtomicIncU32(&s_cMessages) == ASMAtomicReadU32(&s_cThreshold))
            {
                LogRel(("%s: too many linked descriptors; check if the guest arranges descriptors in a loop.\n",
                        INSTANCE(pState)));
                if (ASMAtomicReadU32(&s_cMessages) != 1)
                    LogRel(("%s: (the above error has occured %u times so far)\n",
                            INSTANCE(pState), ASMAtomicReadU32(&s_cMessages)));
                ASMAtomicWriteU32(&s_cThreshold, ASMAtomicReadU32(&s_cThreshold) * 10);
            }
            break;
        }
        RT_UNTRUSTED_VALIDATED_FENCE();

        vringReadDesc(pState, &pQueue->VRing, idx, &desc);
        if (desc.u16Flags & VRINGDESC_F_WRITE)
        {
            Log2(("%s vqueueGet: %s IN  seg=%u desc_idx=%u addr=%p cb=%u\n", INSTANCE(pState),
                  QUEUENAME(pState, pQueue), pElem->nIn, idx, desc.u64Addr, desc.uLen));
            pSeg = &pElem->aSegsIn[pElem->nIn++];
        }
        else
        {
            Log2(("%s vqueueGet: %s OUT seg=%u desc_idx=%u addr=%p cb=%u\n", INSTANCE(pState),
                  QUEUENAME(pState, pQueue), pElem->nOut, idx, desc.u64Addr, desc.uLen));
            pSeg = &pElem->aSegsOut[pElem->nOut++];
        }

        pSeg->addr = desc.u64Addr;
        pSeg->cb   = desc.uLen;
        pSeg->pv   = NULL;

        idx = desc.u16Next;
    } while (desc.u16Flags & VRINGDESC_F_NEXT);

    Log2(("%s vqueueGet: %s head_desc_idx=%u nIn=%u nOut=%u\n", INSTANCE(pState),
          QUEUENAME(pState, pQueue), pElem->uIndex, pElem->nIn, pElem->nOut));
    return true;
}

uint16_t vringReadUsedIndex(PVPCISTATE pState, PVRING pVRing)
{
    uint16_t tmp;
    PDMDevHlpPhysRead(pState->CTX_SUFF(pDevIns),
                      pVRing->addrUsed + RT_UOFFSETOF(VRINGUSED, uIndex),
                      &tmp, sizeof(tmp));
    return tmp;
}

void vringWriteUsedIndex(PVPCISTATE pState, PVRING pVRing, uint16_t u16Value)
{
    PDMDevHlpPCIPhysWrite(pState->CTX_SUFF(pDevIns),
                          pVRing->addrUsed + RT_UOFFSETOF(VRINGUSED, uIndex),
                          &u16Value, sizeof(u16Value));
}

void vringWriteUsedElem(PVPCISTATE pState, PVRING pVRing, uint32_t uIndex, uint32_t uId, uint32_t uLen)
{
    VRINGUSEDELEM elem;

    elem.uId = uId;
    elem.uLen = uLen;
    PDMDevHlpPCIPhysWrite(pState->CTX_SUFF(pDevIns),
                          pVRing->addrUsed + RT_UOFFSETOF_DYN(VRINGUSED, aRing[uIndex % pVRing->uSize]),
                          &elem, sizeof(elem));
}


void vqueuePut(PVPCISTATE pState, PVQUEUE pQueue,
               PVQUEUEELEM pElem, uint32_t uTotalLen, uint32_t uReserved)
{
    Log2(("%s vqueuePut: %s"
          " desc_idx=%u acb=%u (%u)\n",
          INSTANCE(pState), QUEUENAME(pState, pQueue),
          pElem->uIndex, uTotalLen, uReserved));

    Assert(uReserved < uTotalLen);

    uint32_t cbLen = uTotalLen - uReserved;
    uint32_t cbSkip = uReserved;

    for (unsigned i = 0; i < pElem->nIn && cbLen > 0; ++i)
    {
        if (cbSkip >= pElem->aSegsIn[i].cb) /* segment completely skipped? */
        {
            cbSkip -= pElem->aSegsIn[i].cb;
            continue;
        }

        uint32_t cbSegLen = pElem->aSegsIn[i].cb - cbSkip;
        if (cbSegLen > cbLen)   /* last segment only partially used? */
            cbSegLen = cbLen;

        /*
         * XXX: We should assert pv != NULL, but we need to check and
         * fix all callers first.
         */
        if (pElem->aSegsIn[i].pv != NULL)
        {
            Log2(("%s vqueuePut: %s"
                  " used_idx=%u seg=%u addr=%p pv=%p cb=%u acb=%u\n",
                  INSTANCE(pState), QUEUENAME(pState, pQueue),
                  pQueue->uNextUsedIndex, i,
                  (void *)pElem->aSegsIn[i].addr, pElem->aSegsIn[i].pv,
                  pElem->aSegsIn[i].cb, cbSegLen));

            PDMDevHlpPCIPhysWrite(pState->CTX_SUFF(pDevIns),
                                  pElem->aSegsIn[i].addr + cbSkip,
                                  pElem->aSegsIn[i].pv,
                                  cbSegLen);
        }

        cbSkip = 0;
        cbLen -= cbSegLen;
    }

    Log2(("%s vqueuePut: %s"
          " used_idx=%u guest_used_idx=%u id=%u len=%u\n",
          INSTANCE(pState), QUEUENAME(pState, pQueue),
          pQueue->uNextUsedIndex, vringReadUsedIndex(pState, &pQueue->VRing),
          pElem->uIndex, uTotalLen));

    vringWriteUsedElem(pState, &pQueue->VRing,
                       pQueue->uNextUsedIndex++,
                       pElem->uIndex, uTotalLen);
}


void vqueueNotify(PVPCISTATE pState, PVQUEUE pQueue)
{
    LogFlow(("%s vqueueNotify: %s availFlags=%x guestFeatures=%x vqueue is %sempty\n",
             INSTANCE(pState), QUEUENAME(pState, pQueue),
             vringReadAvailFlags(pState, &pQueue->VRing),
             pState->uGuestFeatures, vqueueIsEmpty(pState, pQueue)?"":"not "));
    if (!(vringReadAvailFlags(pState, &pQueue->VRing) & VRINGAVAIL_F_NO_INTERRUPT)
        || ((pState->uGuestFeatures & VPCI_F_NOTIFY_ON_EMPTY) && vqueueIsEmpty(pState, pQueue)))
    {
        int rc = vpciRaiseInterrupt(pState, VERR_INTERNAL_ERROR, VPCI_ISR_QUEUE);
        if (RT_FAILURE(rc))
            Log(("%s vqueueNotify: Failed to raise an interrupt (%Rrc).\n", INSTANCE(pState), rc));
    }
    else
    {
        STAM_COUNTER_INC(&pState->StatIntsSkipped);
    }

}

void vqueueSync(PVPCISTATE pState, PVQUEUE pQueue)
{
    Log2(("%s vqueueSync: %s old_used_idx=%u new_used_idx=%u\n", INSTANCE(pState),
          QUEUENAME(pState, pQueue), vringReadUsedIndex(pState, &pQueue->VRing), pQueue->uNextUsedIndex));
    vringWriteUsedIndex(pState, &pQueue->VRing, pQueue->uNextUsedIndex);
    vqueueNotify(pState, pQueue);
}

void vpciReset(PVPCISTATE pState)
{
    pState->uGuestFeatures = 0;
    pState->uQueueSelector = 0;
    pState->uStatus        = 0;
    pState->uISR           = 0;

    for (unsigned i = 0; i < pState->nQueues; i++)
        vqueueReset(&pState->Queues[i]);
}


/**
 * Raise interrupt.
 *
 * @param   pState      The device state structure.
 * @param   rcBusy      Status code to return when the critical section is busy.
 * @param   u8IntCause  Interrupt cause bit mask to set in PCI ISR port.
 */
int vpciRaiseInterrupt(VPCISTATE *pState, int rcBusy, uint8_t u8IntCause)
{
    RT_NOREF_PV(rcBusy);
    // int rc = vpciCsEnter(pState, rcBusy);
    // if (RT_UNLIKELY(rc != VINF_SUCCESS))
    //     return rc;

    STAM_COUNTER_INC(&pState->StatIntsRaised);
    LogFlow(("%s vpciRaiseInterrupt: u8IntCause=%x\n",
             INSTANCE(pState), u8IntCause));

    pState->uISR |= u8IntCause;
    PDMDevHlpPCISetIrq(pState->CTX_SUFF(pDevIns), 0, 1);
    // vpciCsLeave(pState);
    return VINF_SUCCESS;
}

/**
 * Lower interrupt.
 *
 * @param   pState      The device state structure.
 */
static void vpciLowerInterrupt(VPCISTATE *pState)
{
    LogFlow(("%s vpciLowerInterrupt\n", INSTANCE(pState)));
    PDMDevHlpPCISetIrq(pState->CTX_SUFF(pDevIns), 0, 0);
}

DECLINLINE(uint32_t) vpciGetHostFeatures(PVPCISTATE pState,
                                         PFNGETHOSTFEATURES pfnGetHostFeatures)
{
    return pfnGetHostFeatures(pState)
        | VPCI_F_NOTIFY_ON_EMPTY;
}

/**
 * Port I/O Handler for IN operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      Pointer to the device state structure.
 * @param   Port        Port number used for the IN operation.
 * @param   pu32        Where to store the result.
 * @param   cb          Number of bytes read.
 * @param   pCallbacks  Pointer to the callbacks.
 * @thread  EMT
 */
int vpciIOPortIn(PPDMDEVINS         pDevIns,
                 void              *pvUser,
                 RTIOPORT           Port,
                 uint32_t          *pu32,
                 unsigned           cb,
                 PCVPCIIOCALLBACKS  pCallbacks)
{
    VPCISTATE  *pState = PDMINS_2_DATA(pDevIns, VPCISTATE *);
    int         rc     = VINF_SUCCESS;
    STAM_PROFILE_ADV_START(&pState->CTX_SUFF(StatIORead), a);
    RT_NOREF_PV(pvUser);

    /*
     * We probably do not need to enter critical section when reading registers
     * as the most of them are either constant or being changed during
     * initialization only, the exception being ISR which can be raced by all
     * threads but I see no big harm in it. It also happens to be the most read
     * register as it gets read in interrupt handler. By dropping cs protection
     * here we gain the ability to deliver RX packets to the guest while TX is
     * holding cs transmitting queued packets.
     *
    rc = vpciCsEnter(pState, VINF_IOM_R3_IOPORT_READ);
    if (RT_UNLIKELY(rc != VINF_SUCCESS))
    {
        STAM_PROFILE_ADV_STOP(&pState->CTX_SUFF(StatIORead), a);
        return rc;
        }*/

    Port -= pState->IOPortBase;
    switch (Port)
    {
        case VPCI_HOST_FEATURES:
            /* Tell the guest what features we support. */
            *pu32 = vpciGetHostFeatures(pState, pCallbacks->pfnGetHostFeatures)
                    | VPCI_F_BAD_FEATURE;
            break;

        case VPCI_GUEST_FEATURES:
            *pu32 = pState->uGuestFeatures;
            break;

        case VPCI_QUEUE_PFN:
            *pu32 = pState->Queues[pState->uQueueSelector].uPageNumber;
            break;

        case VPCI_QUEUE_NUM:
            Assert(cb == 2);
            *(uint16_t*)pu32 = pState->Queues[pState->uQueueSelector].VRing.uSize;
            break;

        case VPCI_QUEUE_SEL:
            Assert(cb == 2);
            *(uint16_t*)pu32 = pState->uQueueSelector;
            break;

        case VPCI_STATUS:
            Assert(cb == 1);
            *(uint8_t*)pu32 = pState->uStatus;
            break;

        case VPCI_ISR:
            Assert(cb == 1);
            *(uint8_t*)pu32 = pState->uISR;
            pState->uISR = 0; /* read clears all interrupts */
            vpciLowerInterrupt(pState);
            break;

        default:
            if (Port >= VPCI_CONFIG)
                rc = pCallbacks->pfnGetConfig(pState, Port - VPCI_CONFIG, cb, pu32);
            else
            {
                *pu32 = 0xFFFFFFFF;
                rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "%s vpciIOPortIn: no valid port at offset port=%RTiop cb=%08x\n",
                                       INSTANCE(pState), Port, cb);
            }
            break;
    }
    Log3(("%s vpciIOPortIn:  At %RTiop in  %0*x\n", INSTANCE(pState), Port, cb*2, *pu32));
    STAM_PROFILE_ADV_STOP(&pState->CTX_SUFF(StatIORead), a);
    //vpciCsLeave(pState);
    return rc;
}


/**
 * Port I/O Handler for OUT operations.
 *
 * @returns VBox status code.
 *
 * @param   pDevIns     The device instance.
 * @param   pvUser      User argument.
 * @param   Port        Port number used for the IN operation.
 * @param   u32         The value to output.
 * @param   cb          The value size in bytes.
 * @param   pCallbacks  Pointer to the callbacks.
 * @thread  EMT
 */
int vpciIOPortOut(PPDMDEVINS                pDevIns,
                  void                     *pvUser,
                  RTIOPORT                  Port,
                  uint32_t                  u32,
                  unsigned                  cb,
                  PCVPCIIOCALLBACKS         pCallbacks)
{
    VPCISTATE  *pState = PDMINS_2_DATA(pDevIns, VPCISTATE *);
    int         rc     = VINF_SUCCESS;
    bool        fHasBecomeReady;
    STAM_PROFILE_ADV_START(&pState->CTX_SUFF(StatIOWrite), a);
    RT_NOREF_PV(pvUser);

    Port -= pState->IOPortBase;
    Log3(("%s virtioIOPortOut: At %RTiop out          %0*x\n", INSTANCE(pState), Port, cb*2, u32));

    switch (Port)
    {
        case VPCI_GUEST_FEATURES:
        {
            const uint32_t uHostFeatures = vpciGetHostFeatures(pState, pCallbacks->pfnGetHostFeatures);

            if (RT_LIKELY((u32 & ~uHostFeatures) == 0))
            {
                pState->uGuestFeatures = u32;
            }
            else
            {
                /*
                 * Guest requests features we don't advertise.  Stick
                 * to the minimum if negotiation looks completely
                 * botched, otherwise restrict to advertised features.
                 */
                Log(("%s Guest asked for features host does not support! (host=%x guest=%x)\n",
                        INSTANCE(pState), uHostFeatures, u32));
                pState->uGuestFeatures = u32 & uHostFeatures;
            }
            pCallbacks->pfnSetHostFeatures(pState, pState->uGuestFeatures);
            break;
        }

        case VPCI_QUEUE_PFN:
            /*
             * The guest is responsible for allocating the pages for queues,
             * here it provides us with the page number of descriptor table.
             * Note that we provide the size of the queue to the guest via
             * VIRTIO_PCI_QUEUE_NUM.
             */
            pState->Queues[pState->uQueueSelector].uPageNumber = u32;
            if (u32)
                vqueueInit(&pState->Queues[pState->uQueueSelector], u32);
            else
                rc = pCallbacks->pfnReset(pState);
            break;

        case VPCI_QUEUE_SEL:
            Assert(cb == 2);
            u32 &= 0xFFFF;
            if (u32 < pState->nQueues)
                pState->uQueueSelector = u32;
            else
                Log3(("%s vpciIOPortOut: Invalid queue selector %08x\n", INSTANCE(pState), u32));
            break;

        case VPCI_QUEUE_NOTIFY:
#ifdef IN_RING3
            Assert(cb == 2);
            u32 &= 0xFFFF;
            if (u32 < pState->nQueues)
            {
                RT_UNTRUSTED_VALIDATED_FENCE();
                if (pState->Queues[u32].VRing.addrDescriptors)
                {
                    // rc = vpciCsEnter(pState, VERR_SEM_BUSY);
                    // if (RT_LIKELY(rc == VINF_SUCCESS))
                    // {
                        pState->Queues[u32].pfnCallback(pState, &pState->Queues[u32]);
                    //     vpciCsLeave(pState);
                    // }
                }
                else
                    Log(("%s The queue (#%d) being notified has not been initialized.\n",
                         INSTANCE(pState), u32));
            }
            else
                Log(("%s Invalid queue number (%d)\n", INSTANCE(pState), u32));
#else
            rc = VINF_IOM_R3_IOPORT_WRITE;
#endif
            break;

        case VPCI_STATUS:
            Assert(cb == 1);
            u32 &= 0xFF;
            fHasBecomeReady = !(pState->uStatus & VPCI_STATUS_DRV_OK) && (u32 & VPCI_STATUS_DRV_OK);
            pState->uStatus = u32;
            /* Writing 0 to the status port triggers device reset. */
            if (u32 == 0)
                rc = pCallbacks->pfnReset(pState);
            else if (fHasBecomeReady)
            {
                /* Older hypervisors were lax and did not enforce bus mastering. Older guests
                 * (Linux prior to 2.6.34, NetBSD 6.x) were lazy and did not enable bus mastering.
                 * We automagically enable bus mastering on driver initialization to make existing
                 * drivers work.
                 */
                PDMPciDevSetCommand(&pState->pciDevice, PDMPciDevGetCommand(&pState->pciDevice) | PCI_COMMAND_BUSMASTER);

                pCallbacks->pfnReady(pState);
            }
            break;

        default:
            if (Port >= VPCI_CONFIG)
                rc = pCallbacks->pfnSetConfig(pState, Port - VPCI_CONFIG, cb, &u32);
            else
                rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "%s vpciIOPortOut: no valid port at offset Port=%RTiop cb=%08x\n",
                                       INSTANCE(pState), Port, cb);
            break;
    }

    STAM_PROFILE_ADV_STOP(&pState->CTX_SUFF(StatIOWrite), a);
    return rc;
}

#ifdef IN_RING3

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
void *vpciQueryInterface(struct PDMIBASE *pInterface, const char *pszIID)
{
    VPCISTATE *pThis = IFACE_TO_STATE(pInterface, IBase);
    Assert(&pThis->IBase == pInterface);

    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pThis->IBase);
    return NULL;
}

#if 0 /* unused */
/**
 * Sets 32-bit register in PCI configuration space.
 * @param   refPciDev   The PCI device.
 * @param   uOffset     The register offset.
 * @param   u32Value    The value to store in the register.
 * @thread  EMT
 */
DECLINLINE(void) vpciCfgSetU32(PDMPCIDEV& refPciDev, uint32_t uOffset, uint32_t u32Value)
{
    Assert(uOffset+sizeof(u32Value) <= sizeof(refPciDev.config));
    *(uint32_t*)&refPciDev.config[uOffset] = u32Value;
}
#endif /* unused */


#ifdef DEBUG
static void vpciDumpState(PVPCISTATE pState, const char *pcszCaller)
{
    Log2(("vpciDumpState: (called from %s)\n"
          "  uGuestFeatures = 0x%08x\n"
          "  uQueueSelector = 0x%04x\n"
          "  uStatus        = 0x%02x\n"
          "  uISR           = 0x%02x\n",
          pcszCaller,
          pState->uGuestFeatures,
          pState->uQueueSelector,
          pState->uStatus,
          pState->uISR));

    for (unsigned i = 0; i < pState->nQueues; i++)
        Log2((" %s queue:\n"
              "  VRing.uSize           = %u\n"
              "  VRing.addrDescriptors = %p\n"
              "  VRing.addrAvail       = %p\n"
              "  VRing.addrUsed        = %p\n"
              "  uNextAvailIndex       = %u\n"
              "  uNextUsedIndex        = %u\n"
              "  uPageNumber           = %x\n",
              pState->Queues[i].pcszName,
              pState->Queues[i].VRing.uSize,
              pState->Queues[i].VRing.addrDescriptors,
              pState->Queues[i].VRing.addrAvail,
              pState->Queues[i].VRing.addrUsed,
              pState->Queues[i].uNextAvailIndex,
              pState->Queues[i].uNextUsedIndex,
              pState->Queues[i].uPageNumber));
}
#else
# define vpciDumpState(x, s)  do {} while (0)
#endif

/**
 * Saves the state of device.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   pSSM        The handle to the saved state.
 */
int vpciSaveExec(PVPCISTATE pState, PSSMHANDLE pSSM)
{
    int rc;

    vpciDumpState(pState, "vpciSaveExec");

    rc = SSMR3PutU32(pSSM, pState->uGuestFeatures);
    AssertRCReturn(rc, rc);
    rc = SSMR3PutU16(pSSM, pState->uQueueSelector);
    AssertRCReturn(rc, rc);
    rc = SSMR3PutU8( pSSM, pState->uStatus);
    AssertRCReturn(rc, rc);
    rc = SSMR3PutU8( pSSM, pState->uISR);
    AssertRCReturn(rc, rc);

    /* Save queue states */
    rc = SSMR3PutU32(pSSM, pState->nQueues);
    AssertRCReturn(rc, rc);
    for (unsigned i = 0; i < pState->nQueues; i++)
    {
        rc = SSMR3PutU16(pSSM, pState->Queues[i].VRing.uSize);
        AssertRCReturn(rc, rc);
        rc = SSMR3PutU32(pSSM, pState->Queues[i].uPageNumber);
        AssertRCReturn(rc, rc);
        rc = SSMR3PutU16(pSSM, pState->Queues[i].uNextAvailIndex);
        AssertRCReturn(rc, rc);
        rc = SSMR3PutU16(pSSM, pState->Queues[i].uNextUsedIndex);
        AssertRCReturn(rc, rc);
    }

    return VINF_SUCCESS;
}

/**
 * Loads a saved device state.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   pSSM        The handle to the saved state.
 * @param   uVersion    The data unit version number.
 * @param   uPass       The data pass.
 */
int vpciLoadExec(PVPCISTATE pState, PSSMHANDLE pSSM, uint32_t uVersion, uint32_t uPass, uint32_t nQueues)
{
    int rc;

    if (uPass == SSM_PASS_FINAL)
    {
        /* Restore state data */
        rc = SSMR3GetU32(pSSM, &pState->uGuestFeatures);
        AssertRCReturn(rc, rc);
        rc = SSMR3GetU16(pSSM, &pState->uQueueSelector);
        AssertRCReturn(rc, rc);
        rc = SSMR3GetU8( pSSM, &pState->uStatus);
        AssertRCReturn(rc, rc);
        rc = SSMR3GetU8( pSSM, &pState->uISR);
        AssertRCReturn(rc, rc);

        /* Restore queues */
        if (uVersion > VIRTIO_SAVEDSTATE_VERSION_3_1_BETA1)
        {
            rc = SSMR3GetU32(pSSM, &pState->nQueues);
            AssertRCReturn(rc, rc);
        }
        else
            pState->nQueues = nQueues;
        AssertLogRelMsgReturn(pState->nQueues <= VIRTIO_MAX_NQUEUES, ("%#x\n", pState->nQueues), VERR_SSM_LOAD_CONFIG_MISMATCH);
        AssertLogRelMsgReturn(pState->uQueueSelector < pState->nQueues || (pState->nQueues == 0 && pState->uQueueSelector),
                              ("uQueueSelector=%u nQueues=%u\n", pState->uQueueSelector, pState->nQueues),
                              VERR_SSM_LOAD_CONFIG_MISMATCH);

        for (unsigned i = 0; i < pState->nQueues; i++)
        {
            rc = SSMR3GetU16(pSSM, &pState->Queues[i].VRing.uSize);
            AssertRCReturn(rc, rc);
            rc = SSMR3GetU32(pSSM, &pState->Queues[i].uPageNumber);
            AssertRCReturn(rc, rc);

            if (pState->Queues[i].uPageNumber)
                vqueueInit(&pState->Queues[i], pState->Queues[i].uPageNumber);

            rc = SSMR3GetU16(pSSM, &pState->Queues[i].uNextAvailIndex);
            AssertRCReturn(rc, rc);
            rc = SSMR3GetU16(pSSM, &pState->Queues[i].uNextUsedIndex);
            AssertRCReturn(rc, rc);
        }
    }

    vpciDumpState(pState, "vpciLoadExec");

    return VINF_SUCCESS;
}

/**
 * Set and populate the PCI capability list
 * 
 * Adds pci capabilities according to VIRTIO standard
 * 
 * @param   pci      Reference to PCI device structure
 * @param   cap_base location of the first capability
 */
void vpciSetCapabilityList(PPDMPCIDEV pci, uint8_t cap_base) {
    PDMPciDevSetStatus(pci, VBOX_PCI_STATUS_CAP_LIST);
    PDMPciDevSetCapabilityList(pci, cap_base);
    struct virtio_pci_cap tmp_cap = { 
        VBOX_PCI_CAP_ID_VNDR,       //cap_vndr
        cap_base+0x10,              //cap_next
        sizeof(virtio_pci_cap),     //cap_len
        VIRTIO_PCI_CAP_COMMON_CFG,  //cfg_type
        2,                          //bar
        {0,0,0},                    //padding
        RT_H2LE_U32(0x00001000),    //offset
        RT_H2LE_U32(0x00000800),    //length
    };
    memcpy(&pci->abConfig[0x40], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = cap_base+0x20;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_ISR_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00001800);
    tmp_cap.length = RT_H2LE_U32(0x00000800);
    memcpy(&pci->abConfig[0x50], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = cap_base+0x30;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_DEVICE_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00002000);
    tmp_cap.length = RT_H2LE_U32(0x00001000);
    memcpy(&pci->abConfig[0x60], &tmp_cap, sizeof(tmp_cap));

    tmp_cap.cap_next = 0x0;
    tmp_cap.cfg_type = VIRTIO_PCI_CAP_NOTIFY_CFG;
    tmp_cap.offset = RT_H2LE_U32(0x00003000);
    tmp_cap.length = RT_H2LE_U32(0x00001000);
    memcpy(&pci->abConfig[0x70], &tmp_cap, sizeof(tmp_cap));
}

/**
 * Set PCI configuration space registers.
 *
 * @param   pci          Reference to PCI device structure.
 * @param   uDeviceId    VirtiO Device Id
 * @param   uClass       Class of PCI device (network, etc)
 * @thread  EMT
 */
static DECLCALLBACK(void) vpciConfigure(PDMPCIDEV& pci,
                                        uint16_t uDeviceId,
                                        uint16_t uClass)
{
    /* Configure PCI Device, assume 32-bit mode ******************************/
    PDMPciDevSetVendorId(&pci, DEVICE_PCI_VENDOR_ID);
    PDMPciDevSetDeviceId(&pci, DEVICE_PCI_BASE_ID + uDeviceId);
    PDMPciDevSetSubSystemVendorId(&pci, DEVICE_PCI_SUBSYSTEM_VENDOR_ID);
    PDMPciDevSetSubSystemId(&pci, DEVICE_PCI_SUBSYSTEM_BASE_ID + uDeviceId);
    PDMPciDevSetWord(&pci, VBOX_PCI_CLASS_DEVICE, uClass);

    PDMPciDevSetRevisionId(&pci, 0x01);
    PDMPciDevSetClassProg(&pci, 0x00);
    /* Interrupt Pin: INTA# */
    PDMPciDevSetInterruptPin(&pci, 0x01);

    PDMPciDevSetBaseAddress(&pci, 2, false, true, true, 0x00000000);
    vpciSetCapabilityList(&pci, CAP_BASE);
}

#ifdef VBOX_WITH_STATISTICS
/* WARNING! This function must never be used in multithreaded context! */
static const char *vpciCounter(const char *pszDevFmt,
                               const char *pszCounter)
{
    static char s_szCounterName[80];

    RTStrPrintf(s_szCounterName, sizeof(s_szCounterName),
                "/Devices/%s/%s", pszDevFmt, pszCounter);

    return s_szCounterName;
}
#endif

PDMBOTHCBDECL(int) virtioModernCommonCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernCommonCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernISRWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernISRRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernDeviceCfgWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernDeviceCfgRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernNotifyWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}
PDMBOTHCBDECL(int) virtioModernNotifyRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS GCPhysAddr, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns, pvUser, GCPhysAddr, pv, cb);
    return VINF_SUCCESS;
}

DECLCALLBACK(int) virtioModernMap(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t iRegion,
                                RTGCPHYS GCPhysAddress, RTGCPHYS cb, PCIADDRESSSPACE enmType) {
    VPCISTATE *pThis = PDMINS_2_DATA(pDevIns, VPCISTATE *);
    
    int rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x0000, 0x1000, NULL, 0,
                                   virtioModernCommonCfgWrite, virtioModernCommonCfgRead, "VirtioPCICommonCfg");
    Assert(rc == VINF_SUCCESS);

    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x1000, 0x1000, NULL, 0,
                                   virtioModernISRWrite, virtioModernISRRead, "VirtioPCIISR");
    Assert(rc == VINF_SUCCESS);

    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x2000, 0x1000, NULL, 0,
                                   virtioModernDeviceCfgWrite, virtioModernDeviceCfgRead, "VirtioPCIDeviceCfg");
    Assert(rc == VINF_SUCCESS);
    
    rc = PDMDevHlpMMIORegister(pDevIns, GCPhysAddress+0x3000, 0x1000, NULL, 0,
                                   virtioModernNotifyWrite, virtioModernNotifyRead, "VirtioPCINotify");
    Assert(rc == VINF_SUCCESS);

    return rc;
}

/// @todo header
int vpciConstruct(PPDMDEVINS pDevIns, VPCISTATE *pState,
                  int iInstance, const char *pcszNameFmt,
                  uint16_t uDeviceId, uint16_t uClass,
                  uint32_t nQueues)
{
    /* Init handles and log related stuff. */
    RTStrPrintf(pState->szInstance, sizeof(pState->szInstance),
                pcszNameFmt, iInstance);

    pState->pDevInsR3    = pDevIns;
    pState->pDevInsR0    = PDMDEVINS_2_R0PTR(pDevIns);
    pState->pDevInsRC    = PDMDEVINS_2_RCPTR(pDevIns);

    /* Initialize critical section. */
    int rc = PDMDevHlpCritSectInit(pDevIns, &pState->cs, RT_SRC_POS, "%s", pState->szInstance);
    if (RT_FAILURE(rc))
        return rc;

    /* Set PCI config registers */
    vpciConfigure(pState->pciDevice, uDeviceId, uClass);
    /* Register PCI device */
    rc = PDMDevHlpPCIRegister(pDevIns, &pState->pciDevice);
    if (RT_FAILURE(rc))
        return rc;
    /* Register required MMIO Regions */
    rc = PDMDevHlpPCIIORegionRegister(pDevIns, 2, 0x0000000000004000, PCI_ADDRESS_SPACE_MEM_PREFETCH, virtioModernMap);
    if (RT_FAILURE(rc))
        return rc;
#ifdef VBOX_WITH_MSI_DEVICES
#if 0
    {
        PDMMSIREG aMsiReg;

        RT_ZERO(aMsiReg);
        aMsiReg.cMsixVectors = 1;
        aMsiReg.iMsixCapOffset = 0x80;
        aMsiReg.iMsixNextOffset = 0x0;
        aMsiReg.iMsixBar = 0;
        rc = PDMDevHlpPCIRegisterMsi(pDevIns, &aMsiReg);
        if (RT_FAILURE (rc))
            PCIDevSetCapabilityList(&pState->pciDevice, 0x0);
    }
#endif
#endif

    /* Status driver */
    PPDMIBASE pBase;
    rc = PDMDevHlpDriverAttach(pDevIns, PDM_STATUS_LUN, &pState->IBase, &pBase, "Status Port");
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach the status LUN"));

    pState->nQueues = nQueues;

#if defined(VBOX_WITH_STATISTICS)
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOReadR3,           STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO reads in R3",      vpciCounter(pcszNameFmt, "IO/ReadR3"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOReadR0,           STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO reads in R0",      vpciCounter(pcszNameFmt, "IO/ReadR0"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOReadRC,           STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO reads in RC",      vpciCounter(pcszNameFmt, "IO/ReadRC"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOWriteR3,          STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO writes in R3",     vpciCounter(pcszNameFmt, "IO/WriteR3"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOWriteR0,          STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO writes in R0",     vpciCounter(pcszNameFmt, "IO/WriteR0"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIOWriteRC,          STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling IO writes in RC",     vpciCounter(pcszNameFmt, "IO/WriteRC"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIntsRaised,         STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES,     "Number of raised interrupts",   vpciCounter(pcszNameFmt, "Interrupts/Raised"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatIntsSkipped,        STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES,     "Number of skipped interrupts",   vpciCounter(pcszNameFmt, "Interrupts/Skipped"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatCsR3,               STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling CS wait in R3",      vpciCounter(pcszNameFmt, "Cs/CsR3"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatCsR0,               STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling CS wait in R0",      vpciCounter(pcszNameFmt, "Cs/CsR0"), iInstance);
    PDMDevHlpSTAMRegisterF(pDevIns, &pState->StatCsRC,               STAMTYPE_PROFILE, STAMVISIBILITY_ALWAYS, STAMUNIT_TICKS_PER_CALL, "Profiling CS wait in RC",      vpciCounter(pcszNameFmt, "Cs/CsRC"), iInstance);
#endif /* VBOX_WITH_STATISTICS */

    return rc;
}

/**
 * Destruct PCI-related part of device.
 *
 * We need to free non-VM resources only.
 *
 * @returns VBox status code.
 * @param   pState      The device state structure.
 */
int vpciDestruct(VPCISTATE* pState)
{
    Log(("%s Destroying PCI instance\n", INSTANCE(pState)));

    if (PDMCritSectIsInitialized(&pState->cs))
        PDMR3CritSectDelete(&pState->cs);

    return VINF_SUCCESS;
}

/**
 * Device relocation callback.
 *
 * When this callback is called the device instance data, and if the
 * device have a GC component, is being relocated, or/and the selectors
 * have been changed. The device must use the chance to perform the
 * necessary pointer relocations and data updates.
 *
 * Before the GC code is executed the first time, this function will be
 * called with a 0 delta so GC pointer calculations can be one in one place.
 *
 * @param   pDevIns     Pointer to the device instance.
 * @param   offDelta    The relocation delta relative to the old location.
 *
 * @remark  A relocation CANNOT fail.
 */
void vpciRelocate(PPDMDEVINS pDevIns, RTGCINTPTR offDelta)
{
    RT_NOREF(offDelta);
    VPCISTATE *pState = PDMINS_2_DATA(pDevIns, VPCISTATE*);
    pState->pDevInsRC = PDMDEVINS_2_RCPTR(pDevIns);
    // TBD
}

PVQUEUE vpciAddQueue(VPCISTATE* pState, unsigned uSize, PFNVPCIQUEUECALLBACK pfnCallback, const char *pcszName)
{
    PVQUEUE pQueue = NULL;
    /* Find an empty queue slot */
    for (unsigned i = 0; i < pState->nQueues; i++)
    {
        if (pState->Queues[i].VRing.uSize == 0)
        {
            pQueue = &pState->Queues[i];
            break;
        }
    }

    if (!pQueue)
    {
        Log(("%s Too many queues being added, no empty slots available!\n", INSTANCE(pState)));
    }
    else
    {
        pQueue->VRing.uSize = uSize;
        pQueue->VRing.addrDescriptors = 0;
        pQueue->uPageNumber = 0;
        pQueue->pfnCallback = pfnCallback;
        pQueue->pcszName = pcszName;
    }

    return pQueue;
}

#endif /* IN_RING3 */

#endif /* VBOX_DEVICE_STRUCT_TESTCASE */

