/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>

#include <ti/drivers/dma/UDMAMSP432.h>

#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/dma.h>

extern const UDMAMSP432_Config UDMAMSP432_config[];

static bool dmaInitialized = false;

/* Reference count for open calls */
static uint32_t          refCount = 0;

/*
 *  ======== UDMAMSP432_close ========
 */
void UDMAMSP432_close(UDMAMSP432_Handle handle)
{
    UDMAMSP432_Object    *object = handle->object;
    uintptr_t             key;

    key = HwiP_disable();

    refCount--;

    if (refCount == 0) {
        object->isOpen = false;
    }

    HwiP_restore(key);
}

/*
 *  ======== UDMAMSP432_init ========
 */
void UDMAMSP432_init()
{
    HwiP_Params           hwiParams;
    UDMAMSP432_Handle     handle = (UDMAMSP432_Handle)&(UDMAMSP432_config[0]);
    UDMAMSP432_HWAttrs    const *hwAttrs = handle->hwAttrs;
    UDMAMSP432_Object    *object = handle->object;

    if (!dmaInitialized) {
        object->isOpen = false;

        HwiP_Params_init(&hwiParams);
        hwiParams.priority = hwAttrs->intPriority;

        /* Will check in UDMAMSP432_open() if this failed */
        object->hwiHandle = HwiP_create(hwAttrs->intNum, hwAttrs->dmaErrorFxn,
                &hwiParams);
        if (object->hwiHandle == NULL) {
            DebugP_log0("Failed to create uDMA error Hwi!!\n");
        }
        else {
            dmaInitialized = true;
        }
    }
}

/*
 *  ======== UDMAMSP432_open ========
 */
UDMAMSP432_Handle UDMAMSP432_open()
{
    UDMAMSP432_Handle     handle = (UDMAMSP432_Handle)&(UDMAMSP432_config);
    UDMAMSP432_Object    *object = handle->object;
    UDMAMSP432_HWAttrs    const *hwAttrs = handle->hwAttrs;
    uintptr_t             key;

    if (!dmaInitialized) {
        return (NULL);
    }

    key = HwiP_disable();

    /*
     *  If the UDMA has not been opened yet, create the error Hwi
     *  and initialize the control table base address.
     */
    if (object->isOpen == false) {
        MAP_DMA_enableModule();
        MAP_DMA_setControlBase(hwAttrs->controlBaseAddr);

        object->isOpen = true;
    }

    refCount++;

    HwiP_restore(key);

    return (handle);
}
