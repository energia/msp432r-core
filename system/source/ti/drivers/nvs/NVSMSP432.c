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

/*
 *  ======== NVSMSP432.c ========
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>  /* for string support */
#include <stdlib.h>

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

/* driverlib header files */
#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/flash.h>

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSMSP432.h>

#define FLASH_PAGE_SIZE  (0x1000)  /* 4-KB */

#define BANK0_END_ADDRESS (0x0001ffff)
#define BANK1_END_ADDRESS (0x0003ffff)

#define GET_BANK(address) ((address > BANK0_END_ADDRESS) ? \
        FLASH_MAIN_MEMORY_SPACE_BANK1 : FLASH_MAIN_MEMORY_SPACE_BANK0)

/* NVSMSP432 functions */
void        NVSMSP432_close(NVS_Handle handle);
int         NVSMSP432_control(NVS_Handle handle, unsigned int cmd,
                                uintptr_t arg);
void        NVSMSP432_exit(NVS_Handle handle);
int         NVSMSP432_getAttrs(NVS_Handle handle, NVS_Attrs *attrs);
void        NVSMSP432_init(NVS_Handle handle);
NVS_Handle  NVSMSP432_open(NVS_Handle handle, NVS_Params *params);
int         NVSMSP432_read(NVS_Handle handle, size_t offset, void *buffer,
                             size_t bufferSize);
int         NVSMSP432_write(NVS_Handle handle, size_t offset, void *buffer,
                              size_t bufferSize, unsigned int flags);

/* NVS function table for NVSMSP432 implementation */
const NVS_FxnTable NVSMSP432_fxnTable = {
    NVSMSP432_close,
    NVSMSP432_control,
    NVSMSP432_exit,
    NVSMSP432_getAttrs,
    NVSMSP432_init,
    NVSMSP432_open,
    NVSMSP432_read,
    NVSMSP432_write
};

static uint32_t getFlashSector(uint32_t address);

static uint32_t flashSector[32] = {
    FLASH_SECTOR0,
    FLASH_SECTOR1,
    FLASH_SECTOR2,
    FLASH_SECTOR3,
    FLASH_SECTOR4,
    FLASH_SECTOR5,
    FLASH_SECTOR6,
    FLASH_SECTOR7,
    FLASH_SECTOR8,
    FLASH_SECTOR9,
    FLASH_SECTOR10,
    FLASH_SECTOR11,
    FLASH_SECTOR12,
    FLASH_SECTOR13,
    FLASH_SECTOR14,
    FLASH_SECTOR15,
    FLASH_SECTOR16,
    FLASH_SECTOR17,
    FLASH_SECTOR18,
    FLASH_SECTOR19,
    FLASH_SECTOR20,
    FLASH_SECTOR21,
    FLASH_SECTOR22,
    FLASH_SECTOR23,
    FLASH_SECTOR24,
    FLASH_SECTOR25,
    FLASH_SECTOR26,
    FLASH_SECTOR27,
    FLASH_SECTOR28,
    FLASH_SECTOR29,
    FLASH_SECTOR30,
    FLASH_SECTOR31,
};

/*
 *  Semaphore to synchronize access to flash block.
 */
static Semaphore_Struct  writeSem;
static bool isInitialized = false;

/*
 *  ======== NVSMSP432_close ========
 */
void NVSMSP432_close(NVS_Handle handle)
{
}

/*
 *  ======== NVSMSP432_control ========
 */
int NVSMSP432_control(NVS_Handle handle, unsigned int cmd, uintptr_t arg)
{
    NVSMSP432_HWAttrs *hwAttrs = (NVSMSP432_HWAttrs *)(handle->hwAttrs);
    NVSMSP432_CmdSetCopyBlockArgs *cmdArgs = (NVSMSP432_CmdSetCopyBlockArgs *)arg;
    uint8_t *copyBlock = (uint8_t *)(cmdArgs->copyBlock);

    if (cmd == NVSMSP432_CMD_SET_COPYBLOCK) {
        if ((copyBlock == NULL) || ((uint32_t)copyBlock & 0x3)) {
            return (NVSMSP432_STATUS_ECOPYBLOCK);
        }
        hwAttrs->copyBlock = cmdArgs->copyBlock;
        hwAttrs->isRam = cmdArgs->isRam;

        return (NVS_STATUS_SUCCESS);
    }

    return (NVS_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== NVSMSP432_exit ========
 */
void NVSMSP432_exit(NVS_Handle handle)
{
}

/*
 *  ======== NVSMSP432_getAttrs ========
 */
int NVSMSP432_getAttrs(NVS_Handle handle, NVS_Attrs *attrs)
{
    NVSMSP432_HWAttrs const  *hwAttrs = handle->hwAttrs;

    attrs->pageSize   = FLASH_PAGE_SIZE;
    attrs->blockSize  = hwAttrs->blockSize;

    return (NVS_SOK);
}

/*
 *  ======== NVSMSP432_init ========
 */
void NVSMSP432_init(NVS_Handle handle)
{
    if (!isInitialized) {
        Semaphore_construct(&writeSem, 1, NULL);
        isInitialized = true;
    }
}

/*
 *  ======== NVSMSP432_open =======
 */
NVS_Handle NVSMSP432_open(NVS_Handle handle, NVS_Params *params)
{
    NVSMSP432_Object         *object = handle->object;
    NVSMSP432_HWAttrs const  *hwAttrs = handle->hwAttrs;
    bool                      status;

    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    if (object->opened == true) {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) already in use.", (IArg)(hwAttrs->block));
        return (NULL);
    }

    /* The block must lie in the main memory (0 - 0x40000) */
    if ((uint32_t)(hwAttrs->block) > BANK1_END_ADDRESS) {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) Block is out of range (0 - 0x40000).",
                (IArg)(hwAttrs->block));
        return (NULL);
    }

    /* The block must be aligned on a flaah page boundary */
    if ((uint32_t)(hwAttrs->block) & (FLASH_PAGE_SIZE - 1)) {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) block not aligned on flash page boundary.",
                (IArg)(hwAttrs->block));
        return (NULL);
    }

    /* The block cannot be larger than a flash page */
    if ((uint32_t)(hwAttrs->blockSize) > FLASH_PAGE_SIZE) {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) blockSize must not be greater than page size.",
                (IArg)(hwAttrs->block));
        return (NULL);
    }

    /* Check flash copy block */
    if (hwAttrs->copyBlock && !(hwAttrs->isRam)) {
        /* Flash copy block must be aligned on a flaah page boundary */
        if (((uint32_t)(hwAttrs->copyBlock) & (FLASH_PAGE_SIZE - 1))) {
            Semaphore_post(Semaphore_handle(&writeSem));

            Log_warning1("NVS:(%p) Flash copyBlock not page boundary aligned.",
                    (IArg)(hwAttrs->block));
            return (NULL);
        }

        /* Flash copy block must be in main memory */
        if ((uint32_t)(hwAttrs->copyBlock) > BANK1_END_ADDRESS) {
            Semaphore_post(Semaphore_handle(&writeSem));

            Log_warning1("NVS:(%p) Copy block is out of range (0 - 0x40000).",
                    (IArg)(hwAttrs->block));
            return (NULL);
        }

        if ((uint32_t)(hwAttrs->copyBlock) == (uint32_t)(hwAttrs->block)) {
            Semaphore_post(Semaphore_handle(&writeSem));

            Log_warning1("NVS:(%p) Bad copy block address.",
                    (IArg)(hwAttrs->block));
            return (NULL);
        }
    }

    /* Ram copy block must be 4-byte aligned */
    if ((uint32_t)(hwAttrs->copyBlock) & 0x3) {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) copyBlock not 4-byte aligned.",
                (IArg)(hwAttrs->block));
        return (NULL);
    }

    object->opened = true;
    object->bank = GET_BANK((uint32_t)(hwAttrs->block));
    object->sector = getFlashSector((uint32_t)(hwAttrs->block));

    if (!hwAttrs->isRam) {
        object->copyBank = GET_BANK((uint32_t)(hwAttrs->copyBlock));
        object->copySector = getFlashSector((uint32_t)(hwAttrs->copyBlock));
    }
    else {
        object->copyBank = (uint32_t)(-1);
        object->copySector = (uint32_t)(-1);
    }

    Semaphore_post(Semaphore_handle(&writeSem));

    if (params->eraseOnOpen == true) {
        MAP_FlashCtl_unprotectSector(object->bank, object->sector);
        status = MAP_FlashCtl_eraseSector((uint32_t)hwAttrs->block);
        MAP_FlashCtl_protectSector(object->bank, object->sector);

        if (!status) {
            Log_warning1("NVS:(%p) FlashCtl_eraseSector() failed.",
                    (IArg)(hwAttrs->block));
        }
    }

    return (handle);
}

/*
 *  ======== NVSMSP432_read =======
 */
int NVSMSP432_read(NVS_Handle handle, size_t offset, void *buffer,
        size_t bufferSize)
{
    NVSMSP432_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int retval = NVS_SOK;

    /* Validate offset and bufferSize */
    if (offset + bufferSize > hwAttrs->blockSize) {
        return (NVS_EOFFSET);
    }

    /*
     *  Get exclusive access to the block.  We don't want someone
     *  else to erase the block while we are reading it.
     */
    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    memcpy(buffer, (Char *)(hwAttrs->block) + offset, bufferSize);

    Semaphore_post(Semaphore_handle(&writeSem));

    return (retval);
}

/*
 *  ======== NVSMSP432_write =======
 */
int NVSMSP432_write(NVS_Handle handle, size_t offset, void *buffer,
                      size_t bufferSize, unsigned int flags)
{
    NVSMSP432_HWAttrs const  *hwAttrs = handle->hwAttrs;
    NVSMSP432_Object         *object = handle->object;
    unsigned int size;
    bool status = true;
    int i;
    uint8_t *srcBuf, *dstBuf;
    int retval = NVS_SOK;

    /* Buffer to copy into flash must be 4-byte aligned */
    if ((uint32_t)buffer & 0x3) {
        Log_warning1("NVS:(%p) Buffer must be 4-byte aligned.",
                (IArg)(hwAttrs->block));
        return (NVS_EALIGN);
    }

    /* Error if bufferSize is not a multiple of 4 */
    if (bufferSize & 0x3) {
        Log_warning1("NVS:(%p) Buffer size must be 4-byte aligned.",
                (IArg)(hwAttrs->block));
        return (NVS_EALIGN);
    }

    /* Check if offset is not a multiple of 4 */
    if (offset & 0x3) {
        Log_warning1("NVS:(%p) offset size must be 4-byte aligned.",
                (IArg)(hwAttrs->block));
        return (NVS_EALIGN);
    }

    /* Validate offset and bufferSize */
    if (offset + bufferSize > hwAttrs->blockSize) {
        return (NVS_EOFFSET);
    }

    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    if (buffer == NULL) {
        /* NULL buffer ==> Erase the block */
        MAP_FlashCtl_unprotectSector(object->bank, object->sector);
        status = MAP_FlashCtl_eraseSector((uint32_t)hwAttrs->block);
        MAP_FlashCtl_protectSector(object->bank, object->sector);

        Semaphore_post(Semaphore_handle(&writeSem));

        if (!status) {
            Log_warning1("NVS:(%p) FlashCtl_eraseSector() failed.",
                    (IArg)(hwAttrs->block));
        }
        return (status ? NVS_SOK : NVS_EFAIL);
    }

    /*
     *  If exclusive write, check that the region has not been
     *  written to since the last erase.  (Erasing leaves flash
     *  set to 0xFF)
     */
    if (flags & NVS_WRITE_EXCLUSIVE) {
        dstBuf = (uint8_t *)((uint32_t)(hwAttrs->block) + offset);
        for (i = 0; i < bufferSize; i++) {
            if (dstBuf[i] != 0xFF) {
                Semaphore_post(Semaphore_handle(&writeSem));
                return (NVS_EALREADYWRITTEN);
            }
        }
    }

    /* If erase is set, determine whether to use RAM or the flash copyBlock */
    if (flags & NVS_WRITE_ERASE) {

        /* Must have copy block for erase */
        if (hwAttrs->copyBlock == NULL) {
            Semaphore_post(Semaphore_handle(&writeSem));
            Log_warning1("NVS:(%p) copyBlock must be non-NULL.",
                    (IArg)(hwAttrs->block));
            return (NVS_ECOPYBLOCK);
        }

        srcBuf = (uint8_t *)(hwAttrs->copyBlock);

        if (hwAttrs->isRam) {
            /* Copy flash contents up to the offset into temporary buffer */
            memcpy(srcBuf, hwAttrs->block, offset);

            /* Update the temporary buffer with the data to be written */
            memcpy((void *)((uint32_t)srcBuf + offset), buffer, bufferSize);

            /* Copy remaining flash contents into temporary buffer */
            memcpy(srcBuf + offset + bufferSize,
                    (void *)((uint32_t)hwAttrs->block + offset + bufferSize),
                    hwAttrs->blockSize - bufferSize - offset);
        }
        else {
            /* Using extra flash region to maintain copy - erase copy block */
            MAP_FlashCtl_unprotectSector(object->copyBank, object->copySector);
            status = MAP_FlashCtl_eraseSector((uint32_t)hwAttrs->copyBlock);

            /*  Copy up to offset */
            status &= MAP_FlashCtl_programMemory(
                    hwAttrs->block,                 /* src  */
                    hwAttrs->copyBlock,             /* dst  */
                    (uint32_t)offset);              /* size */

            /*  Copy buffer */
            status &= MAP_FlashCtl_programMemory(buffer,
                    (void *)((uint32_t)(hwAttrs->copyBlock) + offset),
                    (uint32_t)bufferSize);

            /*  Copy after offset + bufferSize */
            status &= MAP_FlashCtl_programMemory(
                (void *)((uint32_t)hwAttrs->block + offset + bufferSize),
                (void *)((uint32_t)(hwAttrs->copyBlock) + offset + bufferSize),
                hwAttrs->blockSize - bufferSize - offset);

            MAP_FlashCtl_protectSector(object->copyBank, object->copySector);
        }

        /* Erase the block */
        MAP_FlashCtl_unprotectSector(object->bank, object->sector);
        status &= MAP_FlashCtl_eraseSector((uint32_t)hwAttrs->block);
        MAP_FlashCtl_protectSector(object->bank, object->sector);

        dstBuf = hwAttrs->block;
        size = hwAttrs->blockSize;
    }
    else {
        /* Not erasing the block before writing */
        srcBuf = buffer;
        size   = bufferSize;
        dstBuf = (uint8_t *)((uint32_t)(hwAttrs->block) + offset);
    }

    MAP_FlashCtl_unprotectSector(object->bank, object->sector);
    status &= MAP_FlashCtl_programMemory((void *)srcBuf, (void *)dstBuf, size);
    MAP_FlashCtl_protectSector(object->bank, object->sector);

    if (!status) {
        retval = NVS_EFAIL;
    }
    else if ((flags & NVS_WRITE_VALIDATE)) {
        /*
         *  Note: This validates the entire block even on erase mode.
         */
        for (i = 0; i < size; i++) {
            if (srcBuf[i] != dstBuf[i]) {
                retval = NVS_EFAIL;
                break;
            }
        }
    }

    Semaphore_post(Semaphore_handle(&writeSem));

    return (retval);
}

/*
 *  ======== getFlashSector ========
 */
static uint32_t getFlashSector(uint32_t address)
{
    int index;

    index = (address & 0x1FFFF) >> 12;
    return (flashSector[index]);
}
