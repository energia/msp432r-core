/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

#include <stdint.h>

/*
 * By default disable both asserts and log for this module.
 * This must be done before DebugP.h is included.
 */
#ifndef DebugP_ASSERT_ENABLED
#define DebugP_ASSERT_ENABLED 0
#endif
#ifndef DebugP_LOG_ENABLED
#define DebugP_LOG_ENABLED 0
#endif
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432DMA.h>
#include <ti/drivers/dma/UDMAMSP432.h>

/* driverlib header files */
#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/spi.h>
#include <ti/devices/msp432p4xx/driverlib/dma.h>
#include <ti/devices/msp432p4xx/driverlib/pmap.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>

#define PinConfigValue(config) (((config) >> 10) & 0x1F)
#define PinConfigModuleFunction(config) (((config) >> 8) & 0x3)
#define PinConfigPort(config) (((config) >> 4) & 0xF)
#define PinConfigPin(config) (1 << ((config) & 0x7))

/* SPIMSP432DMA functions */
void SPIMSP432DMA_close(SPI_Handle handle);
int_fast16_t SPIMSP432DMA_control(SPI_Handle handle, uint_fast16_t cmd, void *arg);
void SPIMSP432DMA_init(SPI_Handle handle);
SPI_Handle SPIMSP432DMA_open(SPI_Handle handle, SPI_Params *params);
bool SPIMSP432DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction);
void SPIMSP432DMA_transferCancel(SPI_Handle handle);
static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction);
static void configDMA(SPIMSP432DMA_Object *object,
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs, SPI_Transaction *transaction);
static void initHw(SPIMSP432DMA_Object *object,
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs, uint32_t inputClkFreq);
static int perfChangeNotifyFxn(unsigned int eventType, uintptr_t eventArg,
    uintptr_t clientArg);

static void mapPin(uint8_t port, uint8_t pin, uint8_t value);

/* SPI function table for SPIMSP432DMA implementation */
const SPI_FxnTable SPIMSP432DMA_fxnTable = {
    SPIMSP432DMA_close,
    SPIMSP432DMA_control,
    SPIMSP432DMA_init,
    SPIMSP432DMA_open,
    SPIMSP432DMA_transfer,
    SPIMSP432DMA_transferCancel
};

static const uint16_t frameFormat[] = {
    /* SPI_POL0_PHA0 */
    EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW |
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,

    /* SPI_POL0_PHA1 */
    EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW |
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,

    /* SPI_POL1_PHA0 */
    EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH |
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,

    /* SPI_POL1_PHA1 */
    EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH |
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,

    (0),    /* SPI_TI - Not supported */
    (0)     /* SPI_MW - Not supported */
};

/*
 *  ======== blockingTransferCallback ========
 *  Callback function for when the SPI is in SPI_MODE_BLOCKING
 *
 *  @pre    Function assumes that the handle is not NULL
 */
static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction)
{
    SPIMSP432DMA_Object *object = handle->object;

    SemaphoreP_post(object->transferComplete);

    DebugP_log1("SPI:(%p) posting transferComplete semaphore",
        ((SPIMSP432DMA_HWAttrsV1 const *)(handle->hwAttrs))->baseAddr);
}

/*
 *  ======== configDMA ========
 *  This functions configures the transmit and receive DMA channels for a given
 *  SPI_Handle and SPI_Transaction
 *
 *  @pre    Function assumes that the handle and transaction is not NULL
 */
static void configDMA(SPIMSP432DMA_Object *object,
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs, SPI_Transaction *transaction)
{
    uintptr_t key;
    void     *buf;
    uint32_t  channelControlOptions;

    if (transaction->txBuf) {
        channelControlOptions = UDMA_SIZE_8 | UDMA_SRC_INC_8 |
            UDMA_DST_INC_NONE | UDMA_ARB_1;
        buf = transaction->txBuf;
    }
    else {
        channelControlOptions = UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_NONE | UDMA_ARB_1;
        object->scratchBuffer = hwAttrs->defaultTxBufValue;
        buf = &(object->scratchBuffer);
    }

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelControl(hwAttrs->txDMAChannelIndex | UDMA_PRI_SELECT,
        channelControlOptions);
    MAP_DMA_setChannelTransfer(hwAttrs->txDMAChannelIndex | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC, buf,
        (void *) MAP_SPI_getTransmitBufferAddressForDMA(hwAttrs->baseAddr),
        transaction->count);

    if (transaction->rxBuf) {
        channelControlOptions = UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_8 | UDMA_ARB_1;
        buf = transaction->rxBuf;
    }
    else {
        channelControlOptions = UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_NONE | UDMA_ARB_1;
        buf = &(object->scratchBuffer);
    }

    /* Setup the RX transfer characteristics & buffers */
    MAP_DMA_setChannelControl(hwAttrs->rxDMAChannelIndex | UDMA_PRI_SELECT,
        channelControlOptions);
    MAP_DMA_setChannelTransfer(hwAttrs->rxDMAChannelIndex | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        (void *) MAP_SPI_getReceiveBufferAddressForDMA(hwAttrs->baseAddr),
        buf, transaction->count);

    /* A lock is needed because we are accessing shared uDMA registers.*/
    key = HwiP_disable();

    MAP_DMA_assignChannel(hwAttrs->rxDMAChannelIndex);
    MAP_DMA_assignChannel(hwAttrs->txDMAChannelIndex);

    /* Enable DMA interrupt */
    MAP_DMA_assignInterrupt(hwAttrs->dmaIntNum, hwAttrs->rxDMAChannelIndex & 0x0F);
    MAP_DMA_clearInterruptFlag(hwAttrs->rxDMAChannelIndex & 0x0F);
    MAP_DMA_enableInterrupt(hwAttrs->dmaIntNum);

    MAP_DMA_enableChannel(hwAttrs->rxDMAChannelIndex & 0x0F);
    MAP_DMA_enableChannel(hwAttrs->txDMAChannelIndex & 0x0F);

    HwiP_restore(key);

    DebugP_log1("SPI:(%p) DMA transfer enabled", hwAttrs->baseAddr);
    DebugP_log3("SPI:(%p) DMA transaction: %p, Count: %d", hwAttrs->baseAddr,
        (uintptr_t) transaction, (uintptr_t) transaction->count);
}

/*
 *  ======== initHW ========
 *
 *  Configures SPI peripheral
 */
static void initHw(SPIMSP432DMA_Object *object,
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs, uint32_t inputClkFreq)
{
    union {
        eUSCI_SPI_MasterConfig masterCfg;
        eUSCI_SPI_SlaveConfig  slaveCfg;
    } cfgStructs;

    if (object->spiMode == SPI_MASTER) {
        cfgStructs.masterCfg.selectClockSource = hwAttrs->clockSource;
        cfgStructs.masterCfg.clockSourceFrequency = inputClkFreq;
        cfgStructs.masterCfg.desiredSpiClock = object->bitRate;
        cfgStructs.masterCfg.msbFirst = hwAttrs->bitOrder;
        cfgStructs.masterCfg.clockPhase = object->clockPhase;
        cfgStructs.masterCfg.clockPolarity = object->clockPolarity;
        cfgStructs.masterCfg.spiMode = hwAttrs->pinMode;
        MAP_SPI_initMaster(hwAttrs->baseAddr, &(cfgStructs.masterCfg));

        if (hwAttrs->pinMode != EUSCI_SPI_3PIN) {
            MAP_SPI_selectFourPinFunctionality(hwAttrs->baseAddr, EUSCI_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);
        }
    }
    else { /* SPI_SLAVE */
        cfgStructs.slaveCfg.msbFirst = hwAttrs->bitOrder;
        cfgStructs.slaveCfg.clockPhase = object->clockPhase;
        cfgStructs.slaveCfg.clockPolarity = object->clockPolarity;
        cfgStructs.slaveCfg.spiMode = hwAttrs->pinMode;
        MAP_SPI_initSlave(hwAttrs->baseAddr, &(cfgStructs.slaveCfg));
    }

    DebugP_log3("SPI:(%p) CPU freq: %d; SPI freq to %d", hwAttrs->baseAddr,
        inputClkFreq, object->bitRate);
}

/*
 *  ======== perfChangeNotifyFxn ========
 *
 *  Called by Power module before and after performance level is changed.
 */
static int perfChangeNotifyFxn(unsigned int eventType, uintptr_t eventArg,
    uintptr_t clientArg)
{
    uint32_t                      clockFreq;
    PowerMSP432_Freqs             powerFreqs;
    SPIMSP432DMA_Object          *object = ((SPI_Handle) clientArg)->object;
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = ((SPI_Handle) clientArg)->hwAttrs;

    /* Get new performance level clock frequencies */
    PowerMSP432_getFreqs((unsigned int) eventArg, &powerFreqs);
    clockFreq = (hwAttrs->clockSource == EUSCI_B_SPI_CLOCKSOURCE_SMCLK) ?
        powerFreqs.SMCLK : powerFreqs.ACLK;

    /* Reconfigure SPI peripheral */
    initHw(object, hwAttrs, clockFreq);

    return (Power_NOTIFYDONE);
}

/*
 *  ======== SPIMSP432DMA_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void SPIMSP432DMA_close(SPI_Handle handle)
{
    unsigned int                  i;
    SPIMSP432DMA_Object          *object = handle->object;
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = handle->hwAttrs;

    MAP_SPI_disableModule(hwAttrs->baseAddr);

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
    }

    if (object->transferComplete) {
        SemaphoreP_delete(object->transferComplete);
    }

    if (object->dmaHandle) {
        UDMAMSP432_close(object->dmaHandle);
    }

    /* Remove power constraints */
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);
    for (i = 0; object->perfConstraintMask; i++) {
        if (object->perfConstraintMask & 0x01) {
            Power_releaseConstraint(PowerMSP432_DISALLOW_PERFLEVEL_0 + i);
        }
        object->perfConstraintMask >>= 1;
    }
    Power_unregisterNotify(&object->perfChangeNotify);

    object->isOpen = false;

    DebugP_log1("SPI:(%p) closed", hwAttrs->baseAddr);
}

/*
 *  ======== SPIMSP432DMA_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t SPIMSP432DMA_control(SPI_Handle handle, uint_fast16_t cmd, void *arg)
{
    /* No implementation yet */
    return (SPI_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== SPIMSP432DMA_hwiFxn ========
 *  ISR for the DMA Hwi
 */
void SPIMSP432DMA_hwiFxn(uintptr_t arg)
{
    SPI_Transaction              *msg;
    SPIMSP432DMA_Object          *object = ((SPI_Handle) arg)->object;
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = ((SPI_Handle) arg)->hwAttrs;

    DebugP_log1("SPI:(%p) interrupt context start", hwAttrs->baseAddr);

    /* Determine if the RX DMA channels has completed */
    if ((object->transaction) &&
        !(MAP_DMA_isChannelEnabled(hwAttrs->rxDMAChannelIndex & 0x0F))) {
        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;

        /* Perform callback */
        object->transferCallbackFxn((SPI_Handle) arg, msg);

        /* Remove constraints set during transfer */
        Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
        Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

        /* Disable peripheral to allow device to enter low power modes */
        MAP_SPI_disableModule(hwAttrs->baseAddr);

        DebugP_log2("SPI:(%p) DMA transaction: %p complete",
            hwAttrs->baseAddr, (uintptr_t) msg);
    }

    DebugP_log1("SPI:(%p) interrupt context end", hwAttrs->baseAddr);
}

/*
 *  ======== SPIMSP432DMA_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void SPIMSP432DMA_init(SPI_Handle handle)
{
    /* Relying on ELF to set object->isOpen to false */

    UDMAMSP432_init();
}

/*
 *  ======== SPIMSP432DMA_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
SPI_Handle SPIMSP432DMA_open(SPI_Handle handle, SPI_Params *params)
{
    unsigned int                  i;
    uintptr_t                     key;
    uint32_t                      clockFreq;
    uint8_t                       numPerfLevels;
    union {
        SemaphoreP_Params         semParams;
        HwiP_Params               hwiParams;
    } portsParams;
    PowerMSP432_Freqs             powerFreqs;
    SPIMSP432DMA_Object          *object = handle->object;
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uint16_t                      pin;
    uint16_t                      port;
    uint16_t                      value;
    uint16_t                      moduleFunction;

    /* Check for callback function if using async mode */
    DebugP_assert(params->transferMode != SPI_MODE_CALLBACK ||
        params->transferCallbackFxn != NULL);

    /* Determine if the device index was already opened */
    key = HwiP_disable();

    if (object->isOpen) {
        HwiP_restore(key);
        DebugP_log1("SPI:(%p) already open.", hwAttrs->baseAddr);
        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Ensure a supported clock source is used */
    if (hwAttrs->clockSource != EUSCI_B_SPI_CLOCKSOURCE_ACLK &&
        hwAttrs->clockSource != EUSCI_B_SPI_CLOCKSOURCE_SMCLK) {
        DebugP_log1("SPI:(%p) Error! Using unsupported clock source.",
            hwAttrs->baseAddr);
        object->isOpen = false;
        return (NULL);
    }

    /* Map the pins */
    /* CLK */
    port = PinConfigPort(hwAttrs->clkPin);
    value = PinConfigValue(hwAttrs->clkPin);

    if (value != 0) {
        moduleFunction = GPIO_PRIMARY_MODULE_FUNCTION;
        pin = (hwAttrs->clkPin) & 0x7;
        mapPin(port, pin, value);
    }
    else {
        moduleFunction = PinConfigModuleFunction(hwAttrs->clkPin);
    }

    pin  = PinConfigPin(hwAttrs->clkPin);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port,
            pin, moduleFunction);

    /* STE */
    if (hwAttrs->pinMode != EUSCI_SPI_3PIN) {
        port = PinConfigPort(hwAttrs->stePin);
        value = PinConfigValue(hwAttrs->stePin);

        if (value != 0) {
            moduleFunction = GPIO_PRIMARY_MODULE_FUNCTION;
            pin = (hwAttrs->stePin) & 0x7;
            mapPin(port, pin, value);
        }
        else {
            moduleFunction = PinConfigModuleFunction(hwAttrs->stePin);
        }

        pin  = PinConfigPin(hwAttrs->stePin);
        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port,
                pin, moduleFunction);
    }

    /* SIMO */
    port = PinConfigPort(hwAttrs->simoPin);
    value = PinConfigValue(hwAttrs->simoPin);

    if (value != 0) {
        moduleFunction = GPIO_PRIMARY_MODULE_FUNCTION;
        pin = (hwAttrs->simoPin) & 0x7;
        mapPin(port, pin, value);
    }
    else {
        moduleFunction = PinConfigModuleFunction(hwAttrs->simoPin);
    }

    pin  = PinConfigPin(hwAttrs->simoPin);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port,
            pin, moduleFunction);

    /* SOMI */
    port = PinConfigPort(hwAttrs->somiPin);
    value = PinConfigValue(hwAttrs->somiPin);

    if (value != 0) {
        moduleFunction = GPIO_PRIMARY_MODULE_FUNCTION;
        pin = (hwAttrs->somiPin) & 0x7;
        mapPin(port, pin, value);
    }
    else {
        moduleFunction = PinConfigModuleFunction(hwAttrs->somiPin);
    }

    pin  = PinConfigPin(hwAttrs->somiPin);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(port,
            pin, moduleFunction);
    /*
     * Add power management support - Disable performance transitions while
     * opening the driver.
     */
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /*
     * Verify that the driver can be opened at current performance level and
     * set constraints for other performance levels.
     */
    numPerfLevels = PowerMSP432_getNumPerfLevels();
    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &powerFreqs);
    if (hwAttrs->clockSource == EUSCI_B_SPI_CLOCKSOURCE_ACLK) {
        /*
         * Verify if driver can be opened with ACLK; ACLK does not change
         * in any performance level.
         */
        if (params->bitRate >= powerFreqs.ACLK) {
            DebugP_log3("SPI:(%p) ACLK input (%d) too low for bit rate (%d)",
                hwAttrs->baseAddr, powerFreqs.ACLK, params->bitRate);
            Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
            object->isOpen = false;
            return (NULL);
        }
        clockFreq = powerFreqs.ACLK;
    }
    else {    /* hwAttrs->clockSource == EUSCI_B_SPI_CLOCKSOURCE_SMCLK */
        if (params->bitRate >= powerFreqs.SMCLK) {
            DebugP_log3("SPI:(%p) SMCLK input (%d) too low for bit rate (%d)",
                hwAttrs->baseAddr, powerFreqs.SMCLK, params->bitRate);
            Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
            object->isOpen = false;
            return (NULL);
        }
        clockFreq = powerFreqs.SMCLK;

        /*
         * SMCLK changes with performance levels.  Set constraints for
         * unsupported performance levels.
         */
        for (i = 0; i < numPerfLevels; i++) {
            PowerMSP432_getFreqs(i, &powerFreqs);
            if (params->bitRate >= powerFreqs.SMCLK) {
                /* Set constraint and keep track of it in perfConstraintMask */
                object->perfConstraintMask |= (1 << i);
                Power_setConstraint(PowerMSP432_DISALLOW_PERFLEVEL_0 + i);
            }
        }
    }

    /* Shutdown not supported while driver is open */
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);

    /* Register function to reconfigure peripheral on perf level changes */
    Power_registerNotify(&object->perfChangeNotify,
        PowerMSP432_DONE_CHANGE_PERF_LEVEL, perfChangeNotifyFxn,
        (uintptr_t) handle);

    object->transferComplete = NULL;
    object->dmaHandle = NULL;

    /* Create the Hwi for this SPI peripheral */
    HwiP_Params_init(&(portsParams.hwiParams));
    portsParams.hwiParams.arg = (uintptr_t) handle;
    portsParams.hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->dmaIntNum, SPIMSP432DMA_hwiFxn,
        &(portsParams.hwiParams));
    if (!object->hwiHandle) {
        DebugP_log1("SPI:(%p) HwiP_create() failed", hwAttrs->baseAddr);
        Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
        SPIMSP432DMA_close(handle);
        return (NULL);
    }

    object->dmaHandle = UDMAMSP432_open();
    if (object->dmaHandle == NULL) {
        DebugP_log1("SPI:(%p) UDMAMSP432_open() failed.", hwAttrs->baseAddr);
        Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
        SPIMSP432DMA_close(handle);
        return (NULL);
    }

    /* Configure driver to Callback or Blocking operating mode */
    if (params->transferMode == SPI_MODE_CALLBACK) {
        object->transferCallbackFxn = params->transferCallbackFxn;
        DebugP_log1("SPI:(%p) in SPI_MODE_CALLBACK mode", hwAttrs->baseAddr);
    }
    else {
        /* Semaphore to block task for the duration of the SPI transfer */
        SemaphoreP_Params_init(&(portsParams.semParams));
        portsParams.semParams.mode = SemaphoreP_Mode_BINARY;
        object->transferComplete = SemaphoreP_create(0, &(portsParams.semParams));
        if (!object->transferComplete) {
            DebugP_log1("SPI:(%p) transfer SemaphoreP_create() failed.",
                hwAttrs->baseAddr);
            Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
            SPIMSP432DMA_close(handle);
            return (NULL);
        }

        object->transferCallbackFxn = blockingTransferCallback;
        DebugP_log1("SPI:(%p) in SPI_MODE_BLOCKING mode", hwAttrs->baseAddr);
    }

    /*
     * Store SPI parameters & initialize peripheral.  These are used to
     * re/initialize the peripheral when opened or changing performance level.
     */
    object->spiMode = params->mode;
    object->bitRate = params->bitRate;
    object->clockPhase = frameFormat[params->frameFormat] & UCCKPH;
    object->clockPolarity = frameFormat[params->frameFormat] & UCCKPL;
    object->transferMode = params->transferMode;
    object->transaction = NULL;
    initHw(object, hwAttrs, clockFreq);

    /* Allow performance level changes */
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    DebugP_log1("SPI:(%p) opened", hwAttrs->baseAddr);

    return (handle);
}

/*
 *  ======== SPIMSP432DMA_transfer ========
 *  @pre    Function assumes that handle and transaction is not NULL
 */
bool SPIMSP432DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    uintptr_t                     key;
    SPIMSP432DMA_Object          *object = handle->object;
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = handle->hwAttrs;

    /* Check the transaction arguments */
    if ((transaction->count == 0) ||
        !(transaction->rxBuf || transaction->txBuf)) {
        return (false);
    }

    /* Check if a transfer is in progress */
    key = HwiP_disable();

    if (object->transaction) {
        /* Transfer is in progress */
        HwiP_restore(key);
        DebugP_log1("SPI:(%p) Error! Transaction still in progress",
            hwAttrs->baseAddr);
        return (false);
    }
    else {
        /* Save the pointer to the transaction */
        object->transaction = transaction;
    }

    HwiP_restore(key);

    /*
     * Set power constraints to keep peripheral active during transfer and
     * to prevent a performance level change
     */
    Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    MAP_SPI_enableModule(hwAttrs->baseAddr);

    configDMA(object, hwAttrs, transaction);

    if (object->transferMode == SPI_MODE_BLOCKING) {
        DebugP_log1("SPI:(%p) transfer pending on transferComplete semaphore",
            hwAttrs->baseAddr);

        SemaphoreP_pend(object->transferComplete, SemaphoreP_WAIT_FOREVER);
    }

    return (true);
}

/*
 *  ======== SPIMSP432DMA_transferCancel ========
 *  A function to cancel a transaction (if one is in progress) when the driver
 *  is in SPI_MODE_CALLBACK.
 *
 *  @pre    Function assumes that the handle is not NULL
 */
void SPIMSP432DMA_transferCancel(SPI_Handle handle)
{
    SPIMSP432DMA_HWAttrsV1 const *hwAttrs = handle->hwAttrs;

    /* No implementation yet */
    DebugP_assert(false);

    /* Remove constraints set during transfer */
    Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Disable peripheral to allow device to enter low power modes */
    MAP_SPI_disableModule(hwAttrs->baseAddr);
}

/*
 *  ======== mapPin ========
 *  NOTE: This function may go away when DriverLib supports mapping
 *  an individual pin.
 */
static void mapPin(uint8_t port, uint8_t pin, uint8_t value)
{
    volatile uint8_t                     pmap;

    pmap = port * 0x8;  // 2 -> 0x10, 3 -> 0x18, 7 -> 0x38

    //portMapReconfigure = PMAP_DISABLE_RECONFIGURATION; // ?

    /*  Code from pmap.c: */
    //Get write-access to port mapping registers:
    PMAP->KEYID = PMAP_KEYID_VAL;

    //Enable/Disable reconfiguration during runtime
//    PMAP->CTL = (PMAP->CTL & ~PMAP_CTL_PRECFG) | PMAP_DISABLE_RECONFIGURATION;
    PMAP->CTL = (PMAP->CTL & ~PMAP_CTL_PRECFG) | PMAP_ENABLE_RECONFIGURATION;
    HWREG8((uint32_t)PMAP_BASE + pin + pmap) = value;
    //Disable write-access to port mapping registers:

    PMAP->KEYID = 0;
}
