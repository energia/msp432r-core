/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
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
#include <ti/drivers/capture/CaptureMSP432.h>
#include <ti/drivers/timer/TimerMSP432.h>
#include <ti/drivers/power/PowerMSP432.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include <ti/devices/msp432p4xx/driverlib/pmap.h>
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/interrupt.h>
#include <ti/drivers/dpl/HwiP.h>
#include <stdint.h>

/* Hwi Function */
void CaptureMSP432_hwiIntFunction(uintptr_t arg);

/* Function for port mapping */
static void mapPin(uint8_t port, uint8_t pin, uint8_t value);

/* Function table */
Capture_FxnTable CaptureMSP432_captureFxnTable =
{
    .closeFxn =   CaptureMSP432_close,
    .openFxn  =   CaptureMSP432_open,
    .startFxn =   CaptureMSP432_start,
    .stopFxn =    CaptureMSP432_stop,
    .initFxn =    CaptureMSP432_init,
    .controlFxn = CaptureMSP432_control
};

/* Timer_A Capture Mode Configuration Parameter */
static Timer_A_CaptureModeConfig captureModeConfig =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        /* CC Register 1 */
    TIMER_A_CAPTUREMODE_RISING_EDGE,          /* Rising Edge */
    TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        /* CCIxB Input Select */
    TIMER_A_CAPTURE_SYNCHRONOUS,              /* Synchronized Capture */
    TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  /* Enable interrupt */
    TIMER_A_OUTPUTMODE_OUTBITVALUE            /* Output bit value */
};

/* Timer_A Continuous Mode Configuration Parameter */
static Timer_A_ContinuousModeConfig continuousModeConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,           /* SMCLK Clock Source */
    TIMER_A_CLOCKSOURCE_DIVIDER_1,       /* SMCLK/1 = 3MHz */
    TIMER_A_TAIE_INTERRUPT_DISABLE,      /* Disable Timer ISR */
    TIMER_A_DO_CLEAR                     /*  Clear Counter */
};

/*
 *  ======== Capture_close ========
 */
void CaptureMSP432_close(Capture_Handle handle)
{
    CaptureMSP432_HWAttrs *myAttrs = (CaptureMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_freeTimerResource(myAttrs->timerBaseAddress);

    /* Remove power constraints */
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);
}

/*
 *  ======== Capture_control ========
 */
int_fast16_t CaptureMSP432_control(Capture_Handle handle, uint_fast16_t cmd,
        void *arg)
{
    uintptr_t key;
    key = HwiP_disable();

    switch (cmd)
    {
    default:
        HwiP_restore(key);
        return CAPTURE_STATUS_UNDEFINEDCMD;
    }
}

/*
 *  ======== Capture_hwiIntFunction ========
 */
void CaptureMSP432_hwiIntFunction(uintptr_t arg)
{
    Capture_Handle handle = (Capture_Handle) arg;
    PowerMSP432_Freqs curFrequencies;
    CaptureMSP432_HWAttrs *hwAttrs = (CaptureMSP432_HWAttrs*) handle->hwAttrs;
    CaptureMSP432_Object *myObject = (CaptureMSP432_Object*) handle->object;
    uint32_t baseAddress, returnCount, periodUnits, clockFreq;

    /* Parsing out relevant areas of the object/hardware */
    baseAddress = hwAttrs->timerBaseAddress;
    periodUnits = myObject->periodUnits;

    /* Clearing our interrupt */
    Timer_A_clearCaptureCompareInterrupt(baseAddress,
            myObject->ccrRegister);


    /* If we are on the first capture, we want to record our value and return */
    if (myObject->captureCount == 0)
    {
        myObject->captureCount = Timer_A_getCaptureCompareCount(baseAddress,
                myObject->ccrRegister);
        return;
    }

    /* Otherwise calculate the difference and invoke the callback */
    returnCount = Timer_A_getCaptureCompareCount(baseAddress,
            myObject->ccrRegister) - myObject->captureCount;

    /* Resetting the return count */
    myObject->captureCount = 0;

    /* If the units are set to counts, we just need to return the count */
    if (periodUnits == CAPTURE_PERIOD_COUNTS)
    {
        myObject->callBack(handle, returnCount);
        return;
    }

    /* Otherwise we need to do some calculation with the clock source */
    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &curFrequencies);
    if (hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_SMCLK)
    {
        clockFreq = curFrequencies.SMCLK / hwAttrs->clockDivider;
    } else if (hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_ACLK)
    {
        clockFreq = curFrequencies.ACLK / hwAttrs->clockDivider;
    }

    /* Calculating the correct return value in Hz or uS */
    if (periodUnits == CAPTURE_PERIOD_HZ)
    {
        returnCount = returnCount / clockFreq;
    } else if (periodUnits == CAPTURE_PERIOD_US)
    {
        returnCount = (uint32_t)((returnCount) * (1000000.f/ clockFreq));
    }

    /* Invoke the callback */
    myObject->callBack(handle, returnCount);
}

/*
 *  ======== Capture_init ========
 */
void CaptureMSP432_init(Capture_Handle handle)
{
    CaptureMSP432_Object *resource = (CaptureMSP432_Object*) handle->object;
    resource->config = handle;
}

/*
 *  ======== Capture_open ========
 */
Capture_Handle CaptureMSP432_open(Capture_Handle handle, Capture_Params *params)
{
    CaptureMSP432_Object *myObject = (CaptureMSP432_Object*) handle->object;
    CaptureMSP432_HWAttrs *myAttrs =
            (CaptureMSP432_HWAttrs*) myObject->config->hwAttrs;
    uint32_t mode, baseAddr, capturePort, port, pin, portMap, periphValue;
    uintptr_t key;

    baseAddr = myAttrs->timerBaseAddress;

    /* Trying to allocate the Timer resource and returning false if we can't. */
    if (!TimerMSP432_allocateTimerResource(baseAddr))
    {
        return NULL;
    }

    key = HwiP_disable();

    /*
     * Add power management support - Disable performance transitions while
     * opening the driver.
     */
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Shutdown not supported while driver is open */
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);

    /* Setting up the port settings and parsing out relevant information
     * from the HW Attr pin parameter */
    capturePort = myAttrs->capturePort;
    myObject->ccrRegister = (capturePort >> CAPTUREMSP432_CCR_OFS) & 0xF;
    myObject->intNum = (capturePort >> CAPTUREMSP432_INT_OFS) & 0xFF;
    port = (capturePort >> 4) & 0xF;
    pin = (capturePort & 0xF);
    portMap = (capturePort >> CAPTUREMSP432_PMAP_OFS) & 0xFF;

    /* If this is a port mapped configuration, we need to configure the
     * port mapper.
     */
    if(portMap)
    {
        periphValue = GPIO_PRIMARY_MODULE_FUNCTION;
        mapPin(port, pin, portMap);
    }
    else if(capturePort == CaptureMSP432_P8_0_TA1 ||
                capturePort == CaptureMSP432_P8_1_TA2)
    {
        periphValue = GPIO_SECONDARY_MODULE_FUNCTION;
    }
    else
    {
        periphValue = GPIO_PRIMARY_MODULE_FUNCTION;
    }

    /* Setting actual pin configuration in capture mode */
    pin = 1 << pin;
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(port,
            pin, periphValue);

    /* Setting up the timer  hardware for continuous mode*/
    continuousModeConfig.clockSource = myAttrs->clockSource;
    continuousModeConfig.clockSourceDivider = myAttrs->clockDivider;
    MAP_Timer_A_configureContinuousMode(baseAddr, &continuousModeConfig);

    /* Setting up the capture block */
    mode = params->mode;
    captureModeConfig.captureRegister = myObject->ccrRegister;

    if (mode == CAPTURE_MODE_RISING_RISING)
    {
        captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    } else if (mode == CAPTURE_MODE_FALLING_FALLING)
    {
        captureModeConfig.captureMode = TIMER_A_CAPTUREMODE_FALLING_EDGE;
    } else if (mode == CAPTURE_MODE_ANY_EDGE)
    {
        captureModeConfig.captureMode =
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE;
    } else
    {
        Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
        TimerMSP432_freeTimerResource(baseAddr);
        HwiP_restore(key);
        return NULL;
    }

    /* Initializing the capture */
    MAP_Timer_A_initCapture(baseAddr, &captureModeConfig);

    /* Creating the Hwi Handle if it is not already there */
    if (myObject->hwiHandle == NULL)
    {
        HwiP_Params_init(&myObject->hwiParams);
        myObject->hwiParams.arg = (uintptr_t) handle;
        myObject->hwiParams.priority = myAttrs->intPriority;
        myObject->hwiParams.name = "TA_CAP";
        myObject->hwiHandle = HwiP_create(myObject->intNum,
                CaptureMSP432_hwiIntFunction, &myObject->hwiParams);
    }

    /* Saving off the callback */
    myObject->callBack = params->callbackFxn;

    HwiP_restore(key);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
    return handle;
}

/*
 *  ======== Capture_start ========
 */
void CaptureMSP432_start(Capture_Handle handle)
{
    CaptureMSP432_HWAttrs *myAttrs = (CaptureMSP432_HWAttrs*) handle->hwAttrs;
    CaptureMSP432_Object *myObj = (CaptureMSP432_Object*) handle->object;
    uintptr_t key;

    key = HwiP_disable();

    /*
     * Set power constraints to keep peripheral active during transfer
     * and to prevent a performance level change
     */
    Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    HwiP_enableInterrupt(myObj->intNum);
    MAP_Timer_A_clearCaptureCompareInterrupt(myAttrs->timerBaseAddress,
                                             myObj->ccrRegister);
    MAP_Timer_A_enableCaptureCompareInterrupt(myAttrs->timerBaseAddress,
                                             myObj->ccrRegister);
    MAP_Timer_A_startCounter(myAttrs->timerBaseAddress,
            TIMER_A_CONTINUOUS_MODE);

    HwiP_restore(key);
}

/*
 *  ======== Capture_stop ========
 */
void CaptureMSP432_stop(Capture_Handle handle)
{
    CaptureMSP432_HWAttrs *myAttrs = (CaptureMSP432_HWAttrs*) handle->hwAttrs;
    CaptureMSP432_Object *myObj = (CaptureMSP432_Object*) handle->object;
    uintptr_t key;

    key = HwiP_disable();

    MAP_Timer_A_stopTimer(myAttrs->timerBaseAddress);
    HwiP_disableInterrupt(myObj->intNum);
    MAP_Timer_A_clearCaptureCompareInterrupt(myAttrs->timerBaseAddress,
                                             myObj->ccrRegister);
    MAP_Timer_A_disableCaptureCompareInterrupt(myAttrs->timerBaseAddress,
                                               myObj->ccrRegister);
    Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
    HwiP_restore(key);
}

static void mapPin(uint8_t port, uint8_t pin, uint8_t value)
{
    volatile uint8_t                     pmap;

    pmap = port * 0x08;  // 2 -> 0x10, 3 -> 0x18, 7 -> 0x38

    /*  Code from pmap.c: */
    //Get write-access to port mapping registers:
    PMAP->KEYID = PMAP_KEYID_VAL;

    PMAP->CTL = (PMAP->CTL & ~PMAP_CTL_PRECFG) | PMAP_ENABLE_RECONFIGURATION;
    HWREG8((uint32_t)PMAP_BASE + pin + pmap) = value;
    //Disable write-access to port mapping registers:

    PMAP->KEYID = 0;
}
