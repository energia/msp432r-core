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
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/timer32.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/timer/TimerMSP432.h>
#include <ti/drivers/power/PowerMSP432.h>
#include <stdint.h>
#include <stddef.h>

/* Function table of function to handle Timer_A */
Timer_FxnTable TimerMSP432_Timer_A_fxnTable =
{
    .closeFxn = TimerMSP432_close,
    .openFxn =  TimerMSP432_open,
    .startFxn = TimerMSP432_Timer_A_start,
    .stopFxn = TimerMSP432_Timer_A_stop,
    .initFxn = TimerMSP432_init,
    .getCountFxn = TimerMSP432_Timer_A_getCount,
    .controlFxn = TimerMSP432_Timer_A_control
};

/* Function table of function to handle Timer32 */
Timer_FxnTable TimerMSP432_Timer32_fxnTable =
{
    .closeFxn = TimerMSP432_close,
    .openFxn = TimerMSP432_open,
    .startFxn = TimerMSP432_Timer32_start,
    .stopFxn = TimerMSP432_Timer32_stop,
    .controlFxn = TimerMSP432_Timer32_control,
    .getCountFxn = TimerMSP432_Timer32_getCount,
    .initFxn = TimerMSP432_init
};

/* Divider arrays used for timer period calculation */
static const uint32_t timer32Dividers[] =
{
    TIMER32_PRESCALER_256,
    TIMER32_PRESCALER_16,
    TIMER32_PRESCALER_1
};

static const uint32_t timerADividers[] =
{
    TIMER_A_CLOCKSOURCE_DIVIDER_64,
    TIMER_A_CLOCKSOURCE_DIVIDER_56,
    TIMER_A_CLOCKSOURCE_DIVIDER_48,
    TIMER_A_CLOCKSOURCE_DIVIDER_40,
    TIMER_A_CLOCKSOURCE_DIVIDER_32,
    TIMER_A_CLOCKSOURCE_DIVIDER_28,
    TIMER_A_CLOCKSOURCE_DIVIDER_24,
    TIMER_A_CLOCKSOURCE_DIVIDER_20,
    TIMER_A_CLOCKSOURCE_DIVIDER_16,
    TIMER_A_CLOCKSOURCE_DIVIDER_14,
    TIMER_A_CLOCKSOURCE_DIVIDER_12,
    TIMER_A_CLOCKSOURCE_DIVIDER_10,
    TIMER_A_CLOCKSOURCE_DIVIDER_8,
    TIMER_A_CLOCKSOURCE_DIVIDER_7,
    TIMER_A_CLOCKSOURCE_DIVIDER_6,
    TIMER_A_CLOCKSOURCE_DIVIDER_5,
    TIMER_A_CLOCKSOURCE_DIVIDER_4,
    TIMER_A_CLOCKSOURCE_DIVIDER_3,
    TIMER_A_CLOCKSOURCE_DIVIDER_2,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
};

/* Internal/Static Functions */
static bool configureTimerHardware(Timer_Handle handle, Timer_Params *params);
static inline uint32_t getT32DividerValue(uint32_t div);
static bool isTimer32Bit(uint32_t baseAddress);
static bool setIdealT32TickCount(TimerMSP432_HWAttrs *attr, uint32_t period,
        Timer_PeriodUnits units);
static bool setIdealTATickCount(TimerMSP432_HWAttrs *attr, uint32_t period,
        Timer_PeriodUnits units);
static void TimerMSP432_Timer_A_hwiIntFunction(uintptr_t arg);
static void TimerMSP432_Timer32_hwiIntFunction(uintptr_t arg);

/* Structure used for Timer_A Configuration */
static Timer_A_UpModeConfig upConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,           /* SMCLK Clock Source */
    TIMER_A_CLOCKSOURCE_DIVIDER_1,       /*  Default to a 1 divider */
    0xFFFF,                              /* Default to the max period */
    TIMER_A_TAIE_INTERRUPT_DISABLE,      /* Disable Timer interrupt */
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,  /* Enable CCR0 interrupt */
    TIMER_A_DO_CLEAR                     /* Clear value */
};

/*
 *  ======== configureTimerHardware ========
 */
static bool configureTimerHardware(Timer_Handle handle, Timer_Params *params)
{
    TimerMSP432_HWAttrs *hwAttr = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *myObject;
    uint32_t baseAddress;

    /* Parsing out the assigned object */
    myObject = (TimerMSP432_Object*) handle->object;

    /* Storing the base address */
    baseAddress = hwAttr->timerBaseAddress;

    /* Checking to see if it is Timer_A. Timer_A has a base address that is
     * at a lower memory address than Timer32
     */
    if (!isTimer32Bit(baseAddress))
    {
        /* Creating the Hwi Handle if it is not already there */
        if (myObject->hwiHandle == NULL)
        {
            HwiP_Params_init(&myObject->hwiParams);
            myObject->hwiParams.arg = (uintptr_t) handle;
            myObject->hwiParams.priority = hwAttr->intPriority;
            myObject->hwiParams.name = "TA";
            myObject->hwiHandle = HwiP_create(hwAttr->intNum,
                    TimerMSP432_Timer_A_hwiIntFunction, &myObject->hwiParams);
        }

        /* If the user specified timer counts, we can simply set the timer
         * period to that of the user period. Otherwise we need to do some
         * math to get the optimal divider.
         */
        if (params->periodUnits == Timer_PERIOD_COUNTS)
        {
            upConfig.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;

            if (params->period > UINT16_MAX)
            {
                return false;
            }

            upConfig.timerPeriod = params->period;

        } else
        {
            if (!setIdealTATickCount(hwAttr, params->period,
                    params->periodUnits))
            {
                return false;
            }
        }

        /* Configuring the timer in Up Mode with CCR0 interrupting */
        upConfig.clockSource = hwAttr->clockSource;
        upConfig.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
        MAP_Timer_A_configureUpMode(baseAddress, &upConfig);

    }
    /* Timer 32 is our sole 32-bit timer */
    else
    {
        /* If the period unit is period counts, we can just set the timer period
         * to the user period. Otherwise, we have to do math to figure out the
         * optimal divider.
         */
        if (params->periodUnits == Timer_PERIOD_COUNTS)
        {
            MAP_Timer32_initModule(hwAttr->timerBaseAddress,
            TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
            MAP_Timer32_setCount(baseAddress, params->period);
        } else
        {
            if (!setIdealT32TickCount(hwAttr, params->period,
                    params->periodUnits))
            {
                return false;
            }
        }

        /* Creating the Hwi Handle if it is not already there */
        if (myObject->hwiHandle == NULL)
        {
            HwiP_Params_init(&myObject->hwiParams);
            myObject->hwiParams.arg = (uintptr_t) handle;
            myObject->hwiParams.priority = hwAttr->intPriority;
            myObject->hwiParams.name = "T32";
            myObject->hwiHandle = HwiP_create(hwAttr->intNum,
                    TimerMSP432_Timer32_hwiIntFunction, &myObject->hwiParams);
        }
    }

    return true;
}

/*
 *  ======== getT32DividerValue ========
 */
static inline uint32_t getT32DividerValue(uint32_t div)
{
    switch (div)
    {
    case TIMER32_PRESCALER_256:
        return 256;
    case TIMER32_PRESCALER_16:
        return 16;
    case TIMER32_PRESCALER_1:
        return 1;
    default:
        return 0;
    }
}

/*
 *  ======== isTimer32Bit ========
 */
static inline bool isTimer32Bit(uint32_t baseAddress)
{
    if (baseAddress < TIMER32_BASE)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*
 *  ======== setIdealT32TickCount ========
 */
static bool setIdealT32TickCount(TimerMSP432_HWAttrs *attr, uint32_t period,
        Timer_PeriodUnits units)
{
    PowerMSP432_Freqs curFrequencies;
    uint32_t ii;
    uint32_t clockFreq, curCalClock;
    uint32_t bestDifference, bestDivider, curDifference;

    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &curFrequencies);
    clockFreq = curFrequencies.MCLK;
    bestDifference = 0;

    /* Finding what the idea "divider" would be for the given time. We
     * start with the largest divider and go down to "stretch out" the
     * number of ticks in the timer. */
    for (ii = 0; ii < sizeof(timer32Dividers) / sizeof(uint32_t); ii++)
    {
        curCalClock = (clockFreq / getT32DividerValue(timer32Dividers[ii]));

        if (units == Timer_PERIOD_US)
        {
            curDifference = (uint32_t)((period / 1000000.0f) * (curCalClock));
        } else
        {
            curDifference = curCalClock / period;
        }

        if (curDifference > bestDifference)
        {
            bestDifference = curDifference;
            bestDivider = timer32Dividers[ii];
        }
    }

    if (bestDifference == 0)
    {
        return false;
    }

    MAP_Timer32_initModule(attr->timerBaseAddress, bestDivider, TIMER32_32BIT,
                            TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(attr->timerBaseAddress, bestDifference);

    return true;
}

/*
 *  ======== setIdealTATickCount ========
 */
static bool setIdealTATickCount(TimerMSP432_HWAttrs *attr, uint32_t period,
        Timer_PeriodUnits units)
{
    PowerMSP432_Freqs curFrequencies;
    uint32_t ii;
    uint32_t clockFreq, curCalClock;
    uint32_t bestDifference, bestDivider, curDifference;

    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &curFrequencies);

    if (attr->clockSource == TIMER_A_CLOCKSOURCE_SMCLK)
    {
        clockFreq = curFrequencies.SMCLK;
    } else if (attr->clockSource == TIMER_A_CLOCKSOURCE_ACLK)
    {
        clockFreq = curFrequencies.ACLK;
    }

    bestDifference = 0;

    /* Finding what the idea "divider" would be for the given time. We
     * start with the largest divider and go down to "stretch out" the
     * number of ticks in the timer. */
    for (ii = 0; ii < sizeof(timerADividers) / sizeof(uint32_t); ii++)
    {
        curCalClock = (clockFreq / timerADividers[ii]);

        if (units == Timer_PERIOD_US)
        {
            curDifference = (uint32_t)((period / 1000000.0f) * (curCalClock));
        } else
        {
            curDifference = curCalClock / period;
        }

        if (curDifference > UINT16_MAX)
        {
            continue;
        }

        if (curDifference > bestDifference)
        {
            bestDifference = curDifference;
            bestDivider = timerADividers[ii];
        }
    }

    if (bestDifference == 0)
    {
        return false;
    }

    upConfig.timerPeriod = bestDifference;
    upConfig.clockSourceDivider = bestDivider;

    return true;
}


/*
 *  ======== TimerMSP432_allocateTimerResource ========
 */
bool TimerMSP432_allocateTimerResource(uint32_t timerBase)
{
    uint32_t ii;
    TimerMSP432_HWAttrs *curAttr;

    for (ii = 0; ii < Timer_count; ii++)
    {
        curAttr = (TimerMSP432_HWAttrs*) timerMSP432Objects[ii].config->hwAttrs;

        if (curAttr->timerBaseAddress == timerBase)
        {
            if (timerMSP432Objects[ii].resourceAvailable)
            {
                timerMSP432Objects[ii].resourceAvailable = false;
                return true;
            } else
            {
                return false;
            }

        }
    }

    /* If we cannot find the base address, the resource is not managed
     * by the timer resource and the other module can use it
     */
    return true;
}

/*
 *  ======== TimerMSP432_close ========
 */
void TimerMSP432_close(Timer_Handle handle)
{
    TimerMSP432_Object *myObject;
    uintptr_t key;

    key = HwiP_disable();

    /* Stopping the Timer before closing it */
    handle->fxnTablePtr->stopFxn(handle);

    /* Parsing out the assigned object */
    myObject = (TimerMSP432_Object*) handle->object;

    /* Closing out all of the parameters inside the driver object */
    myObject->resourceAvailable = true;
    myObject->callBack = NULL;

    /* Deleting/Freeing the Semaphore if it is there */
    if (myObject->timerSem != NULL)
    {
        SemaphoreP_delete(myObject->timerSem);
    }

    HwiP_restore(key);

    /* Remove power constraints */
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);
}

/*
 *  ======== TimerMSP432_freeTimerResource ========
 */
void TimerMSP432_freeTimerResource(uint32_t timerBase)
{
    uint32_t ii;
    TimerMSP432_HWAttrs *curAttr;

    for (ii = 0; ii < Timer_count; ii++)
    {
        curAttr = (TimerMSP432_HWAttrs*) timerMSP432Objects[ii].config->hwAttrs;

        if (curAttr->timerBaseAddress == timerBase)
        {
            timerMSP432Objects[ii].resourceAvailable = true;
            return;
        }
    }
}

/*
 *  ======== TimerMSP432_init ========
 */
void TimerMSP432_init(Timer_Handle handle)
{
    TimerMSP432_Object *resource = (TimerMSP432_Object*) handle->object;
    resource->resourceAvailable = true;
    resource->config = handle;
}

/*
 *  ======== TimerMSP432_open ========
 */
Timer_Handle TimerMSP432_open(Timer_Handle handle, Timer_Params *params)
{
    TimerMSP432_Object *myObject = (TimerMSP432_Object*) handle->object;
    SemaphoreP_Params semParams;
    uintptr_t key;
    Timer_Mode timerMode;

    /* Checking to make sure that the given parameters don't have anything
     * fundamentally wrong with them (ie no callback for callback modes).
     */
    if ((params->timerMode == Timer_ONESHOT_CALLBACK
            || params->timerMode == Timer_CONTINUOUS_CALLBACK)
            && !params->timerCallback)
    {
        return NULL;
    }

    /* Making sure the timer is not in use by another resource */
    if (!myObject->resourceAvailable)
    {
        return NULL;
    }

    /*
     * Add power management support - Disable performance transitions while
     * opening the driver.
     */
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Shutdown not supported while driver is open */
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
    Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);

    key = HwiP_disable();

    /* Grabbing and recording the timer mode (it is needed in the Hwi) */
    timerMode = params->timerMode;
    myObject->timerMode = timerMode;

    /* Creating the semaphore if mode is blocking */
    if (timerMode == Timer_ONESHOT_BLOCKING)
    {
        SemaphoreP_Params_init(&semParams);
        semParams.mode = SemaphoreP_Mode_BINARY;
        myObject->timerSem = SemaphoreP_create(0, &semParams);

        if (myObject->timerSem == NULL)
        {
            HwiP_restore(key);
            Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
            return NULL;
        }
    }
    /* For callback modes set the callback */
    else if (timerMode == Timer_CONTINUOUS_CALLBACK ||
             timerMode == Timer_ONESHOT_CALLBACK) {
        myObject->callBack = params->timerCallback;
    } else {
        myObject->callBack = NULL;
    }

    /* Configuring the initial hardware settings */
    if (!configureTimerHardware(handle, params))
    {
        HwiP_restore(key);
        Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
        return NULL;
    }

    /* Marking the resource as used */
    myObject->resourceAvailable = false;

    HwiP_restore(key);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);
    return handle;
}

/*
 *  ======== TimerMSP432_Timer_A_control ========
 */
int_fast16_t TimerMSP432_Timer_A_control(Timer_Handle handle,
    uint_fast16_t cmd, void *arg)
{
        return Timer_STATUS_UNDEFINEDCMD;
}

/*
 *  ======== TimerMSP432_Timer_A_getCount ========
 */
uint32_t TimerMSP432_Timer_A_getCount(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *attrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    uint32_t count;
    uintptr_t key;

    key = HwiP_disable();

    count = MAP_Timer_A_getCounterValue(attrs->timerBaseAddress);

    HwiP_restore(key);

    return count;
}

/*
 *  ======== TimerMSP432_TimerA_hwiIntFunction ========
 */
void TimerMSP432_Timer_A_hwiIntFunction(uintptr_t arg)
{
    Timer_Handle handle = (Timer_Handle) arg;
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *myObject = (TimerMSP432_Object*) handle->object;
    uint32_t baseAddress, mode;

    /* Grabbing the base address and handling the interrupt */
    baseAddress = hwAttrs->timerBaseAddress;
    MAP_Timer_A_clearCaptureCompareInterrupt(baseAddress,
    TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* Restarting or halting the timer depending on the mode */
    mode = myObject->timerMode;

    if (mode == Timer_ONESHOT_CALLBACK || mode == Timer_ONESHOT_BLOCKING)
    {
        MAP_Timer_A_stopTimer(baseAddress);
        HwiP_disableInterrupt(hwAttrs->intNum);
        MAP_Timer_A_disableCaptureCompareInterrupt(baseAddress,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
    }

    /* Invoking the callback if needed */
    if (mode == Timer_ONESHOT_CALLBACK || mode == Timer_CONTINUOUS_CALLBACK)
    {
        myObject->callBack(handle);
    } else if (mode == Timer_ONESHOT_BLOCKING)
    {
        SemaphoreP_post(myObject->timerSem);
    }
}

/*
 *  ======== TimerMSP432_Timer_A_start ========
 */
int32_t TimerMSP432_Timer_A_start(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *myObj = (TimerMSP432_Object*) handle->object;
    uint32_t baseAddress;
    uintptr_t key;

    key = HwiP_disable();

    /* Grabbing the base address and starting the timer */
    baseAddress = hwAttrs->timerBaseAddress;

    MAP_Timer_A_clearTimer(baseAddress);

    if (myObj->timerMode != Timer_FREE_RUNNING)
    {
        MAP_Timer_A_enableCaptureCompareInterrupt(baseAddress,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
        HwiP_enableInterrupt(hwAttrs->intNum);
    }

    MAP_Timer_A_startCounter(baseAddress, TIMER_A_UP_MODE);

    HwiP_restore(key);

    /*
     * Set power constraints to keep peripheral active during transfer and
     * to prevent a performance level change
     */
    Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    return (Timer_STATUS_SUCCESS);
}

/*
 *  ======== TimerMSP432_Timer_A_stop ========
 */
void TimerMSP432_Timer_A_stop(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    uint32_t baseAddress;
    uintptr_t key;

    key = HwiP_disable();

    /* Remove constraints set during transfer */
    Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Grabbing the base address and stopping the timer */
    baseAddress = hwAttrs->timerBaseAddress;
    MAP_Timer_A_stopTimer(baseAddress);
    MAP_Timer_A_clearTimer(baseAddress);
    MAP_Timer_A_disableCaptureCompareInterrupt(baseAddress,
    TIMER_A_CAPTURECOMPARE_REGISTER_0);

    HwiP_restore(key);
}

/*
 *  ======== TimerMSP432_Timer32_control ========
 */
int_fast16_t TimerMSP432_Timer32_control(Timer_Handle handle,
        uint_fast16_t cmd, void *arg)
{
    uintptr_t key;

    key = HwiP_disable();

    switch (cmd)
    {
    default:
        HwiP_restore(key);
        return Timer_STATUS_UNDEFINEDCMD;
    }
}

/*
 *  ======== TimerMSP432_Timer32_getCount ========
 */
uint32_t TimerMSP432_Timer32_getCount(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *attrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    uint32_t count;
    uintptr_t key;

    key = HwiP_disable();

    count = MAP_Timer32_getValue(attrs->timerBaseAddress);

    HwiP_restore(key);

    return count;
}

/*
 *  ======== TimerMSP432_Timer32_hwiIntFunction ========
 */
void TimerMSP432_Timer32_hwiIntFunction(uintptr_t arg)
{
    Timer_Handle handle = (Timer_Handle) arg;
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *myObject = (TimerMSP432_Object*) handle->object;
    uint32_t baseAddress;
    Timer_Mode mode;

    baseAddress = hwAttrs->timerBaseAddress;

    MAP_Timer32_clearInterruptFlag(baseAddress);

    /* Restarting or halting the timer depending on the mode */
    mode = myObject->timerMode;

    if (mode == Timer_ONESHOT_CALLBACK || mode == Timer_ONESHOT_BLOCKING)
    {
        MAP_Timer32_haltTimer(baseAddress);
        HwiP_disableInterrupt(hwAttrs->intNum);
        MAP_Timer32_disableInterrupt(baseAddress);
    }

    /* Invoking the callback if needed */
    if (mode == Timer_ONESHOT_CALLBACK || mode == Timer_CONTINUOUS_CALLBACK)
    {
        myObject->callBack(handle);
    } else if (mode == Timer_ONESHOT_BLOCKING)
    {
        SemaphoreP_post(myObject->timerSem);
    }
}

/*
 *  ======== TimerMSP432_Timer32_start ========
 */
int32_t TimerMSP432_Timer32_start(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *object = (TimerMSP432_Object*) handle->object;
    uint32_t baseAddress;
    uintptr_t key;

    key = HwiP_disable();

    baseAddress = hwAttrs->timerBaseAddress;

    if (object->timerMode != Timer_FREE_RUNNING)
    {
        MAP_Timer32_enableInterrupt(baseAddress);
        HwiP_enableInterrupt(hwAttrs->intNum);
    }

    if (object->timerMode == Timer_ONESHOT_BLOCKING
            || object->timerMode == Timer_ONESHOT_CALLBACK)
    {
        MAP_Timer32_startTimer(baseAddress, true);
    }
    else
    {
        MAP_Timer32_startTimer(baseAddress, false);
    }

    HwiP_restore(key);

    /*
     * Set power constraints to keep peripheral active during transfer and
     * to prevent a performance level change
     */
    Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    return (Timer_STATUS_SUCCESS);
}

/*
 *  ======== TimerMSP432_Timer32_stop ========
 */
void TimerMSP432_Timer32_stop(Timer_Handle handle)
{
    TimerMSP432_HWAttrs *hwAttrs = (TimerMSP432_HWAttrs*) handle->hwAttrs;
    TimerMSP432_Object *object = (TimerMSP432_Object*) handle->object;
    uint32_t baseAddress;
    uintptr_t key;

    key = HwiP_disable();

    /* Remove constraints set during transfer */
    Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Grabbing the base address and stopping the timer */
    baseAddress = hwAttrs->timerBaseAddress;

    MAP_Timer32_haltTimer(baseAddress);
    MAP_Timer32_disableInterrupt(baseAddress);

    if (object->timerMode != Timer_FREE_RUNNING)
    {
        HwiP_disableInterrupt(hwAttrs->intNum);
    }

    HwiP_restore(key);
}
