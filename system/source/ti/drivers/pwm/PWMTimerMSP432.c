/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>
#include <ti/drivers/timer/TimerMSP432.h>
#include <ti/drivers/pwm/PWMTimerMSP432.h>

#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/pmap.h>

#define PinConfigValue(config)            (((config) >> 10) & 0x1F)
#define PinConfigModuleFunction(config)   (((config) >> 8) & 0x3)
#define PinConfigPort(config)             (((config) >> 4) & 0xF)
#define PinConfigPin(config)              (1 << ((config) & 0x7))
#define PinConfigTimerId(config)          (((config) >> 16) & 0xF)
#define PinConfigCompareRegister(config)  (((config) >> 20) & 0xF)

void PWMTimerMSP432_close(PWM_Handle handle);
int_fast16_t PWMTimerMSP432_control(PWM_Handle handle, uint_fast16_t cmd, void *arg);
void PWMTimerMSP432_init(PWM_Handle handle);
PWM_Handle PWMTimerMSP432_open(PWM_Handle handle, PWM_Params *params);
int_fast16_t PWMTimerMSP432_setDuty(PWM_Handle handle, uint32_t dutyValue);
int_fast16_t PWMTimerMSP432_setPeriod(PWM_Handle handle, uint32_t periodValue);
void PWMTimerMSP432_start(PWM_Handle handle);
void PWMTimerMSP432_stop(PWM_Handle handle);

static void mapPin(uint8_t port, uint8_t pin, uint8_t value);

/* PWM function table for PWMTimerMSP432 implementation */
const PWM_FxnTable PWMTimerMSP432_fxnTable = {
    PWMTimerMSP432_close,
    PWMTimerMSP432_control,
    PWMTimerMSP432_init,
    PWMTimerMSP432_open,
    PWMTimerMSP432_setDuty,
    PWMTimerMSP432_setPeriod,
    PWMTimerMSP432_start,
    PWMTimerMSP432_stop
};

static const uint32_t pwmTimerBaseAddr[] = {
    TIMER_A0_BASE,
    TIMER_A1_BASE,
    TIMER_A2_BASE,
    TIMER_A3_BASE
};

/*
 * Internal value to notify an error has occurred while calculating a duty
 * or period.
 */
static const uint32_t PWM_INVALID_VALUE = (~0);

/*
 * Timer_A peripheral registers have 16 bit resolution.  The max register value
 * which be set is 65535.
 */
static const uint16_t PWM_MAX_MATCH_REG_VALUE = (~0);

/*
 * Timer_A peripheral has a max prescalar of 64.
 */
static const uint16_t PWM_MAX_PRESCALAR = (64);

/* Internal Timer status structures */
static PWMTimerMSP432_Status pwmTimerStatus[PWMTimerMSP432_NUM_TIMERS] = {0};

/*
 *  ======== calculatePrescalar ========
 *  Calculates timer prescalar for a given period.
 *
 *  @param  period      in timer ticks
 *  @return prescalar   required to generate period
 */
static inline uint8_t calculatePrescalar(uint32_t period)
{
    /* Initialize to a prescalar of 1 */
    uint8_t prescalar = 1;

    while (period > PWM_MAX_MATCH_REG_VALUE) {
        prescalar <<= 1;
        period /=2;
    }

    return (prescalar);
}

/*
 *  ======== corroborateDuty ========
 */
static int corroborateDuty(PWM_Handle handle, uint32_t period, uint32_t duty)
{
    if (duty == PWM_INVALID_VALUE) {
        DebugP_log1("PWM:(%p) duty units could not be determined.",
            (uintptr_t) handle);

        return (PWM_STATUS_ERROR);
    }

    if (duty > period) {
        DebugP_log1("PWM:(%p) duty is out of range.", (uintptr_t) handle);

        return (PWM_STATUS_INVALID_DUTY);
    }

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== corroboratePeriod ========
 */
static int corroboratePeriod(PWM_Handle handle, uint32_t period,
    uint8_t prescalar)
{
    if (period == PWM_INVALID_VALUE) {
        DebugP_log1("PWM:(%p) period units could not be determined.",
            (uintptr_t) handle);

        return (PWM_STATUS_ERROR);
    }

    if ((period == 0) || (prescalar > PWM_MAX_PRESCALAR)) {
        DebugP_log1("PWM:(%p) period is out of range.", (uintptr_t) handle);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== getDutyCounts ========
 */
static uint32_t getDutyCounts(PWM_Duty_Units dutyUnits, uint32_t dutyValue,
    uint32_t periodCounts, uint32_t clockFreq)
{
    uint32_t duty = 0;

    switch (dutyUnits) {
        case PWM_DUTY_COUNTS:
            duty = dutyValue;
            break;

        case PWM_DUTY_FRACTION:
            duty = (((uint64_t) dutyValue) * ((uint64_t) periodCounts)) /
                PWM_DUTY_FRACTION_MAX;
            break;

        case PWM_DUTY_US:
            duty = dutyValue * (clockFreq/1000000);
            break;

        default:
            /* Unsupported duty units return an invalid duty */
            duty = PWM_INVALID_VALUE;
    }

    return (duty);
}

/*
 *  ======== getPeriodCounts ========
 */
static uint32_t getPeriodCounts(PWM_Period_Units periodUnits,
    uint32_t periodValue, uint32_t clockFreq)
{
    uint32_t period = 0;

    switch (periodUnits) {
        case PWM_PERIOD_COUNTS:
            period = periodValue;
            break;

        case PWM_PERIOD_HZ:
            if (periodValue && periodValue <= clockFreq) {
                period = clockFreq / periodValue;
            }
            break;

        case PWM_PERIOD_US:
            period = periodValue * (clockFreq/1000000);
            break;

        default:
            /* Unsupported period units return an invalid period */
            period = PWM_INVALID_VALUE;
    }

    return (period);
}

/*
 *  ======== initHw ========
 */
static int initHw(PWM_Handle handle, uint32_t period, uint32_t duty)
{
    uintptr_t                       key;
    int32_t                         result;
    uint32_t                        dutyTicks;
    uint32_t                        periodTicks;
    uint8_t                         prescalar;
    uint32_t                        clockFreq;
    Timer_A_PWMConfig               pwmConfig;
    PowerMSP432_Freqs               powerFreqs;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;

    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &powerFreqs);
    clockFreq = (hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_SMCLK) ?
        powerFreqs.SMCLK : powerFreqs.ACLK;

    periodTicks = getPeriodCounts(object->periodUnits, period, clockFreq);
    dutyTicks = getDutyCounts(object->dutyUnits, duty, periodTicks, clockFreq);
    prescalar = calculatePrescalar(periodTicks);

    result = corroboratePeriod(handle, periodTicks, prescalar);
    if (result != PWM_STATUS_SUCCESS) {
        return (result);
    }

    result = corroborateDuty(handle, periodTicks, dutyTicks);
    if (result != PWM_STATUS_SUCCESS) {
        return (result);
    }

    /* Trying to allocate the timer resource. If the timer that the PWM is using
     *  is in use by either the Capture or Timer driver, this will return false.
     */
    if(!TimerMSP432_allocateTimerResource(object->timerBaseAddr))
    {
        return PWM_STATUS_ERROR;
    }

    key = HwiP_disable();

    /*
     * Verify if timer has been initialized by another PWM instance.  If so,
     * make sure PWM periods & prescalars are the same, do not open driver if
     * otherwise.
     */
    if ((object->timerStatusStruct)->period &&
        ((object->timerStatusStruct)->period != periodTicks ||
            (object->timerStatusStruct)->prescalar != prescalar)) {
        HwiP_restore(key);

        DebugP_log1("PWM:(%p) differing PWM periods, cannot open driver.",
            (uintptr_t) handle);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    /*
     * Store configuration & mark PWM instance as active (prevents other
     * instances from shutting off the timer in PWM_close()).
     */
    (object->timerStatusStruct)->period = periodTicks;
    (object->timerStatusStruct)->prescalar = prescalar;
    (object->timerStatusStruct)->duties[object->compareOutputNum] = dutyTicks;
    (object->timerStatusStruct)->activeOutputsMask |=
        (1 << object->compareOutputNum);

    /*
     * This condition ensures that the output will remain active if the duty
     * is equal to the period.
     */
    duty = dutyTicks / (object->timerStatusStruct)->prescalar;
    period = periodTicks / (object->timerStatusStruct)->prescalar;
    if (duty == period) {
        duty++;
    }

    pwmConfig.clockSource = hwAttrs->clockSource;
    pwmConfig.clockSourceDivider = prescalar;
    pwmConfig.timerPeriod = period;
    pwmConfig.compareRegister = PinConfigCompareRegister(hwAttrs->pwmPin);
    pwmConfig.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    pwmConfig.dutyCycle = duty;
    MAP_Timer_A_generatePWM(object->timerBaseAddr, &pwmConfig);

    HwiP_restore(key);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMTimerMSP432_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PWMTimerMSP432_close(PWM_Handle handle)
{
    uintptr_t                       key;
    PWMTimerMSP432_Object          *object  = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;

    PWMTimerMSP432_stop(handle);

    key = HwiP_disable();

    /* Remove power constraints */
    Power_releaseConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Mark the PWM as inactive */
    (object->timerStatusStruct)->activeOutputsMask &=
        ~(1 << object->compareOutputNum);
    (object->timerStatusStruct)->duties[object->compareOutputNum] = 0;

    /* Stop timer & clear all status if no other PWM instances are being used */
    if ((object->timerStatusStruct)->activeOutputsMask == 0) {
        MAP_Timer_A_stopTimer(object->timerBaseAddr);
        (object->timerStatusStruct)->period = 0;
        (object->timerStatusStruct)->prescalar = 0;
    }

    /* If the pin was mapped in open, restore to PMAP_NONE */
    if (PinConfigValue(hwAttrs->pwmPin) != 0) {

        mapPin(PinConfigPort(hwAttrs->pwmPin), (hwAttrs->pwmPin) & 0x7,
            PMAP_NONE);
    }

    object->isOpen = false;

    /* Freeing up the resource with the Timer driver */
    TimerMSP432_freeTimerResource(object->timerBaseAddr);

    HwiP_restore(key);

    DebugP_log1("PWM:(%p) closed", (uintptr_t) handle);
}

/*
 *  ======== PWMTimerMSP432_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t PWMTimerMSP432_control(PWM_Handle handle, uint_fast16_t cmd, void *arg)
{
    /* No implementation yet */
    return (PWM_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== PWMTimerMSP432_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PWMTimerMSP432_init(PWM_Handle handle)
{
}

/*
 *  ======== PWMTimerMSP432_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
PWM_Handle PWMTimerMSP432_open(PWM_Handle handle, PWM_Params *params)
{
    uintptr_t                       key;
    bool                            timerIsRunning;
    uint8_t                         structIndex;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;
    uint16_t                        pin;
    uint16_t                        port;
    uint16_t                        value;

    /* Assign corresponding status structure to the PWM instance */
    structIndex = PinConfigTimerId(hwAttrs->pwmPin);
    object->timerStatusStruct = &(pwmTimerStatus[structIndex]);
    object->timerBaseAddr = pwmTimerBaseAddr[structIndex];
    object->compareOutputNum = (PinConfigCompareRegister(hwAttrs->pwmPin) / 2) - 2;

    key = HwiP_disable();

    /*
     * Before opening the PWM instance, we must verify that the Timer is not
     * already open or being used by another source (possibly the Kernel).
     * Additionally, the Timer peripheral could have already been initialized
     * by another PWM instance, so we must verify if any other PWM driver
     * (on the same Timer) is initialized.
     */
    timerIsRunning =
        (TIMER_A_CMSIS(object->timerBaseAddr)->CTL & TIMER_A_CTL_MC_3) != TIMER_A_STOP_MODE;
    if (object->isOpen ||
        (timerIsRunning && (object->timerStatusStruct)->activeOutputsMask == 0)) {
        /* Timer already opened or used by source other than PWM driver */
        HwiP_restore(key);

        DebugP_log1("PWM:(%p) timer used by another source.",
            (uintptr_t) handle);

        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    if (PinConfigCompareRegister(hwAttrs->pwmPin) == TIMER_A_CAPTURECOMPARE_REGISTER_0) {
        object->isOpen = false;

        DebugP_log1("PWM:(%p) Cannot use COMPARE_REGISTER_0 to generate PWM.",
            (uintptr_t) handle);

        return (NULL);
    }

    if (hwAttrs->clockSource != TIMER_A_CLOCKSOURCE_ACLK &&
        hwAttrs->clockSource != TIMER_A_CLOCKSOURCE_SMCLK) {
        object->isOpen = false;

        DebugP_log1("PWM:(%p) Unsupported PWM clock source.",
            (uintptr_t) handle);

        return (NULL);
    }

    if ((hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_ACLK) &&
        ((params->periodUnits == PWM_PERIOD_US) ||
            (params->dutyUnits == PWM_DUTY_US))) {
        object->isOpen = false;

        DebugP_log1("PWM:(%p) Microseconds units unsupported with ACLK source.",
            (uintptr_t) handle);

        return (NULL);
    }

    /*
     * Add power management support - PWM driver does not allow performance
     * level changes while open.
     */
    Power_setConstraint(PowerMSP432_DISALLOW_PERF_CHANGES);

    /* Map the pin, only if its a mappable pin. */
    value = PinConfigValue(hwAttrs->pwmPin);

    if (value != 0) {
        port = PinConfigPort(hwAttrs->pwmPin);
        pin = (hwAttrs->pwmPin) & 0x7;
        mapPin(port, pin, value);
    }

    /* Store PWM configuration */
    object->dutyUnits = params->dutyUnits;
    object->idleLevel = params->idleLevel;
    object->periodUnits = params->periodUnits;
    object->pwmStarted = false;

    /* Initialize the peripheral & set the period & duty */
    if (initHw(handle, params->periodValue, params->dutyValue) !=
        PWM_STATUS_SUCCESS) {
        PWMTimerMSP432_close(handle);

        DebugP_log1("PWM:(%p) Failed set initial PWM configuration.",
            (uintptr_t) handle);

        return (NULL);
    }

    /* Called to set the initial idleLevel */
    PWMTimerMSP432_stop(handle);

    DebugP_log3("PWM:(%p) opened; period set to: %d; duty set to: %d",
        (uintptr_t) handle, params->periodValue, params->dutyValue);

    return (handle);
}

/*
 *  ======== PWMTimerMSP432_setDuty ========
 *  @pre    Function assumes that handle is not NULL
 */
int_fast16_t PWMTimerMSP432_setDuty(PWM_Handle handle, uint32_t dutyValue)
{
    uintptr_t                       key;
    int32_t                         result;
    uint32_t                        duty;
    uint32_t                        period;
    uint32_t                        clockFreq;
    PowerMSP432_Freqs               powerFreqs;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;

    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &powerFreqs);
    clockFreq = (hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_SMCLK) ?
        powerFreqs.SMCLK : powerFreqs.ACLK;

    key = HwiP_disable();

    period = (object->timerStatusStruct)->period;
    duty = getDutyCounts(object->dutyUnits, dutyValue, period, clockFreq);
    result = corroborateDuty(handle, period, duty);
    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);

        return (result);
    }

    /*
     * Set & store the new duty.  IMPORTANT: this must be saved before the
     * duty is divided by the prescalar & the duty = period corner case.
     */
    (object->timerStatusStruct)->duties[object->compareOutputNum] = duty;

    /*
     * This condition ensures that the output will remain active if the duty
     * is equal to the period.
     */
    duty /= (object->timerStatusStruct)->prescalar;
    period /= (object->timerStatusStruct)->prescalar;
    if (duty == period) {
        duty++;
    }

    MAP_Timer_A_setCompareValue(object->timerBaseAddr,
            PinConfigCompareRegister(hwAttrs->pwmPin), duty);

    HwiP_restore(key);

    DebugP_log2("PWM:(%p) duty set to: %d", (uintptr_t) handle, duty);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMTimerMSP432_setPeriod ========
 *  @pre    Function assumes that handle is not NULL
 */
int_fast16_t PWMTimerMSP432_setPeriod(PWM_Handle handle, uint32_t periodValue)
{
    uintptr_t                       key;
    int8_t                          i;
    uint8_t                         prescalar;
    int32_t                         result;
    uint32_t                        period;
    uint32_t                        clockFreq;
    PowerMSP432_Freqs               powerFreqs;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;

    PowerMSP432_getFreqs(Power_getPerformanceLevel(), &powerFreqs);
    clockFreq = (hwAttrs->clockSource == TIMER_A_CLOCKSOURCE_SMCLK) ?
        powerFreqs.SMCLK : powerFreqs.ACLK;

    key = HwiP_disable();

    period = getPeriodCounts(object->periodUnits, periodValue, clockFreq);
    prescalar = calculatePrescalar(period);

    result = corroboratePeriod(handle, period, prescalar);
    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);

        return (result);
    }

    /* Ensure the new period can be generated with the current prescalar. */
    if (prescalar != (object->timerStatusStruct)->prescalar) {
        HwiP_restore(key);

        DebugP_log1("PWM:(%p) period is out of range.", (uintptr_t) handle);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    /*
     * Due to Timer_A peripherals generating multiple PWM outputs on a single
     * timer, we need to ensure the new period is greater than all the duties
     * currently set on the timer peripheral.
     */
    for (i = 0; i < PWMTimerMSP432_NUM_PWM_OUTPUTS; i++) {
        if ((object->timerStatusStruct)->duties[i] &&
            period <= (object->timerStatusStruct)->duties[i]) {
            HwiP_restore(key);

            DebugP_log1("PWM:(%p) period is out of range.", (uintptr_t) handle);

            return (PWM_STATUS_INVALID_PERIOD);
        }
    }

    (object->timerStatusStruct)->period = period;
    MAP_Timer_A_setCompareValue(object->timerBaseAddr,
        TIMER_A_CAPTURECOMPARE_REGISTER_0, period/prescalar);

    HwiP_restore(key);

    DebugP_log2("PWM:(%p) period set to: %d", (uintptr_t) handle, periodValue);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMTimerMSP432_start ========
 *  @pre    Function assumes that handle is not NULL
 */
void PWMTimerMSP432_start(PWM_Handle handle)
{
    uintptr_t                       key;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;
    uint16_t                        pin;
    uint16_t                        port;
    uint16_t                        moduleFunction;

    key = HwiP_disable();

    /*
     * Power management - do not allow low power modes or shutdown while PWM
     * is started.
     */
    if (!(object->pwmStarted)) {
        Power_setConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
        Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
        Power_setConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);
        object->pwmStarted = true;
    }

    /* Start the timer & set pinmux to PWM mode */
    MAP_Timer_A_startCounter(object->timerBaseAddr, TIMER_A_UP_MODE);

    port = PinConfigPort(hwAttrs->pwmPin);
    moduleFunction = (PinConfigValue(hwAttrs->pwmPin) == 0) ?
        PinConfigModuleFunction(hwAttrs->pwmPin) :
        GPIO_PRIMARY_MODULE_FUNCTION;
    pin  = PinConfigPin(hwAttrs->pwmPin);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(port, pin, moduleFunction);

    HwiP_restore(key);

    DebugP_log1("PWM:(%p) started.", (uintptr_t) handle);
}

/*
 *  ======== PWMTimerMSP432_stop ========
 *  @pre    Function assumes that handle is not NULL
 */
void PWMTimerMSP432_stop(PWM_Handle handle)
{
    uintptr_t                       key;
    PWMTimerMSP432_Object          *object = handle->object;
    PWMTimerMSP432_HWAttrsV2 const *hwAttrs = handle->hwAttrs;
    uint16_t                        pin;
    uint16_t                        port;

    key = HwiP_disable();

    /* Remove the dependency to allow low power modes & shutdown */
    if (object->pwmStarted) {
        Power_releaseConstraint(PowerMSP432_DISALLOW_DEEPSLEEP_0);
        Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_0);
        Power_releaseConstraint(PowerMSP432_DISALLOW_SHUTDOWN_1);
        object->pwmStarted = false;
    }

    /* Set pin as GPIO with IdleLevel value & stop the timer */
    pin = PinConfigPin(hwAttrs->pwmPin);
    port = PinConfigPort(hwAttrs->pwmPin);

    MAP_GPIO_setAsOutputPin(port, pin);
    MAP_GPIO_setDriveStrengthHigh(port, pin);
    if (object->idleLevel) {
        MAP_GPIO_setOutputHighOnPin(port, pin);
    }
    else {
        MAP_GPIO_setOutputLowOnPin(port, pin);
    }

    HwiP_restore(key);

    DebugP_log1("PWM:(%p) stopped.", (uintptr_t) handle);
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
