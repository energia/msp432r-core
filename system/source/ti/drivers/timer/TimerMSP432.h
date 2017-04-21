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
 /** ===========================================================================
 *  @file       TimerMSP432.h
 *
 *  @brief      Timer driver interface for MSP432 devices
 *
 *  # Operation #
 *  The Timer implementation for MSP432 leverages two different hardware
 *  peripherals: Timer_A and Timer32. For each hardware instance, the
 *  configuration of clock dividers and prescalars is taken care of in the
 *  TimerMSP432_open function. This function will examine the provided period
 *  and clock resource and intelligently calculate the best clock divider for
 *  the period and clock source combination.
 *
 *  # Resource Allocation #
 *  Each Timer instantiation can only be associated with one exclusive hardware
 *  Timer. For example, if a Timer is declared that uses Timer_A1, no other
 *  Timer instance can leverage Timer_A1 without introducing unreliable
 *  behavior. This behavior is managed through a set of simple resource
 *  allocation APIs. For example, the TimerMSP432_allocateTimerResource API will
 *  accept a Timer's base address and "allocate" that timer for exclusive use.
 *  Any attempt to allocate this resource in the future will result in a "false"
 *  value being returned from the allocation API. To release and "free" the
 *  timer resource, the TimerMSP432_freeTimerResource is used.
 *
 *  ============================================================================
 */
#ifndef ti_driver_timer_TimerMSP432__include
#define ti_driver_timer_TimerMSP432__include

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/*!
 *  @brief TimerMSP432 Hardware Attributes
 *
 *  Timer hardware attributes that tell the TimerMSP432 driver specific hardware
 *  configurations and interrupt/priority settings.
 *
 *  A sample structure is shown below:
 *  @code
 *  const TimerMSP432_HWAttrs timerMSP432HWAttrs[] =
 *  {
 *      {
 *          .timerBaseAddress = TIMER32_0_BASE,
 *          .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
 *          .intNum = INT_T32_INT1,
 *          .intPriority = ~0
 *      }
 *  };
 *  @endcode
 */
typedef struct _TimerMSP432_HWAttr
{
    uint32_t timerBaseAddress;
    uint32_t clockSource;
    uint32_t intNum;
    uint32_t intPriority;
} TimerMSP432_HWAttrs;

/*!
 *  @brief TimerMSP432 Object
 *  Driver specific structure that takes care of various driver parameters such
 *  as hardware interrupts and callbacks. The application should not modify any
 *  members of this structure.
 */
typedef struct TimerMSP432_Object
{
    bool resourceAvailable;
    Timer_Config *config;
    HwiP_Params hwiParams;
    HwiP_Handle hwiHandle;
    Timer_CallBackFxn callBack;
    SemaphoreP_Handle timerSem;
    Timer_Mode timerMode;
} TimerMSP432_Object;

/* Function Declarations */
extern bool TimerMSP432_allocateTimerResource(uint32_t timerBase);
extern void TimerMSP432_close(Timer_Handle handle);
extern void TimerMSP432_freeTimerResource(uint32_t timerBase);
extern void TimerMSP432_init(Timer_Handle handle);
extern Timer_Handle TimerMSP432_open(Timer_Handle handle, Timer_Params *params);
extern int_fast16_t TimerMSP432_Timer_A_control(Timer_Handle handle,
        uint_fast16_t cmd, void *arg);
extern uint32_t TimerMSP432_Timer_A_getCount(Timer_Handle handle);
extern int32_t TimerMSP432_Timer_A_start(Timer_Handle handle);
extern void TimerMSP432_Timer_A_stop(Timer_Handle handle);
extern int_fast16_t TimerMSP432_Timer32_control(Timer_Handle handle,
        uint_fast16_t cmd, void *arg);
extern uint32_t TimerMSP432_Timer32_getCount(Timer_Handle handle);
extern int32_t TimerMSP432_Timer32_start(Timer_Handle handle);
extern void TimerMSP432_Timer32_stop(Timer_Handle handle);

/* External Hardware Configuration Variables */
extern TimerMSP432_Object timerMSP432Objects[];
extern const uint8_t Timer_count;
extern Timer_FxnTable TimerMSP432_Timer_A_fxnTable;
extern Timer_FxnTable TimerMSP432_Timer32_fxnTable;

#ifdef __cplusplus
}
#endif

#endif /* ti_driver_timer_TimerMSP432__include */
