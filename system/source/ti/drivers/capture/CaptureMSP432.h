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
  /** ===========================================================================
 *  @file       CaptureMSP432.h
 *
 *  @brief      Capture driver interface for MSP432 devices
 *
 *  # Operation #
 * The Capture driver for MSP432 leverages the Timer_A peripheral of the MSP432
 * devive. It is important to note that each instance of the Capture driver
 * will occupy exactly one Timer_A peripheral on the device. This is done to
 * reduce contingencies and conflicts that might arise from using more than one
 * Capture/Compare registers on a Timer_A peripheral. The specific
 * Capture/Compare register that is used by the Capture module is determined
 * automatically by the pin assignment. Pin assignments can also take advantage
 * of the pin mapping peripheral of the MSP432. If a pin mapped pin is given as
 * the capturePort parameter of the CaptureMSP432_HWAttrs structure, the
 * implementation of the driver will automatically configure the PMAP module
 * with the correct/corresponding pin assignments.
 *
 *  ============================================================================
 */
#ifndef ti_driver_capture_CaptureMSP432__include
#define ti_driver_capture_CaptureMSP432__include

#ifdef __cplusplus
extern "C"
{
#endif

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/timer/TimerMSP432.h>
#include <ti/drivers/Capture.h>

/* Port definitions for MSP432 TimerA pins. For P2, P3, and P7 we are able to
 * use the port mapper to map any TA0/TA1 pin to any pin. For the other ports
 * we are limited by the pin assignment.
 */
#define CAPTUREMSP432_INT_OFS                                               24
#define CAPTUREMSP432_CCR_OFS                                               16
#define CAPTUREMSP432_PMAP_OFS                                               8


/* Port 2 Settings */
#define CaptureMSP432_P2_0_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x20 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_0_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x20 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_1_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x21 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_1_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x21 | (INT_TA2_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_2_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x22 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_2_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x22 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_3_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x23 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_3_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x23 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_4_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x24 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_4_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x24 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_5_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x25 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_5_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x25 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_6_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x26 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_6_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x26 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_7_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x27 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P2_7_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x27 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))

/* Port 3 Settings */
#define CaptureMSP432_P3_0_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x30 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_0_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x30 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_1_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x31 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_1_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x31 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_2_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x32 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_2_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x32 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_3_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x33 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_3_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x33 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_4_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x34 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_4_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x34 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_5_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x35 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_5_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x35 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_6_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x36 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_6_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x36 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_7_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x37 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P3_7_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x37 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))

/* Port 5 Settings */
#define CaptureMSP432_P5_6_TA2 (0x56 | (INT_TA2_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P5_7_TA2 (0x57 | (INT_TA2_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 << CAPTUREMSP432_CCR_OFS))

/* Port 6 Settings */
#define CaptureMSP432_P6_6_TA2 (0x66 | (INT_TA2_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P6_7_TA2 (0x67 | (INT_TA2_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 << CAPTUREMSP432_CCR_OFS))

/* Port 7 Settings */
#define CaptureMSP432_P7_0_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x70 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_0_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x70 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_1_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x71 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_1_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x71 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_2_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x72 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_2_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x72 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_3_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x73 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_3_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x73 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_4_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x74 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_4_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x74 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_5_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x75 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_5_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x75 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_6_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x76 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_6_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x76 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_7_TA0 ((PMAP_TA0CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x77 | (INT_TA0_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P7_7_TA1 ((PMAP_TA1CCR1A << CAPTUREMSP432_PMAP_OFS) |    \
                               0x77 | (INT_TA1_N << CAPTUREMSP432_INT_OFS) |   \
                               (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))

/* Port 8 Settings */
#define CaptureMSP432_P8_0_TA1 (0x80 | (INT_TA1_0 << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_0 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P8_1_TA2 (0x81 | (INT_TA2_0 << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_0 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P8_2_TA3 (0x82 | (INT_TA3_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 << CAPTUREMSP432_CCR_OFS))

/* Port 9 Settings */
#define CaptureMSP432_P9_2_TA3 (0x92 | (INT_TA3_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P9_3_TA3 (0x93 | (INT_TA3_N << CAPTUREMSP432_INT_OFS) |  \
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 << CAPTUREMSP432_CCR_OFS))

/* Port 10 Settings */
#define CaptureMSP432_P10_4_TA3 (0xA4 | (INT_TA3_0 << CAPTUREMSP432_INT_OFS) | \
        (TIMER_A_CAPTURECOMPARE_REGISTER_0 << CAPTUREMSP432_CCR_OFS))
#define CaptureMSP432_P10_5_TA3 (0xA5 | (INT_TA3_1 << CAPTUREMSP432_INT_OFS) | \
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 << CAPTUREMSP432_CCR_OFS))

/*!
 *  @brief CaptureMSP432 Hardware Attributes
 *
 *  Capture hardware attributes that tell the CaptureMSP432 driver specific
 *  hardware configurations and interrupt/priority settings.
 *
 *  A sample structure is shown below:
 *  @code
 *  const TimerMSP432_HWAttrs timerMSP432HWAttrs[] =
 *  {
 *      {
 *          .timerBaseAddress = TIMER_A1_BASE,
 *          .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
 *          .clockDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64,
 *          .capturePort = CaptureMSP432_P7_7_TA1,
 *          .intPriority = ~0
 *      }
 *  };
 *  @endcode
 */
typedef struct CaptureMSP432_HWAttrs
{
    uint32_t timerBaseAddress;
    uint32_t clockSource;
    uint32_t clockDivider;
    uint32_t capturePort;
    uint32_t intPriority;
} CaptureMSP432_HWAttrs;

/*!
 *  @brief CaptureMSP432 Object
 *  Driver specific structure that takes care of various driver parameters such
 *  as hardware interrupts and callbacks. The application should not modify any
 *  members of this structure.
 */
typedef struct CaptureMSP432_Object
{
    Capture_Config *config;
    HwiP_Params hwiParams;
    HwiP_Handle hwiHandle;
    Capture_CallBackFxn callBack;
    Capture_Period_Unit periodUnits;
    uint32_t captureCount;
    uint32_t ccrRegister;
    uint32_t intNum;
} CaptureMSP432_Object;

extern void CaptureMSP432_close(Capture_Handle handle);
extern int_fast16_t CaptureMSP432_control(Capture_Handle handle,
        uint_fast16_t cmd, void *arg);
extern void CaptureMSP432_init(Capture_Handle handle);
extern Capture_Handle CaptureMSP432_open(Capture_Handle handle,
        Capture_Params *params);
extern void CaptureMSP432_start(Capture_Handle handle);
extern void CaptureMSP432_stop(Capture_Handle handle);

/* External Hardware Configuration */
extern CaptureMSP432_Object captureMSP432Objects[];
extern TimerMSP432_Object timerMSP432Objects[];
extern const uint8_t Capture_count;
extern Capture_FxnTable CaptureMSP432_captureFxnTable;

#endif /* ti_driver_capture_CaptureMSP432__include */
