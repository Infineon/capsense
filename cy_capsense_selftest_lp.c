/***************************************************************************//**
* \file cy_capsense_selftest_lp.c
* \version 7.0
*
* \brief
* This file provides the source code to the Built-in Self-test (BIST)
* functions.
*
********************************************************************************
* \copyright
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <stddef.h>
#include <string.h>
#include "cy_syslib.h"
#include "cy_sysclk.h"
#include "cycfg_capsense_defines.h"
#include "cycfg_peripherals.h"
#include "cy_capsense_common.h"
#include "cy_capsense_sensing.h"
#include "cy_capsense_sensing_lp.h"
#include "cy_capsense_generator_lp.h"
#include "cy_capsense_selftest_lp.h"
#include "cy_capsense_sm_base_full_wave_lp.h"
#include "cy_gpio.h"

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #include "cy_msclp.h"
#endif

#if (defined(CY_IP_M0S8MSCV3LP))

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN)

/*******************************************************************************
* Common local definitions for self-test
*******************************************************************************/

/* Port Data Register macros for BIST */
#define CY_CAPSENSE_BIST_DR_PIN2GND                             (0u)
#define CY_CAPSENSE_BIST_DR_PIN2VDD                             (1u)
/* Electrode capacitance measurement macros for BIST */
#define CY_CAPSENSE_BIST_CAP_SLOT_SCAN                          (0u)
#define CY_CAPSENSE_BIST_CAP_ELTD_SCAN                          (1u)
#define CY_CAPSENSE_BIST_CAP_SHIELD_SCAN                        (2u)
/** The number of BIST sense mode configurations for capacitance measurements (CSD + CSX) */
#define CY_CAPSENSE_BIST_SENSE_MODE_CONFIG_NUMBER               (2u)
#define CY_CAPSENSE_BIST_ELTD_CAP_MODCLK_DIV_DEFAULT            (1u)
#define CY_CAPSENSE_BIST_ELTD_CAP_MAX_MODCLK                    (48000000u)
#define CY_CAPSENSE_BIST_ELTD_CAP_SNSCLK_DIV_DEFAULT            (512u)
#define CY_CAPSENSE_BIST_SHIELD_CAP_SNSCLK_DIV_DEFAULT          (1024u)
#define CY_CAPSENSE_BIST_SNS_CLK_MIN_DIVIDER                    (4u)
#define CY_CAPSENSE_BIST_SNS_CLK_MAX_DIVIDER                    (4096u)
#define CY_CAPSENSE_BIST_ELTD_CAP_SUBCONV_NUM_DEFAULT           (100u)
#define CY_CAPSENSE_BIST_SHIELD_CAP_REF_CDAC_DEFAULT            (200u)
#define CY_CAPSENSE_BIST_ELTD_CAP_REF_CDAC_DEFAULT              (100u)
#define CY_CAPSENSE_BIST_MUTUAL_CAP_REF_CDAC_DEFAULT            (50u)
#define CY_CAPSENSE_BIST_WATCHDOG_MARGIN_COEFF                  (3u)
#define CY_CAPSENSE_BIST_CAP_MEAS_WDT_CYCLES_PER_LOOP           (5u)
#define CY_CAPSENSE_BIST_V3_ELTD_CAP_CYCLES_NUM                 (1u)
#define CY_CAPSENSE_BIST_CAP_MEAS_CDAC_LSB_FF_DIV_1000          (8870u)
#define CY_CAPSENSE_BIST_CP_MAX_VALUE                           (275000u)
#define CY_CAPSENSE_BIST_CSH_MAX_VALUE                          (1160000u)
#define CY_CAPSENSE_BIST_PROMILLE_FACTOR                        (1000u)
#define CY_CAPSENSE_BIST_ELTD_CAP_NUM_EPI_CYCLES                (2u)

/*******************************************************************************
* Macros for the external capacitor capacitance measurement test
*******************************************************************************/

#define CY_CAPSENSE_BIST_EXT_CAP_REF_CDAC_DEFAULT                               (250u)
#define CY_CAPSENSE_BIST_EXT_CAP_NUM_SUB_DEFAULT                                (8u)
#define CY_CAPSENSE_BIST_EXT_CAP_IMO_CLK_MHZ                                    (12u)
#define CY_CAPSENSE_BIST_EXT_CAP_WDT_FACTOR                                     (5u)
#define CY_CAPSENSE_BIST_EXT_CAP_ACCURACY_FACTOR                                (1150u)
#define CY_CAPSENSE_BIST_EXT_CAP_DISCHARGE_FACTOR                               (2u)
#define CY_CAPSENSE_BIST_EXT_CAP_RESULT_FACTOR                                  ((123uL * CY_CAPSENSE_REF_CDAC_LSB_X100 *\
                                                                                  CY_CAPSENSE_BIST_EXT_CAP_REF_CDAC_DEFAULT) /\
                                                                                  CY_CAPSENSE_CONVERSION_KILO)
#define CY_CAPSENSE_BIST_EXT_CAP_RESULT_DIVIDER                                 (CY_CAPSENSE_CONVERSION_HECTO *\
                                                                                 CY_CAPSENSE_CONVERSION_HECTO)
#define CY_CAPSENSE_BIST_EXT_CAP_MAX_CAP                                        (25u)
#define CY_CAPSENSE_BIST_EXT_CAP_CMOD_MAX_VALUE                                 (5u)

/*******************************************************************************
* Macros for the BIST VDDA measurement test
*******************************************************************************/

#define CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_MAX_POSSIBLE_VALUE                     (20u)
#define CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_DISCHARGE_TIME                         ((CY_CAPSENSE_BIST_EXT_CAP_DISCHARGE_FACTOR *\
                                                                                  CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_MAX_POSSIBLE_VALUE *\
                                                                                  CY_CAPSENSE_BIST_EXT_CAP_SERIAL_RESISTANCE) /\
                                                                                 CY_CAPSENSE_CONVERSION_KILO)
#define CY_CAPSENSE_BIST_VDDA_MEAS_MAX_RAW                                      (((((CY_CAPSENSE_BIST_EXT_CAP_ACCURACY_FACTOR *\
                                                                                     CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_MAX_POSSIBLE_VALUE) *\
                                                                                    CY_CAPSENSE_CONVERSION_KILO) /\
                                                                                   CY_CAPSENSE_BIST_EXT_CAP_REF_CDAC_DEFAULT) *\
                                                                                  CY_CAPSENSE_CONVERSION_KILO) /\
                                                                                 CY_CAPSENSE_BIST_CAP_MEAS_CDAC_LSB_FF_DIV_1000)
#define CY_CAPSENSE_BIST_VDDA_MEAS_NUM_SNS_CLK                                  ((CY_CAPSENSE_BIST_VDDA_MEAS_MAX_RAW /\
                                                                                 (MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk >>\
                                                                                  MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos)) + 1u)
#define CY_CAPSENSE_BIST_VDDA_MEAS_BGREF_VALUE                                  (1200u)
#define CY_CAPSENSE_BIST_VDDA_MEAS_RESULT_FACTOR                                (1715u)
#define CY_CAPSENSE_BIST_VDDA_MEAS_RESULT_SHIFT                                 (710u)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    static cy_en_capsense_bist_status_t Cy_CapSense_CheckAllWidgetCRC(
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
    static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckSensor(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint32_t mode,
                    cy_stc_capsense_context_t * context);
    static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckElectrode(
                    uint32_t widgetId,
                    uint32_t ioSnsId,
                    uint32_t mode,
                    const cy_stc_capsense_context_t * context);
    static void Cy_CapSense_SnsShortUpdateTestResult(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    cy_stc_capsense_context_t * context);
    static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckAllSensors(
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
        static void Cy_CapSense_BistSwitchHwConfig(
                        cy_en_capsense_bist_hw_config_t hwCfg,
                        uint8_t bistSenseGroup,
                        uint8_t bistScanMode,
                        cy_stc_capsense_context_t * context);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
        static void Cy_CapSense_BistSetPinDr(
                        cy_stc_capsense_pin_config_t const *ioPtr,
                        uint32_t value);
        static void Cy_CapSense_BistSetPinPc(
                        cy_stc_capsense_pin_config_t const *ioPtr,
                        uint32_t value);
    #endif

    static void Cy_CapSense_BistSwitchAllSnsPinState(
                    cy_en_capsense_bist_io_state_t desiredPinState,
                    const cy_stc_capsense_context_t * context);
    static void Cy_CapSense_BistSwitchAllExternalCapPinState(
                    cy_en_capsense_bist_io_state_t desiredPinState,
                    const cy_stc_capsense_context_t * context);
    void Cy_CapSense_BistSetAllSnsPinsState(
                    uint32_t desiredDriveMode,
                    uint32_t desiredPinOutput,
                    en_hsiom_sel_t desiredHsiom,
                    const cy_stc_capsense_context_t * context);
    void Cy_CapSense_BistSetAllCmodPinsState(
                    uint32_t desiredDriveMode,
                    uint32_t desiredPinOutput,
                    en_hsiom_sel_t desiredHsiom,
                    const cy_stc_capsense_context_t * context);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        static void Cy_CapSense_BistSwitchAllShieldPinState(
                        cy_en_capsense_bist_io_state_t desiredPinState,
                        const cy_stc_capsense_context_t * context);
        void Cy_CapSense_BistSetAllShieldPinsState(
                        uint32_t desiredDriveMode,
                        uint32_t desiredPinOutput,
                        en_hsiom_sel_t desiredHsiom,
                        const cy_stc_capsense_context_t * context);
    #endif

    static void Cy_CapSense_BistSetDmHsiomPinState(
                    cy_en_capsense_bist_io_state_t desiredPinState,
                    const cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
    static cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceAllElectrodes(
                cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSlotSensorsInternal(
                uint32_t slotId,
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context);
    static cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceAllSensors(
                cy_stc_capsense_context_t * context);
#endif

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
    static uint32_t Cy_CapSense_BistWatchdogPeriodCalc(
                        const cy_stc_capsense_context_t * context);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
        static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceSensor(
                        uint32_t * cpPtr,
                        uint32_t snsFrameType,
                        cy_stc_capsense_context_t * context);
    #endif

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || \
         ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
        static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceSlot(
                        uint32_t slotId,
                        uint32_t snsFrameType,
                        cy_stc_capsense_context_t * context);
    #endif
    static void Cy_CapSense_BistGenerateBaseConfig(
                    cy_stc_capsense_context_t * context);
    static void Cy_CapSense_BistGenerateSensorConfig(
                    uint32_t * ptrSensorCfg,
                    uint32_t snsFrameType,
                    cy_stc_capsense_context_t * context);
    static void Cy_CapSense_BistGenerateSnsCfgMaskReg(
                    const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                    uint32_t * ptrSensorCfg,
                    uint32_t cswFuncNum);
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
           ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN))
    static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceCapRun(
                    uint32_t * ptrExtCapValue,
                    cy_stc_capsense_context_t * context);
    static void Cy_CapSense_BistGenerateBaseCmodConfig(
                    cy_stc_capsense_context_t * context);
    void Cy_CapSense_DischargeCmod(
                    GPIO_PRT_Type * portCmod,
                    uint32_t pinCmod,
                    cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN)
    static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceCapAll(
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
    /* Measure VDDA test-related functions */
    static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureVddaRun(
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */

/** \} \endcond */


/*******************************************************************************
* Function Name: Cy_CapSense_RunSelfTest_V3Lp
****************************************************************************//**
*
* Runs built-in self-tests for specified by the test enable mask.
*
* The function performs various self-tests on all the enabled widgets
* and sensors in the project. Select the required set of tests
* using the bit-mask in testEnMask parameter.
*
* Use CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK to execute
* all the self-tests or any combination of the masks
* (defined in testEnMask parameter) to specify the desired test list.
*
* To execute a single-element test (i.e. for one widget or one sensor),
* the following low-level functions are available:
* * Cy_CapSense_CheckCRCWidget()
* * Cy_CapSense_CheckIntegritySensorRawcount()
* * Cy_CapSense_CheckIntegritySensorBaseline()
* * Cy_CapSense_CheckIntegritySensorPins()
* * Cy_CapSense_MeasureCapacitanceSensorElectrode()
* * Cy_CapSense_MeasureCapacitanceSlotSensors()
* * Cy_CapSense_MeasureCapacitanceShieldElectrode()
* * Cy_CapSense_MeasureCapacitanceCap()
* * Cy_CapSense_MeasureVdda()
*
* Refer to these functions descriptions for detail information
* on the corresponding test.
*
* \note
* The function is available only for the fifth-generation CAPSENSE&trade;.
*
* \param testEnMask
* Specifies the tests to be executed. Each bit corresponds to one test. It is
* possible to launch the function with any combination of the available tests.
* - CY_CAPSENSE_BIST_CRC_WDGT_MASK       - Verifies the RAM widget structure CRC
*                                          for all the widgets.
* - CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK  - Checks all the sensor pins for a short
*                                          to GND / VDD / other sensors.
* - CY_CAPSENSE_BIST_SNS_CAP_MASK        - Measures all the sensors capacitance.
* - CY_CAPSENSE_BIST_ELTD_CAP_MASK       - Measures all the electrodes capacitance.
* - CY_CAPSENSE_BIST_SHIELD_CAP_MASK     - Measures the shield capacitance.
* - CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK
*                                        - Executes all available tests.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a bit-mask with a status of execution of the specified tests:
* - CY_CAPSENSE_BIST_SUCCESS_E      - All the tests passed successfully.
* - CY_CAPSENSE_BIST_BAD_PARAM_E    - A non-defined test was requested in the
*                                     testEnMask parameter or the context is
*                                     a NULL pointer. The function
*                                     was not performed.
* - CY_CAPSENSE_BIST_HW_BUSY_E      - The CAPSENSE&trade; HW block is busy with a previous
*                                     operation. The function was not performed.
* - CY_CAPSENSE_BIST_ERROR_E        - An unexpected fault occurred during
*                                     the measurement, you may need to repeat
*                                     the measurement.
* - CY_CAPSENSE_BIST_FAIL_E         - Any of tests specified by the testEnMask
*                                     parameters has faulted.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_RunSelfTest_V3Lp(
                uint32_t testEnMask,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;

    if (0u == (testEnMask & (~CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK)))
    {
        if (NULL != context)
        {
            if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
            {
                result = CY_CAPSENSE_BIST_SUCCESS_E;

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
                    if (0u != (CY_CAPSENSE_BIST_CRC_WDGT_MASK & testEnMask))
                    {
                        if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_CheckAllWidgetCRC(context))
                        {
                            result = CY_CAPSENSE_BIST_FAIL_E;
                            context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_CRC_WDGT_MASK;
                        }
                    }
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN) */

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
                    /* The next group of tests is hardware-dependent and they need to switch the configuration to BIST one */
                    if (CY_CAPSENSE_STATUS_SUCCESS == Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context))
                    {

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
                            if (0u != (CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK & testEnMask))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_SnsShortCheckAllSensors(context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK;
                                }
                            }
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN) */

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
                            if (0u != (CY_CAPSENSE_BIST_VDDA_MASK  & testEnMask))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureVdda_V3Lp(NULL, context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_VDDA_MASK;
                                }
                            }
                        #endif

                        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN))
                            if (0u != (CY_CAPSENSE_BIST_EXTERNAL_CAP_MASK & testEnMask))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_BistMeasureCapacitanceCapAll(context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_EXTERNAL_CAP_MASK;
                                }
                            }
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
                            if (0u != (CY_CAPSENSE_BIST_SNS_CAP_MASK & testEnMask))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceAllSensors(context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_SNS_CAP_MASK;
                                }
                            }
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
                            if (0u != (CY_CAPSENSE_BIST_ELTD_CAP_MASK & testEnMask))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceAllElectrodes(context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_ELTD_CAP_MASK;
                                }
                            }
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) */

                        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
                             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))
                            if ((0u != (CY_CAPSENSE_BIST_SHIELD_CAP_MASK & testEnMask)) &&
                                (CY_CAPSENSE_DISABLE != context->ptrCommonConfig->csdShieldMode))
                            {
                                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceShieldElectrode_V3Lp(0u, context))
                                {
                                    result = CY_CAPSENSE_BIST_FAIL_E;
                                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_SHIELD_CAP_MASK;
                                }
                            }
                        #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
                                   (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)) */
                    }
                    else
                    {
                        result = CY_CAPSENSE_BIST_ERROR_E;
                    }
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */
            }
            else
            {
                result = CY_CAPSENSE_BIST_HW_BUSY_E;
            }
        }
    }

    return result;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CheckCRCWidget_V3Lp
****************************************************************************//**
*
* Checks the stored CRC of the \ref cy_stc_capsense_widget_context_t data
* structure of the specified widget.
*
* This function validates the data integrity of the
* \ref cy_stc_capsense_widget_context_t data structure of the specified widget
* by calculating the CRC and comparing it with the stored CRC value of the
* specified widget.
*
* Initially, after the device power up, the Cy_CapSense_Enable() function
* calculates CRC for each widget and stores them in the .ptrWdgtCrc[] array
* of the \ref cy_stc_capsense_bist_context_t structure. The test execution
* compares this stored CRC value with the newly calculated and if the stored
* and calculated CRC values differ:
* 1. The calculated CRC is stored to the .wdgtCrcCalc field
*    of the \ref cy_stc_capsense_bist_context_t data structure.
* 2. The widget ID is stored to the .crcWdgtId field.
* 3. The CY_CAPSENSE_BIST_CRC_WDGT_MASK bit is set in the .testResultMask field.
*
* The function never clears the CY_CAPSENSE_BIST_CRC_WDGT_MASK bit.
* If the CY_CAPSENSE_BIST_CRC_WDGT_MASK bit is set, the wdgtCrcCalc
* and .crcWdgtId fields are not updated.
*
* It is recommended to use the Cy_CapSense_SetParam() function to change
* the value of the \ref cy_stc_capsense_widget_context_t data structure elements
* as the CRC is updated by Cy_CapSense_SetParam() function.
*
* You can initiate this test by the Cy_CapSense_RunSelfTest() function with
* the CY_CAPSENSE_BIST_CRC_WDGT_MASK mask as an input.
*
* The function clears the CY_CAPSENSE_WD_WORKING_MASK bit of the .status
* field in \ref cy_stc_capsense_widget_context_t structure if the calculated
* CRC value differs to the stored CRC value.
* Those non-working widgets are skipped by the high-level scanning and
* processing functions. Restoring a widget to its working state should
* be done by the application level.
*
* For details of the used CRC algorithm, refer to the Cy_CapSense_GetCRC()
* function.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The stored CRC matches
*                                         the calculated CRC.
* - CY_CAPSENSE_BIST_FAIL_E             - The widget CRC differs to
*                                         the stored CRC.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameters are invalid.
*                                         The test was not executed.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_CheckCRCWidget_V3Lp(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint16_t crcValue;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;

    if (NULL != context)
    {
        if (context->ptrCommonConfig->numWd > widgetId)
        {
            crcValue = Cy_CapSense_GetCrcWidget(widgetId, context);
            if ((context->ptrBistContext->ptrWdgtCrc[widgetId]) != crcValue)
            {
                /* Write to the self-test data structure widgetId of the first badly-tested widget */
                if (0uL == (context->ptrBistContext->testResultMask & CY_CAPSENSE_BIST_CRC_WDGT_MASK))
                {
                    context->ptrBistContext->wdgtCrcCalc = crcValue;
                    context->ptrBistContext->crcWdgtId = (uint8_t)widgetId;
                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_CRC_WDGT_MASK;
                }
                /* Marks widget non-working */
                (void)Cy_CapSense_SetWidgetStatus(widgetId, 0u, CY_CAPSENSE_WD_WORKING_MASK, context);
                result = CY_CAPSENSE_BIST_FAIL_E;
            }
            else
            {
                result = CY_CAPSENSE_BIST_SUCCESS_E;
            }
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CheckAllWidgetCRC
****************************************************************************//**
*
* The internal function that checks CRC of all widget structures.
*
* The function calculates CRC of all widget structures and compare it
* to the stored CRCs. It is called by the Cy_CapSense_RunSelfTest() function.
* In the first case of failed comparison the function updates
* testResultMask and returns the status. Next widgets are not checked.
* The function use the Cy_CapSense_CheckCRCWidget() function.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the test processing:
* - CY_CAPSENSE_BIST_SUCCESS_E if all widget CRC are OK;
* - CY_CAPSENSE_BIST_FAIL_E if any widget CRC is wrong.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_CheckAllWidgetCRC(
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_SUCCESS_E;
    uint32_t widgetId;

    for (widgetId = 0u; widgetId < context->ptrCommonConfig->numWd; widgetId++)
    {
        if (CY_CAPSENSE_BIST_SUCCESS_E != (Cy_CapSense_CheckCRCWidget_V3Lp(widgetId, context)))
        {
            result = CY_CAPSENSE_BIST_FAIL_E;
            break;
        }
    }
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_UpdateCrcWidget
****************************************************************************//**
*
* The internal function updates the CRC
* of the \ref cy_stc_capsense_widget_context_t data structure
* for the specified widget.
*
* The function implements the following functionality:
* - Executes the Cy_CapSense_GetCRC() routine for the specified widget.
* - Updates the self-test CRC array with the CRC value, calculated for the
*   specified widget.
*
* The CRC value is stored in the special wdgtCrc[CY_CAPSENSE_WIDGET_COUNT] array
* declared in the cycfg_capsense.c file.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_UpdateCrcWidget(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint16_t crcValue;

    crcValue = Cy_CapSense_GetCrcWidget(widgetId, context);

    /* Write the calculated CRC value to the self-test CRC array */
    context->ptrBistContext->ptrWdgtCrc[widgetId] = crcValue;
}


/*******************************************************************************
* Function Name: Cy_CapSense_UpdateAllWidgetCrc
****************************************************************************//**
*
* The internal function that updates CRC of all widget structures.
*
* The function implements the following functionality:
* - Executes the Cy_CapSense_UpdateCrcWidget() for all widgets.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_UpdateAllWidgetCrc(cy_stc_capsense_context_t * context)
{
    uint32_t wdIndex;
    uint32_t wdNum = (uint32_t)context->ptrCommonConfig->numWd;

    /* Initialize CRC and status for all widgets */
    for (wdIndex = 0u; wdIndex < wdNum; wdIndex++)
    {
        Cy_CapSense_UpdateCrcWidget(wdIndex, context);
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CheckIntegritySensorBaseline_V3Lp
****************************************************************************//**
*
* Checks if the baseline of the specified sensor is not corrupted
* by comparing it with its inverse copy and checks if the baseline is
* within the specified range.
*
* The function checks whether or not the baseline binary inverted to
* its inverse copy is saved to the self-test baseline-inverse structure
* and is within the user-defined limits. If the baseline does
* not match its inverse copy or if the baseline is out of the user-defined
* limits, the function sets the CY_CAPSENSE_BIST_BSLN_INTEGRITY_MASK bit
* in the .testResultMask field of the \ref cy_stc_capsense_bist_context_t
* structure.
*
* The test is integrated into the CAPSENSE&trade; Middleware. All CAPSENSE&trade;
* processing functions like Cy_CapSense_ProcessAllWidgets()
* or Cy_CapSense_UpdateSensorBaseline() automatically verify the baseline
* value before using it and update its inverse copy after processing.
* If a baseline update fails, a CY_CAPSENSE_STATUS_BAD_DATA result
* is returned. The baseline initialization functions do not verify the
* baseline and update the baseline inverse copy.
*
* This function does not update the CY_CAPSENSE_WD_WORKING_MASK bit of
* the .status field in \ref cy_stc_capsense_widget_context_t structure
* and is not available in the Cy_CapSense_RunSelfTest() function.
*
* Use this function to verify the uniformity of sensors, for example, at
* mass-production or during an operation phase together with the
* Cy_CapSense_CheckIntegritySensorRawcount() function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \note
* This function does not support Liquid Level widget.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param sensorId
* Specifies the ID number of the sensor within the widget.
* A macro for the sensor ID within the specified widget can be found in
* the CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_SNS<SensorNumber>_ID.
*
* \param baselineHighLimit
* Specifies the upper limit for a baseline.
*
* \param baselineLowLimit
* Specifies the lower limit for a baseline.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The baseline is within the
*                                         specified range.
* - CY_CAPSENSE_BIST_FAIL_E             - The test failed and the baseline
*                                         is not binary inverted to its inverse
*                                         copy or is out of the specified limits.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The test was not executed.
* - CY_CAPSENSE_BIST_FEATURE_DISABLED_E - The selected widget is not supported.
*                                         The test was not executed.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorBaseline_V3Lp(
                uint32_t widgetId,
                uint32_t sensorId,
                uint16_t baselineHighLimit,
                uint16_t baselineLowLimit,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;
    uint16_t bslnInv;
    uint32_t cxtOffset;
    const cy_stc_capsense_sensor_context_t *ptrSnsCxt;
    cy_stc_capsense_widget_config_t const *ptrWdgtCfg;

    if (NULL != context)
    {
        if ((context->ptrCommonConfig->numWd > widgetId) &&
            (context->ptrWdConfig[widgetId].numSns > sensorId))
        {
            if ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != context->ptrWdConfig[widgetId].wdType)
            {
                /* Get a pointer to the specified widget configuration structure */
                ptrWdgtCfg = &context->ptrWdConfig[widgetId];
                /* Get a pointer to the specified sensor context structure */
                ptrSnsCxt = &ptrWdgtCfg->ptrSnsContext[sensorId];

                /* Check baselines */

                cxtOffset = sensorId;
                bslnInv = (uint16_t)(~(ptrWdgtCfg->ptrBslnInv[cxtOffset]));
                if ((ptrSnsCxt->bsln != bslnInv) ||
                    (ptrSnsCxt->bsln > baselineHighLimit) ||
                    (ptrSnsCxt->bsln < baselineLowLimit))
                {
                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_BSLN_INTEGRITY_MASK;
                    result = CY_CAPSENSE_BIST_FAIL_E;
                }

                if (CY_CAPSENSE_BIST_FAIL_E != result)
                {
                    result = CY_CAPSENSE_BIST_SUCCESS_E;
                }
            }
            else
            {
                result = CY_CAPSENSE_BIST_FEATURE_DISABLED_E;
            }
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CheckIntegritySensorRawcount_V3Lp
****************************************************************************//**
*
* Checks the raw count of the specified widget/sensor is within the specified
* range.
*
* The raw count is within a specific range (based on the calibration target)
* for good units. The function checks whether or not the raw count is within
* the user-defined limits in the ranges function arguments.
* If the raw count is out of limits, this function sets the
* CY_CAPSENSE_BIST_RAW_INTEGRITY_MASK bit in the .testResultMask field of the
* \ref cy_stc_capsense_bist_context_t structure.
*
* This function does not update the CY_CAPSENSE_WD_WORKING_MASK bit of
* the .status field in \ref cy_stc_capsense_widget_context_t structure
* and is not available in the Cy_CapSense_RunSelfTest() function.
*
* Use this function to verify the uniformity of sensors, for example, at
* mass-production or during an operation phase together with the
* Cy_CapSense_CheckIntegritySensorBaseline() function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \note
* This function does not support Liquid Level widget.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param sensorId
* Specifies the ID number of the sensor within the widget.
* A macro for the sensor ID within the specified widget can be found in
* the CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_SNS<SensorNumber>_ID.
*
* \param rawcountHighLimit
* Specifies the upper limit for the widget/sensor raw count.
*
* \param rawcountLowLimit
* Specifies the lower limit for the widget/sensor raw count.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The raw count is within the
*                                         specified range.
* - CY_CAPSENSE_BIST_FAIL_E             - The test failed and raw count is out
*                                         of the specified limits.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The test was not executed.
* - CY_CAPSENSE_BIST_FEATURE_DISABLED_E - The selected widget is not supported.
*                                         The test was not executed.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorRawcount_V3Lp(
                uint32_t widgetId,
                uint32_t sensorId,
                uint16_t rawcountHighLimit,
                uint16_t rawcountLowLimit,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;
    const cy_stc_capsense_sensor_context_t *ptrSnsCxt;
    cy_stc_capsense_widget_config_t const *ptrWdgtCfg;

    if (NULL != context)
    {
        if ((context->ptrCommonConfig->numWd > widgetId) &&
            (context->ptrWdConfig[widgetId].numSns > sensorId))
        {
            if ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != context->ptrWdConfig[widgetId].wdType)
            {
                /* Find a pointer to the specified widget configuration structure */
                ptrWdgtCfg = &context->ptrWdConfig[widgetId];
                /* Find a pointer to the specified sensor context structure */
                ptrSnsCxt = &ptrWdgtCfg->ptrSnsContext[sensorId];
                /* Check raw counts */
                if ((ptrSnsCxt->raw  > rawcountHighLimit) ||
                    (ptrSnsCxt->raw  < rawcountLowLimit))
                {
                    context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_RAW_INTEGRITY_MASK;
                    result = CY_CAPSENSE_BIST_FAIL_E;
                }

                if (CY_CAPSENSE_BIST_FAIL_E != result)
                {
                    result = CY_CAPSENSE_BIST_SUCCESS_E;
                }
            }
            else
            {
                result = CY_CAPSENSE_BIST_FEATURE_DISABLED_E;
            }
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CheckIntegritySensorPins_V3Lp
****************************************************************************//**
*
* Checks the specified widget/sensor for shorts to GND, VDD or other sensors.
*
* This function performs several sub-tests to verify the specified sensor
* is not electrically shorted and is in a good condition to reliably detect
* user interactions.
*
* This function performs tests to check if the specified sensor is shorted to:
* * GND
* * VDD
* * Other GPIOs used by CAPSENSE&trade; (such as sensors, Tx, Rx,
*   shield electrodes, and external capacitors)
* * Other non-CAPSENSE&trade; GPIOs (only if they are configured
*   in a strong high or low state during the test execution).
*
* The absolute resistance of an electrical short must be less than 1500 Ohm
* including all series resistors on a sensor for a short to be detected
* to GND, VDD or GPIOs. For example, if a series resistor on a sensor is
* 560 Ohm (as recommended) and the sensor is shorted with another sensor,
* the function can detect a short with a short resistance up to 380 Ohm as
* there are two 560 ohm resistors between the shorted sensor GPIOs.
*
* The function executes the following flow to detect a short:
* * Configures all CAPSENSE&trade; controlled GPIOs to strong-drive-high,
*   and the specified sensor GPIO to resistive pull down mode.
* * Waits for a delay (defined by .snsIntgShortSettlingTime field
*   of the \ref cy_stc_capsense_bist_context_t structure) to get established
*   all transient processes.
* * Checks the status of the specified sensor for the expected state
*   (logic low).
* * Configures all CAPSENSE&trade; controlled GPIOs to strong-drive-low,
*   and the specified sensor GPIO to resistive pull up mode.
* * Waits for the above mentioned delay.
* * Checks the status of the specified sensor for the expected state
*   (logic high).
* * Stores the test result in the CAPSENSE&trade; Data Structure.
*   A short is reported only when the sensor status check returns
*   an unexpected state.
*
* Due to the sensor parasitic capacitance and internal pull-up/down resistance,
* logic high-to-low (and vice versa) transitions require a settling time before
* checking the sensor status. A 2us delay is used as a settling time and can
* be changed using the .snsIntgShortSettlingTime field
* of the cy_stc_capsense_bist_context_t structure.
*
* If a short is detected this function updates the following statuses:
* * The widget ID is stored to the .shortedWdId field
*   of the \ref cy_stc_capsense_bist_context_t structure.
* * The sensor ID is stored to the .shortedSnsId field
*   of the \ref cy_stc_capsense_bist_context_t structure.
* * The CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK bit is set in the .testResultMask field
*   of the \ref cy_stc_capsense_bist_context_t structure.
* * If CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK is already set due to a previously
*   detected fault on any of the sensor, this function does not update
*   the .shortedWdId and .shortedSnsId fields. For this reason,
*   clear the CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK bit prior calling this function.
* * The widget is disabled by clearing the CY_CAPSENSE_WD_WORKING_MASK bit
*   in the .status field of the \ref cy_stc_capsense_widget_context_t structure
*   of the specified widget.
*   The disabled widget is ignored by high-level functions of scanning / data
*   processing. To restore the widget operation
*   the application layer should manually set the CY_CAPSENSE_WD_WORKING_MASK
*   bit in the .status field of the \ref cy_stc_capsense_widget_context_t structure
*   of the specified widget.
*
* To check all the project sensors at once, use the Cy_CapSense_RunSelfTest()
* function with the CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK mask.
*
* To detect an electrical short or fault condition with resistance
* higher than 1500 ohm, the Cy_CapSense_MeasureCapacitanceSensorElectrode() function can
* be used as the fault condition affects the measured sensor capacitance.
*
* This test can be executed only if the CAPSENSE&trade; Middleware is in the IDLE
* state. This function must not be called while CAPSENSE&trade; Middleware is busy.
*
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
* Rx/Lx electrodes for ISX widgets are excluded from the test as
* they are electrically shorted to GND and the CY_CAPSENSE_BIST_BAD_PARAM_E result
* for such widgets is returned.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param sensorId
* Specifies the ID of the sensor (electrode for CSX widgets) within the widget
* to be tested.
*
* For the CSD widgets, a macro for the sensor ID within the specified widget
* can be found in the CAPSENSE&trade; Configuration header file (cycfg_capsense.h)
* defined as CY_CAPSENSE_<WidgetName>_SNS<SensorNumber>_ID.
*
* For the CSX widgets, sensorId is an electrode ID and is defined as Rx ID
* or Tx ID. The first Rx in a widget corresponds to electrodeId = 0, the
* second Rx in a widget corresponds to electrodeId = 1, and so on.
* The last Tx in a widget corresponds to electrodeId = (RxNum + TxNum - 1).
* Macros for Rx and Tx IDs can be found in the CAPSENSE&trade; Configuration header
* file (cycfg_capsense.h) defined as:
* * CapSense_<WidgetName>_RX<RXNumber>_ID
* * CapSense_<WidgetName>_TX<TXNumber>_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - Sensor pin resistances are in defined range.
* - CY_CAPSENSE_BIST_FAIL_E             - The test failed and Sensor pin resistances
*                                         are out of the defined range.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The test was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation. The function
*                                         was not executed.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorPins_V3Lp(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;
    uint32_t numWdgtElectrodes;

    if (NULL != context)
    {
        if (context->ptrCommonConfig->numWd > widgetId)
        {
            if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
            {
                /* Get a total number of the widget elements: for CSX it is numRows + numCols, for CSD it is totalNumSns */
                if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[widgetId].senseMethod)
                {
                    /* For the CSX widgets, get the index of the Rx electrode */
                    numWdgtElectrodes = context->ptrWdConfig[widgetId].numRows +
                            (uint32_t)context->ptrWdConfig[widgetId].numCols;
                }
                else
                {
                    numWdgtElectrodes = context->ptrWdConfig[widgetId].numSns;
                }
                if (numWdgtElectrodes > sensorId)
                {
                    /* Initialize the result */
                    result = CY_CAPSENSE_BIST_SUCCESS_E;
                    /* Release previously-captured HW resources by the other mode and capture them for BIST */
                    if ((uint32_t)CY_CAPSENSE_SUCCESS_E == Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context))
                    {
                        /* Switch the HW resource configuration to the sensor short test */
                        Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_SHORT_E, CY_CAPSENSE_UNDEFINED_GROUP, 0u,
                                                    context);

                        /* Set all CAPSENSE&trade; pins to strong-high */
                        Cy_CapSense_BistSwitchAllSnsPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                            Cy_CapSense_BistSwitchAllShieldPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);
                        #endif
                        Cy_CapSense_BistSwitchAllExternalCapPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);

                        /* Wait for the maximum possible external capacitor charging time */
                        Cy_SysLib_DelayUs(context->ptrBistContext->snsIntgShortSettlingTime);

                        if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_SnsShortCheckSensor(widgetId, sensorId, CY_CAPSENSE_BIST_DR_PIN2VDD, context))
                        {
                            result = CY_CAPSENSE_BIST_FAIL_E;
                        }

                        /* Set all CAPSENSE&trade; pins to strong-low */
                        Cy_CapSense_BistSwitchAllSnsPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                            Cy_CapSense_BistSwitchAllShieldPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
                        #endif
                        Cy_CapSense_BistSwitchAllExternalCapPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);

                        /* Wait for the maximum possible external capacitor charging time */
                        Cy_SysLib_DelayUs(context->ptrBistContext->snsIntgShortSettlingTime);

                        if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_SnsShortCheckSensor(widgetId, sensorId, CY_CAPSENSE_BIST_DR_PIN2GND, context))
                        {
                            result = CY_CAPSENSE_BIST_FAIL_E;
                        }
                    }
                    else
                    {
                        result = CY_CAPSENSE_BIST_HW_BUSY_E;
                    }
                }
            }
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SnsShortCheckSensor
****************************************************************************//**
*
* The internal function checks if the specified sensor element is shorted
* to the VDD or GND level by configuring each of its electrodes to pull-up or
* pull-down and check their state.
*
* An additional delay is added between configuring the electrode and
* reading its state to establish the transition process for cases
* with big capacitance and short resistance.
* The function assumes all rest electrodes are set to strong drive mode,
* so the sensor-to-sensor short condition is also detected.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param sensorId
* Specifies the ID of the sensor element within the widget to change
* its pin state.
* * For the CSD widgets, use the sensor ID. A macro for the
*   sensor ID within a specified widget can be found in the cycfg_capsense.h
*   file defined as CY_CAPSENSE_<WIDGET_NAME>_SNS<SENSOR_NUMBER>_ID.
* * For the CSX widgets use either Rx ID or Tx ID.
*   The first Rx in a widget corresponds to sensorElement = 0; the second
*   Rx in a widget corresponds to sensorElement = 1, and so on.
*   The last Tx in a widget corresponds to sensorElement = (RxNum + TxNum - 1).
*   A macro for the Rx ID or Tx ID can be found in the cycfg_capsense.h
*   file defined as CY_CAPSENSE_<WIDGET_NAME>_<TX/RX><TX/RX_NUMBER>_ID.
*
* \param mode
* Specifies the test mode, either:
* * CY_CAPSENSE_BIST_DR_PIN2GND means sensor is configured
*   to pull-up and checked for logical 0
* * CY_CAPSENSE_BIST_DR_PIN2VDD means sensor is configured
*   to pull-down and checked for logical 1
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The sensor pin(s) are not shorted.
* - CY_CAPSENSE_BIST_FAIL_E - A short is detected on the specified sensor.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckSensor(
                uint32_t widgetId,
                uint32_t sensorId,
                uint32_t mode,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result;
    uint32_t ioSnsId;

    if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[widgetId].senseMethod)
    {
        /* For the CSX widgets, get the index of the Rx electrode */
        ioSnsId = sensorId / context->ptrWdConfig[widgetId].numRows;
        result = Cy_CapSense_SnsShortCheckElectrode(widgetId, ioSnsId, mode, context);
        if (CY_CAPSENSE_BIST_SUCCESS_E == result)
        {
            /* For the CSX widgets, get the index of the Tx electrode */
            ioSnsId = (uint32_t)(sensorId % context->ptrWdConfig[widgetId].numRows) +
                      (uint32_t)context->ptrWdConfig[widgetId].numCols;
            result = Cy_CapSense_SnsShortCheckElectrode(widgetId, ioSnsId, mode, context);
        }
    }
    else
    {
        result = Cy_CapSense_SnsShortCheckElectrode(widgetId, sensorId, mode, context);
    }

    if (CY_CAPSENSE_BIST_SUCCESS_E != result)
    {
        Cy_CapSense_SnsShortUpdateTestResult(widgetId, sensorId, context);
    }
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SnsShortCheckElectrode
****************************************************************************//**
*
* This internal function checks if a sensor or Rx or Tx electrode is shorted
* to VDD or GND by configuring each of its pins to pull-up or pull-down
* and checks its state.
*
* An additional delay is added between configuring the electrode and
* reading its state to establish the transition process for cases
* with big capacitance and short resistance.
* The function assumes all rest electrodes are set to strong drive mode,
* so the sensor-to-sensor short condition is also detected.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param ioSnsId
* Specifies the ID number of the sensor (Rx or Tx electrode for CSX widgets)
* within the widget to be processed.
*
* \param mode
* Specifies the test mode, either:
* * CY_CAPSENSE_BIST_DR_PIN2GND means sensor is configured
*   to pull-up and checked for logical 0
* * CY_CAPSENSE_BIST_DR_PIN2VDD means sensor is configured
*   to pull-down and checked for logical 1
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The sensor pin(s) are not shorted.
* - CY_CAPSENSE_BIST_FAIL_E - A short is detected on the specified sensor.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckElectrode(
                uint32_t widgetId,
                uint32_t ioSnsId,
                uint32_t mode,
                const cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint32_t eltdNum;
    uint32_t eltdState;
    uint32_t eltdDM = CY_GPIO_DM_PULLDOWN;
    cy_en_capsense_bist_status_t result;
    const cy_stc_capsense_pin_config_t *ioPtr = context->ptrWdConfig[widgetId].ptrEltdConfig[ioSnsId].ptrPin;

    if (CY_CAPSENSE_BIST_DR_PIN2GND == mode)
    {
        eltdDM = CY_GPIO_DM_PULLUP;
    }

    eltdNum = context->ptrWdConfig[widgetId].ptrEltdConfig[ioSnsId].numPins;

    /* Loop through all electrodes of the specified sensor */
    for (i = 0u; i < eltdNum; i++)
    {
        /* Set pin Drive mode and data register */
        Cy_CapSense_BistSetPinDr(ioPtr, (mode ^ 0x01u));
        Cy_CapSense_BistSetPinPc(ioPtr, eltdDM);
        /* Wait for establishing the transition process */
        Cy_SysLib_DelayUs((uint16_t)context->ptrBistContext->snsIntgShortSettlingTime);
        /* Read the electrode state */
        eltdState = Cy_GPIO_Read(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber);
        if (CY_CAPSENSE_BIST_DR_PIN2GND != eltdState)
        {
            eltdState = CY_CAPSENSE_BIST_DR_PIN2VDD;
        }
        /* Revert the electrode to the default Drive mode */
        Cy_CapSense_BistSetPinDr(ioPtr, mode);
        Cy_CapSense_BistSetPinPc(ioPtr, CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF);
        /* Check the electrode state */
        if (mode == eltdState)
        {
            result = CY_CAPSENSE_BIST_FAIL_E;
            break;
        }
        else
        {
            result = CY_CAPSENSE_BIST_SUCCESS_E;
        }
        /* Get the next electrode */
        ioPtr++;
    }
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SnsShortUpdateTestResult
****************************************************************************//**
* The internal function updates the BIST data structure and Widget Working bit
* in the .status field of the cy_stc_capsense_widget_context_t structure.
*
* The function resets a Widget Working bit, checks the .testResultMask field
* of the cy_stc_capsense_bist_context_t structure
* for the CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK bit and if it was not set to 1,
* the function sets it and memorizes widgetId and sensorId
* in corresponding fields of the cy_stc_capsense_bist_context_t structure.
*
* \param widgetId
* Specifies the ID number of the widget to be processed.
*
* \param sensorId
* Specifies the ID number of the sensor within the widget which
* will be processed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_SnsShortUpdateTestResult(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    /* Marks widget non-working */
    (void)Cy_CapSense_SetWidgetStatus(widgetId, 0u, CY_CAPSENSE_WD_WORKING_MASK, context);
    if (0Lu == (context->ptrBistContext->testResultMask & CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK))
    {
        /* Write to the BIST context structure widgetId and sensorId of the first shorted sensor */
        context->ptrBistContext->testResultMask |= CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK;
        context->ptrBistContext->shortedWdId = (uint8_t)widgetId;
        context->ptrBistContext->shortedSnsId = (uint8_t)sensorId;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_SnsShortCheckAllSensors
****************************************************************************//**
*
* The internal function that checks for all the sensors short.
*
* The function that checks for shorts on VDD/GND or to another sensors of all
* sensor (not electrode) by using the Cy_CapSense_SnsShortCheckSensor() function.
* The function is called by the Cy_CapSense_RunSelfTest() function.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the test processing:
* - CY_CAPSENSE_BIST_SUCCESS_E if test passed successfully;
* - CY_CAPSENSE_BIST_FAIL_E if any sensor of any widget is
*   shorted to VDD or GND.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_SnsShortCheckAllSensors(
                cy_stc_capsense_context_t * context)
{
    uint32_t widgetId;
    uint32_t sensorId;
    uint32_t numWdgtElectrodes;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_SUCCESS_E;

    /* Previously-captured HW resources were released by the other mode in the RunSelfTest function */
    /* Switch HW resource configuration to sensor short test */
    Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_SHORT_E, CY_CAPSENSE_UNDEFINED_GROUP, 0u,
                                   context);

    Cy_CapSense_BistSwitchAllSnsPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        Cy_CapSense_BistSwitchAllShieldPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);
    #endif
    Cy_CapSense_BistSwitchAllExternalCapPinState(CY_CAPSENSE_BIST_IO_STRONG_HIGH_E, context);

    /* Wait for the maximum possible external capacitor charging time */
    Cy_SysLib_DelayUs(context->ptrBistContext->snsIntgShortSettlingTime);

    for (widgetId = 0u; widgetId < context->ptrCommonConfig->numWd; widgetId++)
    {
        if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
        {
            /* Get a total number of the widget elements: for CSX it is numRows + numCols, for CSD it is totalNumSns */
            if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[widgetId].senseMethod)
            {
                /* For the CSX widgets, get the index of the Rx electrode */
                numWdgtElectrodes = context->ptrWdConfig[widgetId].numRows +
                        (uint32_t)context->ptrWdConfig[widgetId].numCols;
            }
            else
            {
                numWdgtElectrodes = context->ptrWdConfig[widgetId].numSns;
            }

            for (sensorId = 0u; sensorId < numWdgtElectrodes; sensorId++)
            {
                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_SnsShortCheckSensor(widgetId, sensorId, CY_CAPSENSE_BIST_DR_PIN2VDD, context))
                {
                    result = CY_CAPSENSE_BIST_FAIL_E;
                    break;
                }
            }
        }
    }

    Cy_CapSense_BistSwitchAllSnsPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        Cy_CapSense_BistSwitchAllShieldPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
    #endif
    Cy_CapSense_BistSwitchAllExternalCapPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);

    /* Wait for the maximum possible external capacitor charging time */
    Cy_SysLib_DelayUs(context->ptrBistContext->snsIntgShortSettlingTime);

    for (widgetId = 0u; widgetId < context->ptrCommonConfig->numWd; widgetId++)
    {
        /* Get a total number of the widget elements: for CSX it is numRows + numCols, for CSD it is totalNumSns */
        if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[widgetId].senseMethod)
        {
            /* For the CSX widgets, get the index of the Rx electrode */
            numWdgtElectrodes = context->ptrWdConfig[widgetId].numRows +
                      (uint32_t)context->ptrWdConfig[widgetId].numCols;
        }
        else
        {
            numWdgtElectrodes = context->ptrWdConfig[widgetId].numSns;
        }

        for (sensorId = 0u; sensorId < numWdgtElectrodes; sensorId++)
        {
            if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
            {
                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_SnsShortCheckSensor(widgetId, sensorId, CY_CAPSENSE_BIST_DR_PIN2GND, context))
                {
                    result = CY_CAPSENSE_BIST_FAIL_E;
                    break;
                }
            }
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistSetPinDr
****************************************************************************//**
*
* The internal function that sets a certain pin output data register (DR).
*
* The function sets a pin output data register (DR) in a desired state.
*
* \param *ioPtr
* A pointer to the specified pin in the widget pin configuration structure.
*
* \param value
* A port output data which will be set for the pin.
*
*******************************************************************************/
static void Cy_CapSense_BistSetPinDr(
                cy_stc_capsense_pin_config_t const *ioPtr,
                uint32_t value)
{
    uint32_t  interruptState;

    /* Set a data register */
    interruptState = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_Write(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber, value);
    Cy_SysLib_ExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistSetPinPc
****************************************************************************//**
*
* The internal function that sets a certain pin output drive mode (PC).
*
* The function sets a pin port control register (PC) in a desired state.
*
* \param *ioPtr
* A pointer to the specified pin in the widget pin configuration structure.
*
* \param value
* Drive mode to be set for the pin.
*
*******************************************************************************/
static void Cy_CapSense_BistSetPinPc(
                cy_stc_capsense_pin_config_t const *ioPtr,
                uint32_t value)
{
    uint32_t  interruptState;

    /* Update the port configuration register (Drive mode) */
    interruptState = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_SetDrivemode(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber, value);
    Cy_SysLib_ExitCriticalSection(interruptState);
}
#endif


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistSetAllSnsPinsState
****************************************************************************//**
*
* Sets all CAPSENSE&trade; pins into a desired state.
*
* Sets all the CSD/CSX IOs into a desired state.
* Default state:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - Strong drive.
* - State   - Zero.
*
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredPinOutput
* Specifies the desired pin output data register (DR) state.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistSetAllSnsPinsState(
                uint32_t desiredDriveMode,
                uint32_t desiredPinOutput,
                en_hsiom_sel_t desiredHsiom,
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->numPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr,
            (uint32_t)ptrPinCfg->pinNumber, desiredDriveMode, desiredHsiom, CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN);

        if (0u != desiredPinOutput)
        {
            Cy_GPIO_Set(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        }
        else
        {
            Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        }
        /* Get next electrode */
        ptrPinCfg++;
    }
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistSetAllShieldPinsState
****************************************************************************//**
*
* Sets all shield pins into a desired state.
*
* Sets all the dedicated shield electrodes into a desired state.
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredPinOutput
* Specifies the desired pin output data register (DR) state.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistSetAllShieldPinsState(
                uint32_t desiredDriveMode,
                uint32_t desiredPinOutput,
                en_hsiom_sel_t desiredHsiom,
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrShieldPinConfig;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->csdShieldNumPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr,
            (uint32_t)ptrPinCfg->pinNumber, desiredDriveMode, desiredHsiom, CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN);

        if (0u != desiredPinOutput)
        {
            Cy_GPIO_Set(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        }
        else
        {
            Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        }
        /* Get next electrode */
        ptrPinCfg++;
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) */


/*******************************************************************************
* Function Name: Cy_CapSense_BistSetAllCmodPinsState
****************************************************************************//**
*
* Sets all available MSC Cmod pins connected into a desired state .
*
* Sets all external capacitors connected into a desired state.
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredPinOutput
* Specifies the desired pin output data register (DR) state.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistSetAllCmodPinsState(
                uint32_t desiredDriveMode,
                uint32_t desiredPinOutput,
                en_hsiom_sel_t desiredHsiom,
                const cy_stc_capsense_context_t * context)
{
    uint32_t curChIndex;

    uint8_t cmod1Pin;
    uint8_t cmod2Pin;

    GPIO_PRT_Type * cmod1Port;
    GPIO_PRT_Type * cmod2Port;

    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;

    /* Loop through all electrodes */
    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        cmod1Port = ptrCommonCfg->ptrChConfig[curChIndex].portCmod1;
        cmod1Pin = ptrCommonCfg->ptrChConfig[curChIndex].pinCmod1;

        cmod2Port = ptrCommonCfg->ptrChConfig[curChIndex].portCmod2;
        cmod2Pin = ptrCommonCfg->ptrChConfig[curChIndex].pinCmod2;

        Cy_CapSense_SsConfigPinRegisters(cmod1Port, (uint32_t)cmod1Pin, desiredDriveMode, desiredHsiom, CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN);
        Cy_CapSense_SsConfigPinRegisters(cmod2Port, (uint32_t)cmod2Pin, desiredDriveMode, desiredHsiom, CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN);

        if (0u != desiredPinOutput)
        {
            Cy_GPIO_Set(cmod1Port, (uint32_t)cmod1Pin);
        }
        else
        {
            Cy_GPIO_Clr(cmod1Port, (uint32_t)cmod1Pin);
        }

        if (0u != desiredPinOutput)
        {
            Cy_GPIO_Set(cmod2Port, (uint32_t)cmod2Pin);
        }
        else
        {
            Cy_GPIO_Clr(cmod2Port, (uint32_t)cmod2Pin);
        }
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceSensorElectrode_V3Lp
****************************************************************************//**
*
* Measures the specified CSD sensor / CSX electrode capacitance in femtofarads.
*
* This function measures the sensor capacitance for CSD widgets or the electrode
* capacitance for CSX
* widgets and returns the measurement status. For a CSX sensor, the
* measurement is done on either Rx or Tx electrode.
* For a CSD sensor, measurement is done on a sensor (refer to the
* eltdId parameter description).
* The function does not measure capacitances of Lx/Rx electrodes of the ISX widgets
* as they are electrically connected to the GND.
* The CY_CAPSENSE_BIST_BAD_PARAM_E status for such a widget is returned.
* If the specified sensor (electrode) is a ganged sensor, the capacitance
* is measured for all the pins ganged together that belong to this sensor
* (electrode).
*
* The measured capacitance is stored in the .eltdCap[] array.
* The .ptrEltdCapacitance field of the
* \ref cy_stc_capsense_widget_config_t structure contains a pointer to
* the first widget sensor (electrode) element within the .eltdCap[] array.
*
* In addition to the measuring sensor (electrode) capacitance, this function
* is used to identify various fault conditions with sensors such
* as electrically-opened or -shorted sensors. For example, the PCB track is
* broken or shorted to other nodes in the system - in all of these conditions,
* this function returns changed capacitance which can be compared
* against predetermined capacitance for the sensor to detect a
* fault condition.
*
* The sensor capacitance is measured independently of the sensor scan
* configuration. For the capacitance measurement, the CSD sensing method is used.
* The default scanning parameters are the following:
* * SnsClk divider (256) is the divider for the sensor clock frequency.
* * NumConv (100) is the number of sub-conversions.
* * The reference CDAC capacitance (887 fF) is equivalent to CDAC Code of 100u.
* * The compensation CDAC is disabled.
* * The CIC2 filter is disabled.
* * The dithering is disabled.
* * The chopping is disabled.
*
* The raw count is converted into capacitance using the following equation:
*
*  Cs = Rawcount * RefCDAC capacitance / NumConv
*
* where:
* * Cs is the sensor capacitance.
* * Rawcount is the measured raw count value.
*
* The minimum measurable input by this function is 1pF and the
* maximum is either 275pF or limited by the RC time constant
* (Cs < 1 / (4*5*SnsClk*R), where R is the total sensor series
* resistance that includes on-chip GPIO resistance ~500 Ohm and
* external series resistance). The measurement accuracy is about 30% and
* is defined by the RefCDAC tolerance.
*
* By default, all CAPSENSE&trade; sensors (electrodes) inherit regular
* scanning configuration. For example if the Inactive sensor
* connection parameter of the CSD sensing method is set to GND,
* sensors that are not being measured are set to the GND state.
* The inactive state can be changed in run-time by using
* the Cy_CapSense_SetInactiveElectrodeState() function.
*
* By default, the both Cmod1 and Cmod2 capacitors are used for the measurement.
*
* The sensor measurement can be done on all the electrodes using
* the Cy_CapSense_RunSelfTest() function along with
* the CY_CAPSENSE_BIST_ELTD_CAP_MASK mask.
*
* This function must not be called while the CAPSENSE&trade; MW is busy
* by another scan.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
*
* \param widgetId
* Specifies the ID number of the widget.
* A macro for the widget ID can be found in the
* CAPSENSE&trade; Configuration header file (cycfg_capsense.h) defined as
* CY_CAPSENSE_<WidgetName>_WDGT_ID.
*
* \param eltdId
* Specifies the ID of the electrode within the widget (sensorID for CSD
* widgets and Rx or Tx electrode ID for CSX widgets).
*
* For the CSD widgets, a macro for the sensor ID within the specified widget
* can be found in the CAPSENSE&trade; Configuration header file (cycfg_capsense.h)
* defined as CY_CAPSENSE_<WidgetName>_SNS<SensorNumber>_ID.
*
* For the CSX widgets, eltdId is an electrode ID and is defined as Rx ID
* or Tx ID. The first Rx in a widget corresponds to eltdId = 0, the
* second Rx in a widget corresponds to eltdId = 1, and so on.
* The last Tx in a widget corresponds to eltdId = (RxNum + TxNum - 1).
* Macros for Rx and Tx IDs can be found in the CAPSENSE&trade; Configuration header
* file (cycfg_capsense.h) defined as:
* * CapSense_<WidgetName>_RX<RXNumber>_ID
* * CapSense_<WidgetName>_TX<TXNumber>_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation.
*                                         The measurement was not executed.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSensorElectrode_V3Lp(
                uint32_t widgetId,
                uint32_t eltdId,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;
    uint32_t numWdgtElectrodes = 0u;
    uint32_t snsFrameType = CY_CAPSENSE_SNS_FRAME_ACTIVE;

    if (NULL != context)
    {
        if ((context->ptrCommonConfig->numWd > widgetId))
        {
            /* Get the total widget electrode number: for CSX it is numRows + numCols, for CSD it is totalNumSns */
            switch (context->ptrWdConfig[widgetId].senseMethod)
            {
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                    case CY_CAPSENSE_CSD_GROUP:
                        result = CY_CAPSENSE_BIST_SUCCESS_E;
                        numWdgtElectrodes = context->ptrWdConfig[widgetId].numSns;
                        context->ptrBistContext->currentISC = context->ptrBistContext->intrEltdCapCsdISC;
                        break;
                #endif

                /* For CSX and ISX widgets, get the sum of Rx and Tx electrode numbers */
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    case CY_CAPSENSE_CSX_GROUP:
                        result = CY_CAPSENSE_BIST_SUCCESS_E;
                        numWdgtElectrodes = context->ptrWdConfig[widgetId].numRows +
                                             (uint32_t)context->ptrWdConfig[widgetId].numCols;
                        context->ptrBistContext->currentISC = context->ptrBistContext->intrEltdCapCsxISC;
                        break;
                #endif

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                    case CY_CAPSENSE_ISX_GROUP:
                        /* exit with CY_CAPSENSE_BIST_BAD_PARAM_E */
                #endif
                default:
                    /* Do nothing */
                    break;
            }

            if ((numWdgtElectrodes > eltdId) && (CY_CAPSENSE_BIST_BAD_PARAM_E != result))
            {
                if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
                {
                    /* Set the BUSY status and switch the HW configuration to BIST */
                    context->ptrCommonContext->status = CY_CAPSENSE_BUSY | CY_CAPSENSE_MW_STATE_BIST_MASK;
                    (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context);
                    /* Store the the pointer to the electrode pin configuration to the Bist context structure */
                    context->ptrBistContext->curPtrEltdCfg = &context->ptrWdConfig[widgetId].ptrEltdConfig[eltdId];
                    /* Switch the HW configuration to the sensor electrode capacitance measurement */
                    Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_ELTD_CAP_E,
                                                   CY_CAPSENSE_CSD_GROUP,
                                                   CY_CAPSENSE_BIST_CAP_ELTD_SCAN,
                                                   context);
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == context->ptrWdConfig[widgetId].wdType)
                        {
                            snsFrameType = CY_CAPSENSE_SNS_FRAME_LOW_POWER;
                        }
                    #endif
                    result = Cy_CapSense_BistMeasureCapacitanceSensor(
                                &context->ptrWdConfig[widgetId].ptrEltdCapacitance[eltdId],
                                snsFrameType,
                                context);
                    /* Clear the BUSY flag */
                    context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;
                }
                else
                {
                    result = CY_CAPSENSE_BIST_HW_BUSY_E;
                }
            }
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceAllSensors
****************************************************************************//**
*
* This internal function measures all the sensor capacitances
* (Cp for CSD sensors and Cm for CSX sensors).
*
* This function measures capacitances of all the sensors by using the
* Cy_CapSense_MeasureCapacitanceSlotSensors() function.
* The function stores sensor capacitance values in femtofarads
* in the snsCap array defined
* in the cycfg_capsense.c file. The pointer to the first element of the snsCap
* array that contains the widget sensor capacitances is stored
* in the .ptrSnsCapacitance field of the cy_stc_capsense_widget_config_t
* structure.
* The function is called by the Cy_CapSense_RunSelfTest() function.
*
* \note
* Rx/Lx electrodes for ISX widgets are excluded from measuring.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the measuring process:
* - CY_CAPSENSE_BIST_SUCCESS_E if all the measurements passed successfully.
* - CY_CAPSENSE_BIST_FAIL_E if any measurement was failed.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceAllSensors(
                cy_stc_capsense_context_t * context)
{
    uint32_t slotId;
    uint32_t widgetId;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_SUCCESS_E;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots = context->ptrScanSlots;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        const cy_stc_capsense_scan_slot_t * ptrLpScanSlots = context->ptrLpScanSlots;
    #endif

    /* Loop through all the slots */
    for (slotId = 0u; slotId < CY_CAPSENSE_SLOT_COUNT; slotId++)
    {
        widgetId = ptrScanSlots[slotId].wdId;
        if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
        {
            if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceSlotSensors_V3Lp(slotId, 0u, context))
            {
                if (CY_CAPSENSE_BIST_SUCCESS_E == result)
                {
                    result = CY_CAPSENSE_BIST_FAIL_E;
                }
            }
        }
    }

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        for (slotId = 0u; slotId < CY_CAPSENSE_SLOT_LP_COUNT; slotId++)
        {
            widgetId = ptrLpScanSlots[slotId].wdId;
            if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
            {
                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceLpSlotSensors(slotId, context))
                {
                    if (CY_CAPSENSE_BIST_SUCCESS_E == result)
                    {
                        result = CY_CAPSENSE_BIST_FAIL_E;
                    }
                }
            }
        }
    #endif

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceAllElectrodes
****************************************************************************//**
*
* This internal function measures all the electrodes (sensor for CSD widgets and
* Rx/Tx electrodes for CSX widgets) parasitic capacitance (Cp).
*
* This function measures Cp of all the electrodes by using the
* Cy_CapSense_MeasureCapacitanceSensorElectrode() function.
* The function stores the Cp values in femtofarads in the eltdCap array defined
* in the cycfg_capsense.c file. The pointer to the first element of the eltdCap
* array that contains the widget electrode capacitances is stored
* in the .ptrEltdCapacitance field of the cy_stc_capsense_widget_config_t
* structure.
* The function is called by the Cy_CapSense_RunSelfTest() function.
*
* \note
* Rx/Lx electrodes for ISX widgets are excluded from measuring.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the measuring process:
* - CY_CAPSENSE_BIST_SUCCESS_E if all the measurements passed successfully.
* - CY_CAPSENSE_BIST_FAIL_E if any measurement was failed.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceAllElectrodes(
                cy_stc_capsense_context_t * context)
{
    uint32_t widgetId;
    uint32_t electrodeId;
    uint32_t numWdgtElectrodes;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_SUCCESS_E;

    /* Loop through all the widgets */
    for (widgetId = 0u; widgetId < context->ptrCommonConfig->numWd; widgetId++)
    {
        if (CY_CAPSENSE_ISX_GROUP != context->ptrWdConfig[widgetId].senseMethod)
        {
            /* Get a total number of the widget elements: for CSX, it is numRows + numCols, for CSD, it is totalNumSns */
            if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[widgetId].senseMethod)
            {
                numWdgtElectrodes = context->ptrWdConfig[widgetId].numRows +
                                    (uint32_t)context->ptrWdConfig[widgetId].numCols;
            }
            else
            {
                numWdgtElectrodes = context->ptrWdConfig[widgetId].numSns;
            }
            /* Loop through all the sensor electrodes */
            for (electrodeId = 0u; electrodeId < numWdgtElectrodes; electrodeId++)
            {
                if (CY_CAPSENSE_BIST_SUCCESS_E != Cy_CapSense_MeasureCapacitanceSensorElectrode_V3Lp(widgetId, electrodeId, context))
                {
                    if (CY_CAPSENSE_BIST_SUCCESS_E == result)
                    {
                        result = CY_CAPSENSE_BIST_FAIL_E;
                    }
                }
            }
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceSlotSensors_V3Lp
****************************************************************************//**
*
* Measures the specified slot sensor capacitance in femtofarads. The function
* measures the Cp capacitance for CSD widgets and the Cm capacitance
* for CSX widgets.
*
* This function performs BIST slot scan with predefined parameters,
* back-calculates the slot sensor capacitances (Cp for CSD and Cm for CSX)
* by using the raw-count equation, stores the calculated capacitances
* to the sensor context structure, and returns the measurement status.
* If the specified slot has a ganged sensor, the capacitance
* is measured for all the pins ganged together that belong to this sensor.
* The function does not measure capacitances of Lx/Rx electrodes of the ISX widgets
* as they are electrically connected to the GND. 
* The CY_CAPSENSE_BIST_BAD_PARAM_E status for such a widget is returned.
*
* Besides the sensor capacitance measuring, this function could be
* used to identify various fault conditions with sensors such
* as electrically-opened or -shorted sensors. For example, the PCB track is
* broken or shorted to other nodes in the system - in all of these conditions,
* the function returns changed capacitance which can be compared
* against predetermined capacitance for the sensor to detect a
* fault condition.
*
* The sensor capacitance is measured independently of the sensor regular scan
* configuration. For the capacitance measurement, the BIST specific scan
* parameters are used. They can be found in the Electrode capacitance measurement
* macros group.
* The CDAC code for the CSD sensors is 100u and that provides about 0.887 pF
* of the CDAC value and for CSX sensors the CDAC code is 50u (0.443 pF).
* Compensation CDAC is disabled during the BIST scan.
* Another default scanning parameters are the following:
* * NumConv (100) is the number of sub-conversions.
* * SnsClk divider (512) is the divider for the sensor clock frequency.
*
* The raw count is converted into capacitance using the following equation:
*
*  Cs = Rawcount * CDAC / 2 / NumConv / 2
*
* where:
* * Cs is the sensor capacitance.
* * Rawcount is the measured raw count value.
* * The first divider of 2 is determined by the divided ref_clk frequency usage.
* * The second divider of 2 is used only for CSX sensors.
*
* The minimum measurable input by this function is 0.5 pF and the
* maximum is either 275pF or limited by the RC time constant
* (Cs < 1 / (4*5*SnsClk*R), where R is the total sensor series
* resistance that includes on-chip pin resistance ~500 Ohm and
* external series resistance). The measurement accuracy is about 30%.
*
* By default, all CAPSENSE&trade; sensors (electrodes) inherit regular
* scanning configuration. For example if the Inactive sensor
* connection parameter of the CSD sensing method is set to GND,
* sensors that are not being measured are set to the GND state.
* The inactive state can be changed in run-time by using
* the Cy_CapSense_SetInactiveElectrodeState() function.
*
* By default, the both Cmod1 and Cmod2 capacitors are used for the measurement.
*
* Measured capacitance values (Cp for CSD widgets and Cm for CSX widgets)
* are stored in the ptrSnsCapacitance sensor capacitance array field 
* and in the ptrEltdCapacitance electrode capacitance array
* of the \ref cy_stc_capsense_widget_config_t structure.
*
* The all sensor measurement can be done on all the sensors using
* the Cy_CapSense_RunSelfTest() function along with
* the CY_CAPSENSE_BIST_SNS_CAP_MASK mask.
*
* This function must not be called while the CAPSENSE&trade; MW is busy
* by another scan.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
*
* \param slotId
* Specifies the ID number of the slot to measure sensor capacitance.
* The slot ID should be in the admissible range.
*
* \param skipChMask
* This argument is kept for uniformity and backward compatibility
* and is not used. The function can be called with value NULL.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_ERROR_E            - An unexpected fault occurred during
*                                         the measurement.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSlotSensors_V3Lp(
                uint32_t slotId,
                uint32_t skipChMask,
                cy_stc_capsense_context_t * context)
{
    (void) skipChMask;
    return Cy_CapSense_MeasureCapacitanceSlotSensorsInternal(slotId, CY_CAPSENSE_SNS_FRAME_ACTIVE, context);
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceLpSlotSensors
****************************************************************************//**
*
* Measures the specified low power slot sensor capacitance in femtofarads.
* The function measures the Cp capacitance for CSD widgets and the Cm
* capacitance for CSX widgets.
*
* This function performs BIST slot scan with predefined parameters,
* back-calculates the slot sensor capacitances (Cp for CSD and Cm for CSX)
* by using the raw-count equation, stores the calculated capacitances
* to the sensor context structure, and returns the measurement status.
* If the specified slot has a ganged sensor, the capacitance
* is measured for all the pins ganged together that belong to this sensor.
*
* Besides the sensor capacitance measuring, this function could be
* used to identify various fault conditions with sensors such
* as electrically-opened or -shorted sensors. For example, the PCB track is
* broken or shorted to other nodes in the system - in all of these conditions,
* the function returns changed capacitance which can be compared
* against predetermined capacitance for the sensor to detect a
* fault condition.
*
* The sensor capacitance is measured independently of the sensor regular scan
* configuration. For the capacitance measurement, the BIST specific scan
* parameters are used. They can be found in the Electrode capacitance measurement
* macros group.
* The CDAC code for the CSD sensors is 100u and that provides about 0.887 pF
* of the CDAC value and for CSX sensors the CDAC code is 50u (0.443 pF).
* Compensation CDAC is disabled during the BIST scan.
* Another default scanning parameters are the following:
* * NumConv (100) is the number of sub-conversions.
* * SnsClk divider (256) is the divider for the sensor clock frequency.
*
* The raw count is converted into capacitance using the following equation:
*
*  Cs = Rawcount * CDAC / 2 / NumConv / 2
*
* where:
* * Cs is the sensor capacitance.
* * Rawcount is the measured raw count value.
* * The first divider of 2 is determined by the divided ref_clk frequency usage.
* * The second divider of 2 is used only for CSX sensors.
*
* The minimum measurable input by this function is 0.5 pF and the
* maximum is either 200pF or limited by the RC time constant
* (Cs < 1 / (2*10*SnsClk*R), where R is the total sensor series
* resistance that includes on-chip pin resistance ~500 Ohm and
* external series resistance). The measurement accuracy is about 30%.
*
* By default, all CAPSENSE&trade; sensors (electrodes) inherit regular
* scanning configuration. For example if the Inactive sensor
* connection parameter of the CSD sensing method is set to GND,
* sensors that are not being measured are set to the GND state.
* The inactive state can be changed in run-time by using
* the Cy_CapSense_SetInactiveElectrodeState() function.
*
* By default, the both Cmod1 and Cmod2 capacitors are used for the measurement.
*
* Measured capacitance values (Cp for CSD widgets and Cm for CSX widgets)
* are stored in the ptrSnsCapacitance sensor capacitance array field 
* and in the ptrEltdCapacitance electrode capacitance array
* of the \ref cy_stc_capsense_widget_config_t structure.
*
* The all sensor measurement can be done on all the sensors using
* the Cy_CapSense_RunSelfTest() function along with
* the CY_CAPSENSE_BIST_SNS_CAP_MASK mask.
*
* This function must not be called while the CAPSENSE&trade; MW is busy
* by another scan.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
*
* \param slotId
* Specifies the ID number of the slot to measure sensor capacitance.
* The slot ID should be in the admissible range.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_ERROR_E            - An unexpected fault occurred during
*                                         the measurement.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceLpSlotSensors(
                uint32_t slotId,
                cy_stc_capsense_context_t * context)
{
    return Cy_CapSense_MeasureCapacitanceSlotSensorsInternal(slotId, CY_CAPSENSE_SNS_FRAME_LOW_POWER, context);
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN))  */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceSlotSensorsInternal
****************************************************************************//**
*
* Internal function to measure the specified slot sensor capacitance in
* femtofarads. The function measures the Cp capacitance for CSD widgets and
* the Cm capacitance for CSX widgets.
*
* \param slotId
* Specifies the ID number of the slot to measure sensor capacitance.
* The slot ID should be in the admissible range.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_ERROR_E            - An unexpected fault occurred during
*                                         the measurement.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSlotSensorsInternal(
                uint32_t slotId,
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context)
{
    uint32_t wdIndex;
    uint32_t numSlots;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;

    if (NULL != context)
    {
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
            if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
            {
                ptrScanSlots = context->ptrScanSlots;
                numSlots = CY_CAPSENSE_SLOT_COUNT;
            }
            else
            {
                ptrScanSlots = context->ptrLpScanSlots;
                numSlots = CY_CAPSENSE_SLOT_LP_COUNT;
            }
        #else
            ptrScanSlots = context->ptrScanSlots;
            numSlots = CY_CAPSENSE_SLOT_COUNT;
            (void)snsFrameType;
        #endif
        if (numSlots > slotId)
        {
            if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
            {
                /* Set the BUSY status */
                context->ptrCommonContext->status = CY_CAPSENSE_BUSY | CY_CAPSENSE_MW_STATE_BIST_MASK;
                (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context);
                /* Set the slot measurement mode for the BIST scan */
                context->ptrBistContext->eltdCapScanMode = CY_CAPSENSE_BIST_CAP_SLOT_SCAN;
                /* Store the current slot for the BIST scan */
                context->ptrBistContext->curBistSlotId = (uint16_t)slotId;
                /* Initializes the active sensor structure for the current sensor */
                wdIndex = ptrScanSlots[slotId].wdId;
                /* Define the current ISC for BIST scan.
                    * The CSD or CSX group is the same for all slot sensors
                    */
                switch (context->ptrWdConfig[wdIndex].senseMethod)
                {
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                        case CY_CAPSENSE_CSD_GROUP:
                            result = CY_CAPSENSE_BIST_SUCCESS_E;
                            context->ptrBistContext->currentISC = context->ptrBistContext->intrEltdCapCsdISC;
                            break;
                    #endif

                    /* For CSX widgets get the sum of Rx and Tx electrode numbers */
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                        case CY_CAPSENSE_CSX_GROUP:
                            result = CY_CAPSENSE_BIST_SUCCESS_E;
                            context->ptrBistContext->currentISC = context->ptrBistContext->intrEltdCapCsxISC;
                            break;
                    #endif

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                        case CY_CAPSENSE_ISX_GROUP:
                            /* exit with CY_CAPSENSE_BIST_BAD_PARAM_E */
                    #endif
                    default:
                        /* Do nothing */
                        break;
                }

                if (CY_CAPSENSE_BIST_BAD_PARAM_E != result)
                {
                    /* Switch the HW resource configuration to the sensor element capacitance measurement */
                    Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_ELTD_CAP_E,
                                                    context->ptrWdConfig[wdIndex].senseMethod,
                                                    CY_CAPSENSE_BIST_CAP_SLOT_SCAN,
                                                    context);
                    result = Cy_CapSense_BistMeasureCapacitanceSlot(slotId, snsFrameType, context);
                }
                /* Clear the BUSY flag */
                context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;
            }
            else
            {
                result = CY_CAPSENSE_BIST_HW_BUSY_E;
            }
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureCapacitanceSensor
****************************************************************************//**
*
* This internal function measures a capacitance attached to AMUXBUS.
*
* The function measures Cp of a certain electrode (CSD sensor or CSX
* Rx/Tx electrode) by using CSD FW mode and defined scan configuration,
* sense clock frequency and resolution.
* The range for sensor measuring is 1 to 275 pF.
* The function performs the CSD scan with the fixed CDAC value. This provides
* a possibility of classical raw counts formula usage for the capacitance
* calculation. The function stores the Cp value
* in the corresponding element of the eltdCap[CY_CAPSENSE_ELTD_COUNT] array.
*
* \param cpPtr
* The pointer to the uint32_t to store measured value of the capacitance.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The measurement is completed
*                                successfully, the result is valid.
* - CY_CAPSENSE_BIST_TIMEOUT_E - The scan reached the timeout. It can be caused
*                                by a measured capacitance short or HW block
*                                failure or invalid configuration. You may need
*                                to repeat the measurement after the issue fix.
*                                The result is set to zero.
* - CY_CAPSENSE_BIST_ERROR_E   - The was an overflow or bad conversions during
*                                the scan. It can be caused
*                                by a measured capacitance short or HW block
*                                failure or invalid configuration. You may need
*                                to repeat the measurement after the issue fix.
*                                The result is set to zero.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceSensor(
                uint32_t * cpPtr,
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context)
{
    uint32_t cp;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_TIMEOUT_E;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    MSCLP_Type * ptrHwBase = ptrCommonCfg->ptrChConfig->ptrHwBase;
    uint32_t rawCountTmp;
    uint32_t watchdog;
    uint32_t cdacTrim;
    uint32_t sensorFrame[CY_MSCLP_6_SNS_REGS];

    /* Setup scan parameters: Ref CDAC code for Cp measurement */
    context->ptrBistContext->eltdCapRefCdac = CY_CAPSENSE_BIST_ELTD_CAP_REF_CDAC_DEFAULT;

    Cy_CapSense_BistGenerateSensorConfig(&sensorFrame[0u], snsFrameType, context);
    Cy_CapSense_StartCpuScan((const uint32_t *)sensorFrame, context);

    watchdog = Cy_CapSense_BistWatchdogPeriodCalc(context);
    watchdog = Cy_CapSense_WaitEndOfCpuScan(watchdog, context);

    /* Check if the watchdog timeout happened */
    if (0u == watchdog)
    {
        result = CY_CAPSENSE_BIST_TIMEOUT_E;
    }
    else
    {
        /* Read raw counts */
        rawCountTmp = ptrHwBase->SNS.RESULT_FIFO_RD;

        /* Check for overflow */
        if (MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk == (rawCountTmp & MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk))
        {
            result = CY_CAPSENSE_BIST_ERROR_E;
        }
        else
        {
            rawCountTmp &= MSCLP_SNS_RESULT_FIFO_RD_RAW_COUNT_Msk;
            result = CY_CAPSENSE_BIST_SUCCESS_E;
        }
    }

    /* Disable HW IP */
    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

    /* Check for timeout and if no, then calculate capacitance and store the value to the data structure */
    if (CY_CAPSENSE_BIST_SUCCESS_E == result)
    {
        cp = rawCountTmp * CY_CAPSENSE_REF_CDAC_LSB_X100;
        cp /= context->ptrBistContext->eltdCapSubConvNum;
        cp *= context->ptrBistContext->eltdCapRefCdac;
        cp /= CY_CAPSENSE_CONVERSION_HECTO;

        /* Adds CDAC trimming */
        if (0u != context->ptrCommonContext->cdacTrimCoefficient)
        {
            cdacTrim = context->ptrCommonContext->cdacTrimCoefficient;

            if (0u != cdacTrim)
            {
                /* Scales result with decreased resolution to avoid overflow */
                cp = (uint32_t)(cp << (CY_CAPSENSE_CDAC_TRIM_OFFSET - CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET));
                /* Rounding to the nearest */
                cp += cdacTrim >> (CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET + 1u);
                cp /= cdacTrim >> CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET;
            }
        }

        if (((uint32_t)CY_CAPSENSE_BIST_CP_MAX_VALUE) < cp)
        {
            cp = (uint32_t)CY_CAPSENSE_BIST_CP_MAX_VALUE;
        }
        *cpPtr = (uint32_t)cp;
    }
    else
    {
        *cpPtr = 0u;
    }

    return result;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN_EN) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || \
    ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureCapacitanceSlot
****************************************************************************//**
*
* This internal function measures slot sensor capacitances.
*
* The function measures Cp of a certain CSD sensor or shield
* or Cm of a certain CSX sensor by using the predefined scan configuration.
* The range for sensor measuring is 1 to 275 pF for CSD/CSX sensors and
* 1160 pF for shield electrodes.
* The function performs the CSD or CSX scan with fixed reference CDAC values
* (100u and 50u respectively) and 200u for shield electrodes' capacitance
* measurements. The function stores the Cp or Cm value
* in the corresponding element
* of the cy_capsense_snsCap[CY_CAPSENSE_SENSOR_COUNT] array and
* Cshield values for each channel
* in the corresponding element
* of the cy_capsense_shieldCap[CY_CAPSENSE_TOTAL_CH_NUMBER] array.
* The pointer to the first sensor capacitance value of a widget is
* in the .ptrSnsCapacitance field of the \ref cy_stc_capsense_widget_config_t
* structure.
*
* \param slotId
* Specifies the ID number of the slot to measure sensor capacitance.
* The slot ID should be in the admissible range.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The measurement completes
*                                successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E - The input parameter is invalid.
* - CY_CAPSENSE_BIST_TIMEOUT_E - The scan reached the timeout. It can be caused
*                                by a measured capacitance short or The CAPSENSE&trade; HW block
*                                failure or invalid configuration. You may need to
*                                repeat the measurement after the issue fix.
*                                The result is not affected.
* - CY_CAPSENSE_BIST_ERROR_E   - The was an overflow or bad conversions during
*                                the scan. It can be caused
*                                by a measured capacitance short or HW block
*                                failure or invalid configuration. You may need
*                                to repeat the measurement after the issue fix.
*                                The result is not affected.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceSlot(
                uint32_t slotId,
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context)
{

    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_TIMEOUT_E;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    MSCLP_Type * ptrHwBase = ptrCommonCfg->ptrChConfig->ptrHwBase;
    uint32_t cp;
    uint32_t watchdog;
    uint32_t rawCountTmp;
    uint32_t wdIndex;
    uint32_t snsIndex;
    uint32_t cdacTrim;
    uint32_t sensorFrame[CY_MSCLP_6_SNS_REGS];
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        ptrScanSlots = (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE) ? (context->ptrScanSlots) : (context->ptrLpScanSlots);
    #else
        ptrScanSlots = context->ptrScanSlots;
        (void)snsFrameType;
    #endif
    wdIndex = ptrScanSlots[slotId].wdId;
    snsIndex = ptrScanSlots[slotId].snsId;
    ptrWdCfg = &context->ptrWdConfig[wdIndex];

    /* Setup scan parameters: Ref CDAC code for shield or electrode Cp measurement */
    if (CY_CAPSENSE_BIST_CAP_SHIELD_SCAN == context->ptrBistContext->eltdCapScanMode)
    {
        context->ptrBistContext->eltdCapRefCdac = CY_CAPSENSE_BIST_SHIELD_CAP_REF_CDAC_DEFAULT;
    }
    /* The CSD or CSX group is the same for all slot sensors */
    else if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
    {
        context->ptrBistContext->eltdCapRefCdac = CY_CAPSENSE_BIST_MUTUAL_CAP_REF_CDAC_DEFAULT;
    }
    else
    {
        context->ptrBistContext->eltdCapRefCdac = CY_CAPSENSE_BIST_ELTD_CAP_REF_CDAC_DEFAULT;
    }

    Cy_CapSense_BistGenerateSensorConfig(&sensorFrame[0u], snsFrameType, context);
    Cy_CapSense_StartCpuScan((const uint32_t *)sensorFrame, context);

    watchdog = Cy_CapSense_BistWatchdogPeriodCalc(context);
    watchdog = Cy_CapSense_WaitEndOfCpuScan(watchdog, context);

    /* Check if the watchdog timeout happened */
    if (0u == watchdog)
    {
        result = CY_CAPSENSE_BIST_TIMEOUT_E;
    }
    else
    {
        /* Read raw counts */
        rawCountTmp = ptrHwBase->SNS.RESULT_FIFO_RD;

        /* Check for overflow */
        if (MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk == (rawCountTmp & MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk))
        {
            result = CY_CAPSENSE_BIST_ERROR_E;
        }
        else
        {
            rawCountTmp &= MSCLP_SNS_RESULT_FIFO_RD_RAW_COUNT_Msk;
            result = CY_CAPSENSE_BIST_SUCCESS_E;
        }
    }

    /* Disable HW IP */
    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

    /* Calculate capacitance and store the value to the data structure */
    if (CY_CAPSENSE_BIST_SUCCESS_E == result)
    {
        cp = CY_CAPSENSE_REF_CDAC_LSB_X100 * rawCountTmp;
        cp /= context->ptrBistContext->eltdCapSubConvNum;
        cp *= context->ptrBistContext->eltdCapRefCdac;
        cp /= CY_CAPSENSE_CONVERSION_HECTO;

        /* Adds CDAC trimming */
        if (0u != context->ptrCommonContext->cdacTrimCoefficient)
        {
            cdacTrim = context->ptrCommonContext->cdacTrimCoefficient;

            if (0u != cdacTrim)
            {
                /* Scales result with decreased resolution to avoid overflow */
                cp = (uint32_t)(cp << (CY_CAPSENSE_CDAC_TRIM_OFFSET - CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET));
                /* Rounding to the nearest */
                cp += cdacTrim >> (CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET + 1u);
                cp /= cdacTrim >> CY_CAPSENSE_CDAC_TRIM_OVERFLOW_OFFSET;
            }
        }

        /* Store the Cp value in the appropriate structure */
        if ((CY_CAPSENSE_BIST_CAP_SHIELD_SCAN != context->ptrBistContext->eltdCapScanMode))
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                /* Divide the result by 2 for the FW CSX scanning method */
                if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                {
                    cp /= CY_CAPSENSE_DIVIDER_TWO;
                }
            #endif

            if (((uint32_t)CY_CAPSENSE_BIST_CP_MAX_VALUE) < cp)
            {
                cp = (uint32_t)CY_CAPSENSE_BIST_CP_MAX_VALUE;
            }
            ptrWdCfg->ptrSnsCapacitance[snsIndex] = (uint32_t)cp;
        }
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
            else
            {
                if (((uint32_t)CY_CAPSENSE_BIST_CSH_MAX_VALUE) < cp)
                {
                    cp = (uint32_t)CY_CAPSENSE_BIST_CSH_MAX_VALUE;
                }
                *context->ptrBistContext->ptrChShieldCap = (uint32_t)cp;
            }
        #endif
    }

    return result;
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || \
          ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && \
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_BistGenerateBaseConfig
****************************************************************************//**
*
* Changes the configuration for all registers that have to be changed
* one-time to initialize the MSC block for BIST scan.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
static void Cy_CapSense_BistGenerateBaseConfig(
                cy_stc_capsense_context_t * context)
{
    cy_stc_msclp_base_config_t cy_capsense_smTemplate = CY_CAPSENSE_SENSING_METHOD_BASE_TEMPLATE;
    const cy_stc_msclp_mode_config_t modeArr[CY_CAPSENSE_REG_MODE_NUMBER] = CY_CAPSENSE_SM_MODES_ARR;
    cy_stc_msclp_base_config_t * ptrTemplateCfg = &cy_capsense_smTemplate;
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;

    /* Check for modClk divider value */
    if (1u == context->ptrBistContext->eltdCapModClk)
    {
        ptrTemplateCfg->ctl |= MSCLP_CTL_CLK_MSC_RATIO_Msk;
    }
    else
    {
        ptrTemplateCfg->ctl &= ~MSCLP_CTL_CLK_MSC_RATIO_Msk;
    }
    ptrTemplateCfg->imoCtl |= _VAL2FLD(MSCLP_IMO_CTL_CLOCK_MSC_DIV, (uint32_t)context->ptrBistContext->eltdCapModClk - 1u);

    /* Generating the common configuration for the BIST epilogue cycle number */
    if (0u < context->ptrBistContext->eltdCapNumEpiCycles)
    {
        ptrTemplateCfg->scanCtl2 |= (uint32_t)context->ptrBistContext->eltdCapNumEpiCycles << MSCLP_SCAN_CTL2_NUM_EPI_KREF_DELAY_Pos;
    }
    else
    {
        ptrTemplateCfg->scanCtl2 |= (uint32_t)(1uL << MSCLP_SCAN_CTL2_NUM_EPI_KREF_DELAY_Pos);
    }

    /* Generating the common configuration for the coarse initialization and coarse short phase */
    ptrTemplateCfg->initCtl1 |= (uint32_t)context->ptrBistContext->eltdCapNumCoarseInitChargeCycles <<
                                                                MSCLP_INIT_CTL1_NUM_INIT_CMOD_12_RAIL_CYCLES_Pos;
    ptrTemplateCfg->initCtl1 |= (uint32_t)context->ptrBistContext->eltdCapNumCoarseInitSettleCycles <<
                                                                MSCLP_INIT_CTL1_NUM_INIT_CMOD_12_SHORT_CYCLES_Pos;
    ptrTemplateCfg->initCtl3 |= CY_CAPSENSE_CMOD12_PAIR_SELECTION << MSCLP_INIT_CTL3_CMOD_SEL_Pos;
    ptrTemplateCfg->initCtl3 |= (uint32_t)context->ptrInternalContext->numProOffsetCycles << MSCLP_INIT_CTL3_NUM_PRO_OFFSET_CYCLES_Pos;

    /*
    * Generating the common configuration for the number of sub-conversions to be run during the PRO_DUMMY phase and
    * the modClk period number during the PRO_WAIT phase.
    */
    ptrTemplateCfg->initCtl4 |= (((uint32_t)context->ptrBistContext->eltdCapNumFineInitCycles <<
                                                                  MSCLP_INIT_CTL4_NUM_PRO_DUMMY_SUB_CONVS_Pos) |
                                 ((uint32_t)context->ptrBistContext->eltdCapNumFineInitWaitCycles <<
                                                                  MSCLP_INIT_CTL4_NUM_PRO_WAIT_KREF_DELAY_Pos));

    /* Set switch control functions */
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_GND]     = CY_CAPSENSE_SM_REG_SW_SEL_CSW0_GND_VALUE;
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z]  = CY_CAPSENSE_SM_REG_SW_SEL_CSW1_HIGH_Z_VALUE;
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_TX]      = CY_CAPSENSE_SM_REG_SW_SEL_CSW3_CSX_TX_VALUE;
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_RX]      = CY_CAPSENSE_SM_REG_SW_SEL_CSW2_CSX_RX_VALUE;
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS]     = CY_CAPSENSE_SM_REG_SW_SEL_CSW5_CSD_SNS_VALUE;
    ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2]   = CY_CAPSENSE_SM_REG_SW_SEL_CSW10_VDDA2_VALUE;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        if (CY_CAPSENSE_SHIELD_ACTIVE == context->ptrCommonConfig->csdShieldMode)
        {
            /* Active Shield mode */
            ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD] = CY_CAPSENSE_SM_REG_SW_SEL_CSW8_ACT_SHLD_VALUE;
        }
        else
        {
            /* Passive Shield mode */
            ptrTemplateCfg->swSelCswFunc[CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD] = CY_CAPSENSE_SM_REG_SW_SEL_CSW9_PAS_SHLD_VALUE;
        }
    #endif

    /* Set mode configuration */
    ptrTemplateCfg->mode[CY_CAPSENSE_REG_MODE_CSD] = modeArr[CY_CAPSENSE_REG_MODE_CSD];
    ptrTemplateCfg->mode[CY_CAPSENSE_REG_MODE_CSX] = modeArr[CY_CAPSENSE_REG_MODE_CSX];
    ptrTemplateCfg->mode[CY_CAPSENSE_REG_MODE_ISX] = modeArr[CY_CAPSENSE_REG_MODE_ISX];
    if (CY_CAPSENSE_BIST_IO_VDDA2_E == context->ptrBistContext->currentISC)
    {
        if ((CY_CAPSENSE_BIST_CAP_ELTD_SCAN == context->ptrBistContext->eltdCapScanMode) ||
            (CY_CAPSENSE_CSD_GROUP == context->ptrBistContext->eltdCapSenseGroup))
        {
            /* Close the reference to filter switch for CSD sensing mode */
            ptrTemplateCfg->mode[CY_CAPSENSE_REG_MODE_CSD].swSelSh |= MSCLP_MODE_SW_SEL_SH_SOMB_Msk;
        }
        else
        {
            /* Close the reference to filter switch for CSD sensing mode */
            ptrTemplateCfg->mode[CY_CAPSENSE_REG_MODE_CSX].swSelSh |= MSCLP_MODE_SW_SEL_SH_SOMB_Msk;
        }
    }

    (void)Cy_MSCLP_Configure(ptrHwBase,
                       ptrTemplateCfg,
                       CY_MSCLP_CAPSENSE_KEY,
                       context->ptrCommonConfig->ptrChConfig->ptrHwContext);

    Cy_CapSense_SetupCpuOperatingMode(context);
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
           ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */


 #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_BistGenerateSensorConfig
****************************************************************************//**
*
* Generates the configuration for registers that have to be configured to start
* a scan for a single sensor. It is used the next sensor configuration for BIST
* scan:
* - CIC2 filter is off
* - is used mode structure 0u
* - Multi-channel mode is off
* - System level chopping is off
* - Coarse Init Bypass is off
*
* \param ptrSensorCfg
* Specifies the pointer to the sensor configuration to be filled.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
static void Cy_CapSense_BistGenerateSensorConfig(
                uint32_t * ptrSensorCfg,
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context)
{
    uint32_t modeSel;
    uint32_t snsClkVal;
    uint32_t *ptrSensorCfgLocal = ptrSensorCfg;
    uint32_t i;
    uint32_t padMask = 0u;
    uint32_t cswFuncNum = 0u;
    uint32_t slotStcWdgtIdx;
    uint32_t slotStcSnsIdx;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    const cy_stc_capsense_electrode_config_t * ptrEltdCfg;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
        uint32_t padMaskIsx = 0u;
        uint32_t numEltd;
        uint32_t eltdIndex;
        uint32_t wdIndex;
        uint32_t snsMethod;
        cy_stc_capsense_electrode_config_t const * eltdPinCfg;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        ptrScanSlots = (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE) ? (context->ptrScanSlots) : (context->ptrLpScanSlots);
    #else
        ptrScanSlots = context->ptrScanSlots;
        (void)snsFrameType;
    #endif
    slotStcWdgtIdx = ptrScanSlots[context->ptrBistContext->curBistSlotId].wdId;
    slotStcSnsIdx =  ptrScanSlots[context->ptrBistContext->curBistSlotId].snsId;
    ptrWdCfg = &context->ptrWdConfig[slotStcWdgtIdx];

    /* Clear the sensor configuration structure */
    for (i = 0u; i < CY_MSCLP_6_SNS_REGS; i++)
    {
        ptrSensorCfg[i] = 0u;
    }

    /* CapDAC configuration */
    ptrSensorCfgLocal[CY_CAPSENSE_SNS_CDAC_CTL_INDEX] =
            (_VAL2FLD(MSCLP_SNS_SNS_CDAC_CTL_SEL_RE, context->ptrBistContext->eltdCapRefCdac) |
             (CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_CLOCK_REF_RATE << MSCLP_SNS_SNS_CDAC_CTL_CLOCK_REF_RATE_Pos));

    if ((CY_CAPSENSE_BIST_CAP_ELTD_SCAN == context->ptrBistContext->eltdCapScanMode) ||
        (CY_CAPSENSE_BIST_CAP_SHIELD_SCAN == context->ptrBistContext->eltdCapScanMode))
    {
        modeSel = CY_CAPSENSE_REG_MODE_CSD;
    }
    else
    {
        switch (ptrWdCfg->senseMethod)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                case CY_CAPSENSE_CSD_GROUP:
                    modeSel = CY_CAPSENSE_REG_MODE_CSD;
                    break;
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                case CY_CAPSENSE_CSX_GROUP:
                    modeSel = CY_CAPSENSE_REG_MODE_CSX;
                    break;
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                case CY_CAPSENSE_ISX_GROUP:
                    modeSel = CY_CAPSENSE_REG_MODE_ISX;
                    break;
            #endif

            default:
                modeSel = CY_CAPSENSE_REG_MODE_CSD;
                break;
        }
    }

    snsClkVal = context->ptrBistContext->eltdCapSnsClk;
    if (CY_CAPSENSE_BIST_CAP_SHIELD_SCAN == context->ptrBistContext->eltdCapScanMode)
    {
        snsClkVal = CY_CAPSENSE_BIST_SHIELD_CAP_SNSCLK_DIV_DEFAULT;
    }
    snsClkVal--;

    ptrSensorCfgLocal[CY_CAPSENSE_SNS_SCAN_CTL_INDEX] = _VAL2FLD(MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS, (uint32_t)context->ptrBistContext->eltdCapSubConvNum - 1u) |
                                                        _VAL2FLD(MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV, (uint32_t)snsClkVal); /* Compensation CDAC Divider set to BIST Sns clock divider */
    ptrSensorCfgLocal[CY_CAPSENSE_SNS_CTL_INDEX] = _VAL2FLD(MSCLP_SNS_SNS_CTL_SENSE_MODE_SEL, modeSel) |
                                                   _VAL2FLD(MSCLP_SNS_SNS_CTL_SENSE_DIV, snsClkVal);

    /* Create mask for all project electrodes */
    for (i = 0u; i < context->ptrCommonConfig->numPin; i++)
    {
        padMask |= (0x01uL << ptrPinCfg->padNumber);
        ptrPinCfg++;
    }

    /* Add to mask all shield electrodes (if present) */
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        ptrPinCfg = context->ptrShieldPinConfig;
        for (i = 0u; i < context->ptrCommonConfig->csdShieldNumPin; i++)
        {
            padMask |= (0x01uL << ptrPinCfg->padNumber);
            ptrPinCfg++;
        }
    #endif /*  (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) */

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
        /* Create separate inactive mask of ISX pins only */
        for (wdIndex = 0u; wdIndex < context->ptrCommonConfig->numWd; wdIndex++)
        {
            snsMethod = context->ptrWdConfig[wdIndex].senseMethod;
            if (CY_CAPSENSE_ISX_GROUP == snsMethod)
            {
                numEltd = (uint32_t)context->ptrWdConfig[wdIndex].numRows + context->ptrWdConfig[wdIndex].numCols;
                eltdPinCfg = ptrWdCfg->ptrEltdConfig;

                for (eltdIndex = 0u; eltdIndex < numEltd; eltdIndex++)
                {
                    /* Loop through all pads for this electrode (ganged sensor) */
                    for (i = 0u; i < eltdPinCfg->numPins; i++)
                    {
                        padMaskIsx |= (0x01uL << eltdPinCfg->ptrPin[i].padNumber);
                    }
                    eltdPinCfg++;
                }
            }
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */

    /* Change the control switch function depending on the current inactive sensor connection */
    switch (context->ptrBistContext->currentISC)
    {
        case CY_CAPSENSE_BIST_IO_HIGHZA_E:
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z;
            break;

        case CY_CAPSENSE_BIST_IO_VDDA2_E:
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2;
            break;

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
            case CY_CAPSENSE_BIST_IO_SHIELD_E:
                cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD;
                break;
        #endif

        default:
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_GND;
            break;
    }

    /* Set the control mux switch registers for inactive pins */
    Cy_CapSense_CalculateMaskRegisters(padMask, cswFuncNum, ptrSensorCfg);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
        Cy_CapSense_CalculateMaskRegisters(padMaskIsx, cswFuncNum, ptrSensorCfg);
    #endif

    /* Initializes an active sensor (including ganged sensors) by SNS sensor state */
    if (CY_CAPSENSE_BIST_CAP_SHIELD_SCAN == context->ptrBistContext->eltdCapScanMode)
    {
        /* Generate CTRL_MUX masks for the electrode or CSD sensor connection */
        cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS;
        if (CY_CAPSENSE_BIST_IO_SHIELD_E != context->ptrBistContext->currentISC)
        {
            padMask = 0u;
            for (i = 0u; i < context->ptrCommonConfig->csdShieldNumPin; i++)
            {
                padMask |= 0x01uL << context->ptrShieldPinConfig[i].padNumber;
            }
        }
        Cy_CapSense_CalculateMaskRegisters(padMask, cswFuncNum, ptrSensorCfg);
    }
    else
    {
        if ((CY_CAPSENSE_BIST_CAP_ELTD_SCAN == context->ptrBistContext->eltdCapScanMode) ||
            (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod))
        {
            /* Generate CTRL_MUX masks for the electrode or CSD sensor connection */
            if (CY_CAPSENSE_BIST_CAP_ELTD_SCAN == context->ptrBistContext->eltdCapScanMode)
            {
                ptrEltdCfg = context->ptrBistContext->curPtrEltdCfg;
            }
            else
            {
                ptrEltdCfg = &ptrWdCfg->ptrEltdConfig[slotStcSnsIdx];
            }
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS;
            Cy_CapSense_BistGenerateSnsCfgMaskReg(ptrEltdCfg, ptrSensorCfg, cswFuncNum);
        }
        else
        {
            /* RX ELECTRODE */
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_RX;
            ptrEltdCfg = &ptrWdCfg->ptrEltdConfig[(slotStcSnsIdx / ptrWdCfg->numRows)];
            Cy_CapSense_BistGenerateSnsCfgMaskReg(ptrEltdCfg, ptrSensorCfg, cswFuncNum);

            /* TX ELECTRODE */
            cswFuncNum = CY_CAPSENSE_CTRLMUX_PIN_STATE_TX;
            ptrEltdCfg =  &ptrWdCfg->ptrEltdConfig[ptrWdCfg->numCols + (slotStcSnsIdx % ptrWdCfg->numRows)];
            Cy_CapSense_BistGenerateSnsCfgMaskReg(ptrEltdCfg, ptrSensorCfg, cswFuncNum);
        }
    }
}
#endif /*((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || \
         ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))*/


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_BistGenerateSnsCfgMaskReg
****************************************************************************//**
*
* Calculates the configuration for all sensor configuration mask registers that
* have to be changed to set the desired CTRL-MUX function
* for the specified electrode for BIST scan.
*
* \param ptrEltdCfg
* The pointer to the specified electrode configuration
* \ref cy_stc_capsense_electrode_config_t structure.
*
* \param ptrSensorCfg
* Specifies the pointer to the sensor configuration to be filled.
*
* \param cswFuncNum
* The desired CTRL-MUX Switch Control Global function.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
static void Cy_CapSense_BistGenerateSnsCfgMaskReg(
                const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                uint32_t * ptrSensorCfg,
                uint32_t cswFuncNum)
{
    uint32_t padMask = 0u;
    uint32_t i;

    /* Loop through all pads for this electrode (ganged sensor) */
    for (i = 0u; i < ptrEltdCfg->numPins; i++)
    {
        padMask |= 0x01uL << ptrEltdCfg->ptrPin[i].padNumber;
    }
    Cy_CapSense_CalculateMaskRegisters(padMask, cswFuncNum, ptrSensorCfg);
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
          ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
        (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceShieldElectrode_V3Lp
****************************************************************************//**
*
* Measures shield electrode capacitances in femtofarads.
*
* This function measures the capacitances of all shield electrodes for all
* enabled MSCv3 channels and returns a status of this measurement.
* The function checks if there is any CSD widget in the project and
* if the shield is enabled.
* The measurement results in femtofarads are stored
* in the chShieldCap[CY_MSC_ENABLED_CH_NUMBER] array.
* The pointer to the array is in the .ptrChShieldCap field
* of the \ref cy_stc_capsense_bist_context_t structure,
* the CY_MSC_ENABLED_CH_NUMBER define is in the cycfg_peripherals.h file.
* If the any channel shield consists of several electrodes, the total
* capacitance of all the shield electrodes is measured.
*
* This function uses an algorithm identical to the electrode capacitance
* measurement. Refer to the Cy_CapSense_MeasureCapacitanceSensorElectrode()
* function for more details.
*
* In addition to measuring shield capacitance, this function is used to
* identify various fault conditions with shield electrodes such as an
* electrically-open or -short shield electrodes, e.g. the PCB track is broken or
* shorted to other nodes in the system - in all of these conditions,
* this function returns changed capacitance that can be compared
* against pre-determined capacitance for the shield electrode to
* detect a hardware fault.
*
* By default, all CAPSENSE&trade; sensors (electrodes) inherit regular
* scanning configuration. For example if the Inactive sensor
* connection parameter of the CSD sensing method is set to GND,
* sensors that are not being measured are set to the GND state.
* The inactive state can be changed in run-time by using
* the Cy_CapSense_SetInactiveElectrodeState() function.
* When the inactive sensor (electrode) connection is set
* to the CY_CAPSENSE_SNS_CONNECTION_SHIELD state,
* all the CAPSENSE&trade; electrodes are connected to the shield and
* the total capacitance are measured.
*
* By default, the both Cmod1 and Cmod2 capacitors are used for the measurement.
*
* This test can be executed using the CapSense_RunSelfTest()
* function with the CY_CAPSENSE_BIST_SHIELD_CAP_MASK mask.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
* ISX widgets are excluded from measuring.
*
* \param skipChMask
* Specifies the mask to skip some channels during the shield electrode
* capacitance measurement. If the bit N in the skipChMask is set to 1,
* the channel N will be excluded from measuring and all its shield pins will be
* set to the shield inactive sensor connection state (see the .shieldCapISC
* field of the \ref cy_stc_capsense_bist_context_t structure).
*
* \param context
* The pointer to the CAPSENSE&trade; context
* structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with a
*                                         previous operation.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_BAD_CONFIG_E       - The shield is disabled.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceShieldElectrode_V3Lp(
                uint32_t skipChMask,
                cy_stc_capsense_context_t * context)
{
    (void)skipChMask;
    cy_en_capsense_bist_status_t result = CY_CAPSENSE_BIST_BAD_PARAM_E;

    if (NULL != context)
    {
    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) &&\
         (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_SHIELD_EN))
        if ((0u < context->ptrCommonConfig->csdShieldNumPin) ||
            (CY_CAPSENSE_BIST_IO_SHIELD_E == context->ptrBistContext->intrEltdCapShieldISC))
        {
            context->ptrBistContext->currentISC = context->ptrBistContext->intrEltdCapShieldISC;
            if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
            {
                /* Set the BUSY status */
                context->ptrCommonContext->status = CY_CAPSENSE_BUSY | CY_CAPSENSE_MW_STATE_BIST_MASK;
                (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context);

                /* Store the current slot and the skip channel mask for the BIST scan */
                context->ptrBistContext->curBistSlotId = 0u;
                /* Switch the HW resource configuration to the electrode capacitance measurement */
                Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_ELTD_CAP_E,
                                               CY_CAPSENSE_CSD_GROUP,
                                               CY_CAPSENSE_BIST_CAP_SHIELD_SCAN,
                                               context);
                /* Perform the CSD BIST slot scan with connected all channel shields as active electrodes */
                result = Cy_CapSense_BistMeasureCapacitanceSlot(0u, CY_CAPSENSE_SNS_FRAME_ACTIVE, context);
                /* Clear the BUSY flag */
                context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;
            }
            else
            {
                result = CY_CAPSENSE_BIST_HW_BUSY_E;
            }
        }
    #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) &&\
               (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)) */
    }

    return result;
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureCapacitanceCapRun
****************************************************************************//**
*
* This internal function performs the specific scan for the external capacitor
* capacitance measurement.
*
* \param ptrExtCapValue
* The pointer to the result of the measurement as raw counts.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The measurement completes
*                                successfully, the result is valid.
* - CY_CAPSENSE_BIST_ERROR_E   - The raw counts reach overflow during the
*                                measuring scan. The measurement result is
*                                invalid.
* - CY_CAPSENSE_BIST_TIMEOUT_E - The measuring scan is not finished properly
*                                and reached the timeout.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceCapRun(
                uint32_t * ptrExtCapValue,
                cy_stc_capsense_context_t * context)
{
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    uint32_t tempRawcount;
    cy_en_capsense_bist_status_t bistStatus = CY_CAPSENSE_BIST_TIMEOUT_E;
    uint32_t watchdog;
    uint32_t scanConfigTmp[CY_MSCLP_6_SNS_REGS];

    /*
     * The measurement will be performed with all the sensors disconnected.
     * The code below sets up the configuration of the LO_MASK registers with 0x00uL.
     */
    scanConfigTmp[CY_CAPSENSE_SNS_SW_SEL_CSW_LO_MASK2_INDEX] = 0x00uL;
    scanConfigTmp[CY_CAPSENSE_SNS_SW_SEL_CSW_LO_MASK1_INDEX] = 0x00uL;
    scanConfigTmp[CY_CAPSENSE_SNS_SW_SEL_CSW_LO_MASK0_INDEX] = 0x00uL;

    scanConfigTmp[CY_CAPSENSE_SNS_SCAN_CTL_INDEX] =
            ((((uint32_t)context->ptrBistContext->extCapSubConvNum - 1u) << MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos) |
             MSCLP_SNS_SNS_SCAN_CTL_INIT_BYPASS_Msk);
    scanConfigTmp[CY_CAPSENSE_SNS_CDAC_CTL_INDEX]  =
            ((CY_CAPSENSE_BIST_EXT_CAP_REF_CDAC_DEFAULT << MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos)                         |
             (CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_CLOCK_REF_RATE << MSCLP_SNS_SNS_CDAC_CTL_CLOCK_REF_RATE_Pos));
    scanConfigTmp[CY_CAPSENSE_SNS_CTL_INDEX] = MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk;

    /* Initiate the scan in the CPU operating mode. */
    Cy_CapSense_StartCpuScan((const uint32_t *)scanConfigTmp, context);

    /* Calculate a watchdog time in us */
    watchdog = (((CY_CAPSENSE_BIST_EXT_CAP_WDT_FACTOR *
                  (MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk >> MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos)) *
                   (uint32_t)(context->ptrBistContext->extCapSubConvNum)) / CY_CAPSENSE_BIST_EXT_CAP_IMO_CLK_MHZ);

    watchdog = Cy_CapSense_WaitEndOfCpuScan(watchdog, context);

    if (0u != watchdog)
    {
        tempRawcount = ptrHwBase->SNS.RESULT_FIFO_RD;
        /* Check for an overflow or bad conversions */
        if (0u == (tempRawcount & MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk))
        {
            tempRawcount &= MSCLP_SNS_RESULT_FIFO_RD_RAW_COUNT_Msk;
            bistStatus = CY_CAPSENSE_BIST_SUCCESS_E;
        }
        else
        {
            bistStatus = CY_CAPSENSE_BIST_ERROR_E;
        }
    }

    /* Disable HW IP */
    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

    /* Check for timeout and if no, then calculate capacitance and store the value to the data structure */
    if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
    {
        /* Store raw counts */
        *ptrExtCapValue = tempRawcount;
    }
    else
    {
        *ptrExtCapValue = 0u;
    }

    return bistStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistGenerateBaseCmodConfig
****************************************************************************//**
*
* Changes the configuration for all registers that have to be changed
* one-time to initialize the MSC block for BIST scan.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistGenerateBaseCmodConfig(cy_stc_capsense_context_t * context)
{
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    uint32_t i;

   ptrHwBase->CTL =
            ((CY_CAPSENSE_SM_REG_CTL_FLD_SENSE_EN                               << MSCLP_CTL_SENSE_EN_Pos)              |
             (CY_CAPSENSE_SM_REG_CTL_FLD_MSCCMP_EN                              << MSCLP_CTL_MSCCMP_EN_Pos)             |
             (CY_CAPSENSE_SM_REG_CTL_FLD_OPERATING_MODE                         << MSCLP_CTL_OPERATING_MODE_Pos)        |
             (CY_CAPSENSE_SM_REG_CTL_FLD_BUF_MODE                               << MSCLP_CTL_BUF_MODE_Pos)              |
             (CY_CAPSENSE_SM_REG_CTL_FLD_CLK_MSC_RATIO                          << MSCLP_CTL_CLK_MSC_RATIO_Pos)         |
             (CY_CAPSENSE_SM_REG_CTL_FLD_ENABLED                                << MSCLP_CTL_ENABLED_Pos));
    ptrHwBase->SPARE =                  0x00uL;
    ptrHwBase->SCAN_CTL1 = (CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_RC_STORE_EN        << MSCLP_SCAN_CTL1_RC_STORE_EN_Pos);
    ptrHwBase->SCAN_CTL2 =
             ((CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_CHOP_POL               << MSCLP_SCAN_CTL2_CHOP_POL_Pos)           |
              (CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY     << MSCLP_SCAN_CTL2_NUM_EPI_KREF_DELAY_Pos) |
              (CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY_PRS << MSCLP_SCAN_CTL2_NUM_EPI_KREF_DELAY_PRS_Pos));
    ptrHwBase->INIT_CTL1 =              0x00uL;
    ptrHwBase->INIT_CTL2 =              0x00uL;
    ptrHwBase->INIT_CTL3 =              0x00uL;
    ptrHwBase->INIT_CTL4 =              0x00uL;
    ptrHwBase->SENSE_DUTY_CTL =         0x00uL;
    ptrHwBase->SENSE_PERIOD_CTL =       0x00uL;
    ptrHwBase->FILTER_CTL =             0x00uL;
    ptrHwBase->CCOMP_CDAC_CTL =         0x00uL;
    ptrHwBase->DITHER_CDAC_CTL =        0x00uL;
    ptrHwBase->MSCCMP_CTL =
            ((CY_CAPSENSE_SM_REG_MSCCMP_CTL_FLD_PWR                             << MSCLP_MSCCMP_CTL_PWR_Pos)            |
             (CY_CAPSENSE_SM_REG_MSCCMP_CTL_FLD_FILT                            << MSCLP_MSCCMP_CTL_FILT_Pos));
    ptrHwBase->OBS_CTL =                0x00uL;
    ptrHwBase->AOS_CTL =                0x00uL;
    ptrHwBase->CE_CTL =                 0x00uL;

    /* Check for the system VDDA value and enable PUMP if VDDA is less than the threshold */
    #if (CY_MSCLP_VDDA_PUMP_TRESHOLD > CY_CAPSENSE_VDDA_MV)
        ptrHwBase->PUMP_CTL = MSCLP_PUMP_CTL_PUMP_MODE_Msk;
    #endif

    /* Use 46 MHz IMO frequency with 4 as divider for Cmod measurements */
    ptrHwBase->IMO_CTL =
            ((CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_IMO_CTL_FREQ          << MSCLP_IMO_CTL_FREQ_Pos)                    |
             (CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_IMO_CTL_CLOCK_MSC_DIV << MSCLP_IMO_CTL_CLOCK_MSC_DIV_Pos));

    ptrHwBase->WAKEUP_CMD = 0x00uL;
    ptrHwBase->SW_SEL_GPIO = 0x00uL;
    ptrHwBase->SW_SEL_CDAC_RE =
            ((CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETCC << MSCLP_SW_SEL_CDAC_RE_SW_RETCC_Pos)                |
             (CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RECD  << MSCLP_SW_SEL_CDAC_RE_SW_RECD_Pos)                 |
             (CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETV  << MSCLP_SW_SEL_CDAC_RE_SW_RETV_Pos)                 |
             (CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETG  << MSCLP_SW_SEL_CDAC_RE_SW_RETG_Pos)                 |
             (CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_REBV  << MSCLP_SW_SEL_CDAC_RE_SW_REBV_Pos)                 |
             (CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_REBG  << MSCLP_SW_SEL_CDAC_RE_SW_REBG_Pos));
    ptrHwBase->SW_SEL_CDAC_CO =         0x00uL;
    ptrHwBase->SW_SEL_CDAC_CF =         0x00uL;
    ptrHwBase->SW_SEL_BGR =             0x00uL;

    /* Clear SW_SEL_CSW registers as not needed for BIST Cmod measurement */
    for (i = 0u; i < CY_MSCLP_SENSE_PAD_NUMBER; i++)
    {
        ptrHwBase->SW_SEL_CSW[i] =      0x00uL;
    }

    /* Clear SW_SEL_CSW_FUNC registers as not needed for BIST Cmod measurement */
    for (i = 0u; i < MSCLP_CSW_GLOBAL_FUNC_NR; i++)
    {
        ptrHwBase->SW_SEL_CSW_FUNC[i] = 0x00uL;
    }

    ptrHwBase->CSW_CTL_LO =             0x00uL;
    ptrHwBase->CSW_CTL_HI =             0x00uL;

    /* Configure MODE[0u] for for BIST Cmod measurement */
    ptrHwBase->MODE[0u].SENSE_DUTY_CTL = 0x00uL;
    ptrHwBase->MODE[0u].SW_SEL_CDAC_FL = 0x00uL;
    ptrHwBase->MODE[0u].SW_SEL_TOP =
           ((CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_CACC                      << MSCLP_MODE_SW_SEL_TOP_CACC_Pos)                      |
            (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_CBCD                      << MSCLP_MODE_SW_SEL_TOP_CBCD_Pos)                      |
            (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYA_CTL                   << MSCLP_MODE_SW_SEL_TOP_AYA_CTL_Pos)                   |
            (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYA_EN                    << MSCLP_MODE_SW_SEL_TOP_AYA_EN_Pos)                    |
            (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_RMF                       << MSCLP_MODE_SW_SEL_TOP_RMF_Pos));
    ptrHwBase->MODE[0u].SW_SEL_COMP =
            ((CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPMA                    << MSCLP_MODE_SW_SEL_COMP_CPMA_Pos)                     |
             (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMF                     << MSCLP_MODE_SW_SEL_COMP_CMF_Pos));
    ptrHwBase->MODE[0u].SW_SEL_SH =
            ((CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_CBSO                      << MSCLP_MODE_SW_SEL_SH_CBSO_Pos)                       |
             (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_FSP                       << MSCLP_MODE_SW_SEL_SH_FSP_Pos)                        |
             (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_BUF_SEL                   << MSCLP_MODE_SW_SEL_SH_BUF_SEL_Pos)                    |
             (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_BUF_EN                    << MSCLP_MODE_SW_SEL_SH_BUF_EN_Pos));
    ptrHwBase->MODE[0u].SW_SEL_CMOD1 =   0x00uL;
    ptrHwBase->MODE[0u].SW_SEL_CMOD2 =   0x00uL;
}


/*******************************************************************************
* Function Name: Cy_CapSense_DischargeCmod
****************************************************************************//**
*
* Discharges one of the CAPSENSE Cmod capacitors and restores its previous state.
*
* \param portCmod
* The pointer to the Cmod pin base port register.
*
* \param pinCmod
* The Cmod pin position (bit number) in the port.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_DischargeCmod(
                GPIO_PRT_Type * portCmod,
                uint32_t pinCmod,
                cy_stc_capsense_context_t * context)
{
    en_hsiom_sel_t hsiomReg;
    uint32_t pcReg;
    uint32_t  interruptState;

    /* Disconnect Ext Cap from AMUXBUS-A / AMUXBUSB using HSIOM registers */
    interruptState = Cy_SysLib_EnterCriticalSection();
    hsiomReg = Cy_GPIO_GetHSIOM(portCmod, pinCmod);
    Cy_GPIO_SetHSIOM(portCmod, pinCmod, CY_CAPSENSE_HSIOM_SEL_GPIO);
    Cy_SysLib_ExitCriticalSection(interruptState);

    /* Set port configuration register (drive mode) to STRONG mode */
    interruptState = Cy_SysLib_EnterCriticalSection();
    pcReg = Cy_GPIO_GetDrivemode(portCmod, pinCmod);
    Cy_GPIO_Clr(portCmod, pinCmod);
    Cy_GPIO_SetDrivemode(portCmod, pinCmod, CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF);
    Cy_SysLib_ExitCriticalSection(interruptState);

    /* Now external CSD-related capacitors discharging */
    Cy_SysLib_DelayUs(context->ptrBistContext->extCapDischargeTime);

    /* Restore Ext Cap settings */
    interruptState = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_SetDrivemode(portCmod, pinCmod, pcReg);
    Cy_GPIO_SetHSIOM(portCmod, pinCmod, hsiomReg);
    Cy_SysLib_ExitCriticalSection(interruptState);
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureCapacitanceCap_V3Lp
****************************************************************************//**
*
* The internal function for the external capacitance measurements
* on the fifth-generation low power CAPSENSE&trade; devices. For details, see
* the Cy_CapSense_MeasureCapacitanceCap() description.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
*
* \param integrationCapId
* Indexes of external capacitors to measure their capacitance.
* There are macros for each of them, namely:
* * CY_CAPSENSE_BIST_CMOD01_ID_E for the MSC0 Cmod1 capacitor
* * CY_CAPSENSE_BIST_CMOD02_ID_E for the MSC0 Cmod2 capacitor
*
* \param ptrValue
* The pointer to the result of the measurement. The result is calculated as
* a specified capacitor capacitance value in picofarads. The user
* declares a variable of the uint32_t type and passes the pointer to this
* variable as the function parameter. If the ptrValue parameter is NULL then
* the capacitance value is not returned through the parameter but stored to the
* corresponding field of the \ref cy_stc_capsense_bist_context_t structure.
*
* \param maxCapacitance
* An expected by the user maximum value of the measured capacitance in
* nanofarads in the range from 1 to 25 nF.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement completes
*                                         successfully, the result is valid.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_HW_BUSY_E          - The CAPSENSE&trade; HW block is busy with
*                                         a previous operation. The measurement
*                                         was not executed.
* - CY_CAPSENSE_BIST_TIMEOUT_E          - The timeout was reached during
*                                         the measurement.
*                                         The measurement result is zeroed.
*                                         The capacitor might be shorted to
*                                         GND.
* - CY_CAPSENSE_BIST_ERROR_E            - An unexpected fault occurred during
*                                         the measurement.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceCap_V3Lp(
                cy_en_capsense_bist_external_cap_id_t integrationCapId,
                uint32_t * ptrValue,
                uint32_t maxCapacitance,
                cy_stc_capsense_context_t * context)
{
    uint32_t rawCounts = 0u;
    GPIO_PRT_Type * portCmod;
    uint32_t pinCmod;
    uint16_t * ptrResult = NULL;
    uint32_t nSub;
    uint32_t maxRaw;
    uint32_t cdacTrim;
    cy_en_capsense_bist_status_t bistStatus = CY_CAPSENSE_BIST_SUCCESS_E;

    if ((NULL == context) || (CY_CAPSENSE_BIST_EXT_CAP_MAX_CAP < maxCapacitance) || (0u == maxCapacitance) ||
        (CY_CAPSENSE_BIST_CMOD02_ID_E < integrationCapId))
    {
        bistStatus = CY_CAPSENSE_BIST_BAD_PARAM_E;
    }

    if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
    {
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
        {
            /* Set the BUSY status and switch the HW resource configuration to the BIST mode */
            context->ptrCommonContext->status = CY_CAPSENSE_BUSY | CY_CAPSENSE_MW_STATE_BIST_MASK;
            switch (integrationCapId)
            {
                case CY_CAPSENSE_BIST_CMOD01_ID_E:
                    portCmod = context->ptrCommonConfig->ptrChConfig->portCmod1;
                    pinCmod = (uint32_t)context->ptrCommonConfig->ptrChConfig->pinCmod1;
                    ptrResult = &context->ptrBistContext->cMod01Cap;
                    break;
                case CY_CAPSENSE_BIST_CMOD02_ID_E:
                    portCmod = context->ptrCommonConfig->ptrChConfig->portCmod2;
                    pinCmod = (uint32_t)context->ptrCommonConfig->ptrChConfig->pinCmod2;
                    ptrResult = &context->ptrBistContext->cMod02Cap;
                    break;
                default:
                    bistStatus = CY_CAPSENSE_BIST_BAD_PARAM_E;
                    break;
            }
            if (CY_CAPSENSE_STATUS_SUCCESS !=
                Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context))
            {
                bistStatus = CY_CAPSENSE_BIST_ERROR_E;
            }

            if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
            {
                /* Generate the MSC base configuration for BIST scan and send it to the MSC HW block */
                Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_EXTERNAL_CAP_E, CY_CAPSENSE_UNDEFINED_GROUP, 0u, context);

                /* Discharge the specified capacitor */
                context->ptrBistContext->extCapDischargeTime = (uint16_t)((CY_CAPSENSE_BIST_EXT_CAP_DISCHARGE_FACTOR *
                    maxCapacitance * CY_CAPSENSE_BIST_EXT_CAP_SERIAL_RESISTANCE) / CY_CAPSENSE_CONVERSION_KILO);

                if (0u == context->ptrBistContext->extCapDischargeTime)
                {
                    context->ptrBistContext->extCapDischargeTime = 1u;
                }

                Cy_CapSense_DischargeCmod(portCmod, pinCmod, context);

                /* Calculate the needed max raw counts value */
                maxRaw = ((((CY_CAPSENSE_BIST_EXT_CAP_ACCURACY_FACTOR * maxCapacitance) *
                            CY_CAPSENSE_CONVERSION_KILO) / CY_CAPSENSE_BIST_EXT_CAP_REF_CDAC_DEFAULT) *
                          CY_CAPSENSE_CONVERSION_KILO) / CY_CAPSENSE_BIST_CAP_MEAS_CDAC_LSB_FF_DIV_1000;

                /* Calculate a corresponding number of sub-conversions for the measurement */
                nSub = maxRaw / (((uint32_t)MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk >> MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos) + 1u);

                if (CY_CAPSENSE_14_BIT_MASK < nSub)
                {
                    nSub = CY_CAPSENSE_14_BIT_MASK;
                }
                if (0u == nSub)
                {
                    nSub = 1u;
                }

                /* Number of sub-conversions variable initialization in BIST data structure */
                context->ptrBistContext->extCapSubConvNum = (uint16_t)nSub;

                /* Connect Cmod to AMuxBusA */
                Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                                 CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA,
                                                 CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);

                /* Perform the measurement */
                bistStatus = Cy_CapSense_BistMeasureCapacitanceCapRun(&rawCounts, context);

                /* Disconnect Cmod from AMuxBusA */
                Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                                 CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                 CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);

                if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
                {
                    /* Calculates the capacitance value in pF */
                    rawCounts *= CY_CAPSENSE_BIST_EXT_CAP_RESULT_FACTOR;
                    rawCounts += CY_CAPSENSE_BIST_EXT_CAP_RESULT_DIVIDER / 2u;
                    rawCounts /= CY_CAPSENSE_BIST_EXT_CAP_RESULT_DIVIDER;

                    /* Adds CDAC trimming */
                    if (0u != context->ptrCommonContext->cdacTrimCoefficient)
                    {
                        cdacTrim = context->ptrCommonContext->cdacTrimCoefficient;

                        if (0u != cdacTrim)
                        {
                            /* Scales result */
                            rawCounts = (uint32_t)(rawCounts << CY_CAPSENSE_CDAC_TRIM_OFFSET);
                            /* Rounding to the nearest */
                            rawCounts += (cdacTrim >> 0x1u);
                            rawCounts /= cdacTrim;
                        }
                    }

                    if (CY_CAPSENSE_16_BIT_MASK < rawCounts)
                    {
                        rawCounts = CY_CAPSENSE_16_BIT_MASK;
                    }

                    *ptrResult = (uint16_t)rawCounts;

                    if (NULL != ptrValue)
                    {
                        *ptrValue = *ptrResult;
                    }
                }
                else
                {
                    *ptrResult = 0u;
                    if (NULL != ptrValue)
                    {
                        *ptrValue = 0u;
                    }
                }
            }

            /* Clear the BUSY flag */
            context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;
        }
        else
        {
            bistStatus = CY_CAPSENSE_BIST_HW_BUSY_E;
        }
    }

    return bistStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureCapacitanceCapAll
****************************************************************************//**
*
* This internal function measures capacitance of all external capacitors.
*
* The function measures capacitances of all external capacitors Cmod1 and Cmod2.
* The function stores cap values in the corresponding registers.
* The function is called by the Cy_CapSense_RunSelfTest() function.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The measurement completes
*                                successfully, the result is valid.
* - CY_CAPSENSE_BIST_HW_BUSY_E - The CAPSENSE&trade; HW block is busy with a previous
*                                operation.
* - CY_CAPSENSE_BIST_ERROR_E - An unexpected fault occurred during
*                            the measurement, you may need to repeat the measurement.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureCapacitanceCapAll(
                cy_stc_capsense_context_t * context)
{
    uint32_t capCapacitance = 0u;
    cy_en_capsense_bist_status_t tempStatus;
    cy_en_capsense_bist_status_t bistStatus = CY_CAPSENSE_BIST_SUCCESS_E;

    tempStatus = Cy_CapSense_MeasureCapacitanceCap_V3Lp(CY_CAPSENSE_BIST_CMOD01_ID_E, &capCapacitance,
                                                        CY_CAPSENSE_BIST_EXT_CAP_CMOD_MAX_VALUE, context);
    if (CY_CAPSENSE_BIST_SUCCESS_E != tempStatus)
    {
        bistStatus = tempStatus;
    }

    tempStatus = Cy_CapSense_MeasureCapacitanceCap_V3Lp(CY_CAPSENSE_BIST_CMOD02_ID_E, &capCapacitance,
                                                        CY_CAPSENSE_BIST_EXT_CAP_CMOD_MAX_VALUE, context);
    if (CY_CAPSENSE_BIST_SUCCESS_E != tempStatus)
    {
        bistStatus = tempStatus;
    }

    return bistStatus;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_MeasureVdda_V3Lp
****************************************************************************//**
*
* Measures a VDDA voltage, returns the measured voltage in
* millivolts through the ptrValue argument and stores it to the .vddaVoltage
* field of the \ref cy_stc_capsense_bist_context_t structure.
*
* This function measures the device analog supply voltage (VDDA) without need
* of explicitly connecting VDDA to any additional GPIO input.
* This capability can be used in various cases, for example to monitor
* the battery voltage.
*
* The function uses Cmod1 charging/discharging due to the measurement.
* As the charging is non-linear, the measurement accuracy is about 20%.
*
* A measurement can be done only if the CAPSENSE&trade; middleware is in the
* IDLE state. This function must not be called while the CAPSENSE&trade;
* middleware is busy. The function is blocking, i.e. waits for the conversion
* to be completed prior to returning to the caller.
*
* \note
* This function is available for the forth-generation and the fifth-generation
* low power CAPSENSE&trade;.
*
* \param ptrValue
* The pointer to the uint32_t to store measured VDDA voltage value.
* If the ptrValue parameter is NULL then VDDA voltage value will not be returned
* through the parameter but still it will be stored in the .vddaVoltage
* field of the \ref cy_stc_capsense_bist_context_t structure. If the status
* of the test execution is not CY_CAPSENSE_BIST_SUCCESS_E the value
* the ptrValue parameter is pointed to will be zeroed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E          - The measurement executed
*                                         successfully.
* - CY_CAPSENSE_BIST_BAD_PARAM_E        - The input parameter is invalid.
*                                         The measurement was not executed.
* - CY_CAPSENSE_BIST_ERROR_E            - An unexpected fault occurred during
*                                         the measurement.
*
*******************************************************************************/
cy_en_capsense_bist_status_t Cy_CapSense_MeasureVdda_V3Lp(
                uint32_t * ptrValue,
                cy_stc_capsense_context_t * context)
{
    cy_en_capsense_bist_status_t bistStatus = CY_CAPSENSE_BIST_BAD_PARAM_E;

    if (NULL != context)
    {
        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(context))
        {
            /* Set the BUSY status and switch the HW resource configuration to the BIST mode */
            context->ptrCommonContext->status = CY_CAPSENSE_BUSY | CY_CAPSENSE_MW_STATE_BIST_MASK;
            if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY, context))
            {
                bistStatus = CY_CAPSENSE_BIST_ERROR_E;
            }
            else
            {
                bistStatus = CY_CAPSENSE_BIST_SUCCESS_E;
            }
            if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
            {
                Cy_CapSense_BistSwitchHwConfig(CY_CAPSENSE_BIST_HW_VDDA_E, CY_CAPSENSE_UNDEFINED_GROUP, 0u, context);

                /* Perform two-phase VDDA measurement */
                bistStatus = Cy_CapSense_BistMeasureVddaRun(context);

                if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
                {
                    if (NULL != ptrValue)
                    {
                        *ptrValue = context->ptrBistContext->vddaVoltage;
                    }
                }
                else
                {
                    if (NULL != ptrValue)
                    {
                        *ptrValue = 0u;
                    }
                }
            }
            /* Clear the BUSY flag */
            context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;
        }
        else
        {
            bistStatus = CY_CAPSENSE_BIST_HW_BUSY_E;
        }
    }

    return bistStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureVddaRun
****************************************************************************//**
*
* The internal function performs the two phase voltage measurement
* by converting the internal band gap reference Vbgref and Vdda/2 voltages
* to the raw counts and calculating the last one in millivolts.
*
* The function performs the following tasks:
* * Measures the time to bring Cmod1 up from Vssa to Vbgref
* * Measures the time to bring Cmod1 up from Vssa to Vdda/2
* * Calculates the results of measurements in millivolts.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns a status of the test execution:
* - CY_CAPSENSE_BIST_SUCCESS_E - The measurement completes
*                                successfully, the result is valid.
* - CY_CAPSENSE_BIST_ERROR_E   - An unexpected fault occurred during
*                                the measurement, you may need to repeat
*                                the measurement.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t Cy_CapSense_BistMeasureVddaRun(
                cy_stc_capsense_context_t * context)
{
    uint32_t rawVref = 0u;
    uint32_t rawVdda2 = 0u;
    uint32_t vddaValue = 0u;
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    GPIO_PRT_Type * portCmod = context->ptrCommonConfig->ptrChConfig->portCmod1;
    uint32_t pinCmod = (uint32_t)context->ptrCommonConfig->ptrChConfig->pinCmod1;

    cy_en_capsense_bist_status_t bistStatus = CY_CAPSENSE_BIST_BAD_PARAM_E;

    /*
     * Phase 1:
     * Measures raw counts for Cmod1 charge from VSSA to BGVREF.
     * The .extCapSubConvNum is initialized to the sense clock period number
     * for Cmod measurement
     * of the CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_MAX_POSSIBLE_VALUE
     */
    context->ptrBistContext->extCapSubConvNum = (uint16_t)CY_CAPSENSE_BIST_VDDA_MEAS_NUM_SNS_CLK;

    /* Connect the Plus comparator terminal to the AmuxBusA and the Minus comparator terminal to the BandGapVref */
    ptrHwBase->MODE[0u].SW_SEL_COMP =
            (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPMA                     << MSCLP_MODE_SW_SEL_COMP_CPMA_Pos);
    ptrHwBase->SW_SEL_BGR = (CY_CAPSENSE_BIST_CMOD_MEAS_SW_SEL_BGR_FLD_SW_BGRCM << MSCLP_SW_SEL_BGR_SW_BGRCM_Pos);

    /* Discharge the Cmod1 capacitor */
    context->ptrBistContext->extCapDischargeTime = (uint16_t)(CY_CAPSENSE_BIST_VDDA_MEAS_CMOD1_DISCHARGE_TIME);
    if (0u == context->ptrBistContext->extCapDischargeTime)
    {
        context->ptrBistContext->extCapDischargeTime = 1u;
    }
    Cy_CapSense_DischargeCmod(portCmod, pinCmod, context);

    /* Connect Cmod to AMuxBusA */
    Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                     CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA,
                                     CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);

    /* Perform the measurement */
    bistStatus = Cy_CapSense_BistMeasureCapacitanceCapRun(&rawVref, context);

    /* Disconnect Cmod from AMuxBusA */
    Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                     CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                     CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);

    if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
    {
        /*
         * Phase 2:
         * Measures raw counts for Cmod1 charge from VSSA to VDDA/2
         */

        /* Connect the Vdda/2 to the Minus comparator terminal */
        ptrHwBase->SW_SEL_BGR = 0u;
        ptrHwBase->MODE[0u].SW_SEL_COMP =
                ((CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPMA << MSCLP_MODE_SW_SEL_COMP_CPMA_Pos)   |
                 (CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMF << MSCLP_MODE_SW_SEL_COMP_CMF_Pos));

        /* Discharge Cmod1 to Vssa (GND) */
        Cy_CapSense_DischargeCmod(portCmod, pinCmod, context);

        /* Connect Cmod to AMuxBusA */
        Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                         CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA,
                                         CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);

        /* Perform the measurement */
        bistStatus = Cy_CapSense_BistMeasureCapacitanceCapRun(&rawVdda2, context);

        /* Disconnect Cmod from AMuxBusA */
        Cy_CapSense_SsConfigPinRegisters(portCmod, pinCmod,
                                         CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                         CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS);
        if (CY_CAPSENSE_BIST_SUCCESS_E == bistStatus)
        {
        /*
         * Phase 3:
         * Convert the results of measurements to millivolts taking into consideration that in Phase 2
         * Vdda/2 was measured.
         */
         vddaValue = ((CY_CAPSENSE_BIST_VDDA_MEAS_RESULT_FACTOR * rawVdda2) / rawVref) +
                     CY_CAPSENSE_BIST_VDDA_MEAS_RESULT_SHIFT;

            /* Store the measured value */
            context->ptrBistContext->vddaVoltage = (uint16_t)vddaValue;
        }
        else
        {
            context->ptrBistContext->vddaVoltage = 0u;
        }
    }

    return bistStatus;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */


/*******************************************************************************
* Function Name: Cy_CapSense_BistInitialize
****************************************************************************//**
*
* The internal function initializes some critical parameters
* for Built-in Self-test (BIST) mode.
*
* This function prepares the resource capturing to execute the BIST functions.
* The HW resource configuring is performed by the Cy_CapSense_BistSwitchHwConfig()
* function depending on a the type of the test to be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistInitialize(cy_stc_capsense_context_t * context)
{
    /*
     * This function prepares the BIST HW and pins re-configuring by setting some
     * critical internal parameters to undefined states.
     * Use this function after switching from CSD or CSX sensing modes.
     */
    context->ptrBistContext->hwConfig = CY_CAPSENSE_BIST_HW_UNDEFINED_E;
    context->ptrBistContext->currentISC = CY_CAPSENSE_BIST_IO_UNDEFINED_E;
    context->ptrBistContext->eltdCapSenseGroup = CY_CAPSENSE_UNDEFINED_GROUP;
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistDsInitialize_V3Lp
****************************************************************************//**
*
* This internal function initializes the BIST cy_stc_capsense_bist_context_t
* data structure parameters to be used in the test execution.
*
* The function is called once at the CAPSENSE&trade; start and performs
* the \ref cy_stc_capsense_bist_context_t structure initialization based
* on the configured parameters.
*
* Some of the parameters of the \ref cy_stc_capsense_bist_context_t structure
* can be changed in the run-time, but some changes require repeating the call
* of this function to the re-calculate register values.
* Refer to description of the following functions which parameters are used
* as an input and which are outputs:
* * Cy_CapSense_BistMeasureCapacitanceSensorInit()
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistDsInitialize_V3Lp(cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
        uint32_t wdIndex;
        uint32_t wdNum = (uint32_t)context->ptrCommonConfig->numWd;

        /* Initialize CRC for all widgets */
        for (wdIndex = 0u; wdIndex < wdNum; wdIndex++)
        {
            Cy_CapSense_UpdateCrcWidget(wdIndex, context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)) */

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
        Cy_CapSense_BistMeasureCapacitanceSensorInit(context);
    #endif

    context->ptrBistContext->hwConfig = CY_CAPSENSE_BIST_HW_UNDEFINED_E;
    context->ptrBistContext->currentISC = CY_CAPSENSE_BIST_IO_UNDEFINED_E;
    context->ptrBistContext->eltdCapSenseGroup = CY_CAPSENSE_UNDEFINED_GROUP;
    context->ptrBistContext->intrEltdCapCsdISC = context->ptrBistContext->eltdCapCsdISC;
    context->ptrBistContext->intrEltdCapCsxISC = context->ptrBistContext->eltdCapCsxISC;
    context->ptrBistContext->intrEltdCapShieldISC = context->ptrBistContext->shieldCapISC;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistDisableMode
****************************************************************************//**
*
* This internal function releases HW resources and pins captured by BIST
* to be used by other CAPSENSE&trade; modes.
*
* This function releases the shared HW resources like connection
* to the analog bus in the AMUX mode.
* The function does not configure HW block registers to the default state.
* It is used by the Cy_CapSense_SwitchHwConfiguration() function only at switching
* to another sensing mode.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistDisableMode(cy_stc_capsense_context_t * context)
{
    Cy_CapSense_BistSwitchAllSnsPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        Cy_CapSense_BistSwitchAllShieldPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);
    #endif
    Cy_CapSense_BistSwitchAllExternalCapPinState(CY_CAPSENSE_BIST_IO_STRONG_E, context);

    context->ptrBistContext->hwConfig = CY_CAPSENSE_BIST_HW_UNDEFINED_E;
    context->ptrBistContext->currentISC = CY_CAPSENSE_BIST_IO_UNDEFINED_E;
}

/*******************************************************************************
* Function Name: Cy_CapSense_BistSwitchHwConfig
****************************************************************************//**
*
* This internal function switches the MSCv3 HW block configuration for
* BIST operations.
*
* The function checks the current MSCv3 HW block configuration.
* If it differs from a desired configuration, the function disables the current
* configuration and sets the desired one.
*
* \param hwCfg
* Specifies the desired configuration.
*
* \param bistSenseGroup
* Specifies the sense group to be scanned during the sensor or electrode
* capacitance measurement scan.
* The parameter does not matter for short tests.
*
* \param bistScanMode
* Specifies the scanning mode (by electrodes or by slots) during the sensor
* or electrode capacitance measurement scan.
* The parameter does not matter for short tests.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistSwitchHwConfig(
                cy_en_capsense_bist_hw_config_t hwCfg,
                uint8_t bistSenseGroup,
                uint8_t bistScanMode,
                cy_stc_capsense_context_t * context)
{
    if ((context->ptrBistContext->hwConfig != hwCfg) ||
        (context->ptrBistContext->eltdCapSenseGroup != bistSenseGroup) ||
        (context->ptrBistContext->eltdCapScanMode != bistScanMode))
    {
        context->ptrBistContext->hwConfig = hwCfg;
        context->ptrBistContext->eltdCapSenseGroup = bistSenseGroup;
        context->ptrBistContext->eltdCapScanMode = bistScanMode;
        /* Enable the specified mode */
        switch (hwCfg)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
                case CY_CAPSENSE_BIST_HW_SHORT_E:
                {
                    break;
                }
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN) */

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)

                #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
                    ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
                    case CY_CAPSENSE_BIST_HW_ELTD_CAP_E:
                    {
                        Cy_CapSense_BistGenerateBaseConfig(context);

                        Cy_CapSense_SetIOsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO, 0u,
                                                         CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                            Cy_CapSense_SetShieldPinsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                                    CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) */

                        Cy_CapSense_SetCmodInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                          CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                        break;
                    }
                #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
                           ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN)
                    case CY_CAPSENSE_BIST_HW_EXTERNAL_CAP_E:
                    {
                        Cy_CapSense_SetIOsInDesiredState(CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF,
                                                         CY_CAPSENSE_HSIOM_SEL_GPIO, 0u,
                                                         CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                            Cy_CapSense_SetShieldPinsInDesiredState(CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF,
                                                                    CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                                    CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);
                        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) */

                        context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
                        Cy_CapSense_SetCmodInDesiredState(CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                          CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);
                        Cy_CapSense_BistGenerateBaseCmodConfig(context);
                        Cy_CapSense_SetupCpuOperatingMode(context);
                        break;
                    }
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */

            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN) */

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
                case CY_CAPSENSE_BIST_HW_VDDA_E:
                {
                    Cy_CapSense_SetIOsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO, 0u,
                                                     CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                        Cy_CapSense_SetShieldPinsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                                CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);
                    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) */

                    context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
                    Cy_CapSense_SetCmodInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                                                      CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS, context);

                    Cy_CapSense_BistGenerateBaseCmodConfig(context);
                    Cy_CapSense_SetupCpuOperatingMode(context);
                    break;
                }
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */

            default:
            {
                /* Nothing to do */
                break;
            }
        }
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistMeasureCapacitanceSensorInit
****************************************************************************//**
*
* This internal function calculates the parameters for sensor, shield or
* electrode capacitance measurements.
*
* It is called by Cy_CapSense_BistDsInitialize() once at the CAPSENSE&trade; MW start.
*
* The following parameters of cy_stc_capsense_bist_context_t are used as
* an input for the calculation:
* * .eltdCapModClk
* * .eltdCapSnsClk
*
* The function checks for parameter limitations and corrects the values before
* the calculation if they exceed.
*
* The following parameters of cy_stc_capsense_bist_context_t are updated
* by the calculation:
* * .eltdCapModClk
* * .eltdCapSnsClk
* * .eltdCapSubConvNum
*
* Restarting CAPSENSE&trade; MW or calling of the Cy_CapSense_BistDsInitialize()
* function overwrites the output parameters.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_BistMeasureCapacitanceSensorInit(
                cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_bist_context_t * ptrBistCxt = context->ptrBistContext;

    /* ModClk */
    if (0u == ptrBistCxt->eltdCapModClk)
    {
        ptrBistCxt->eltdCapModClk = (uint16_t)CY_CAPSENSE_BIST_ELTD_CAP_MODCLK_DIV_DEFAULT;
    }

    /* SnsClk */
    if (0u == ptrBistCxt->eltdCapSnsClk)
    {
        ptrBistCxt->eltdCapSnsClk = (uint16_t)CY_CAPSENSE_BIST_ELTD_CAP_SNSCLK_DIV_DEFAULT;
    }
    /* Check the snsClk divider value */
    if (CY_CAPSENSE_BIST_SNS_CLK_MIN_DIVIDER > ptrBistCxt->eltdCapSnsClk)
    {
        ptrBistCxt->eltdCapSnsClk = CY_CAPSENSE_BIST_SNS_CLK_MIN_DIVIDER;
    }
    if (CY_CAPSENSE_BIST_SNS_CLK_MAX_DIVIDER < ptrBistCxt->eltdCapSnsClk)
    {
        ptrBistCxt->eltdCapSnsClk = CY_CAPSENSE_BIST_SNS_CLK_MAX_DIVIDER;
    }
    /* NumSubConv */
    ptrBistCxt->eltdCapSubConvNum = (uint16_t)CY_CAPSENSE_BIST_ELTD_CAP_SUBCONV_NUM_DEFAULT;
    ptrBistCxt->eltdCapNumEpiCycles = CY_CAPSENSE_BIST_ELTD_CAP_NUM_EPI_CYCLES;
    ptrBistCxt->eltdCapNumFineInitWaitCycles = ptrBistCxt->eltdCapNumEpiCycles;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistSwitchAllSnsPinState
****************************************************************************//**
*
* This internal function sets all the port control (PC), data (DR) and HSIOM
* registers of all sensor pins to the desired state.
*
* The function sets the desired state for the pin port control register (PC),
* the output data register (DR) and the HSIOM register for all sensor pins
* (Drive Mode, output state, and HSIOM state).
*
* \param desiredPinState
* Specifies the desired pin state. See the possible states
* in the cy_en_capsense_bist_io_state_t enum description.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistSwitchAllSnsPinState(
                cy_en_capsense_bist_io_state_t desiredPinState,
                const cy_stc_capsense_context_t * context)
{
    uint32_t desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2GND;

    Cy_CapSense_BistSetDmHsiomPinState(desiredPinState, context);
    if (CY_CAPSENSE_BIST_IO_STRONG_HIGH_E == desiredPinState)
    {
        desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2VDD;
    }
    Cy_CapSense_BistSetAllSnsPinsState(context->ptrBistContext->eltdInactiveDm, desiredPinOutput,
                                       context->ptrBistContext->eltdInactiveHsiom, context);
}


/*******************************************************************************
* Function Name: Cy_CapSense_BistSetDmHsiomPinState
****************************************************************************//**
*
* This internal function changes the port control (PC) and HSIOM
* register values needed to set pins to the desired state.
*
* The function changes the .eltdInactiveDm and .eltdInactiveHsiom field values
* of the \ref cy_stc_capsense_bist_context_t structure on ones needed to set
* pins to the desired pin state (Drive Mode and HSIOM state).
*
* \param desiredPinState
* Specifies the desired pin state. See the possible states
* in the cy_en_capsense_bist_io_state_t enum description.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistSetDmHsiomPinState(
                cy_en_capsense_bist_io_state_t desiredPinState,
                const cy_stc_capsense_context_t * context)
{
    context->ptrBistContext->eltdInactiveDm = CY_CAPSENSE_DM_GPIO_ANALOG;
    context->ptrBistContext->eltdInactiveHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;

    /* Change Drive mode and HSIOM depending on the current inactive sensor connection */
    if ((CY_CAPSENSE_BIST_IO_STRONG_E == desiredPinState) ||
        (CY_CAPSENSE_BIST_IO_STRONG_HIGH_E == desiredPinState))
    {
        context->ptrBistContext->eltdInactiveDm = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
    }

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        if (CY_CAPSENSE_BIST_IO_SHIELD_E == desiredPinState)
        {
            context->ptrBistContext->eltdInactiveHsiom = CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD;
            if (CY_CAPSENSE_SHIELD_PASSIVE == context->ptrCommonConfig->csdShieldMode)
            {
                context->ptrBistContext->eltdInactiveDm = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
            }
        }
    #endif
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)        && \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_BistSwitchAllShieldPinState
****************************************************************************//**
*
* This internal function sets all the port control (PC), data (DR) and HSIOM
* registers of all shield pins to the desired state.
*
* The function sets the desired state for the pin port control register (PC),
* the output data register (DR) and the HSIOM register for all shield pins
* (Drive Mode, output state, and HSIOM state).
*
* \param desiredPinState
* Specifies the desired pin state. See the possible states
* in the cy_en_capsense_bist_io_state_t enum description.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistSwitchAllShieldPinState(
                cy_en_capsense_bist_io_state_t desiredPinState,
                const cy_stc_capsense_context_t * context)
{
    uint32_t desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2GND;

    if (0u < context->ptrCommonConfig->csdShieldNumPin)
    {
        Cy_CapSense_BistSetDmHsiomPinState(desiredPinState, context);
        if (CY_CAPSENSE_BIST_IO_STRONG_HIGH_E == desiredPinState)
        {
            desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2VDD;
        }
        Cy_CapSense_BistSetAllShieldPinsState(context->ptrBistContext->eltdInactiveDm, desiredPinOutput,
                                              context->ptrBistContext->eltdInactiveHsiom, context);
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && \
          (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_BistSwitchAllExternalCapPinState
****************************************************************************//**
*
* The internal function sets all the port control (PC), data (DR) and HSIOM
* registers of all external capacitor pins to the desired state.
*
* The function sets the desired state for the pin port control register (PC),
* the output data register (DR) and the HSIOM register for all external
* capacitor pins (Drive Mode, output state, and HSIOM state).
*
* \param desiredPinState
* Specifies the desired pin state. See the possible states
* in the cy_en_capsense_bist_io_state_t enum description.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
static void Cy_CapSense_BistSwitchAllExternalCapPinState(
                cy_en_capsense_bist_io_state_t desiredPinState,
                const cy_stc_capsense_context_t * context)
{
    uint32_t desiredDriveMode = CY_CAPSENSE_DM_GPIO_ANALOG;
    en_hsiom_sel_t desiredHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;
    uint32_t desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2GND;

    /* Change Drive mode and HSIOM depending on the current inactive sensor connection */
    if (CY_CAPSENSE_BIST_IO_STRONG_E == desiredPinState)
    {
        desiredDriveMode = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
        desiredHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;
        desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2GND;
    }
    else if (CY_CAPSENSE_BIST_IO_STRONG_HIGH_E == desiredPinState)
    {
        desiredDriveMode = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
        desiredHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;
        desiredPinOutput = CY_CAPSENSE_BIST_DR_PIN2VDD;
    }
    else
    {
        /* Do nothing */
    }
    Cy_CapSense_BistSetAllCmodPinsState(desiredDriveMode, desiredPinOutput, desiredHsiom, context);
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) || \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_BistWatchdogPeriodCalc
****************************************************************************//**
*
* The internal function calculates the watch-dog period for BIST sensor
* capacity measurement scans in CPU cycles.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the calculated watch-dog period for BIST sensor capacity measurement
* scans in CPU cycles:
*******************************************************************************/
static uint32_t Cy_CapSense_BistWatchdogPeriodCalc(
                const cy_stc_capsense_context_t * context)
{
    uint64_t isBusyWatchdogTimeUs;
    uint32_t modClkDivider = context->ptrBistContext->eltdCapModClk;
    uint32_t watchdogPeriod;

    if (0u == modClkDivider)
    {
        modClkDivider = 1u;
    }

    isBusyWatchdogTimeUs  = ((uint64_t)context->ptrBistContext->eltdCapSnsClk) *
                            ((uint64_t)context->ptrBistContext->eltdCapSubConvNum);
    isBusyWatchdogTimeUs *= (uint64_t)modClkDivider;
    isBusyWatchdogTimeUs /= CY_CAPSENSE_IMO_CLK_25_MHZ;
    if (0u == isBusyWatchdogTimeUs)
    {
        isBusyWatchdogTimeUs = 1u;
    }
    isBusyWatchdogTimeUs *= CY_CAPSENSE_BIST_WATCHDOG_MARGIN_COEFF;
    watchdogPeriod = Cy_CapSense_WatchdogCyclesNum((uint32_t)isBusyWatchdogTimeUs,
                                                   context->ptrCommonConfig->cpuClkHz / CY_CAPSENSE_CONVERSION_MEGA,
                                                   CY_CAPSENSE_BIST_CAP_MEAS_WDT_CYCLES_PER_LOOP);
    return watchdogPeriod;
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) \
           ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))) */

#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) */

#endif /* (defined(CY_IP_M0S8MSCV3LP)) */


/* [] END OF FILE */
