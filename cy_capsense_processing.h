/***************************************************************************//**
* \file cy_capsense_processing.h
* \version 9.10.0
*
* \brief
* This file provides the function prototypes for the Data Processing module.
* The Data Processing module is responsible for the low level raw counts
* processing provided by the sensing module, maintaining baseline and
* difference values and performing high-level widget processing like updating
* button status or calculating slider position.
*
********************************************************************************
* \copyright
 * (c) 2018-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/


#if !defined(CY_CAPSENSE_PROCESSING_H)
#define CY_CAPSENSE_PROCESSING_H

#include "cycfg_capsense_defines.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_lib.h"

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))


#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************/
/** \addtogroup group_capsense_high_level *//** \{ */
/******************************************************************************/
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_GESTURE_EN)
    void Cy_CapSense_DecodeWidgetGestures(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
#endif
/** \} */

/******************************************************************************/
/** \addtogroup group_capsense_low_level *//** \{ */
/******************************************************************************/
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_GESTURE_EN)
    void Cy_CapSense_InitializeWidgetGestures(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
#endif

void Cy_CapSense_InitializeAllStatuses(const cy_stc_capsense_context_t * context);
void Cy_CapSense_InitializeWidgetStatus(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_TX_ENABLED) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED))
        cy_capsense_status_t Cy_CapSense_ProcessWidgetMpDeconvolution(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
    #endif

    void Cy_CapSense_PreProcessSensor(
                            uint32_t widgetId,
                            uint32_t sensorId,
                            const cy_stc_capsense_context_t * context);
    void Cy_CapSense_PreProcessWidget(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN)
        cy_capsense_status_t Cy_CapSense_RunMfsMedian(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
    #endif /* CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN */
#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */

/** \} */

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/
#if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_BUTTON_EN) || \
     (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN) || \
     (CY_CAPSENSE_DISABLE != CY_CAPSENSE_WBX_BUTTON_EN))
    void Cy_CapSense_DpProcessButton(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
    void Cy_CapSense_DpProcessCsxTouchpad(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_PROXIMITY_EN)
    void Cy_CapSense_DpProcessProximity(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN)
    cy_capsense_status_t Cy_CapSense_DpProcessCsdTouchpad(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    const cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN)
    void Cy_CapSense_DpProcessSlider(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_EN)
    cy_capsense_status_t Cy_CapSense_DpProcessLiquidLevel(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN)
    void Cy_CapSense_DpProcessCsdMatrix(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

uint32_t Cy_CapSense_DpProcessWidgetRawCounts(
                    uint32_t widgetId,
                    const cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
cy_capsense_status_t Cy_CapSense_DpProcessCsdWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
void Cy_CapSense_DpProcessCsxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
void Cy_CapSense_DpProcessIsxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
void Cy_CapSense_DpProcessWbxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN) */

uint32_t Cy_CapSense_DpProcessSensorRawCountsExt(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                cy_stc_capsense_sensor_context_t * ptrSnsContext,
                uint16_t * ptrSnsRawHistory,
                uint8_t * ptrSnsRawHistoryLow,
                uint32_t mode,
                uint16_t * ptrSnsBslnInv,
                const cy_stc_capsense_context_t * context);

void Cy_CapSense_DpUpdateDifferences(
                const cy_stc_capsense_widget_context_t * ptrWdContext,
                cy_stc_capsense_sensor_context_t * ptrSnsContext);
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_REGULAR_RC_CMF_FILTER_EN)
    void Cy_CapSense_DpUpdateDifferencesCmf(
                    cy_stc_capsense_sensor_context_t * ptrSnsContext);
#endif
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
void Cy_CapSense_ProcessWbxCorrection(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context);
void Cy_CapSense_WbxCorrectionProfile(
                volatile cy_stc_capsense_wbx_profile_t * ptrProfile);
#endif

void Cy_CapSense_DpUpdateThresholds(
                cy_stc_capsense_widget_context_t * ptrWdContext,
                const cy_stc_capsense_smartsense_csd_noise_envelope_t * ptrNoiseEnvelope,
                uint32_t startFlag);

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) && \
    (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_SCAN_EN))
    void Cy_CapSense_RunMfsFiltering(
                    cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    const cy_stc_capsense_context_t * context);
#endif

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
void Cy_CapSense_PreProcessWidgetInvertRaw(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
void Cy_CapSense_PreProcessSensorInvertRaw(
                uint32_t widgetId,
                uint32_t sensorId,
                const cy_stc_capsense_context_t * context);
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)) */

void Cy_CapSense_PreProcessWidgetLimitRaw(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context);
void Cy_CapSense_PreProcessSensorLimitRaw(
                uint32_t widgetId,
                uint32_t sensorId,
                const cy_stc_capsense_context_t * context);

#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */

/** \} \endcond */

#if defined(__cplusplus)
}
#endif

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */

#endif /* CY_CAPSENSE_PROCESSING_H */


/* [] END OF FILE */
