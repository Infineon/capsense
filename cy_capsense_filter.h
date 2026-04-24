/***************************************************************************//**
* \file cy_capsense_filter.h
* \version 9.10.0
*
* \brief
* This file contains the definitions for all the filters implementation.
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


#if !defined(CY_CAPSENSE_FILTER_H)
#define CY_CAPSENSE_FILTER_H

#include "cy_syslib.h"

#include "cycfg_capsense_defines.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_common.h"

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* LOW LEVEL FUNCTIONS
*******************************************************************************/

/******************************************************************************/
/** \addtogroup group_capsense_low_level *//** \{ */
/******************************************************************************/

cy_capsense_status_t Cy_CapSense_InitializeAllBaselines(cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_InitializeWidgetBaseline(uint32_t widgetId, cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_InitializeSensorBaseline(uint32_t widgetId, uint32_t sensorId, cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
    void Cy_CapSense_InitializeAllFilters(const cy_stc_capsense_context_t * context);
    void Cy_CapSense_InitializeWidgetFilter(uint32_t widgetId, const cy_stc_capsense_context_t * context);
#endif

cy_capsense_status_t Cy_CapSense_UpdateAllBaselines(const cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_UpdateWidgetBaseline(uint32_t widgetId, const cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_UpdateSensorBaseline(uint32_t widgetId, uint32_t sensorId, const cy_stc_capsense_context_t * context);

/** \} */


/*******************************************************************************
* Function Prototypes - Internal functions
*******************************************************************************/

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/
void Cy_CapSense_FtInitializeBaseline(
                cy_stc_capsense_sensor_context_t * ptrSnsContext);
cy_capsense_status_t Cy_CapSense_FtUpdateBaseline(
                cy_stc_capsense_widget_context_t * ptrWdContext,
                cy_stc_capsense_sensor_context_t * ptrSnsContext,
                uint16_t * ptrSnsBslnInv,
                const cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_POS_MEDIAN_FILTER_EN || CY_CAPSENSE_REGULAR_RC_MEDIAN_FILTER_EN \
    || CY_CAPSENSE_PROX_RC_MEDIAN_FILTER_EN || CY_CAPSENSE_MULTI_FREQUENCY_SCAN_EN || CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN)
    uint32_t Cy_CapSense_FtMedian(uint32_t x1, uint32_t x2, uint32_t x3);
#endif

uint32_t Cy_CapSense_FtIIR1stOrder(uint32_t input, uint32_t prevOutput, uint32_t n);

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_POS_JITTER_FILTER_EN)
    uint32_t Cy_CapSense_FtJitter(uint32_t input, uint32_t prevOutput);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
    void Cy_CapSense_FtRunEnabledFiltersInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory,
                    uint8_t * ptrSnsRawHistoryLow);
#endif

#if (CY_CAPSENSE_REGULAR_RC_IIR_FILTER_EN || CY_CAPSENSE_PROX_RC_IIR_FILTER_EN)
    void Cy_CapSense_InitializeIIRInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    const cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory,
                    uint8_t * ptrSnsRawHistoryLow);
    void Cy_CapSense_RunIIRInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory,
                    uint8_t * ptrSnsRawHistoryLow);
#endif

#if (CY_CAPSENSE_REGULAR_RC_MEDIAN_FILTER_EN || CY_CAPSENSE_PROX_RC_MEDIAN_FILTER_EN)
    void Cy_CapSense_InitializeMedianInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    const cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory);
    void Cy_CapSense_RunMedianInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory);
#endif

#if (CY_CAPSENSE_REGULAR_RC_AVERAGE_FILTER_EN || CY_CAPSENSE_PROX_RC_AVERAGE_FILTER_EN)
    void Cy_CapSense_InitializeAverageInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    const cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory);
    void Cy_CapSense_RunAverageInternal(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_sensor_context_t * ptrSnsContext,
                    uint16_t * ptrSnsRawHistory);
#endif

/** \} \endcond */

#if defined(__cplusplus)
}
#endif

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */

#endif /* CY_CAPSENSE_FILTER_H */


/* [] END OF FILE */
