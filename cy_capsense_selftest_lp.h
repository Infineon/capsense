/***************************************************************************//**
* \file cy_capsense_selftest_lp.h
* \version 7.0
*
* \brief
* This file provides the function prototypes of the BIST module.
*
********************************************************************************
* \copyright
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_CAPSENSE_SELFTEST_LP_H)
#define CY_CAPSENSE_SELFTEST_LP_H

#include "cy_syslib.h"
#include "cycfg_capsense_defines.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"

#if (defined(CY_IP_M0S8MSCV3LP))

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN)

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/******************************************************************************/
/** \addtogroup group_capsense_low_level *//** \{ */
/******************************************************************************/

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN))
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceLpSlotSensors(
                    uint32_t slotId,
                    cy_stc_capsense_context_t * context);
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)) */

/** \} */

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/

cy_en_capsense_bist_status_t Cy_CapSense_RunSelfTest_V3Lp(
                uint32_t testEnMask,
                cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckCRCWidget_V3Lp(
                    uint32_t widgetId,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorRawcount_V3Lp(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint16_t rawcountHighLimit,
                    uint16_t rawcountLowLimit,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorBaseline_V3Lp(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint16_t baselineHighLimit,
                    uint16_t baselineLowLimit,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorPins_V3Lp(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN) */

void Cy_CapSense_BistInitialize(
                cy_stc_capsense_context_t * context);
void Cy_CapSense_BistDsInitialize_V3Lp(
                cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
    void Cy_CapSense_BistDisableMode(
                    cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
    void Cy_CapSense_BistMeasureCapacitanceSensorInit(
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    void Cy_CapSense_UpdateCrcWidget(
                    uint32_t widgetId,
                    cy_stc_capsense_context_t * context);
    void Cy_CapSense_UpdateAllWidgetCrc(
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureVdda_V3Lp(
                uint32_t * ptrValue,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSensorElectrode_V3Lp(
                    uint32_t widgetId,
                    uint32_t eltdId,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_ELTD_CAP_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSlotSensors_V3Lp(
                    uint32_t slotId,
                    uint32_t skipChMask,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceShieldElectrode_V3Lp(
                    uint32_t skipChMask,
                    cy_stc_capsense_context_t * context);
#endif

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceCap_V3Lp(
                cy_en_capsense_bist_external_cap_id_t integrationCapId,
                uint32_t * ptrValue,
                uint32_t maxCapacitance,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */

/** \} \endcond */


#if defined(__cplusplus)
}
#endif

#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) */

#endif /* (defined(CY_IP_M0S8MSCV3LP)) */

#endif /* CY_CAPSENSE_SELFTEST_LP_H */


/* [] END OF FILE */
