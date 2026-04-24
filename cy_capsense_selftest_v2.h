/***************************************************************************//**
* \file cy_capsense_selftest_v2.h
* \version 9.10.0
*
* \brief
* This file provides the function prototypes of the BIST module.
*
********************************************************************************
* \copyright
 * (c) 2019-2026, Infineon Technologies AG, or an affiliate of Infineon
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


#if !defined(CY_CAPSENSE_SELFTEST_V2_H)
#define CY_CAPSENSE_SELFTEST_V2_H

#include "cy_syslib.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cycfg_capsense_defines.h"

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2))

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN)

#if defined(__cplusplus)
extern "C" {
#endif

/* Sense Clock frequency in Hz for sensors capacitance measurement. */
#define CY_CAPSENSE_BIST_ELTD_CAP_SNSCLK_DEFAULT      (375000uL)
/* The default maximum possible external capacitor charge/discharge time in microseconds */
#define CY_CAPSENSE_BIST_CAP_SETTLING_TIME_DEFAULT    (2u)


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************/
/** \addtogroup group_capsense_low_level *//** \{ */
/******************************************************************************/

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSensor(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint32_t * ptrValue,
                    cy_stc_capsense_context_t * context);
#endif

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SH_CAP_EN))
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceShield(
                    uint32_t * ptrValue,
                    cy_stc_capsense_context_t * context);
#endif

/** \} */

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/

cy_en_capsense_bist_status_t Cy_CapSense_RunSelfTest_V2(
                uint32_t testEnMask,
                cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
cy_en_capsense_bist_status_t Cy_CapSense_CheckCRCWidget_V2(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorRawcount_V2(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint16_t rawcountHighLimit,
                    uint16_t rawcountLowLimit,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_RAW_INTEGRITY_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorBaseline_V2(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    uint16_t baselineHighLimit,
                    uint16_t baselineLowLimit,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_BSLN_INTEGRITY_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_CheckIntegritySensorPins_V2(
                    uint32_t widgetId,
                    uint32_t sensorId,
                    cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_SHORT_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceCap_V2(
                cy_en_capsense_bist_external_cap_id_t integrationCapId,
                uint32_t * ptrValue,
                uint32_t maxCapacitance,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_EXTERNAL_CAP_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN)
    cy_en_capsense_bist_status_t Cy_CapSense_MeasureVdda_V2(
                uint32_t * ptrValue,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_VDDA_EN) */

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN)
cy_en_capsense_bist_status_t Cy_CapSense_MeasureCapacitanceSensorExt(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_bist_custom_parameters_t * ptrScanConfig,
                uint32_t * ptrValue,
                cy_stc_capsense_context_t * context);
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_SNS_CAP_EN) */

void Cy_CapSense_BistInitialize(
                cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
    void Cy_CapSense_BistDisableMode(
                    cy_stc_capsense_context_t * context);
#endif

void Cy_CapSense_BistDsInitialize_V2(
                cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN)
    void Cy_CapSense_UpdateCrcWidget(
                    uint32_t widgetId,
                    cy_stc_capsense_context_t * context);
#endif


/** \} \endcond */

#if defined(__cplusplus)
}
#endif

#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) */

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2)) */

#endif /* CY_CAPSENSE_SELFTEST_V2_H */


/* [] END OF FILE */
