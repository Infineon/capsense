/***************************************************************************//**
* \file cy_capsense_csd_v2.h
* \version 9.10.0
*
* \brief
* This file provides the function prototypes specific to the CSD sensing
* method implementation.
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


#if !defined(CY_CAPSENSE_CSD_V2_H)
#define CY_CAPSENSE_CSD_V2_H

#include "cy_syslib.h"
#include "cycfg_capsense_defines.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2))

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)

#if defined(__cplusplus)
extern "C" {
#endif


/****************************************************************************
* Register and mode mask definition
****************************************************************************/
#define CY_CAPSENSE_DEFAULT_CSD_SW_DSI_SEL                      (0x00000000uL)
#define CY_CAPSENSE_DEFAULT_CSD_INTR_SET                        (0x00000000uL)
#define CY_CAPSENSE_DEFAULT_SW_HS_N_SEL                         (0x00000000uL)
#define CY_CAPSENSE_DEFAULT_CSD_ADC_CTL                         (0x00000000uL)

#define CY_CAPSENSE_EXT_CAP_DISCHARGE_TIME                      (1u)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN)
cy_capsense_status_t Cy_CapSense_CSDCalibrateWidget(uint32_t widgetId, uint32_t target, cy_stc_capsense_context_t * context);
#endif
void Cy_CapSense_CSDSetupWidget(uint32_t widgetId, cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDSetupWidgetExt(uint32_t widgetId, uint32_t sensorId, cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDScan(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDScanExt(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDConnectSns(const cy_stc_capsense_pin_config_t * snsAddrPtr, const cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDDisconnectSns(const cy_stc_capsense_pin_config_t * snsAddrPtr, const cy_stc_capsense_context_t * context);

void Cy_CapSense_CSDDisableMode(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDInitialize(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDStartSample(cy_stc_capsense_context_t * context);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
void Cy_CapSense_CSDDisableShieldElectrodes(const cy_stc_capsense_context_t * context);
#endif

uint32_t Cy_CapSense_CSDGetNumberOfConversions(uint32_t snsClkDivider, uint32_t resolution, uint32_t snsClkSrc);
void Cy_CapSense_CSDSetUpIdacs(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDSnsStateCheck(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDCalculateScanDuration(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDConnectSnsExt(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDDisconnectSnsExt(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDConfigClock(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDClearSensors(const cy_stc_capsense_context_t * context);

void Cy_CapSense_CSDSetWidgetSenseClkSrc(const cy_stc_capsense_widget_config_t * ptrWdConfig);
uint32_t Cy_CapSense_CSDCalcPrsSize(uint32_t snsClkDivider, uint32_t resolution);

void Cy_CapSense_CSDScanISR(void * capsenseContext);
void Cy_CapSense_CSDCmodPrecharge(cy_stc_capsense_context_t * context);
void Cy_CapSense_CSDDischargeCmod(cy_stc_capsense_context_t * context);

/** \} \endcond */

#if defined(__cplusplus)
}
#endif

#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2)) */

#endif /* CY_CAPSENSE_CSD_V2_H */


/* [] END OF FILE */
