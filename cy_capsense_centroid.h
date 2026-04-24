/***************************************************************************//**
* \file cy_capsense_centroid.h
* \version 9.10.0
*
* \brief
* This file provides the function prototypes for the centroid calculation
* methods.
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


#if !defined(CY_CAPSENSE_CENTROID_H)
#define CY_CAPSENSE_CENTROID_H

#include "cy_capsense_lib.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cycfg_capsense_defines.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))
#if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_LIQUID_LEVEL_EN))

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_LINEAR_SLIDER_EN)
    void Cy_CapSense_DpCentroidLinear(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_RADIAL_SLIDER_EN)
    void Cy_CapSense_DpCentroidRadial(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_DIPLEX_SLIDER_EN)
    void Cy_CapSense_DpCentroidDiplex(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN)
    void Cy_CapSense_DpCentroidTouchpad(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ADVANCED_CENTROID_5X5_EN)
    void Cy_CapSense_DpAdvancedCentroidTouchpad(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
    void Cy_CapSense_DpFindLocalMaxDd(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
    void Cy_CapSense_DpCalcTouchPadCentroid(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
    void Cy_CapSense_DpTouchTracking(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
    void Cy_CapSense_DpFilterTouchRecord(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_POSITION_FILTER_EN)
    void Cy_CapSense_InitPositionFilters(
                    uint32_t filterConfig,
                    const cy_stc_capsense_position_t * ptrInput,
                    cy_stc_capsense_position_t * ptrHistory);
    void Cy_CapSense_RunPositionFilters(
                    const cy_stc_capsense_widget_config_t * ptrWdConfig,
                    cy_stc_capsense_position_t * ptrInput,
                    cy_stc_capsense_position_t * ptrHistory);
    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_POSITION_FILTER_EN)
        void Cy_CapSense_RunPositionFiltersRadial(
                        const cy_stc_capsense_widget_config_t * ptrWdConfig,
                        cy_stc_capsense_position_t * ptrInput,
                        cy_stc_capsense_position_t * ptrHistory);
    #endif
    void Cy_CapSense_ProcessPositionFilters(
                    cy_stc_capsense_touch_t * newTouch,
                    const cy_stc_capsense_widget_config_t * ptrWdConfig);
#endif
/** \} \endcond */

#endif /* ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_LIQUID_LEVEL_EN)) */
#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */

#if defined(__cplusplus)
}
#endif

#endif /* CY_CAPSENSE_CENTROID_H */
/* [] END OF FILE */
