/***************************************************************************//**
* \file cy_capsense_structure.c
* \version 1.1
*
* \brief
* This file defines the data structure global variables and provides the
* implementation for the functions of the Data Structure module.
*
********************************************************************************
* \copyright
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <stddef.h>
#include <string.h>
#include "cy_syslib.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_lib.h"
#include "cy_csd.h"


/*******************************************************************************
* Function Name: Cy_CapSense_IsAnyWidgetActive
****************************************************************************//**
*
* Reports whether any widget has detected touch.
*
* This function reports whether any widget has detected a touch by extracting
* information from the widget status registers. This function does 
* not process widget data but extracts previously processed results 
* from the \ref group_capsense_structures.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the touch detection status of all the widgets:
* - Zero - No touch is detected in any of the widgets or sensors.
* - Non-zero - At least one widget or sensor has detected a touch.
*
*******************************************************************************/
uint32_t Cy_CapSense_IsAnyWidgetActive(const cy_stc_capsense_context_t * context)
{
    uint32_t result = 0u;
    uint32_t wdIndex;

    for (wdIndex = context->ptrCommonConfig->numWd; wdIndex-- > 0u;)
    {
        result |= (uint32_t)context->ptrWdContext[wdIndex].status & CY_CAPSENSE_WD_ACTIVE_MASK;
    }

    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_IsWidgetActive
****************************************************************************//**
*
* Reports whether the specified widget detected touch on any of its sensors.
*
* This function reports whether the specified widget has detected a touch by
* extracting information from the widget status register.
* This function does not process widget data but extracts previously processed 
* results from the \ref group_capsense_structures.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found 
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the touch detection status of the specified widgets:
* - Zero - No touch is detected in the specified widget or a wrong widgetId
* is specified.
* - Non-zero if at least one sensor of the specified widget is active, i.e.
* a touch is detected.
*
*******************************************************************************/
uint32_t Cy_CapSense_IsWidgetActive(
                uint32_t widgetId, 
                const cy_stc_capsense_context_t * context)
{
    uint32_t result = 0uL;

    if (widgetId < context->ptrCommonConfig->numWd)
    {
        result = (uint32_t)context->ptrWdContext[widgetId].status & CY_CAPSENSE_WD_ACTIVE_MASK;
    }
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_IsSensorActive
****************************************************************************//**
*
* Reports whether the specified sensor in the widget detected touch.
*
* This function reports whether the specified sensor in the widget has detected a
* touch by extracting information from the widget status register.
* This function does not process widget or sensor data but extracts previously  
* processed results from the \ref group_capsense_structures.
*
* For proximity sensors, this function returns the proximity detection status. 
* To get the touch status of proximity sensors, use the
* Cy_CapSense_IsProximitySensorActive() function.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found 
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
* 
* \param sensorId
* Specifies the ID number of the sensor within the widget. A macro for the 
* sensor ID within a specified widget can be found in the cycfg_capsense.h 
* file defined as CY_CAPSENSE_<WIDGET_NAME>_SNS<SENSOR_NUMBER>_ID.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the touch detection status of the specified sensor/widget:
* - Zero if no touch is detected in the specified sensor/widget or a wrong
* widget ID/sensor ID is specified.
* - Non-zero if the specified sensor is active, i.e. touch is detected. If the
* specific sensor belongs to a proximity widget, the proximity detection
* status is returned.
*
*******************************************************************************/
uint32_t Cy_CapSense_IsSensorActive(
                uint32_t widgetId, 
                uint32_t sensorId, 
                const cy_stc_capsense_context_t * context)
{
    uint32_t result = 0uL;

    if (widgetId < context->ptrCommonConfig->numWd)
    {
        if (sensorId < context->ptrWdConfig[widgetId].numSns)
        {
            result = context->ptrWdConfig[widgetId].ptrSnsContext[sensorId].status;
        }
    }
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_IsProximitySensorActive
****************************************************************************//**
*
* Reports the status of the specified proximity widget/sensor.
*
* This function reports whether the specified proximity sensor has detected 
* a touch or proximity event by extracting information from the widget 
* status register. This function is used only with proximity widgets. 
* This function does not process widget data but extracts previously processed 
* results from the \ref group_capsense_structures.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found 
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param sensorId
* Specifies the ID number of the sensor within the widget. A macro for the 
* sensor ID within a specified widget can be found in the cycfg_capsense.h 
* file defined as CY_CAPSENSE_<WIDGET_NAME>_SNS<SENSOR_NUMBER>_ID.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the specified sensor of the proximity widget. Zero
* indicates that no touch is detected in the specified sensor/widget or a
* wrong widgetId/proxId is specified.
* - Bits [31..2] are reserved.
* - Bit [1] indicates that a touch is detected.
* - Bit [0] indicates that a proximity is detected.
*
*******************************************************************************/
uint32_t Cy_CapSense_IsProximitySensorActive(
                uint32_t widgetId, 
                uint32_t sensorId, 
                const cy_stc_capsense_context_t * context)
{
    uint32_t result = 0uL;

    if (widgetId < context->ptrCommonConfig->numWd)
    {
        if ((uint8_t)CY_CAPSENSE_WD_PROXIMITY_E == context->ptrWdConfig[widgetId].wdType)
        {
            if (sensorId < context->ptrWdConfig[widgetId].numSns)
            {
                result |= context->ptrWdConfig[widgetId].ptrSnsContext[sensorId].status;
            }
        }
    }
    
    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetTouchInfo
****************************************************************************//**
*
* Reports the details of touch position detected on the specified touchpad, 
* matrix buttons or slider widgets.
*
* This function does not process widget data but extracts previously processed 
* results from the \ref group_capsense_structures.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found 
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the pointer to widget cy_stc_capsense_touch_t structure that 
* contains number of positions and data about each position.
* 
*
*******************************************************************************/
cy_stc_capsense_touch_t * Cy_CapSense_GetTouchInfo(
                uint32_t widgetId, 
                const cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_touch_t * ptrTouch = NULL;
    
    const cy_stc_capsense_widget_config_t * ptrWdCfg;

    if (widgetId < context->ptrCommonConfig->numWd)
    {
        ptrWdCfg = &context->ptrWdConfig[widgetId];
        switch (ptrWdCfg->wdType)
        {
            case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
            case (uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E:
            case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
            case (uint8_t)CY_CAPSENSE_WD_RADIAL_SLIDER_E:
                ptrTouch = &ptrWdCfg->ptrWdContext->wdTouch;
                break;
            default:
                break;
        }
    }
    
    return ptrTouch;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CheckConfigIntegrity
****************************************************************************//**
*
* Performs verification of CapSense data structure initialization.
*
* \param context
* The pointer to the CapSense context structure \ref cy_stc_capsense_context_t.
*
* \return status
* Returns status of operation:
* - Zero        - Indicates successful initialization.
* - Non-zero    - One or more errors occurred in the initialization process.
*
*******************************************************************************/
cy_status Cy_CapSense_CheckConfigIntegrity(const cy_stc_capsense_context_t * context)
{
    cy_status result = CY_RET_SUCCESS;
    
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    const cy_stc_capsense_common_context_t * ptrCommonCxt = context->ptrCommonContext;
    const cy_stc_capsense_internal_context_t * ptrInternalCxt = context->ptrInternalContext;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = context->ptrWdConfig;
    const cy_stc_capsense_widget_context_t * ptrWdCxt = context->ptrWdContext;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;
    const cy_stc_capsense_pin_config_t * ptrShieldPinCfg = context->ptrShieldPinConfig;
    const cy_stc_active_scan_sns_t * ptrActScanSns = context->ptrActiveScanSns;
    
    if (ptrCommonCfg == NULL)       {result |= CY_RET_BAD_DATA;}
    if (ptrCommonCxt == NULL)       {result |= CY_RET_BAD_DATA;}
    if (ptrInternalCxt == NULL)     {result |= CY_RET_BAD_DATA;}
    if (ptrWdCfg == NULL)           {result |= CY_RET_BAD_DATA;}
    if (ptrWdCxt == NULL)           {result |= CY_RET_BAD_DATA;}
    if (ptrPinCfg == NULL)          {result |= CY_RET_BAD_DATA;}
    if (ptrCommonCfg->csdShieldEn != 0u)
    {
        if((ptrCommonCfg->csdShieldNumPin > 0u) && (ptrShieldPinCfg == NULL))
        {
            result |= CY_RET_BAD_DATA;
        }
    }
    if (ptrActScanSns == NULL)      {result |= CY_RET_BAD_DATA;}
    
    
    return (result);
}


/* [] END OF FILE */
