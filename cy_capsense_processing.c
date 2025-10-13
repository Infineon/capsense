/***************************************************************************//**
* \file cy_capsense_processing.c
* \version 8.0.0
*
* \brief
* This file provides the source code for the Data Processing module functions.
* The Data Processing module is responsible for the low-level raw count
* processing provided by the sensing module, maintaining baseline and
* difference values and performing high-level widget processing like
* updating button status or calculating slider position.
*
********************************************************************************
* \copyright
* Copyright 2018-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#include "cy_syslib.h"
#include <stddef.h>
#include <string.h>
#include "cycfg_capsense_defines.h"
#include "cy_capsense_common.h"
#include "cy_capsense_processing.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_lib.h"
#include "cy_capsense_centroid.h"

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))


/*******************************************************************************
* Local definition
*******************************************************************************/
/* Raw data normalization and scaling */
#define CY_CAPSENSE_SCALING_SHIFT               (15)
#define CY_CAPSENSE_MAX_TX_PATTERN_NUM          (32)
#define CY_CAPSENSE_LLW_NOT_VALID_DATA          (0xFFFFu)
#define CY_CAPSENSE_RAW_COUNT_MAX_VALUE         (CY_CAPSENSE_LLW_NOT_VALID_DATA)

/* CDAC LSB in 10^-15 F */
#define CY_CAPSENSE_REF_CDAC_LSB                (8870u)
#define CY_CAPSENSE_FINE_CDAC_LSB               (2600u)

/* Scaling Coefficients */
#define CY_CAPSENSE_SCALE_10                    (10)
#define CY_CAPSENSE_SCALE_100                   (100)
#define CY_CAPSENSE_SCALE_1000                  (1000)
#define CY_CAPSENSE_SCALE_10E4                  (10000)
#define CY_CAPSENSE_SCALE_10E6                  (1000000)
#define CY_CAPSENSE_SCALE_10E9                  (1000000000)

/* Limitation Numbers */
#define CY_CAPSENSE_HI_LIMIT_NOM                (20000000)
#define CY_CAPSENSE_LO_LIMIT_NOM                (200000000)
#define CY_CAPSENSE_HI_LIMIT_DEN                (20000000)
#define CY_CAPSENSE_LO_LIMIT_DEN                (2000000)
#define CY_CAPSENSE_LO_LIMIT                    (20000)

#define CY_CAPSENSE_ADV_TOUCHPAD_MIN_SNS_NUMBER (3u) 

/*******************************************************************************
* Function Name: Cy_CapSense_InitializeAllStatuses
****************************************************************************//**
*
* Performs initialization of all statuses and related modules including
* debounce counters and touch positions of all the widgets.
*
* The initialization includes the following tasks:
* * Reset the debounce counters of all the widgets.
* * Reset the number of touches.
* * Reset the position filter history for slider and touchpad widgets.
* * Clear all status of widgets and sensors.
* * Enable all the widgets.
*
* Calling this function is accompanied by
* * Cy_CapSense_InitializeAllBaselines().
* * Cy_CapSense_InitializeAllFilters().
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitializeAllStatuses(const cy_stc_capsense_context_t * context)
{
    uint32_t widgetId;

    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        #if (CY_CAPSENSE_LP_EN)
            context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_LP_ACTIVE_MASK); 
        #endif
    #endif    

    for (widgetId = CY_CAPSENSE_TOTAL_WIDGET_COUNT; widgetId-- > 0u;)
    {
        Cy_CapSense_InitializeWidgetStatus(widgetId, context);
        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_GESTURE_EN)
            Cy_CapSense_InitializeWidgetGestures(widgetId, context);
        #endif
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_InitializeWidgetStatus
****************************************************************************//**
*
* Performs initialization of all statuses, debounce counters, and touch positions
* of the specified widget.
*
* The initialization includes:
* * Resets the debounce counter of the widget.
* * Resets the number of touches.
* * Resets the position filter history for slider and touchpad widgets.
* * Clears widget and sensor statuses.
* * Enables the widget.
*
* The Button and Matrix Button widgets have individual debounce counters per
* sensor for the CSD widgets and per node for the CSX and ISX widgets.
*
* The Slider and Touchpad widgets have a single debounce counter per widget.
*
* The Proximity widget has two debounce counters per sensor. One is for the
* proximity event and the second is for the touch event.
*
* All debounce counters during initialization are set to the value of the
* onDebounce widget parameter.
*
* Calling this function is accompanied by
* * Cy_CapSense_InitializeWidgetBaseline().
* * Cy_CapSense_InitializeWidgetFilter().
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \note For the fifth-generation low power CAPSENSE&trade; widgets
* of the \ref CY_CAPSENSE_WD_LOW_POWER_E type are not processed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitializeWidgetStatus(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    uint32_t snsIndex;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdCfg->ptrWdContext;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdCfg->ptrSnsContext;
    uint32_t snsNumber = ptrWdCfg->numSns;

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN)
        uint32_t filterSize;
        cy_stc_capsense_position_t * ptrHistory;
    #endif

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType)
#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */
    {
        /* Clear widget active status */
        ptrWdCxt->status &= (uint8_t)~(CY_CAPSENSE_WD_ACTIVE_MASK);
        /* Clear sensor status */
        for (snsIndex = snsNumber; snsIndex-- >0u;)
        {
            ptrSnsCxt->status &= (uint8_t)~(CY_CAPSENSE_SNS_TOUCH_STATUS_MASK | CY_CAPSENSE_SNS_TOUCH_PROX_STATUS_MASK);
            ptrSnsCxt++;
        }

        /* Reset debounce counters */
        switch (ptrWdCfg->wdType)
        {
            #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_BUTTON_EN) ||\
                (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MATRIX_EN))
                case (uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E:
                case (uint8_t)CY_CAPSENSE_WD_BUTTON_E:
                    /* Each button requires one debounce counter */
                    (void)memset(ptrWdCfg->ptrDebounceArr, (int32_t)ptrWdCxt->onDebounce, (size_t)snsNumber);
                    break;
            #endif
            #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN) ||\
                (CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN))
                case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
                case (uint8_t)CY_CAPSENSE_WD_RADIAL_SLIDER_E:
                case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
                    /* Each widget requires one debounce counter */
                    switch (ptrWdCfg->senseMethod)
                    {
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                            case CY_CAPSENSE_CSD_GROUP:
                                *(ptrWdCfg->ptrDebounceArr) = ptrWdCxt->onDebounce;
                                break;
                        #endif

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                            case CY_CAPSENSE_CSX_GROUP:
                                if ((uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E == ptrWdCfg->wdType)
                                {
                                    *(ptrWdCfg->ptrDebounceArr) = ptrWdCxt->onDebounce;
                                }

                                /*
                                * CSX Touchpad has debounce located in another place. Moreover,
                                * debounce counter is initialized at ID assignment, so no need
                                * to do it here.
                                */
                                break;
                        #endif

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                            case CY_CAPSENSE_ISX_GROUP:
                                *(ptrWdCfg->ptrDebounceArr) = ptrWdCxt->onDebounce;
                                break;
                        #endif

                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
                            case CY_CAPSENSE_WBX_GROUP:
                                *(ptrWdCfg->ptrDebounceArr) = ptrWdCxt->onDebounce;
                                break;
                        #endif

                        default:
                            /* No action */
                            break;
                    }
                    break;
            #endif
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_PROXIMITY_EN)
                case (uint8_t)CY_CAPSENSE_WD_PROXIMITY_E:
                    /* Proximity widgets have 2 debounce counters per sensor (for touch and prox detection) */
                    (void)memset(ptrWdCfg->ptrDebounceArr, (int32_t)ptrWdCxt->onDebounce, (size_t)(snsNumber << 1u));
                    break;
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_EN)
                case (uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E:
                    ptrWdCfg->ptrWdContext->wdTouch.numPosition = 1u;
                    ptrWdCfg->ptrWdContext->status |= (uint8_t)CY_CAPSENSE_WD_ACTIVE_MASK;
                    break;
            #endif

            default:
                /* No other widget types */
                break;
        }

        /* Reset touch numbers */
        #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN) ||\
            (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MATRIX_EN) ||\
            (CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN))
            switch (ptrWdCfg->wdType)
            {
                case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
                case (uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E:
                case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
                case (uint8_t)CY_CAPSENSE_WD_RADIAL_SLIDER_E:
                    /* Clean number of touches */
                    ptrWdCxt->wdTouch.numPosition = CY_CAPSENSE_POSITION_NONE;
                    if (0u != (ptrWdCfg->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_MASK))
                    {
                        ptrWdCfg->ptrPosFilterHistory->numPosition = CY_CAPSENSE_POSITION_NONE;
                    }
                    break;
                default:
                    /* No action on other widget types */
                    break;
            }
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_BALLISTIC_MULTIPLIER_EN)
            /* Reset ballistic displacement */
            if (0u != (ptrWdCfg->centroidConfig & CY_CAPSENSE_CENTROID_BALLISTIC_MASK))
            {
                ptrWdCxt->xDelta = 0;
                ptrWdCxt->yDelta = 0;
                ptrWdCfg->ptrBallisticContext->oldTouchNumber = 0u;
            }
        #endif

        /* Reset touch history */
        if (0u != (ptrWdCfg->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_MASK))
        {
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_TOUCHPAD_EN)
                switch (ptrWdCfg->wdType)
                {
                    case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
                        /* Clean position filter history */
                        if (ptrWdCfg->senseMethod == CY_CAPSENSE_CSX_GROUP)
                        {
                            /* Reset all history IDs to undefined state */
                            ptrHistory = ptrWdCfg->ptrPosFilterHistory->ptrPosition;
                            filterSize = (ptrWdCfg->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_SIZE_MASK) >> CY_CAPSENSE_POSITION_FILTERS_SIZE_OFFSET;
                            for (snsIndex = 0u; snsIndex < CY_CAPSENSE_MAX_CENTROIDS; snsIndex++)
                            {
                                ptrHistory->id = CY_CAPSENSE_CSX_TOUCHPAD_ID_UNDEFINED;
                                ptrHistory += filterSize;
                            }
                        }
                        break;
                    default:
                        /* No action on other widget types */
                        break;
                }
            #endif

            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ADAPTIVE_FILTER_EN)
                /* Init Adaptive IIR filter */
                if (0u != (ptrWdCfg->posFilterConfig & CY_CAPSENSE_POSITION_AIIR_MASK))
                {
                    Cy_CapSense_AdaptiveFilterInitialize_Lib(&ptrWdCfg->aiirConfig,
                                                             ptrWdCfg->ptrPosFilterHistory->ptrPosition);
                }
            #endif
        }
    }
}

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_GESTURE_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_InitializeWidgetGestures
****************************************************************************//**
*
* Performs initialization of all gestures for the specified widget.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitializeWidgetGestures(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdCfg->ptrWdContext;

    if (NULL != ptrWdCfg->ptrGestureConfig)
    {
        if (0u != (ptrWdCfg->ptrGestureConfig->gestureEnableMask & CY_CAPSENSE_GESTURE_ALL_GESTURES_MASK))
        {
            ptrWdCxt->gestureDetected = 0u;
            ptrWdCxt->gestureDirection = 0u;
            Cy_CapSense_Gesture_ResetState(ptrWdCfg->ptrGestureContext);
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_DecodeWidgetGestures
****************************************************************************//**
*
* Performs decoding of all gestures for the specified widget.
*
* This function should be called by application program only after all sensors
* are scanned and all data processing is executed using
* Cy_CapSense_ProcessAllWidgets() or Cy_CapSense_ProcessWidget() functions
* for the widget. Calling this function multiple times without a new sensor
* scan and process causes unexpected behavior.
*
* \note The function (Gesture detection functionality) requires a timestamp
* for its operation. The timestamp should be initialized and maintained
* in the application program prior to calling this function. See the
* descriptions of the Cy_CapSense_SetGestureTimestamp() and
* Cy_CapSense_IncrementGestureTimestamp() functions for details.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the detected Gesture mask and direction of detected gestures.
* The same information is stored in ptrWdContext->gestureDetected and
* ptrWdContext->gestureDirection registers. Corresponding macros could be found
* \ref group_capsense_macros_gesture.
* * bit[0..15] - detected gesture masks gesture
*   * bit[0] - one-finger single click gesture
*   * bit[1] - one-finger double click gesture
*   * bit[2] - one-finger click and drag gesture
*   * bit[3] - two-finger single click gesture
*   * bit[4] - one-finger scroll gesture
*   * bit[5] - two-finger scroll gesture
*   * bit[6] - one-finger edge swipe
*   * bit[7] - one-finger flick
*   * bit[8] - one-finger rotate
*   * bit[9] - two-finger zoom
*   * bit[10] - one-finger long press
*   * bit[13] - touchdown event
*   * bit[14] - liftoff event
* * bit[16..31] - gesture direction if detected
*    * bit[0..1] - direction of one-finger scroll gesture
*    * bit[2..3] - direction of two-finger scroll gesture
*    * bit[4..5] - direction of one-finger edge swipe gesture
*    * bit[6] - direction of one-finger rotate gesture
*    * bit[7] - direction of two-finger zoom gesture
*    * bit[8..10] - direction of one-finger flick gesture
*
* \funcusage
*
* An example of gesture decoding:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Gesture
*
* An example of gesture status parsing:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Gesture_Macro
*
*******************************************************************************/
uint32_t Cy_CapSense_DecodeWidgetGestures(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    uint32_t gestureStatus = 0u;
    uint32_t posIndex;
    uint32_t positionNum;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdCfg->ptrWdContext;
    cy_stc_capsense_gesture_position_t position[CY_CAPSENSE_MAX_CENTROIDS];

    if (NULL != ptrWdCfg->ptrGestureConfig)
    {
        if (0u != (ptrWdCfg->ptrGestureConfig->gestureEnableMask & CY_CAPSENSE_GESTURE_ALL_GESTURES_MASK))
        {
            if (((uint8_t)CY_CAPSENSE_WD_BUTTON_E == ptrWdCfg->wdType) ||
                ((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdCfg->wdType))
            {
                positionNum = (uint32_t)ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK;
                for (posIndex = 0u; posIndex < positionNum; posIndex++)
                {
                    position[posIndex].x = 0u;
                    position[posIndex].y = 0u;
                }
            }
            else
            {
                positionNum = ptrWdCxt->wdTouch.numPosition;
                if (positionNum > CY_CAPSENSE_MAX_CENTROIDS)
                {
                    positionNum = 0u;
                }
                for (posIndex = 0u; posIndex < positionNum; posIndex++)
                {
                    position[posIndex].x = ptrWdCxt->wdTouch.ptrPosition[posIndex].x;
                    position[posIndex].y = ptrWdCxt->wdTouch.ptrPosition[posIndex].y;
                }
            }
            Cy_CapSense_Gesture_Decode(context->ptrCommonContext->timestamp, positionNum,
                    &position[0u], ptrWdCfg->ptrGestureConfig, ptrWdCfg->ptrGestureContext);
            ptrWdCxt->gestureDetected = ptrWdCfg->ptrGestureContext->detected;
            ptrWdCxt->gestureDirection = ptrWdCfg->ptrGestureContext->direction;
            gestureStatus = (uint32_t)ptrWdCxt->gestureDetected | ((uint32_t)ptrWdCxt->gestureDirection << CY_CAPSENSE_GESTURE_DIRECTION_OFFSET);
        }
    }

    return gestureStatus;
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_GESTURE_EN) */


/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessWidgetRawCounts
****************************************************************************//**
*
* Performs default processing of the raw counts of the specified widget.
*
* The processing includes the following tasks:
* - Run Filters.
* - Update Baselines.
* - Update Differences.
* The same process is applied to all the sensors of the specified widget.
* For liquid level widget is applied only filtering, updating baseline and 
* differences are skipped.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the specified widget processing operation:
* - Zero - if operation was successfully completed.
* - Non-zero - if baseline processing of any sensor of the specified widget
* failed. The result is concatenated with the index of failed sensor.
*
*******************************************************************************/
uint32_t Cy_CapSense_DpProcessWidgetRawCounts(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    uint32_t result = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t snsIndex;
    uint32_t freqChIndex;
    uint16_t * ptrBslnInvSns;
    uint8_t * ptrHistoryLowSns = NULL;
    cy_stc_capsense_sensor_context_t * ptrSnsCxtSns;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;

    ptrWdCfg = &context->ptrWdConfig[widgetId];

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
        uint32_t snsHistorySize;
        uint16_t * ptrHistorySns;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ACTIVE_FULL_EN)
        cy_stc_capsense_smartsense_csd_noise_envelope_t * ptrNEHistory = ptrWdCfg->ptrNoiseEnvelope;
    #endif

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
        snsHistorySize = (uint32_t)ptrWdCfg->rawFilterConfig & CY_CAPSENSE_RC_FILTER_SNS_HISTORY_SIZE_MASK;
    #endif

    for (freqChIndex = 0u; freqChIndex < CY_CAPSENSE_CONFIGURED_FREQ_NUM; freqChIndex++)
    {
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_REGULAR_RC_CMF_FILTER_EN)
            if (0u != (ptrWdCfg->rawFilterConfig & CY_CAPSENSE_RC_FILTER_CM_EN_MASK))
            {
                /* Runs common mode filter */
                Cy_CapSense_CommonModeFilter_Lib(ptrWdCfg->cmfThreshold, ptrWdCfg->numSns,
                        (uint16_t *)ptrWdCfg->ptrSnsContext);
            }
        #endif

        ptrSnsCxtSns = &ptrWdCfg->ptrSnsContext[freqChIndex * context->ptrCommonConfig->numSns];
        ptrBslnInvSns = &ptrWdCfg->ptrBslnInv[freqChIndex * context->ptrCommonConfig->numSns];
        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
            ptrHistorySns = &ptrWdCfg->ptrRawFilterHistory[freqChIndex * (CY_CAPSENSE_RAW_HISTORY_SIZE / CY_CAPSENSE_CONFIGURED_FREQ_NUM)];
        #endif
        if (CY_CAPSENSE_IIR_FILTER_PERFORMANCE == (ptrWdCfg->rawFilterConfig & CY_CAPSENSE_RC_FILTER_IIR_MODE_MASK))
        {
            ptrHistoryLowSns = &ptrWdCfg->ptrRawFilterHistoryLow[freqChIndex *
                    (CY_CAPSENSE_IIR_HISTORY_LOW_SIZE / CY_CAPSENSE_CONFIGURED_FREQ_NUM)];
        }

        for (snsIndex = 0u; snsIndex < ptrWdCfg->numSns; snsIndex++)
        {
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
                Cy_CapSense_FtRunEnabledFiltersInternal(ptrWdCfg, ptrSnsCxtSns, ptrHistorySns, ptrHistoryLowSns);
            #endif

            /* Run auto-tuning activities */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ACTIVE_FULL_EN)
                if ((((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType) && ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != ptrWdCfg->wdType)) &&
                   (((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_CSD_ACTIVE_FULL_EN)) ||
                    ((CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ISX_ACTIVE_FULL_EN))))
                {
                    Cy_CapSense_RunNoiseEnvelope_Lib(ptrSnsCxtSns->raw, ptrWdCfg->ptrWdContext->sigPFC, ptrNEHistory);
                    Cy_CapSense_DpUpdateThresholds(ptrWdCfg->ptrWdContext, ptrNEHistory, snsIndex);
                    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_PROXIMITY_EN)
                        if ((uint8_t)CY_CAPSENSE_WD_PROXIMITY_E == ptrWdCfg->wdType)
                        {
                            ptrWdCfg->ptrWdContext->proxTh = ptrWdCfg->ptrWdContext->fingerTh;
                            ptrWdCfg->ptrWdContext->fingerTh = (uint16_t)(((uint32_t)ptrWdCfg->ptrWdContext->fingerTh *
                                context->ptrCommonConfig->proxTouchCoeff) / CY_CAPSENSE_PERCENTAGE_100);
                        }
                    #endif
                    ptrNEHistory++;
                }
            #endif

            if ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != ptrWdCfg->wdType)
            {
                result |= Cy_CapSense_FtUpdateBaseline(ptrWdCfg->ptrWdContext, ptrSnsCxtSns, ptrBslnInvSns, context);

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_REGULAR_RC_CMF_FILTER_EN)
                    if (0u != (ptrWdCfg->rawFilterConfig & CY_CAPSENSE_RC_FILTER_CM_EN_MASK))
                    {
                        Cy_CapSense_DpUpdateDifferencesCmf(ptrSnsCxtSns);
                    }
                    else
                    {
                        Cy_CapSense_DpUpdateDifferences(ptrWdCfg->ptrWdContext, ptrSnsCxtSns);
                    }
                #else
                    Cy_CapSense_DpUpdateDifferences(ptrWdCfg->ptrWdContext, ptrSnsCxtSns);
                #endif
            }

            ptrSnsCxtSns++;
            ptrBslnInvSns++;
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
                ptrHistorySns += snsHistorySize;
            #endif
            if (NULL != ptrHistoryLowSns)
            {
                ptrHistoryLowSns++;
            }
        }
    }

    #if ((CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) && \
        (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_SCAN_EN))
        ptrSnsCxtSns = ptrWdCfg->ptrSnsContext;
        for (snsIndex = ptrWdCfg->numSns; snsIndex-- > 0u;)
        {
            Cy_CapSense_RunMfsFiltering(ptrSnsCxtSns, context);
            ptrSnsCxtSns++;
        }
    #endif

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN) && \
         ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)))
        (void)Cy_CapSense_RunMfsMedian(widgetId, context);
    #endif

    return result;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessCsxWidgetStatus
****************************************************************************//**
*
* Updates the status of the CSX widget in the Data Structure.
*
* This function determines the type of widget and runs the appropriate function
* that implements the status update algorithm for this type of widget.
*
* When the widget-specific processing completes this function updates the
* sensor and widget status registers in the data structure.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessCsxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    switch (ptrWdConfig->wdType)
    {
        #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_BUTTON_EN) ||\
             (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN))
            case (uint8_t)CY_CAPSENSE_WD_BUTTON_E:
            case (uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E:
                Cy_CapSense_DpProcessButton(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_LINEAR_SLIDER_EN)
            case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
                Cy_CapSense_DpProcessSlider(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
            case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
                Cy_CapSense_DpProcessCsxTouchpad(ptrWdConfig);
                break;
        #endif

    default:
        /* Nothing to process since widget type is not valid */
        break;
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessCsdWidgetStatus
****************************************************************************//**
*
* Updates the status of the CSD widget in the Data Structure.
*
* This function determines the type of widget and runs the appropriate function
* that implements the status update algorithm for this type of widget.
*
* When the widget-specific processing completes this function updates the
* sensor and widget status registers in the data structure.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_LLW_BAD_CONFIG   - Liquid level configuration is not valid 
*                                         or calculation fail.
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_DpProcessCsdWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                cy_stc_capsense_context_t * context)
{
    (void)context;
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    switch (ptrWdConfig->wdType)
    {
        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_BUTTON_EN)
            case (uint8_t)CY_CAPSENSE_WD_BUTTON_E:
                Cy_CapSense_DpProcessButton(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_SLIDER_EN)
            case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
            case (uint8_t)CY_CAPSENSE_WD_RADIAL_SLIDER_E:
                Cy_CapSense_DpProcessSlider(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN)
            case (uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E:
                Cy_CapSense_DpProcessCsdMatrix(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN)
            case (uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E:
                status = Cy_CapSense_DpProcessCsdTouchpad(ptrWdConfig, context);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_PROXIMITY_EN)
            case (uint8_t)CY_CAPSENSE_WD_PROXIMITY_E:
                Cy_CapSense_DpProcessProximity(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_EN)
            case (uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E:
                status = Cy_CapSense_DpProcessLiquidLevel(ptrWdConfig, context);
                break;
        #endif

        default:
            /* No other widget types */
            break;
    }

    return status;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessIsxWidgetStatus
****************************************************************************//**
*
* Updates the status of the ISX widget in the Data Structure.
*
* This function determines the type of widget and runs the appropriate function
* that implements the status update algorithm for this type of widget.
*
* When the widget-specific processing completes this function updates the
* sensor and widget status registers in the data structure.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessIsxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    switch (ptrWdConfig->wdType)
    {
        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ISX_BUTTON_EN)
            case (uint8_t)CY_CAPSENSE_WD_BUTTON_E:
                Cy_CapSense_DpProcessButton(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ISX_LINEAR_SLIDER_EN)
            case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
                Cy_CapSense_DpProcessSlider(ptrWdConfig);
                break;
        #endif

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ISX_PROXIMITY_EN)
            case (uint8_t)CY_CAPSENSE_WD_PROXIMITY_E:
                Cy_CapSense_DpProcessProximity(ptrWdConfig);
                break;
        #endif

        default:
            /* No other widget types */
            break;
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessWbxWidgetStatus
****************************************************************************//**
*
* Updates the status of the WBX widget in the Data Structure.
*
* This function determines the type of widget and runs the appropriate function
* that implements the status update algorithm for this type of widget.
*
* When the widget-specific processing completes this function updates the
* sensor and widget status registers in the data structure.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessWbxWidgetStatus(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    switch (ptrWdConfig->wdType)
    {
        case (uint8_t)CY_CAPSENSE_WD_WHEATSTONE_BRIDGE_E:
            Cy_CapSense_DpProcessButton(ptrWdConfig);
            break;

        default:
            /* No other widget types */
            break;
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN) */


/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessSensorRawCountsExt
****************************************************************************//**
*
* Performs customized processing of the sensor raw counts.
*
* If all bits are set at once, the default processing order will take place.
* For a custom order, this function can be called multiple times and execute
* one task at a time.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
* \param ptrSnsContext
* The pointer to the sensor context structure.
*
* \param ptrSnsRawHistory
* The pointer to the filter history.
*
* \param ptrSnsRawHistoryLow
* The pointer to the extended filter history.
*
* \param mode
* The bit-mask with the data processing tasks to be executed.
* The mode parameters can take the following values:
* - CY_CAPSENSE_PROCESS_FILTER     (0x01) Run Firmware Filter
* - CY_CAPSENSE_PROCESS_BASELINE   (0x02) Update Baselines
* - CY_CAPSENSE_PROCESS_DIFFCOUNTS (0x04) Update Difference Counts
* - CY_CAPSENSE_PROCESS_CALC_NOISE (0x08) Calculate the noise
*                   (only in full auto-tuning mode)
* - CY_CAPSENSE_PROCESS_THRESHOLDS (0x10) Update the thresholds
*                   (only in full auto-tuning mode)
* - CY_CAPSENSE_PROCESS_ALL               Execute all tasks
*
* \param ptrSnsBslnInv
* The pointer to the sensor baseline inversion used for BIST if enabled.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the specified sensor processing operation:
* - CY_CAPSENSE_STATUS_SUCCESS if operation was successfully completed;
* - Non-zero - if baseline processing of any
* sensor of the specified widget failed. The result is concatenated with the index
* of failed sensor.
*
*******************************************************************************/
uint32_t Cy_CapSense_DpProcessSensorRawCountsExt(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                cy_stc_capsense_sensor_context_t * ptrSnsContext,
                uint16_t * ptrSnsRawHistory,
                uint8_t * ptrSnsRawHistoryLow,
                uint32_t mode,
                uint16_t * ptrSnsBslnInv,
                const cy_stc_capsense_context_t * context)
{
    uint32_t  result = CY_CAPSENSE_STATUS_SUCCESS;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ACTIVE_FULL_EN)
        cy_stc_capsense_smartsense_csd_noise_envelope_t * ptrNEHistory = ptrWdConfig->ptrNoiseEnvelope;
    #endif

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
        if (0u != (mode & CY_CAPSENSE_PROCESS_FILTER))
        {
            Cy_CapSense_FtRunEnabledFiltersInternal(ptrWdConfig, ptrSnsContext,
                                                    ptrSnsRawHistory, ptrSnsRawHistoryLow);
        }
    #else
        (void)ptrSnsRawHistory;
        (void)ptrSnsRawHistoryLow;
    #endif

    /* Run auto-tuning activities */
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ACTIVE_FULL_EN)
        if ((((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdConfig->wdType) && ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != ptrWdConfig->wdType)) &&
           (((CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_CSD_ACTIVE_FULL_EN)) ||
            ((CY_CAPSENSE_ISX_GROUP == ptrWdConfig->senseMethod) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_ISX_ACTIVE_FULL_EN))))
        {
            Cy_CapSense_RunNoiseEnvelope_Lib(ptrSnsContext->raw, ptrWdCxt->sigPFC, ptrNEHistory);
            Cy_CapSense_DpUpdateThresholds(ptrWdCxt, ptrNEHistory, 0u);
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_PROXIMITY_EN)
                if ((uint8_t)CY_CAPSENSE_WD_PROXIMITY_E == ptrWdConfig->wdType)
                {
                    ptrWdCxt->proxTh = ptrWdCxt->fingerTh;
                    ptrWdCxt->fingerTh = (uint16_t)(((uint32_t)ptrWdCxt->fingerTh *
                        context->ptrCommonConfig->proxTouchCoeff) / CY_CAPSENSE_PERCENTAGE_100);
                }
            #endif
            ptrNEHistory++;
        }
    #endif

    if ((uint8_t)CY_CAPSENSE_WD_LIQUID_LEVEL_E != ptrWdConfig->wdType)
    {
        if (0u != (mode & CY_CAPSENSE_PROCESS_BASELINE))
        {
            result = Cy_CapSense_FtUpdateBaseline(ptrWdCxt, ptrSnsContext, ptrSnsBslnInv, context);
        }
        if (0u != (mode & CY_CAPSENSE_PROCESS_DIFFCOUNTS))
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_REGULAR_RC_CMF_FILTER_EN)
                if (0u != (ptrWdConfig->rawFilterConfig & CY_CAPSENSE_RC_FILTER_CM_EN_MASK))
                {
                    Cy_CapSense_DpUpdateDifferencesCmf(ptrSnsContext);
                }
                else
            #endif
            {
                Cy_CapSense_DpUpdateDifferences(ptrWdCxt, ptrSnsContext);
            }
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: Cy_CapSense_DpUpdateThresholds
****************************************************************************//**
*
* Updates noise and finger thresholds for a specified widget.
*
* Used for smart sensing algorithm.
*
* \param ptrWdContext
* The pointer to the widget context structure.
*
* \param ptrNoiseEnvelope
* The pointer to the noise-envelope history structure.
*
* \param startFlag
* The flag indicates when a new widget is processed.
*
*******************************************************************************/
void Cy_CapSense_DpUpdateThresholds(
                cy_stc_capsense_widget_context_t * ptrWdContext,
                const cy_stc_capsense_smartsense_csd_noise_envelope_t * ptrNoiseEnvelope,
                uint32_t startFlag)
{
    cy_stc_capsense_smartsense_update_thresholds_t thresholds;

    /* Calculate Thresholds */
    thresholds.fingerTh = ptrWdContext->fingerTh;
    Cy_CapSense_UpdateThresholds_Lib(ptrNoiseEnvelope, &thresholds, ptrWdContext->sigPFC, startFlag);

    /* Update CAPSENSE&trade; context */
    ptrWdContext->fingerTh = thresholds.fingerTh;
    ptrWdContext->noiseTh = (uint16_t)thresholds.noiseTh;
    ptrWdContext->nNoiseTh = (uint16_t)thresholds.nNoiseTh;
    ptrWdContext->hysteresis = (uint16_t)thresholds.hysteresis;
}


/*******************************************************************************
* Function Name: Cy_CapSense_DpUpdateDifferences
****************************************************************************//**
*
* Calculates new difference values.
*
* This function calculates the difference between the baseline and raw counts.
* If the difference is positive (raw > baseline), and higher than the widget
* noise threshold, it is saved into the data structure for further processing.
* Otherwise the difference is set to zero. The final difference value is saved
* with the subtracted noise threshold value.
*
* \param ptrWdContext
* The pointer to the widget context structure.
*
* \param ptrSnsContext
* The pointer to the sensor context structure.
*
*******************************************************************************/
void Cy_CapSense_DpUpdateDifferences(
                const cy_stc_capsense_widget_context_t * ptrWdContext,
                cy_stc_capsense_sensor_context_t * ptrSnsContext)
{
    ptrSnsContext->diff = 0u;
    if ((uint32_t)ptrSnsContext->raw > ((uint32_t)ptrSnsContext->bsln + ptrWdContext->noiseTh))
    {
        ptrSnsContext->diff = ptrSnsContext->raw - ptrSnsContext->bsln;
    }
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_REGULAR_RC_CMF_FILTER_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpUpdateDifferencesCmf
****************************************************************************//**
*
* Calculates new difference values.
*
* This function calculates the difference between the baseline and raw counts.
* If the difference is positive (raw > baseline), it is saved into the data
* structure for further processing. Otherwise the difference is set to zero.
*
* \param ptrSnsContext
* The pointer to the sensor context structure.
*
*******************************************************************************/
void Cy_CapSense_DpUpdateDifferencesCmf(
                cy_stc_capsense_sensor_context_t * ptrSnsContext)
{
    ptrSnsContext->diff = 0u;
    if ((uint32_t)ptrSnsContext->raw > (uint32_t)ptrSnsContext->bsln)
    {
        ptrSnsContext->diff = ptrSnsContext->raw - ptrSnsContext->bsln;
    }
}
#endif


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_ProcessWbxCorrection
****************************************************************************//**
*
* Executes Wheatstone bridge raw count correction to compensate 
* sensor-to-sensor variation.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ProcessWbxCorrection(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_widget_config_t * ptrWdCfg = ptrWdConfig;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdCfg->ptrWdContext;

    cy_stc_capsense_wbx_profile_t profile;

    int32_t cCorrZero;
    int32_t cCorrClbr;
    int32_t cCorrRaw;

    profile.rLoad = ptrWdCfg->wbxResistorSeries;
    profile.rBridge = ptrWdCfg->wbxResistorBridge;

    /* Ideal equation: R_REF = 1 / (REF_CDAC * LSB_CDAC * F_MOD)
     * REF_CDAC = [1..255]
     * F_MOD = [1M..46M]
     * LSB_CDAC = 8870*10^-18
     * scale = 1
     * Range = [9.6k to 113M] * scale */
    profile.rRef = CY_CAPSENSE_SCALE_10E9 / CY_CAPSENSE_REF_CDAC_LSB;
    profile.rRef *= CY_CAPSENSE_SCALE_1000;
    profile.rRef /= ptrWdCxt->cdacRef;
    profile.rRef *= context->ptrInternalContext->modClk;
    profile.rRef /= CY_CAPSENSE_MOD_CLOCK_MHZ;

    /* Ideal equation: K_CUT = 2 * COMP_CDAC / REF_CDAC / COMP_DIV
     * COMP_DIV = [3..240]
     * Scale = 1000000
     * Range = [-170 to +170] * scale */
    profile.kCut = (int32_t)CY_CAPSENSE_SCALE_10E6 * 2;
    profile.kCut *= ptrWdCfg->ptrSnsContext->cdacComp;
    profile.kCut /= ((int32_t)ptrWdCxt->cdacCompDivider * ptrWdCxt->cdacRef);
    if (0u == (CY_CAPSENSE_WD_WBX_COMPENSATION_DIRECTION_MASK & ptrWdCfg->ptrWdContext->status))
    {
        /* Adds sign based on direction: Direct = -1 */
        profile.kCut *= -1;
    }

    /* Profile #1: Zero */
    profile.dc = 0;
    Cy_CapSense_WbxCorrectionProfile(&profile);
    cCorrZero = profile.cCorr;

    /* Profile #2: Calibration */
    /* Ideal equation: DC = CALIBRATION_RAW / MAX_RAW
     * Scale = 1000000
     * Range = [0 to 1] * scale */
    profile.dc = (int32_t)ptrWdCxt->maxRawCountRow * CY_CAPSENSE_SCALE_10E6;
    profile.dc /= ptrWdCxt->maxRawCount;
    Cy_CapSense_WbxCorrectionProfile(&profile);

    cCorrClbr = cCorrZero * profile.cCorr;
    cCorrClbr /= CY_CAPSENSE_SCALE_10E4;
    cCorrClbr *= profile.dCorr;
    cCorrClbr /= CY_CAPSENSE_SCALE_10E4;
    cCorrClbr = CY_CAPSENSE_SCALE_10E4 - cCorrClbr;
    cCorrClbr *= (int32_t)ptrWdCxt->maxRawCountRow;
    cCorrClbr /= CY_CAPSENSE_SCALE_10E4;

    /* Profile #3: Rawcounts */
    /* Ideal equation: DC = RAW / MAX_RAW
     * Scale = 1000000
     * Range = [0 to 1] * scale */
    profile.dc = (int32_t)ptrWdCfg->ptrSnsContext->raw * CY_CAPSENSE_SCALE_10E6;
    profile.dc /= ptrWdCxt->maxRawCount;
    Cy_CapSense_WbxCorrectionProfile(&profile);

    cCorrRaw = cCorrZero * profile.cCorr;
    cCorrRaw /= CY_CAPSENSE_SCALE_10E4;
    cCorrRaw *= profile.dCorr;
    cCorrRaw /= CY_CAPSENSE_SCALE_10E4;
    cCorrRaw *= (int32_t)ptrWdCfg->ptrSnsContext->raw;
    cCorrRaw /= CY_CAPSENSE_SCALE_10E4;
    cCorrRaw += cCorrClbr;

    if (0 > cCorrRaw)
    {
        ptrWdCfg->ptrSnsContext->raw = 0u;
    }
    else if ((int32_t)ptrWdCxt->maxRawCount < cCorrRaw)
    {
        ptrWdCfg->ptrSnsContext->raw = ptrWdCxt->maxRawCount;
    }
    else
    {
        ptrWdCfg->ptrSnsContext->raw = (uint16_t)cCorrRaw;
    }
}

/*******************************************************************************
* Function Name: Cy_CapSense_WbxCorrectionProfile
****************************************************************************//**
*
* Executes correction coefficient calculation for Wheatstone bridge widget
* to compensate sensor-to-sensor variation.
*
* The function assumes input arguments already scaled as follows:
* R_REF, R_BRIDGE and R_LOAD without scaling
* K_CUT and DC scaled to 10^6 (means real value is for 1M smaller)
* Results: D_COEFF and C_COEFF scaled to 10^4
*
* \param ptrProfile
* The pointer to the CAPSENSE&trade; structure \ref cy_stc_capsense_wbx_profile_t.
*
*******************************************************************************/

void Cy_CapSense_WbxCorrectionProfile(volatile cy_stc_capsense_wbx_profile_t * ptrProfile)
{
    int32_t bCoeff;
    int32_t bCoeff_den;
    int32_t rxCoeff;
    int32_t rxCoeff_den;
    int32_t cCoeff_den;
    int32_t scale;
    int32_t i;

    /* B_COEFF = R_REF / (K_CUT - DC)
     * Scale = [Run-time defined]
     * Range = [NA] * 10^scale */
    scale = 6;
    bCoeff = ptrProfile->rRef;
    if ((uint32_t)CY_CAPSENSE_HI_LIMIT_NOM > ptrProfile->rRef)
    {
        bCoeff *= CY_CAPSENSE_SCALE_1000;
        scale -= 3;
    }
    if ((uint32_t)CY_CAPSENSE_LO_LIMIT_NOM > ptrProfile->rRef)
    {
        bCoeff *= CY_CAPSENSE_SCALE_10;
        scale--;
    }

    bCoeff_den = ptrProfile->kCut - ptrProfile->dc;
    if ((CY_CAPSENSE_HI_LIMIT_DEN < bCoeff_den) || 
       (-CY_CAPSENSE_HI_LIMIT_DEN > bCoeff_den))
    {
        bCoeff_den /= CY_CAPSENSE_SCALE_1000;
        scale -= 3;
    }
    if ((CY_CAPSENSE_LO_LIMIT_DEN < bCoeff_den) || 
       (-CY_CAPSENSE_LO_LIMIT_DEN > bCoeff_den))
    {
        bCoeff_den /= CY_CAPSENSE_SCALE_10;
        scale--;
    }

    if (0 == bCoeff_den)
    {
        rxCoeff = (int32_t)ptrProfile->rBridge;
    }
    else
    {
        bCoeff /= bCoeff_den;

        /* Ideal equation: RX_COEFF = R_BRIDGE * (B_COEFF - R_LOAD) / (R_BRIDGE + B_COEFF + R_LOAD)
         * Scale = 0
         * Range = [NA] * 10^scale */
        while ((0 > scale) && (CY_CAPSENSE_LO_LIMIT_DEN < bCoeff))
        {
            bCoeff /= CY_CAPSENSE_SCALE_10;
            scale++;
        }
        while ((0 < scale) && ((CY_CAPSENSE_LO_LIMIT_DEN / CY_CAPSENSE_SCALE_10) > bCoeff))
        {
            bCoeff *= CY_CAPSENSE_SCALE_10;
            scale--;
        }
        rxCoeff = (int32_t)ptrProfile->rLoad;
        rxCoeff_den = (int32_t)ptrProfile->rBridge + ptrProfile->rLoad;
        i = scale;
        while (0 < i)
        {
            rxCoeff /= CY_CAPSENSE_SCALE_10;
            rxCoeff_den /= CY_CAPSENSE_SCALE_10;
            i--;
        }
        while (0 > i)
        {
            rxCoeff *= CY_CAPSENSE_SCALE_10;
            rxCoeff_den *= CY_CAPSENSE_SCALE_10;
            i++;
        }
        rxCoeff_den += bCoeff;
        if (0 != rxCoeff_den)
        {
            rxCoeff = bCoeff - rxCoeff;
            scale = 0;
            while ((CY_CAPSENSE_LO_LIMIT < rxCoeff) ||
                  (-CY_CAPSENSE_LO_LIMIT > rxCoeff))
            {
                rxCoeff /= CY_CAPSENSE_SCALE_10;
                scale++;
            }

            rxCoeff *= ptrProfile->rBridge;
            rxCoeff /= rxCoeff_den;
            while (0 < scale)
            {
                rxCoeff *= CY_CAPSENSE_SCALE_10;
                scale--;
            }
        }
        else 
        {
            rxCoeff = -CY_CAPSENSE_SCALE_10E9;
        }
    }

    /* Ideal equation: D_COEFF = R_BRIDGE / RX_COEFF
     * Scale = 10000
     * Range = [0..2] * scale */
    ptrProfile->dCorr = ((int32_t)ptrProfile->rBridge * CY_CAPSENSE_SCALE_10E4) / rxCoeff;

    /* Ideal equation: C_COEFF = (RX_COEFF + R_BRIDGE) / (2 * R_BRIDGE) + (RX_COEFF - R_BRIDGE) / (4 * R_SERIES + 2 * R_BRIDGE)
     * Scale = 10000
     * Range = [0..2] * scale */
    cCoeff_den = (int32_t)ptrProfile->rBridge * 2;
    ptrProfile->cCorr = (rxCoeff + ptrProfile->rBridge);
    ptrProfile->cCorr *= CY_CAPSENSE_SCALE_10E4;
    ptrProfile->cCorr /= cCoeff_den;
    cCoeff_den += ((int32_t)ptrProfile->rLoad * 4);
    ptrProfile->cCorr += (((rxCoeff - ptrProfile->rBridge) * CY_CAPSENSE_SCALE_10E4) / cCoeff_den);
}
#endif


#if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_BUTTON_EN) || \
     (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN) || \
     (CY_CAPSENSE_DISABLE != CY_CAPSENSE_WBX_BUTTON_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessButton
****************************************************************************//**
*
* Processes the status of the Button widget.
*
* This function processes the status of the CSD/CSX/ISX Button widget and
* CSX Matrix Button widget. It applies the hysteresis and debounce algorithm
* to each sensor difference value. This function is expected to be called
* after each new widget scan. If it is called multiple times for the same
* scan data, the debounce algorithm will not work properly.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessButton(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    uint32_t snsIndex;
    uint32_t touchTh;
    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN)
        uint32_t activeCount = 0u;
        uint32_t startIndex = 0u;
    #endif

    uint8_t * ptrDebounceCnt = ptrWdConfig->ptrDebounceArr;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;

    ptrWdCxt->status &= (uint8_t)~CY_CAPSENSE_WD_ACTIVE_MASK;

    /* Go through all widget's sensors */
    for (snsIndex = 0u; snsIndex < ptrWdConfig->numSns; snsIndex++)
    {
        /* Calculate touch threshold based on current sensor state */
        touchTh = (0u == ptrSnsCxt->status) ?
                  ((uint32_t)ptrWdCxt->fingerTh + ptrWdCxt->hysteresis) :
                  ((uint32_t)ptrWdCxt->fingerTh - ptrWdCxt->hysteresis);

        if (0u < (*ptrDebounceCnt))
        {
            /* Decrement debounce counter */
            (*ptrDebounceCnt)--;
        }

        /* No touch */
        if (ptrSnsCxt->diff <= touchTh)
        {
            /* Reset debounce counter */
            *ptrDebounceCnt = ptrWdCxt->onDebounce;
            ptrSnsCxt->status = 0u;
        }

        /* New touch or touch still exists */
        if (0u == (*ptrDebounceCnt))
        {
            ptrSnsCxt->status = CY_CAPSENSE_SNS_TOUCH_STATUS_MASK;
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN)
                activeCount++;
                startIndex = snsIndex;
            #endif
        }

        /* Update widget status */
        if (0u != ptrSnsCxt->status)
        {
            ptrWdCxt->status |= (uint8_t)CY_CAPSENSE_WD_ACTIVE_MASK;
        }

        ptrSnsCxt++;
        ptrDebounceCnt++;
    }
    /* Update position struct */
    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN)
        if (((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdConfig->wdType) &&
            (CY_CAPSENSE_CSX_GROUP == ptrWdConfig->senseMethod))
        {
            ptrWdCxt->wdTouch.numPosition = (uint8_t)activeCount;
            ptrWdCxt->wdTouch.ptrPosition->id = (uint16_t)startIndex;
            ptrWdCxt->wdTouch.ptrPosition->x = (uint16_t)(startIndex / ptrWdConfig->numRows);
            ptrWdCxt->wdTouch.ptrPosition->y = (uint16_t)(startIndex % ptrWdConfig->numRows);
        }
    #endif
}
#endif /* ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_BUTTON_EN) || 
           (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_MATRIX_EN) ||
           (CY_CAPSENSE_DISABLE != CY_CAPSENSE_WBX_BUTTON_EN)) */

#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_PROXIMITY_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessProximity
****************************************************************************//**
*
* Processes the status of the Proximity widget.
*
* This function processes the status of the CSD/ISX Proximity widget. It applies
* the hysteresis and debounce algorithm to each sensor difference value.
* The proximity and touch events are considered independently so debounce and
* hysteresis are also applied independently.
*
* This function is expected to be called after each new widget scan. If it is
* called multiple times for the same scan data the debounce algorithm
* will not work properly.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessProximity(const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    uint32_t snsTh;
    uint32_t snsIndex;
    uint32_t snsStMask;

    uint8_t * ptrDebounceCnt = ptrWdConfig->ptrDebounceArr;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;

    /* Reset widget status */
    ptrWdCxt->status &= (uint8_t)~CY_CAPSENSE_WD_ACTIVE_MASK;

    /* Go through all sensor's status bits */
    for (snsIndex = 0u; snsIndex < ((uint32_t)ptrWdConfig->numSns << 1u); snsIndex++)
    {
        /*
        * Proximity - even; Touch - odd. Example:
        * Bit [1] indicates that a touch is detected.
        * Bit [0] indicates that a proximity is detected.
        */
        snsTh = ptrWdCxt->fingerTh;
        snsStMask = CY_CAPSENSE_SNS_TOUCH_PROX_STATUS_MASK;
        if (0u == (snsIndex & 0x01u))
        {
            snsTh = ptrWdCxt->proxTh;
            snsStMask = CY_CAPSENSE_SNS_PROX_STATUS_MASK;
        }
        /* Calculate threshold based on current sensor state */
        snsTh = (0u == (snsStMask & ptrSnsCxt->status)) ?
                  (snsTh + ptrWdCxt->hysteresis) :
                  (snsTh - ptrWdCxt->hysteresis);

        if (0u < (*ptrDebounceCnt))
        {
            /* Decrement debounce counter */
            (*ptrDebounceCnt)--;
        }

        /* No touch */
        if (ptrSnsCxt->diff <= snsTh)
        {
            /* Reset debounce counter */
            *ptrDebounceCnt = ptrWdCxt->onDebounce;
            ptrSnsCxt->status &= (uint8_t)(~snsStMask);
        }

        /* New touch or touch still exists */
        if (0u == (*ptrDebounceCnt))
        {
            ptrSnsCxt->status |= (uint8_t)snsStMask;
        }

        /* Update widget status */
        if (0u != (ptrSnsCxt->status & (uint8_t)snsStMask))
        {
            ptrWdCxt->status |= (uint8_t)CY_CAPSENSE_WD_ACTIVE_MASK;
        }

        if (0u != (snsIndex & 0x01u))
        {
            ptrSnsCxt++;
        }
        ptrDebounceCnt++;
    }
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_PROXIMITY_EN) */


#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessSlider
****************************************************************************//**
*
* Processes the status of the Linear/Radial Slider widget.
* The process involves running the Linear/Radial centroid algorithm.
*
* It applies the hysteresis and debounce algorithm to the widget ignoring
* the individual states of the sensors.
*
* This function is expected to be called after each new widget scan. If it is
* called multiple times for the same scan data, the debounce algorithm
* will not work properly.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessSlider(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    uint32_t snsIndex;
    uint32_t touchTh;
    uint32_t wdActive = 0u;
    uint8_t * ptrDebounceCnt = ptrWdConfig->ptrDebounceArr;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;
    uint32_t sensorNumber = ptrWdConfig->numSns;
    cy_stc_capsense_position_t newPosition[CY_CAPSENSE_MAX_CENTROIDS];
    cy_stc_capsense_touch_t newTouch = {&newPosition[0u], CY_CAPSENSE_POSITION_NONE};

    /* Calculate touch threshold based on current slider state */
    touchTh = (0u == (ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK)) ?
              ((uint32_t)ptrWdCxt->fingerTh + ptrWdCxt->hysteresis) :
              ((uint32_t)ptrWdCxt->fingerTh - ptrWdCxt->hysteresis);

    if (0u < (*ptrDebounceCnt))
    {
        /* Decrement debounce counter */
        (*ptrDebounceCnt)--;
    }

    /* Check new widget activity */
    for (snsIndex = sensorNumber; snsIndex-- > 0u;)
    {
        ptrSnsCxt->status = (touchTh < ptrSnsCxt->diff) ? CY_CAPSENSE_SNS_TOUCH_STATUS_MASK : 0u;
        wdActive |= ptrSnsCxt->status;
        ptrSnsCxt++;
    }

    /* No touch detected */
    if (0u == wdActive)
    {
        /* Reset debounce counter */
        (*ptrDebounceCnt) = ptrWdCxt->onDebounce;
        ptrWdCxt->status &= (uint8_t)(~CY_CAPSENSE_WD_ACTIVE_MASK);
    }

    if (0u == (*ptrDebounceCnt))
    {
        /* New touch detected or touch still exists from previous cycle */
        ptrWdCxt->status |= CY_CAPSENSE_WD_ACTIVE_MASK;
    }
    else
    {
        if (0u != wdActive)
        {
            /* Clear sensor state if activity was detected but debounce was not passed */
            ptrSnsCxt = ptrWdConfig->ptrSnsContext;

            for (snsIndex = sensorNumber; snsIndex-- > 0u;)
            {
                ptrSnsCxt->status = 0u;
                ptrSnsCxt++;
            }
        }
    }

    /* Centroid processing */
    if (CY_CAPSENSE_WD_ACTIVE_MASK == (ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK))
    {
        switch (ptrWdConfig->wdType)
        {
            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_RADIAL_SLIDER_EN)
                case (uint8_t)CY_CAPSENSE_WD_RADIAL_SLIDER_E:
                    Cy_CapSense_DpCentroidRadial(&newTouch, ptrWdConfig);
                    break;
            #endif

            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_LINEAR_SLIDER_EN)
                case (uint8_t)CY_CAPSENSE_WD_LINEAR_SLIDER_E:
                    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_DIPLEX_SLIDER_EN)
                        if ((0u != (ptrWdConfig->centroidConfig & CY_CAPSENSE_DIPLEXING_MASK)))
                        {
                            /* Run local maximum search for diplexed slider */
                            Cy_CapSense_DpCentroidDiplex(&newTouch, ptrWdConfig);
                        }
                        else
                        {
                            Cy_CapSense_DpCentroidLinear(&newTouch, ptrWdConfig);
                        }
                    #else
                        Cy_CapSense_DpCentroidLinear(&newTouch, ptrWdConfig);
                    #endif
                    break;
            #endif

            default:
                /* No action on other widget types */
                break;
        }
    }

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_POSITION_FILTER_EN)
        /* Position filtering */
        if (0u != (ptrWdConfig->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_MASK))
        {
            Cy_CapSense_ProcessPositionFilters(&newTouch, ptrWdConfig);
        }
    #endif

    /* Copy positions into public structure */
    ptrWdConfig->ptrWdContext->wdTouch.numPosition = newTouch.numPosition;
    for (snsIndex = 0u; snsIndex < newTouch.numPosition; snsIndex++)
    {
        ptrWdConfig->ptrWdContext->wdTouch.ptrPosition[snsIndex] = newTouch.ptrPosition[snsIndex];
    }
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_SLIDER_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessLiquidLevel
****************************************************************************//**
*
* Processes the status of the liquid level widget.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_LLW_BAD_CONFIG   - Liquid level configuration is not valid 
*                                         or calculation fail.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_DpProcessLiquidLevel(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                cy_stc_capsense_context_t * context)
{
    (void) context;
    uint32_t liquidLevel;
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    cy_stc_capsense_position_t newPosition[CY_CAPSENSE_MAX_CENTROIDS];
    cy_stc_capsense_touch_t newTouch = {&newPosition[0u], CY_CAPSENSE_POSITION_ONE};
    uint32_t scalingFactor = (1uL << CY_CAPSENSE_LLW_RESULT_SHIFT);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_FOAM_REJECTION_EN)
        uint16_t foamCorrection;
    #endif

    const void * ptrLlwContext = ptrWdConfig->ptrSnsContext;

    liquidLevel = Cy_CapSense_GetLiquidLevel_Lib(ptrWdConfig->ptrDiplexTable, (const uint16_t *)ptrLlwContext);

    /* Check liquid level validity */
    if (CY_CAPSENSE_LLW_NOT_VALID_DATA != liquidLevel)
    {
        /* Apply resolution scaling */
        liquidLevel = (((liquidLevel * ptrWdConfig->xResolution) + (scalingFactor >> 1u)) >> CY_CAPSENSE_LLW_RESULT_SHIFT);

        newTouch.ptrPosition->x = (uint16_t)liquidLevel;

        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_POSITION_FILTER_EN)
            /* Position filtering */
            if (0u != (ptrWdConfig->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_MASK))
            {
                Cy_CapSense_ProcessPositionFilters(&newTouch, ptrWdConfig);
            }
        #endif

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_FOAM_REJECTION_EN)
            if (0u != (ptrWdConfig->centroidConfig & CY_CAPSENSE_LLW_FOAM_EN_MASK))
            {
                /* Apply level correction factor */
                foamCorrection = 0u;
                if (((ptrWdConfig - 1u)->ptrWdContext->wdTouch.ptrPosition->x) > newTouch.ptrPosition->x)
                {
                    foamCorrection = (ptrWdConfig - 1u)->ptrWdContext->wdTouch.ptrPosition->x - newTouch.ptrPosition->x;
                }
                foamCorrection = (uint16_t)(((uint32_t)foamCorrection * (uint32_t)ptrWdConfig->ptrWdContext->proxTh) >> 8u);

                if (foamCorrection >= newTouch.ptrPosition->x)
                {
                    newTouch.ptrPosition->x = 0u;
                }
                else
                {
                    newTouch.ptrPosition->x = newTouch.ptrPosition->x - foamCorrection;
                }
            } 
        #endif

        ptrWdConfig->ptrWdContext->wdTouch.ptrPosition->x = newTouch.ptrPosition->x;
    }
    else
    {
        status = CY_CAPSENSE_STATUS_LLW_BAD_CONFIG;
    }

    return status;
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LIQUID_LEVEL_EN) */


#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessCsdMatrix
****************************************************************************//**
*
* Processes the status of the CSD Matrix Button widget.
*
* This function processes the status of the CSD Matrix Button widget.
* It applies the hysteresis and debounce algorithm to each sensor value.
*
* Then the function analyzes how many row and column sensors are active.
* If only one per row and one per column, the function considers this as
* a valid touch and updates the corresponding Data Structure registers
* with the button id and active row and column sensors.
*
* If multiple sensors are active in row or column sensors, this function sets
* the corresponding registers to the CY_CAPSENSE_POSITION_MULTIPLE
* value that indicates that it is not possible to detect the touched button id.
*
* This function is expected to be called after each new widget scan. If it is
* called multiple times for the same scan data, the debounce algorithm
* will not work properly.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessCsdMatrix(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    uint32_t snsIndex;
    uint32_t touchTh;

    uint32_t colNumber = ptrWdConfig->numCols;
    uint8_t * ptrDebounceCnt = ptrWdConfig->ptrDebounceArr;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;
    cy_stc_capsense_position_t * ptrSnsTouch = ptrWdCxt->wdTouch.ptrPosition;

    uint32_t numActiveRows = 0u;
    uint32_t numActiveCols = 0u;

    uint32_t activeRow = 0u;
    uint32_t activeCol = 0u;

    /* Reset status */
    ptrWdCxt->status &= (uint8_t)~CY_CAPSENSE_WD_ACTIVE_MASK;

    /* Go through all widget's sensors */
    for (snsIndex = 0u; snsIndex < ptrWdConfig->numSns; snsIndex++)
    {
        /* Calculate touch threshold based on current sensor state */
        touchTh = (0u == ptrSnsCxt->status) ?
                  ((uint32_t)ptrWdCxt->fingerTh + ptrWdCxt->hysteresis) :
                  ((uint32_t)ptrWdCxt->fingerTh - ptrWdCxt->hysteresis);

        if (0u < (*ptrDebounceCnt))
        {
            /* Decrement debounce counter */
            (*ptrDebounceCnt)--;
        }

        /* No touch */
        if (ptrSnsCxt->diff <= touchTh)
        {
            /* Reset debounce counter */
            *ptrDebounceCnt = ptrWdCxt->onDebounce;
            ptrSnsCxt->status = 0u;
        }

        /* New touch or touch still exists */
        if (0u == (*ptrDebounceCnt))
        {
            ptrSnsCxt->status = CY_CAPSENSE_SNS_TOUCH_STATUS_MASK;
        }

        /* Update information about active row/col sensors */
        if (0u != ptrSnsCxt->status)
        {
            if (snsIndex < colNumber)
            {
                numActiveCols++;
                activeCol = snsIndex;
            }
            else
            {
                numActiveRows++;
                activeRow = snsIndex - colNumber;
            }
        }

        ptrSnsCxt++;
        ptrDebounceCnt++;
    }

    /*
    * Number of touches (numActiveCols * numActiveRows):
    * 0 -> No touch
    * 1 -> One touch
    * 2+ -> Multiple touches
    */

    ptrWdCxt->wdTouch.numPosition = (uint8_t)(numActiveCols * numActiveRows);
    if (ptrWdCxt->wdTouch.numPosition > CY_CAPSENSE_POSITION_ONE)
    {
        ptrWdCxt->wdTouch.numPosition = CY_CAPSENSE_POSITION_MULTIPLE;
    }

    if (CY_CAPSENSE_POSITION_ONE == ptrWdCxt->wdTouch.numPosition)
    {
        ptrSnsTouch->x = (uint16_t)activeCol;
        ptrSnsTouch->y = (uint16_t)activeRow;
        ptrSnsTouch->z = 0u;
        ptrSnsTouch->id = (uint16_t)((activeRow * colNumber) + activeCol);
    }

    /* Update widget status if any activity is detected (even non-valid) */
    if ((0u != numActiveRows) || (0u != numActiveCols))
    {
        ptrWdCxt->status |= (uint8_t)CY_CAPSENSE_WD_ACTIVE_MASK;
    }
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) */


#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessCsdTouchpad
****************************************************************************//**
*
* Processes status of the CSD Touchpad widget. This includes running
* Centroid algorithm and updating status in the Data Structure registers.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_CONFIG       - The function does not suppose to be
*                                         called with the current CAPSENSE&trade;
*                                         configuration.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_DpProcessCsdTouchpad(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t snsIndex;
    uint32_t touchTh;
    uint32_t wdActiveCol = 0uL;
    uint32_t wdActiveRow = 0uL;
    uint8_t * ptrDebounceCnt = ptrWdConfig->ptrDebounceArr;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;
    uint32_t sensorNumber = ptrWdConfig->numSns;
    uint32_t colNumber = ptrWdConfig->numCols;
    uint32_t rowNumber = ptrWdConfig->numRows;
    cy_stc_capsense_position_t newPosition[CY_CAPSENSE_MAX_CENTROIDS];
    cy_stc_capsense_touch_t newTouch = {&newPosition[0u], CY_CAPSENSE_POSITION_NONE};
    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_BALLISTIC_MULTIPLIER_EN)
        cy_stc_capsense_ballistic_delta_t delta;
    #endif

    /* Calculate touch threshold based on current sensor state */
    touchTh = (0u == (ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK)) ?
              ((uint32_t)ptrWdCxt->fingerTh + ptrWdCxt->hysteresis) :
              ((uint32_t)ptrWdCxt->fingerTh - ptrWdCxt->hysteresis);

    if (0u < (*ptrDebounceCnt))
    {
        /* Decrement debounce counter */
        (*ptrDebounceCnt)--;
    }

    /* Widget is considered as active if at least one sensor is active on both axes */
    for (snsIndex = colNumber; snsIndex-- > 0uL;)
    {
        ptrSnsCxt->status = (touchTh < ptrSnsCxt->diff) ? CY_CAPSENSE_SNS_TOUCH_STATUS_MASK : 0u;
        wdActiveCol |= ptrSnsCxt->status;
        ptrSnsCxt++;
    }
    for (snsIndex = rowNumber; snsIndex-- > 0uL;)
    {
        ptrSnsCxt->status = (touchTh < ptrSnsCxt->diff) ? CY_CAPSENSE_SNS_TOUCH_STATUS_MASK : 0u;
        wdActiveRow |= ptrSnsCxt->status;
        ptrSnsCxt++;
    }

    /* No touch detected */
    if ((0uL == wdActiveCol) || (0uL == wdActiveRow))
    {
        /* Reset debounce counter */
        (*ptrDebounceCnt) = ptrWdCxt->onDebounce;
        ptrWdCxt->status &= (uint8_t)(~CY_CAPSENSE_WD_ACTIVE_MASK);
    }
    if (0u == (*ptrDebounceCnt))
    {
        /* New touch detected or touch still exists from previous cycle */
        ptrWdCxt->status |= CY_CAPSENSE_WD_ACTIVE_MASK;
    }
    else
    {
        if ((0uL != wdActiveCol) && (0uL != wdActiveRow))
        {
            /* Clear sensor state if activity was detected but debounce was not passed */
            ptrSnsCxt = ptrWdConfig->ptrSnsContext;
            for (snsIndex = sensorNumber; snsIndex-- > 0uL;)
            {
                ptrSnsCxt->status = 0u;
                ptrSnsCxt++;
            }
        }
    }

    /* If widget is still active after debounce run the centroid algorithm */
    if (CY_CAPSENSE_WD_ACTIVE_MASK == (ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK))
    {
        /* 3x3 CSD Touchpad processing */
        if (0u != (ptrWdConfig->centroidConfig & CY_CAPSENSE_CENTROID_3X3_MASK))
        {
            /* Centroid processing */
            Cy_CapSense_DpCentroidTouchpad(&newTouch, ptrWdConfig);
        }
        #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ADVANCED_CENTROID_5X5_EN)
            if ((CY_CAPSENSE_ADV_TOUCHPAD_MIN_SNS_NUMBER > colNumber) || (CY_CAPSENSE_ADV_TOUCHPAD_MIN_SNS_NUMBER > rowNumber))
            {
                /* 2x2 CSD Touchpad does not support advanced centroid algorithm */
                status = CY_CAPSENSE_STATUS_BAD_CONFIG; 
            }
            else 
            {
                /* 5x5 Advanced CSD Touchpad processing */
                if (0u != (ptrWdConfig->centroidConfig & CY_CAPSENSE_CENTROID_5X5_MASK))
                {
                    /* Centroid processing */
                    Cy_CapSense_DpAdvancedCentroidTouchpad(&newTouch, ptrWdConfig);
                }
            }
        #endif
    }

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_POSITION_FILTER_EN)
        /* Position filtering */
        if (0u != (ptrWdConfig->posFilterConfig & CY_CAPSENSE_POSITION_FILTERS_MASK))
        {
            Cy_CapSense_ProcessPositionFilters(&newTouch, ptrWdConfig);
        }
    #endif

    /* Copy positions into public structure */
    ptrWdCxt->wdTouch.numPosition = newTouch.numPosition;
    if (CY_CAPSENSE_POSITION_MULTIPLE != ptrWdCxt->wdTouch.numPosition)
    {
        for (snsIndex = 0u; snsIndex < newTouch.numPosition; snsIndex++)
        {
            ptrWdCxt->wdTouch.ptrPosition[snsIndex] = newTouch.ptrPosition[snsIndex];
        }
    }


    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_BALLISTIC_MULTIPLIER_EN)
        /* Ballistic Multiplier Filtering */
        if (0u != (ptrWdConfig->centroidConfig & CY_CAPSENSE_CENTROID_BALLISTIC_MASK))
        {
            Cy_CapSense_BallisticMultiplier_Lib(&ptrWdConfig->ballisticConfig,
                                                &ptrWdCxt->wdTouch,
                                                &delta,
                                                context->ptrCommonContext->timestamp,
                                                ptrWdConfig->ptrBallisticContext);
            ptrWdCxt->xDelta = delta.deltaX;
            ptrWdCxt->yDelta = delta.deltaY;
        }
    #else
        (void)context;
    #endif

    return status;
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN) */


#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_DpProcessCsxTouchpad
****************************************************************************//**
*
* Processes the status of the CSX Touchpad widget.
* The process involves running the 3x3 centroid algorithm with the
* tracking of finger id.
*
* \param ptrWdConfig
* The pointer to the widget configuration structure.
*
*******************************************************************************/
void Cy_CapSense_DpProcessCsxTouchpad(
                const cy_stc_capsense_widget_config_t * ptrWdConfig)
{
    uint32_t touchTh;
    uint32_t snsIndex;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdConfig->ptrSnsContext;
    cy_stc_capsense_widget_context_t * ptrWdCxt = ptrWdConfig->ptrWdContext;

    /* Calculate touch threshold based on current sensor state */
    touchTh = (0u == (ptrWdCxt->status & CY_CAPSENSE_WD_ACTIVE_MASK)) ?
              ((uint32_t)ptrWdCxt->fingerTh + ptrWdCxt->hysteresis) :
              ((uint32_t)ptrWdCxt->fingerTh - ptrWdCxt->hysteresis);

    for (snsIndex = 0u; snsIndex < ptrWdConfig->numSns; snsIndex++)
    {
        ptrSnsCxt->status = (touchTh < ptrSnsCxt->diff) ? CY_CAPSENSE_SNS_TOUCH_STATUS_MASK : 0u;
        ptrSnsCxt++;
    }

    /* Check whether sensors are active and located all local maxima */
    Cy_CapSense_DpFindLocalMaxDd(ptrWdConfig);
    /* Calculate centroid position for all found local maxima */
    Cy_CapSense_DpCalcTouchPadCentroid(ptrWdConfig);
    /* Identify all touches and assign them correct ID based on historical data */
    Cy_CapSense_DpTouchTracking(ptrWdConfig);
    /* Filter the position data and write it into data structure */
    Cy_CapSense_DpFilterTouchRecord(ptrWdConfig);
}
#endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_TOUCHPAD_EN) */

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) && \
    (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_SCAN_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_RunMfsFiltering
****************************************************************************//**
*
* Selects the median difference signal when the multi-frequency scan is enabled.
*
* \param ptrSnsContext
* The pointer to the widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_RunMfsFiltering(
                cy_stc_capsense_sensor_context_t * ptrSnsContext,
                const cy_stc_capsense_context_t * context)
{
    ptrSnsContext->diff = (uint16_t)Cy_CapSense_FtMedian((uint32_t)ptrSnsContext->diff,
                  (uint32_t)ptrSnsContext[CY_CAPSENSE_MFS_CH1_INDEX * context->ptrCommonConfig->numSns].diff,
                  (uint32_t)ptrSnsContext[CY_CAPSENSE_MFS_CH2_INDEX * context->ptrCommonConfig->numSns].diff);
}
#endif


#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_TX_ENABLED) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED))
/*******************************************************************************
* Function Name: Cy_CapSense_ProcessWidgetMpDeconvolution
****************************************************************************//**
*
* Performs raw count deconvolution for the specified widget:
* * CSD - when multi-phase Self is enabled
* * CSX - when multi-phase Tx is enabled
*
* This function decodes raw counts received after scanning into normal view by
* performing deconvolution algorithm. If the function is called for a widget with
* disabled multi-phase, the function returns CY_CAPSENSE_STATUS_BAD_DATA.
*
* No need to call this function from application layer since the
* Cy_CapSense_ProcessAllWidgets() and Cy_CapSense_ProcessWidget() functions calls
* deconvolution automatically.
*
* DAC auto-calibration when enabled performs sensor auto-calibration without
* performing deconvolution.
* For the CSX widgets, the deconvolution algorithm for even number of TX electrodes
* decreases raw count level twice (keeping the signal on the same level).
*
* If specific processing is implemented using the Cy_CapSense_ProcessWidgetExt()
* and Cy_CapSense_ProcessSensorExt() function then a call of this function is
* required prior doing the specific processing.
*
* \param widgetId
* Specifies the ID number of the widget.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the processing operation.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ProcessWidgetMpDeconvolution(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    uint32_t idx;
    uint32_t sumIdx;
    uint32_t rotIdx;
    uint32_t snsIdx = 0u;
    int32_t localBuf[CY_CAPSENSE_MULTIPHASE_MAX_ORDER];
    int16_t deconvCoefRot[CY_CAPSENSE_MULTIPHASE_MAX_ORDER * 4u];
    int16_t * ptrDeconvTable = &deconvCoefRot[0u];
    int32_t accum;
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_BAD_DATA;
    uint32_t freqChIndex;
    uint32_t mpOrderLocal;
    cy_stc_capsense_mp_table_t * ptrMpTable;
    uint32_t accumTmp;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED)
        int32_t accum_raw[CY_CAPSENSE_MPSC_MAX_ORDER];
        uint32_t maxCount = ptrWdCfg->ptrWdContext->maxRawCount;
        int32_t diff;
        int32_t min_diff;
        uint16_t min_diff_second;
        int32_t sign;
        uint32_t rawDataShift = (uint32_t)((maxCount * context->ptrCommonConfig->csdRawTarget) / CY_CAPSENSE_PERCENTAGE_100);
        int32_t averBsln;
    #endif

    mpOrderLocal = ptrWdCfg->mpOrder;
    ptrMpTable = ptrWdCfg->ptrMpTable;

    if (mpOrderLocal >= CY_CAPSENSE_MPTX_MIN_ORDER)
    {
        result = CY_CAPSENSE_STATUS_SUCCESS;

        (void)memcpy((void *)&deconvCoefRot[0u], (const void *)&ptrMpTable->deconvCoef[0u], mpOrderLocal * CY_CAPSENSE_BYTES_IN_16_BITS);
        (void)memcpy((void *)&deconvCoefRot[mpOrderLocal], (const void *)&ptrMpTable->deconvCoef[0u], mpOrderLocal * CY_CAPSENSE_BYTES_IN_16_BITS);

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED)
            if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
            {
                ptrMpTable++;
                (void)memcpy((void *)&deconvCoefRot[CY_CAPSENSE_MPSC_MAX_ORDER * 2u], (const void *)&ptrMpTable->deconvCoef[0u], ptrWdCfg->mpOrderRows * CY_CAPSENSE_BYTES_IN_16_BITS);
                (void)memcpy((void *)&deconvCoefRot[(CY_CAPSENSE_MPSC_MAX_ORDER * 2u) + ptrWdCfg->mpOrderRows], (const void *)&ptrMpTable->deconvCoef[0u], ptrWdCfg->mpOrderRows * CY_CAPSENSE_BYTES_IN_16_BITS);
            }
        #endif

        for (freqChIndex = 0u; freqChIndex < CY_CAPSENSE_CONFIGURED_FREQ_NUM; freqChIndex++)
        {
            while (snsIdx < ptrWdCfg->numSns)
            {
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED)
                    if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                    {
                        if (snsIdx >= ptrWdCfg->numCols)
                        {
                            ptrDeconvTable = &deconvCoefRot[CY_CAPSENSE_MPSC_MAX_ORDER * 2u];
                            mpOrderLocal = ptrWdCfg->mpOrderRows;
                        }
                    }
                #endif

                /* Copy vector from sensors to localBuf */
                for (idx = 0u; idx < mpOrderLocal; idx++)
                {
                    localBuf[idx] = (int32_t)ptrWdCfg->ptrSnsContext[idx + snsIdx +
                                    (context->ptrCommonConfig->numSns * freqChIndex)].raw;
                }

                /* Multiply vector stored in localBuf by the matrix of deconvolution coefficients */
                idx = 0u;
                while (idx < mpOrderLocal)
                {
                    accum = 0;
                    rotIdx = idx;
                    sumIdx = 0u;
                    while (sumIdx < mpOrderLocal)
                    {
                        accum += localBuf[sumIdx] * ptrDeconvTable[rotIdx];
                        sumIdx++;
                        rotIdx++;
                    }

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED)
                        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                        {
                            sign = (0 > accum) ? -1 : 1;
                            accum *= sign;
                            accumTmp = (uint32_t)accum;

                            /* Shift the result in such a way that guarantees no overflow */
                            accumTmp >>= CY_CAPSENSE_SCALING_SHIFT;

                            /* Store to the temporary signed raw counts buffer */
                            accum_raw[idx+snsIdx] = (int32_t)accumTmp;
                            accum_raw[idx+snsIdx] *= sign;
                        }
                    #endif

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_TX_ENABLED)
                        if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                        {
                            if (0 > accum)
                            {
                                accumTmp = 0u;
                            }
                            else
                            {
                                accumTmp = (uint32_t)accum;

                                /* Shift the result in such a way that guarantees no overflow */
                                accumTmp >>= CY_CAPSENSE_SCALING_SHIFT;

                                if ((uint32_t)UINT16_MAX < accumTmp)
                                {
                                    accumTmp = UINT16_MAX;
                                }
                            }

                            /* Convert the result to unsigned 16 bit and store in the target buffer */
                            ptrWdCfg->ptrSnsContext[idx + snsIdx +
                                            (context->ptrCommonConfig->numSns * freqChIndex)].raw = (uint16_t)accumTmp;
                        }
                    #endif /* CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_TX_ENABLED */

                    idx++;
                }

                 #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_PHASE_SELF_ENABLED)
                    if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                    {
                        min_diff = accum_raw[0u+snsIdx] - (int32_t)ptrWdCfg->ptrSnsContext[0u+snsIdx].bsln;
                        averBsln = (int32_t)ptrWdCfg->ptrSnsContext[0u+snsIdx].bsln;
                        for (idx = 1u; idx < mpOrderLocal; idx++)
                        {
                            /* Find the lowest diff value */
                            diff = accum_raw[idx+snsIdx] - (int32_t)ptrWdCfg->ptrSnsContext[idx+snsIdx].bsln;
                            if (min_diff > diff)
                            {
                                min_diff = diff;
                            }

                            /* Calculate sum of baselines in the pattern */
                            averBsln += (int32_t)ptrWdCfg->ptrSnsContext[idx+snsIdx].bsln;
                        }

                        /* Calculate average baseline */
                        averBsln += (averBsln > 0) ? ((int32_t)mpOrderLocal / 2) : (-(int32_t)mpOrderLocal / 2);
                        averBsln /= (int32_t)mpOrderLocal;

                        /* Calculate total shift */
                        min_diff = (int32_t)rawDataShift - min_diff - averBsln;
                        if (min_diff > (int32_t)CY_CAPSENSE_RAW_COUNT_MAX_VALUE)
                        {
                            min_diff_second = CY_CAPSENSE_RAW_COUNT_MAX_VALUE;
                        } else if (min_diff < 0)
                        {
                            min_diff_second = 0u;
                        }
                        else
                        {
                            min_diff_second = (uint16_t)min_diff;                  
                        }

                        /* Apply aligning algorithm */
                        for (idx = 0u; idx < mpOrderLocal; idx++)
                        {
                            ptrWdCfg->ptrSnsContext[idx+snsIdx].raw = ((uint16_t)accum_raw[idx+snsIdx] + min_diff_second);
                        }
                    }
                 #endif

                snsIdx += mpOrderLocal;
            }
        }
    }
    return result;
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessWidget
****************************************************************************//**
*
* Executes the pre-processing of scan raw data for specified widgets.
*
* This function is called prior any other processing function for
* the fifth CAPSENSE&trade; and  fifth low power CAPSENSE&trade; HW generation.
* The pre-processing routine implements the following operations:
* - Limits raw count to maximum value.
* - Executes the raw data inversion for the CSX ans ISX sensors.
*
* No need to call this function from application layer since the
* Cy_CapSense_ProcessAllWidgets() and Cy_CapSense_ProcessWidget() functions
* calls it automatically.
*
* If specific processing is implemented using the Cy_CapSense_ProcessWidgetExt()
* and Cy_CapSense_ProcessSensorExt() function then a call of this function is
* required prior doing the specific processing. If Multi-phase is enabled
* then deconvolution should be executed after call of this function using
* the Cy_CapSense_ProcessWidgetMpDeconvolution() function.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \note For the fifth-generation low power CAPSENSE&trade; widgets
* of the \ref CY_CAPSENSE_WD_LOW_POWER_E type are not processed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessWidget(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    #if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) ||\
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
        const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    #endif /* Suppress possible compiler warning */

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType)
#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */
    {
        /* Limit raw count to the maximum possible raw count value */
        Cy_CapSense_PreProcessWidgetLimitRaw(widgetId, context);

        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
            /* Inverts raw count for CSX and ISX widgets */
            if ((CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod) ||
                (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod))
            {
                Cy_CapSense_PreProcessWidgetInvertRaw(widgetId, context);
            }
        #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                   (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_WBX_EN)
            if (((uint8_t)CY_CAPSENSE_WD_WHEATSTONE_BRIDGE_E == ptrWdCfg->wdType) &&
                (0u != ptrWdCfg->wbxCorrectionEn) &&
                (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_INITIALIZATION_MASK)))
            {
                Cy_CapSense_ProcessWbxCorrection(ptrWdCfg, context);
            }
        #endif
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessSensor
****************************************************************************//**
*
* Executes the pre-processing of scan raw data for specified sensor.
*
* This function is called prior any other processing function for
* the fifth CAPSENSE&trade; and fifth low power HW generation.
* The pre-processing routine implements the following operations:
* - Limits raw count to maximum value.
* - Executes the raw data inversion for the CSX and ISX sensors.
*
* No need to call this function from application layer since the
* Cy_CapSense_ProcessAllWidgets() and Cy_CapSense_ProcessWidget() functions
* calls it automatically.
*
* If specific processing is implemented using the Cy_CapSense_ProcessWidgetExt()
* and Cy_CapSense_ProcessSensorExt() function then a call of this function is
* required prior doing the specific processing. If Multi-phase is enabled
* then deconvolution should be executed after pre-processing of all sensors
* of the specified widget using the Cy_CapSense_ProcessWidgetMpDeconvolution()
* function.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \note For the fifth-generation low power CAPSENSE&trade; widgets
* of the \ref CY_CAPSENSE_WD_LOW_POWER_E type are not processed.
*
* \param sensorId
* The sensor ID, for which the pre-processing should be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessSensor(
                uint32_t widgetId,
                uint32_t sensorId,
                const cy_stc_capsense_context_t * context)
{
    #if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) ||\
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
        const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    #endif /* Suppress possible compiler warning */

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType)
#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */
    {
        /* Limit raw count to the maximum possible raw count value */
        Cy_CapSense_PreProcessSensorLimitRaw(widgetId, sensorId, context);

        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
            /* Inverts raw count for CSX and ISX widgets */
            if ((CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod) ||
                (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod))
            {
                Cy_CapSense_PreProcessSensorInvertRaw(widgetId, sensorId, context);
            }
        #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
                   (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)) */
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessWidgetLimitRaw
****************************************************************************//**
*
* This internal function limits the raw data for specified
* widgets to the maximum possible raw data for the specified CAPSENSE&trade;
* configuration.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessWidgetLimitRaw(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    uint32_t snsIndex;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = &ptrWdCfg->ptrSnsContext[0u];
    uint32_t maxCount = ptrWdCfg->ptrWdContext->maxRawCount;

    for (snsIndex = 0u; snsIndex < ptrWdCfg->numSns; snsIndex++)
    {
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
            if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) && (ptrWdCfg->numCols <= snsIndex))
            {
                maxCount = ptrWdCfg->ptrWdContext->maxRawCountRow;
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

        if ((uint32_t)ptrSnsCxt->raw > maxCount)
        {
            ptrSnsCxt->raw = (uint16_t)maxCount;
        }
        ptrSnsCxt++;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessSensorLimitRaw
****************************************************************************//**
*
* This internal function limits the raw data for specified
* widgets to the maximum possible raw data for the specified CAPSENSE&trade;
* configuration.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \param sensorId
* The sensor ID, for which the pre-processing should be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessSensorLimitRaw(
                uint32_t widgetId,
                uint32_t sensorId,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = &ptrWdCfg->ptrSnsContext[sensorId];
    uint32_t maxCount = ptrWdCfg->ptrWdContext->maxRawCount;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) && (ptrWdCfg->numCols <= sensorId))
        {
            maxCount = ptrWdCfg->ptrWdContext->maxRawCountRow;
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

    if ((uint32_t)ptrSnsCxt->raw > maxCount)
    {
        ptrSnsCxt->raw = (uint16_t)maxCount;
    }
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessWidgetInvertRaw
****************************************************************************//**
*
* This internal function executes the raw data inversion for specified
* widgets. Raw data is inverted relative to the theoretical MAX value.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessWidgetInvertRaw(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    uint32_t snsIndex;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = &ptrWdCfg->ptrSnsContext[0u];
    uint32_t maxCount = ptrWdCfg->ptrWdContext->maxRawCount;

    for (snsIndex = 0u; snsIndex < ptrWdCfg->numSns; snsIndex++)
    {
        ptrSnsCxt->raw = (uint16_t)(maxCount - ptrSnsCxt->raw);
        ptrSnsCxt++;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_PreProcessSensorInvertRaw
****************************************************************************//**
*
* This internal function executes the raw data inversion for specified
* sensors. The raw data is inverted relative to the theoretical MAX value.
*
* \param widgetId
* The widget ID, for which the pre-processing should be executed.
*
* \param sensorId
* The sensor ID, for which the pre-processing should be executed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_PreProcessSensorInvertRaw(
                uint32_t widgetId,
                uint32_t sensorId,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = &ptrWdCfg->ptrSnsContext[sensorId];
    uint32_t maxCount = ptrWdCfg->ptrWdContext->maxRawCount;

    ptrSnsCxt->raw = (uint16_t)(maxCount - ptrSnsCxt->raw);
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) ||\
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN) && \
     ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)))
/*******************************************************************************
* Function Name: Cy_CapSense_RunMfsMedian
****************************************************************************//**
*
* Applies the median filter to the specified multi-frequency widget and updates
* the specified widget diff counts.
*
* This function is a low-level function and is called automatically by
* high-level processing functions like Cy_CapSense_ProcessWidget()
* and Cy_CapSense_ProcessAllWidgets().
*
* It is not recommended to use this function directly on application level.
*
* The function applies the median filter to diff count of each sensor of the
* specified widget (with enabled multi-frequency feature) and update the diff
* count of the specified main widget.
*
* This function is needed to implement customer-specific use cases.
*
* \note
* This function is available for the fifth-generation CAPSENSE&trade; and
* fifth-generation low power CAPSENSE&trade;.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \note For the fifth-generation low power CAPSENSE&trade; widgets
* of the \ref CY_CAPSENSE_WD_LOW_POWER_E type are not processed and
* \ref CY_CAPSENSE_STATUS_BAD_PARAM is returned
* if a widget of this type is passed.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the widget processing:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is successfully completed
* - CY_CAPSENSE_STATUS_BAD_PARAM     - The input parameter is invalid
* either widgetId is not valid or multi-frequency is not enabled for this widget
* or the specified widgetId is derivative of the main widget.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_RunMfsMedian(
                uint32_t widgetId,
                const cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_BAD_PARAM;

    uint32_t snsIndex;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt0;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt1;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt2;

    /* Check parameter validity */
    if (widgetId < CY_CAPSENSE_TOTAL_WIDGET_COUNT)
    {
        ptrWdCfg = &context->ptrWdConfig[widgetId];

    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType)
    #endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */
        {
            ptrSnsCxt0 = ptrWdCfg[CY_CAPSENSE_MFS_CH0_INDEX].ptrSnsContext;
            ptrSnsCxt1 = ptrWdCfg[CY_CAPSENSE_MFS_CH1_INDEX].ptrSnsContext;
            ptrSnsCxt2 = ptrWdCfg[CY_CAPSENSE_MFS_CH2_INDEX].ptrSnsContext;

            if (((ptrWdCfg->mfsConfig & CY_CAPSENSE_MFS_EN_MASK) != 0u) &&
                ((ptrWdCfg->mfsConfig & CY_CAPSENSE_MFS_WIDGET_FREQ_ALL_CH_MASK) == 0u))
            {
                /* Calculate median */
                for (snsIndex = ptrWdCfg->numSns; snsIndex-- > 0u;)
                {
                    ptrSnsCxt0->diff = (uint16_t)Cy_CapSense_FtMedian(ptrSnsCxt0->diff,
                                                                      ptrSnsCxt1->diff,
                                                                      ptrSnsCxt2->diff);
                    ptrSnsCxt0++;
                    ptrSnsCxt1++;
                    ptrSnsCxt2++;
                }
                result = CY_CAPSENSE_STATUS_SUCCESS;
            }
        }
    }

    return result;
}
#endif

#endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) */

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */


/* [] END OF FILE */
