/***************************************************************************//**
* \file cy_capsense_sensing_v3.c
* \version 8.10.0
*
* \brief
* This file contains the source of functions common for different scanning
* modes.
*
********************************************************************************
* \copyright
* Copyright 2020-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#include <stddef.h>
#include <string.h>
#include "cy_syslib.h"
#include "cy_sysclk.h"
#include "cy_gpio.h"
#include "cy_device_headers.h"
#include "cycfg_capsense_defines.h"
#include "cy_capsense_sensing.h"
#include "cy_capsense_sensing_v3.h"
#include "cy_capsense_generator_v3.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_processing.h"
#include "cy_capsense_lib.h"
#include "cy_capsense_selftest.h"
#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
    #include "cy_msc.h"
    #include "cy_dmac.h"
#endif

#if (defined(CY_IP_M0S8MSCV3))

/*******************************************************************************
* Local definition
*******************************************************************************/
#define CY_CAPSENSE_CSD_CCOMP_CALC_DIV      (4u * CY_CAPSENSE_PERCENTAGE_100)
#define CY_CAPSENSE_CSX_CCOMP_CALC_DIV      (8u * CY_CAPSENSE_PERCENTAGE_100)

/* CIC2 Filter Divider */
#define CY_CAPSENSE_CIC2_DIVIDER_1          (1u)
#define CY_CAPSENSE_CIC2_DIVIDER_2          (2u)
#define CY_CAPSENSE_CIC2_DIVIDER_4          (4u)
#define CY_CAPSENSE_CIC2_DIVIDER_8          (8u)
#define CY_CAPSENSE_CIC2_DIVIDER_16         (16u)

/* CIC2 Accumulator parameters */
#define CY_CAPSENSE_CIC2_ACC_BIT_NUM        (20u)
#define CY_CAPSENSE_CIC2_ACC_MAX_VAL        ((1uL << CY_CAPSENSE_CIC2_ACC_BIT_NUM) - 1u)

/* The minimum allowed value of CDAC compensation divider */
#define CY_CAPSENSE_CDAC_COMP_DIV_MIN_MSCV3 (3u)

/* 
* Raw count value equals to appropriate sensor capacitance with dithering configuration. 
* Calculated according to the equation:
* rawCount = (snsCap * CY_CAPSENSE_CONVERSION_HECTO) / CY_CAPSENSE_REF_CDAC_LSB_X100;
*/
#define CY_CAPSENSE_DITHERING_SNS_CAP_300_FEMTO_RAW     (33u)
#define CY_CAPSENSE_DITHERING_SNS_CAP_500_FEMTO_RAW     (56u)
#define CY_CAPSENSE_DITHERING_SNS_CAP_1000_FEMTO_RAW    (112u)
#define CY_CAPSENSE_DITHERING_SNS_CAP_3000_FEMTO_RAW    (338u)
#define CY_CAPSENSE_DITHERING_SNS_CAP_5000_FEMTO_RAW    (564u)
#define CY_CAPSENSE_DITHERING_SNS_CAP_10000_FEMTO_RAW   (1128u)

/* CDAC dithering modes */
#define CY_CAPSENSE_CDAC_DITHERING_MODE_DISABLE         (0x0u)
#define CY_CAPSENSE_CDAC_DITHERING_MODE_MANUAL          (0x1u)
#define CY_CAPSENSE_CDAC_DITHERING_MODE_AUTO            (0x2u)

#define CY_CAPSENSE_CDAC_DITHERING_CDAC_REF             (100u)
#define CY_CAPSENSE_CDAC_DITHERING_NUM_SUBCONV          (100u)
#define CY_CAPSENSE_CDAC_DITHERING_COMP_DIV             (256u)
#define CY_CAPSENSE_CDAC_DITHERING_KREF                 (256u)
#define CY_CAPSENSE_CDAC_DITHERING_CLOCK_REF_RATE       (0x1u)

/*
* Watchdog time in mod clocks for CDAC auto-dithering scan is calculated according to the equation:
* WdtTime = (INIT_CMOD + INIT_SHORT + PRO_DUMMY + NUM_SUBCONV * KREF + EPILOGUE) * 3.
* The calculated value is the duration of one sample with the three-time margin.
*/
#define CY_CAPSENSE_CDAC_DITHERING_MOD_CLK_PERIODS      (85000u)

/*******************************************************************************
* Constants
*******************************************************************************/
#if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_USE_CAPTURE)
    const cy_stc_msc_base_config_t cy_capsense_mscCfg = CY_CAPSENSE_MSC_CONFIG_DEFAULT;
#endif

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/
uint32_t Cy_CapSense_GetLfsrBitsAuto(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context);

uint32_t Cy_CapSense_GetClkSrcSSCAuto(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context);

uint32_t Cy_CapSense_GetClkSrcPRSAuto(
                const cy_stc_capsense_widget_context_t * ptrWdContext,
                const cy_stc_capsense_context_t * context);

uint32_t Cy_CapSense_GetLfsrBitsNumber(
                uint32_t snsClkDivider,
                uint32_t snsClkDividerMin,
                uint32_t ditherLimitPercents,
                uint32_t lfsrScale);

uint32_t Cy_CapSense_RunSSCAuto(
                uint32_t autoSelMode,
                uint32_t lfsrDitherCycles,
                uint32_t ditherLimitCycles,
                uint32_t lfsrPolySize,
                uint32_t subConvNumber);

uint32_t Cy_CapSense_GetPolySize(uint32_t lfsrPoly);

#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_EN)
void Cy_CapSense_SsAutoTuneScanDurationAlignment(
                uint64_t widgetMask,
                uint32_t clockDivider,
                uint32_t subConversion,
                cy_stc_capsense_context_t * context);
#endif
#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
void Cy_CapSense_SetCompDivider(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
void Cy_CapSense_SetMaxCompCdac(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
#endif
#endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */

#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE))
void Cy_CapSense_AmuxUpdateCdacComp(
                MSC_Type * base,
                const cy_stc_capsense_sensor_context_t * ptrSnsContext);
#endif

void Cy_CapSense_AmuxUpdateScanConfig(
                uint32_t chIndex,
                uint32_t scanSlot,
                uint32_t prevWdIndex,
                cy_stc_capsense_context_t * context);

void Cy_CapSense_AmuxStartScan(MSC_Type * base);
#endif


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN)
static cy_capsense_status_t Cy_CapSense_CdacDitherScaleCalc(
                cy_stc_capsense_context_t * context);
static cy_capsense_status_t Cy_CapSense_ConfigureAutoDitherMode(
                cy_stc_capsense_context_t * context);
static cy_capsense_status_t Cy_CapSense_CdacDitherConfigScan(
                MSC_Type * ptrHwBase,
                uint32_t chId,
                uint32_t slotId,
                uint32_t snsFrameIdx,
                cy_stc_capsense_context_t * context);
#endif


/** \} \endcond */


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSlots_V3
****************************************************************************//**
*
* Initiates the non-blocking scan of specified slots. 
*
* This function initiates a scan only for the first specified slot for all channels
* and then exits. Scans for the remaining slots in the interrupt-driven scan mode
* are initiated
* in the interrupt service routine (part of middleware) triggered at the end
* of each scan completion for each channel. If the syncMode field in the
* cy_stc_capsense_common_config_t structure is set to CY_CAPSENSE_SYNC_MODE_OFF,
* then the next slot scan for the channel with the fired interrupt,
* will start regardless of the another channel readiness for the next scan.
* If the syncMode field is set to CY_CAPSENSE_SYNC_INTERNAL (for single-chip projects)
* or to CY_CAPSENSE_SYNC_EXTERNAL (for multi-chip projects),
* then the next slot scan for the channel with the fired interrupt,
* will start in lockstep with another channels after they all are ready
* for the next scan.
* The scan for the remaining slots in CS-DMA scan mode are initiated
* by DMAC triggered at the end
* of each scan completion for each channel. The channel scan synchronization is
* performed as in Interrupt-driven scan mode. After all slots are scanned,
* the FRAME interrupt is fired and the interrupt service routine (part of middleware)
* updates the busy status.
* To decrease the start scan time when it is intended to scan the same slot,
* i.e. the startSlotId parameter is the same and numberSlots = 1u, then the scan
* is performed without the MSC HW block reconfiguration. Also, in the AMUX mode
* sensors are not disconnected.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* \param startSlotId
* The slot ID scan will be started from.
*
* \param numberSlots
* The number of slots will be scanned.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanSlots_V3(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    uint32_t curChIndex;
    uint32_t startSlotIdLocal = startSlotId;
    uint32_t numberSlotsLocal = numberSlots;
    uint32_t lastSlot = startSlotIdLocal + numberSlotsLocal - 1u;
    const cy_stc_capsense_common_config_t * ptrCommonCfg;
    cy_stc_capsense_internal_context_t * ptrIntrCxt;
    uint32_t slotValue;
    uint32_t curSlotIndex;
    #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
        uint32_t wdIndex;
        uint32_t snsIndex;
        uint32_t scanSlotIndexValid;
    #endif

    #if ((CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) || \
         (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD))
        uint32_t * ptrSensorFrame;
    #endif

    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
        uint32_t sensorFrame[CY_MSC_6_SNS_REGS] = {0u, 0u, 0u, 0u, 0u, 0u};
    #endif

    if ((NULL != context) && (0u != numberSlotsLocal))
    {
        if (CY_CAPSENSE_SLOT_COUNT > lastSlot)
        {
            ptrIntrCxt = context->ptrInternalContext;
            ptrCommonCfg = context->ptrCommonConfig;

            #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                /* Finds the first enabled slot for scanning */
                while (0u < numberSlotsLocal)
                {
                    if (0u != Cy_CapSense_IsSlotEnabled(startSlotIdLocal, context))
                    {
                        break;
                    }
                    startSlotIdLocal++;
                    numberSlotsLocal--;
                }
                /* Finds the last valid slot */
                while (0u < numberSlotsLocal)
                {
                    if (0u != Cy_CapSense_IsSlotEnabled(lastSlot, context))
                    {
                        break;
                    }
                    lastSlot--;
                    numberSlotsLocal--;
                }
            #endif

            #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                if (0u == numberSlotsLocal)
                {
                    capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
                }
                else
            #endif
            if (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(context))
            {
                /* Previous widget is being scanned. Return error. */
                capStatus = CY_CAPSENSE_STATUS_HW_BUSY;
            }
            else
            {
                if ((CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING == ptrIntrCxt->hwConfigState) &&
                    (ptrIntrCxt->currentSlotIndex == startSlotIdLocal) && (1u == numberSlotsLocal) &&
                    (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_CALIBRATION_MASK)) &&
                    (ptrIntrCxt->scanSingleSlot == CY_CAPSENSE_SCAN_SNGL_SLOT))
                {
                    #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                        /* Initiates the frame start for each channel in interrupt driven scan mode */
                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                            ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->FRAME_CMD =
                              MSC_FRAME_CMD_START_FRAME_Msk;
                        }
                    #endif /* (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

                    /* Initialize all enabled MSC channels for scan */
                    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                    {
                        Cy_CapSense_SetBusyFlags(curChIndex, context);

                        if (NULL != ptrIntrCxt->ptrSSCallback)
                        {
                            ptrIntrCxt->ptrSSCallback((cy_stc_capsense_active_scan_sns_t *)&context->ptrActiveScanSns[curChIndex]);
                        }

                        #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                            ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->SNS_CTL = context->ptrInternalContext->snsCtlReg[curChIndex] |
                                    (MSC_SNS_CTL_START_SCAN_Msk | MSC_SNS_CTL_LAST_Msk);
                        #endif

                        #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                            context->ptrSensorFrameContext[((lastSlot + 1u +
                                ((curChIndex + ptrCommonCfg->channelOffset) * CY_CAPSENSE_SLOT_COUNT)) *
                                    CY_MSC_6_SNS_REGS) - 1u] |= MSC_SNS_CTL_LAST_Msk;
                        #endif
                    }

                    #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                        /* Initiates the frame start for each channel in DMA driven scan mode */
                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                          ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->FRAME_CMD =
                              MSC_FRAME_CMD_START_FRAME_Msk;
                        }
                    #endif
                    capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                }
                else
                {
                    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                            if (CY_CAPSENSE_SNS_CONNECTED == context->ptrActiveScanSns[curChIndex].connectedSnsState)
                            {
                                /* Disconnect the scanned sensor */
                                Cy_CapSense_DisconnectSensor(curChIndex, context);
                            }
                        }
                    #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) */

                    capStatus = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
                    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
                    {

                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                            context->ptrActiveScanSns[curChIndex].currentChannelSlotIndex = (uint16_t)startSlotIdLocal;
                        }

                        #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                            /* Finds number of empty slots requested for scan within DMA raw array */
                            slotValue = 0u;
                            for (curSlotIndex = 0u; curSlotIndex < startSlotIdLocal; curSlotIndex++)
                            {
                                if (0u == Cy_CapSense_IsSlotEnabled(curSlotIndex, context))
                                {
                                    slotValue++;
                                }
                            }

                            /* Stores DMA raw array ID */
                            ptrIntrCxt->firstValidSlot = (uint16_t)(startSlotIdLocal - slotValue);

                            /* Finds number of valid slots to be scanned */
                            slotValue = 0u;
                            for (curSlotIndex = startSlotIdLocal; curSlotIndex < (lastSlot + 1u); curSlotIndex++)
                            {
                                if (0u != Cy_CapSense_IsSlotEnabled(curSlotIndex, context))
                                {
                                    slotValue++;
                                }
                            }

                            if ((ptrIntrCxt->currentSlotIndex != (uint16_t)startSlotIdLocal) ||
                               (ptrIntrCxt->endSlotIndex != (uint16_t)lastSlot) ||
                               (ptrIntrCxt->numValidSlots != (uint16_t)slotValue))
                            {
                                ptrIntrCxt->numValidSlots = (uint16_t)slotValue;
                                ptrIntrCxt->currentSlotIndex = (uint16_t)startSlotIdLocal;
                                ptrIntrCxt->endSlotIndex = (uint16_t)lastSlot;
                                /* Configure DMA resources for each channel */
                                for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                                {
                                    (void)Cy_CapSense_ConfigureDmaResource(curChIndex, context);
                                }
                            }
                        #else /* (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

                            ptrIntrCxt->currentSlotIndex = (uint16_t)startSlotIdLocal;
                            ptrIntrCxt->endSlotIndex = (uint16_t)lastSlot;
                            /* Initiates the frame start for each channel in interrupt driven scan mode */
                            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                            {
                                ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->FRAME_CMD =
                                    MSC_FRAME_CMD_START_FRAME_Msk;
                            }
                        #endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

                        /* Set the single or multiple slot scan mode */
                        if ((1u == numberSlotsLocal) &&
                            (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_CALIBRATION_MASK)))
                        {
                            ptrIntrCxt->scanSingleSlot = CY_CAPSENSE_SCAN_SNGL_SLOT;
                        }
                        else
                        {
                            ptrIntrCxt->scanSingleSlot = CY_CAPSENSE_SCAN_MULTIPLE_SLOT;
                        }

                        /* Initialize all enabled MSC channels for scan */
                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                            Cy_CapSense_SetBusyFlags(curChIndex, context);

                            #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                                curSlotIndex = startSlotIdLocal + ((curChIndex + ptrCommonCfg->channelOffset) * CY_CAPSENSE_SLOT_COUNT);
                                slotValue = context->ptrScanSlots[curSlotIndex].wdId;
                                scanSlotIndexValid = curSlotIndex;
                                if (CY_CAPSENSE_SLOT_EMPTY == slotValue)
                                {
                                    #if (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u)
                                        if (curSlotIndex < CY_CAPSENSE_SLOT_COUNT)
                                        {
                                            scanSlotIndexValid += CY_CAPSENSE_SLOT_COUNT;
                                        }
                                        else
                                        {
                                            scanSlotIndexValid -= CY_CAPSENSE_SLOT_COUNT;
                                        }
                                    #endif
                                }
                                else if (CY_CAPSENSE_SLOT_SHIELD_ONLY <= slotValue)
                                {
                                    scanSlotIndexValid = ((uint32_t)context->ptrScanSlots[curSlotIndex].snsId *
                                                          CY_CAPSENSE_SLOT_COUNT) + startSlotIdLocal;
                                }
                                else
                                {
                                    /* Do nothing */
                                }
                                /* Initializes for each channel the active sensor structure for the current sensor */
                                wdIndex = context->ptrScanSlots[scanSlotIndexValid].wdId;
                                snsIndex = context->ptrScanSlots[scanSlotIndexValid].snsId;
                                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                                    (void)Cy_CapSense_ConfigureAnalogMuxResource(curChIndex,
                                                context->ptrWdConfig[wdIndex].senseMethod, context);
                                #endif
                                Cy_CapSense_InitActivePtr(curChIndex, wdIndex, snsIndex, context);
                            #endif /* (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

                            #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                                if ((slotValue != CY_CAPSENSE_SLOT_EMPTY) &&
                                   (slotValue != CY_CAPSENSE_SLOT_SHIELD_ONLY))
                                {
                                    /* Connect the widget first sensor electrodes */
                                    Cy_CapSense_ConnectSensor(curChIndex, context);
                                }
                                /* Getting the sensor frame configuration */
                                ptrSensorFrame = &sensorFrame[0u];
                                (void)Cy_CapSense_GenerateSensorConfig(curChIndex, curSlotIndex, ptrSensorFrame, context);
                                /* Configure the last slot */
                                if (1u == numberSlotsLocal)
                                {
                                    sensorFrame[CY_CAPSENSE_SNS_CTL_INDEX] |= MSC_SNS_CTL_LAST_Msk;
                                }
                            #else /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */
                                /* Configure the last slot for each channel */
                                context->ptrSensorFrameContext[((lastSlot + 1u +
                                    ((curChIndex + ptrCommonCfg->channelOffset) * CY_CAPSENSE_SLOT_COUNT)) *
                                        CY_MSC_6_SNS_REGS) - 1u] |= MSC_SNS_CTL_LAST_Msk;

                                #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                                    ptrSensorFrame = &context->ptrSensorFrameContext[(startSlotIdLocal +
                                            ((curChIndex + ptrCommonCfg->channelOffset) * CY_CAPSENSE_SLOT_COUNT)) *
                                                                                     CY_MSC_6_SNS_REGS];
                                #endif
                            #endif

                            if (NULL != ptrIntrCxt->ptrSSCallback)
                            {
                                #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                                    ptrIntrCxt->ptrSSCallback((cy_stc_capsense_active_scan_sns_t *)&context->ptrActiveScanSns[0u]);
                                #else
                                    ptrIntrCxt->ptrSSCallback(NULL);
                                #endif
                            }

                            #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                                Cy_MSC_ConfigureScan(ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase,
                                                     CY_MSC_6_SNS_REGS, ptrSensorFrame);
                            #endif
                        }

                        #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                            /* Initiates the frame start for each channel in DMA driven scan mode */
                            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                            {
                                ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->FRAME_CMD =
                                    MSC_FRAME_CMD_START_FRAME_Msk;
                            }
                        #endif
                    }
                }
            }
        }
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllWidgets_V3
****************************************************************************//**
*
* Initiates the non-blocking scan for all widgets/sensors. 
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* The function is the wrapper for the Cy_CapSense_ScanAllSlots() function
* to provide the backward compatibility.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAllWidgets_V3(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        capStatus = Cy_CapSense_ScanSlots(0u, CY_CAPSENSE_SLOT_COUNT, context);
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanWidget_V3
****************************************************************************//**
*
* Initiates the scanning of all sensors in the widget. 
*
* The function uses the Cy_CapSense_ScanSlots() function with the parameters of
* startSlotId and numberSlots retrieved from the firstSlotId and numSlots
* fields of the cy_stc_capsense_widget_config_t structure.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
* This function is available in single-channel solution.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanWidget_V3(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        #if (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER)
            if (widgetId < context->ptrCommonConfig->numWd)
            {
                capStatus = Cy_CapSense_ScanSlots(context->ptrWdConfig[widgetId].firstSlotId,
                        context->ptrWdConfig[widgetId].numSlots, context);
            }
        #else
            (void)widgetId;
        #endif /* (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER) */
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSensor_V3
****************************************************************************//**
*
* Initiates the scanning of the selected sensor in the widget.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
* This function is available in single-channel solution.
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
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanSensor_V3(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        #if (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER)
            if (widgetId < context->ptrCommonConfig->numWd)
            {
                if (sensorId < context->ptrWdConfig[widgetId].numSns)
                {
                    capStatus = Cy_CapSense_ScanSlots((sensorId + context->ptrWdConfig->firstSlotId), 1u, context);
                }
            }
        #else
            (void)widgetId;
            (void)sensorId;
        #endif /* (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER) */
    }

    return capStatus;
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllSlots_V3
****************************************************************************//**
*
* Executes CapDAC auto-calibration for all relevant widgets if enabled.
*
* Having enabled the CDAC auto-calibration algorithm is the most common use
* case. It helps to tune your system considering board-to-board variation,
* temperature drift, etc. CDAC auto-calibration is enabled by default.
*
* The function performs searching of Reference CDAC code, Compensation CDAC
* code Compensation Divider (whichever is enabled) by using a successive
* approximation method to make the sensor's raw count closest to the
* defined targets. The auto-calibration target values are defined
* (by default) as:
* * 85% of the maximum raw count for CSD widgets
* * 40% of the maximum raw count for CSX widgets.
*
* To change calibration targets use the Cy_CapSense_SetCalibrationTarget() function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_CALIBRATION_FAIL - The calibration is failed due to
*                                         the issues with scanning (either
*                                         watchdog timer, interrupt breaking, etc.).
* - CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL - The calibration is failed
*                                         because of rawcount is out of the
*                                         defined range.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateAllSlots_V3(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
    const cy_stc_capsense_common_config_t * ptrCommonCfg;
    uint32_t curWdIndex;

    const cy_stc_capsense_widget_config_t * ptrWdCfg;

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE)     || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE))
        uint32_t curSlotIndex;
    #endif

    if (NULL != context)
    {
        context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;

        ptrCommonCfg = context->ptrCommonConfig;

        calibStatus = CY_CAPSENSE_STATUS_SUCCESS;

        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE))
            context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
            /* RefCDAC auto-calibration in single CDAC mode */
            for (curSlotIndex = 0u; curSlotIndex < CY_CAPSENSE_SLOT_COUNT; curSlotIndex++)
            {
                calibStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex,
                        CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR, context);
            }

            /* Finds max RefCDAC and normalizes RefCDAC */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
                for (curWdIndex = 0u; curWdIndex < ptrCommonCfg->numWd; curWdIndex++)
                {
                    if (CY_CAPSENSE_CSD_GROUP == context->ptrWdConfig[curWdIndex].senseMethod)
                    {
                        /* Normalize widgets for which CompCDAC */
                        calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                    }
                }
            #endif
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
                for (curWdIndex = 0u; curWdIndex < ptrCommonCfg->numWd; curWdIndex++)
                {
                    if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[curWdIndex].senseMethod)
                    {
                        /* Normalize widgets for which CompCDAC */
                        calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                    }
                }
            #endif
            context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
        #endif

        /* Skip rest of the stages for SmartSense as not needed */
        if (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK))
        {
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE) || \
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE))
                    /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_DIV_AUTO_USAGE)
                    for (curWdIndex = 0u; curWdIndex < ptrCommonCfg->numWd; curWdIndex++)
                    {
                        if (CY_CAPSENSE_CSD_GROUP == context->ptrWdConfig[curWdIndex].senseMethod)
                        {
                            calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                        }
                    }
                #endif
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_DIV_AUTO_USAGE)
                    for (curWdIndex = 0u; curWdIndex < ptrCommonCfg->numWd; curWdIndex++)
                    {
                        if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[curWdIndex].senseMethod)
                        {
                            calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                        }
                    }
                #endif

                /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3, #4, #7, #8) */
                if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
                {
                    for (curSlotIndex = 0u; curSlotIndex < CY_CAPSENSE_SLOT_COUNT; curSlotIndex++)
                    {
                        /* Calibrate all sensors in slot */
                        calibStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex,
                              CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR, context);
                    }
                }
            #endif

            /* Check calibration result */
            if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
            {
                for (curWdIndex = 0u; curWdIndex < ptrCommonCfg->numWd; curWdIndex++)
                {
                    ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                    switch (ptrWdCfg->senseMethod)
                    {
                        #if (CY_CAPSENSE_CSD_CDAC_CALIBRATION_USAGE)
                            case CY_CAPSENSE_CSD_GROUP:
                        #endif

                        #if (CY_CAPSENSE_CSX_CDAC_CALIBRATION_USAGE)
                            case CY_CAPSENSE_CSX_GROUP:
                        #endif

                            calibStatus |= Cy_CapSense_VerifyCalibration(curWdIndex, context);
                            break;
                        default:
                            /* Skip widget with disabled auto-calibration */
                            break;
                    }
                }
            }

            /* Update CRC if BIST is enabled */
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) &&\
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN))
                Cy_CapSense_UpdateAllWidgetCrc(context);
            #endif
        }

        if (CY_CAPSENSE_STATUS_SUCCESS != calibStatus)
        {
            calibStatus |= CY_CAPSENSE_STATUS_CALIBRATION_FAIL;
        }

        context->ptrCommonContext->status &= ~(uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;
    }

    return calibStatus;
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_SetCalibrationTarget_V3
****************************************************************************//**
*
* Sets the CapDAC auto-calibration raw count targets for CSD, CSX and/or ISX
* widgets.
*
* The function sets the specified raw count targets if CSD, CSX and/or ISX widgets
* are in the project and the auto-calibration is enabled for them. These targets
* will be used instead the configured ones by Cy_CapSense_CalibrateAllSlots(),
* Cy_CapSense_CalibrateAllWidgets() and Cy_CapSense_CalibrateWidget() functions.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* \param calibrTarget
* The raw counts target in percentage for the specified sensing method.
* It should be more than 0u and less than 100u. If the specified target is outside the
* range, then it will not be updated and the CY_CAPSENSE_STATUS_BAD_PARAM status will be
* returned.
*
* \param snsMethod
* Desired sensing method the calibration target should be updated for:
* * CY_CAPSENSE_CSD_GROUP - CSD sensing method
* * CY_CAPSENSE_CSX_GROUP - CSX sensing method
* * CY_CAPSENSE_ISX_GROUP - ISX sensing method
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - At least one of the input parameter is
*                                         invalid.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SetCalibrationTarget_V3(
                uint32_t calibrTarget,
                uint32_t snsMethod,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        if ((CY_CAPSENSE_PERCENTAGE_100 > calibrTarget) && (0u < calibrTarget))
        {
            switch (snsMethod)
            {
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                    case CY_CAPSENSE_CSD_GROUP:
                        context->ptrInternalContext->intrCsdRawTarget = (uint8_t)calibrTarget;
                        break;
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    case CY_CAPSENSE_CSX_GROUP:
                        context->ptrInternalContext->intrCsxRawTarget = (uint8_t)calibrTarget;
                        break;
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                    case CY_CAPSENSE_ISX_GROUP:
                        context->ptrInternalContext->intrIsxRawTarget = (uint8_t)calibrTarget;
                        break;
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */

                default:
                    /* No action */
                    break;
            }
            calibStatus = CY_CAPSENSE_STATUS_SUCCESS;
        }
    }

    return calibStatus;
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllWidgets_V3
****************************************************************************//**
*
* Calibrates CapDACs for all widgets.
*
* The function is the wrapper for the Cy_CapSense_CalibrateAllSlots() function
* to provide the backward compatibility.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
* This function is available in single-channel solution.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM     - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_CALIBRATION_FAIL - The calibration failed if software
*                                         watchdog timeout occurred
*                                         during any calibration scan,
*                                         the scan was not completed, or
*                                         resulted raw counts
*                                         are outside the limits.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateAllWidgets_V3(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        #if (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER)
            calibStatus = Cy_CapSense_CalibrateAllSlots(context);
        #endif /* (1u >= CY_CAPSENSE_TOTAL_CH_NUMBER) */
    }

    return calibStatus;
}
#endif


#if ((1u == CY_CAPSENSE_TOTAL_CH_NUMBER) && \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
      (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateWidget_V3
****************************************************************************//**
*
* Executes the CapDAC calibration for all the sensors in the specified widget
* to the default target value.
*
* This function performs exactly the same tasks as
* Cy_CapSense_CalibrateAllWidgets(), but only for a specified widget.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
* This function is available in single-channel solution.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS   - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_CALIBRATION_FAIL - The calibration is failed due to
*                                  the issues with scanning (either
*                                  watchdog timer, interrupt breaking, etc.).
* - CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL - The calibration is failed
*                                  because of rawcount is out of the
*                                  defined range.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateWidget_V3(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibrationStatus = CY_CAPSENSE_STATUS_SUCCESS;
    cy_stc_capsense_widget_config_t const * ptrWdCfg;
    uint32_t curSlotIndex;

    if (NULL != context)
    {
        context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;

        if (widgetId < context->ptrCommonConfig->numWd)
        {
            ptrWdCfg = &context->ptrWdConfig[widgetId];

            switch (ptrWdCfg->senseMethod)
            {
                #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && (0u != CY_CAPSENSE_CSD_CALIBRATION_EN))
                    case CY_CAPSENSE_CSD_GROUP:

                        /* Calibrate RefCDAC with disabled CompCDAC */
                        #if (0u != CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
                            context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
                            for (curSlotIndex = ptrWdCfg->firstSlotId;
                                 curSlotIndex < ((uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots); curSlotIndex++)
                            {
                                calibrationStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR, context);
                            }
                            calibrationStatus |= Cy_CapSense_NormalizeCdac(widgetId, context);
                            context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
                        #endif

                        /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                        #if (0u != CY_CAPSENSE_CSD_CDAC_COMP_DIV_AUTO_USAGE)
                            calibrationStatus |= Cy_CapSense_CalibrateCompDivider(widgetId, context);
                        #endif

                        /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3 and #7) */
                        #if (0u != CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                            for (curSlotIndex = ptrWdCfg->firstSlotId;
                                 curSlotIndex < ((uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots); curSlotIndex++)
                            {
                                /* Calibrate all sensors in slot */
                                calibrationStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR, context);
                            }
                        #endif

                        break;

                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

                #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) && (0u != CY_CAPSENSE_CSX_CALIBRATION_EN))
                    case CY_CAPSENSE_CSX_GROUP:

                        /* Calibrate RefCDAC with disabled CompCDAC */
                        #if (0u != CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
                            context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
                            for (curSlotIndex = ptrWdCfg->firstSlotId;
                                 curSlotIndex < ((uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots); curSlotIndex++)
                            {
                                calibrationStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR, context);
                            }
                            calibrationStatus |= Cy_CapSense_NormalizeCdac(widgetId, context);
                            context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
                        #endif

                        /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                        #if (0u != CY_CAPSENSE_CSX_CDAC_COMP_DIV_AUTO_USAGE)
                            calibrationStatus |= Cy_CapSense_CalibrateCompDivider(widgetId, context);
                        #endif

                        /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3 and #7) */
                        #if (0u != CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                            for (curSlotIndex = ptrWdCfg->firstSlotId;
                                 curSlotIndex < ((uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots); curSlotIndex++)
                            {
                                /* Calibrate all sensors in slot */
                                calibrationStatus |= Cy_CapSense_CalibrateSlot(curSlotIndex, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR, context);
                            }
                        #endif

                        break;

                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

                default:
                    calibrationStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
                    break;
            }


            /* Check calibration result */
            calibrationStatus |= Cy_CapSense_VerifyCalibration(widgetId, context);

            /* Update CRC if BIST is enabled */
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) &&\
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN))
                Cy_CapSense_UpdateCrcWidget(widgetId, context);
            #endif
        }

        if (CY_CAPSENSE_STATUS_SUCCESS != calibrationStatus)
        {
            calibrationStatus |= CY_CAPSENSE_STATUS_CALIBRATION_FAIL;
        }

        context->ptrCommonContext->status &= ~(uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;
    }

    return calibrationStatus;
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_WatchdogCyclesNum
****************************************************************************//**
*
* Converts the specified time into number of CPU cycles.
*
* \param desiredTimeUs
* The time (delay) specified in us.
*
* \param cpuFreqMHz
* The CPU frequency in MHz.
*
* \param cyclesPerLoop
* The number of cycles per a loop.
*
*******************************************************************************/
uint32_t Cy_CapSense_WatchdogCyclesNum(
                uint32_t desiredTimeUs,
                uint32_t cpuFreqMHz,
                uint32_t cyclesPerLoop)
{
    uint32_t retVal;

    if (0uL != cyclesPerLoop)
    {
        retVal = (desiredTimeUs * cpuFreqMHz) / cyclesPerLoop;
    }
    else
    {
        retVal = 0xFFFFFFFFuL;
    }

    return retVal;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SsInitialize
****************************************************************************//**
*
* Performs the hardware and firmware initialization required for proper operation
* of the CAPSENSE&trade; middleware. This function is called from
* the Cy_CapSense_Init() prior to calling any other function of the middleware.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return status
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SsInitialize(cy_stc_capsense_context_t * context)
{
    uint32_t curChIndex;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    uint32_t i;

    (void)ptrCommonCfg;
    capStatus |= Cy_CapSense_InitializeSourceSenseClk(context);

    /* Initialize all enabled MSC channels for scan */
    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        /* Reset the sense method of all channels */
        context->ptrActiveScanSns[curChIndex].currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
        context->ptrInternalContext->scanSingleSlot = CY_CAPSENSE_SCAN_MULTIPLE_SLOT;

        /* Configure inactive sensor states */
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
            if (context->ptrInternalContext->intrCsdInactSnsConn != ptrCommonCfg->csdInactiveSnsConnection)
            {
                context->ptrInternalContext->intrCsdInactSnsConn = ptrCommonCfg->csdInactiveSnsConnection;
                Cy_CapSense_SetCsdInactiveState(context);
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            if (context->ptrInternalContext->intrCsxInactSnsConn != ptrCommonCfg->csxInactiveSnsConnection)
            {
                context->ptrInternalContext->intrCsxInactSnsConn = ptrCommonCfg->csxInactiveSnsConnection;
                Cy_CapSense_SetCsxInactiveState(context);
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

        /* Generate base frame configurations for all enabled MSC channels */
        capStatus |= Cy_CapSense_GenerateBaseConfig(curChIndex, context);

        if (CY_CAPSENSE_STATUS_SUCCESS != capStatus)
        {
            break;
        }

        /* Generates sensor frame configuration */
        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
            Cy_CapSense_GenerateAllSensorConfig(curChIndex,
            &context->ptrSensorFrameContext[(CY_CAPSENSE_SLOT_COUNT *
                                            (curChIndex + context->ptrCommonConfig->channelOffset)) *
                                            CY_MSC_6_SNS_REGS], context);
        #endif
    }

    /* Assign the ISR for scan */
    context->ptrInternalContext->ptrISRCallback = &Cy_CapSense_ScanISR;

    /* Call user's callback function if it is registered */
    if (NULL != context->ptrInternalContext->ptrEODsInitCallback)
    {
        context->ptrInternalContext->ptrEODsInitCallback(context);
    }

    /* Resetting the HW configuration in order to trigger the base frame initialization in scope of the
     * next  Cy_CapSense_SwitchHwConfiguration() function call with the desired HW configuration.
     */
    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
    {
        capStatus |= Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_CAPTURED_DEFAULT, context);
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
    {
        capStatus |= Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
    }

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN)
        if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
        {
            capStatus = Cy_CapSense_CdacDitherScaleCalc(context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN) */

    /* Find maximum raw count for each widget */
    for (i = 0u; i < context->ptrCommonConfig->numWd; i++)
    {
        capStatus |= Cy_CapSense_InitializeMaxRaw(i, context);
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_InterruptHandler_V3
****************************************************************************//**
*
* Implements interrupt service routine for CAPSENSE&trade; Middleware.
*
* The MSC HW block generates an interrupt at end of every sensor scan.
* The CAPSENSE&trade; middleware uses this interrupt to implement a
* non-blocking sensor scan method, in which only the first sensor scan is
* initiated by the application program and subsequent sensor scans are
* initiated in the interrupt service routine as soon as the current scan
* is completed. The above stated interrupt service routine is implemented
* as a part of the CAPSENSE&trade; middleware.
*
* The CAPSENSE&trade; middleware does not initialize or modify the priority
* of interrupts. For the operation of middleware, the application program
* must configure MSC interrupt and assign interrupt vector to
* the Cy_CapSense_InterruptHandler() function. Refer to function
* usage example for details.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* The calls of the Start Sample and End Of Scan callbacks
* (see the \ref group_capsense_callbacks section for details) are the part of the
* Cy_CapSense_InterruptHandler() routine and they lengthen its execution. These
* callbacks will lengthen the MSC ISR execution in case of a direct call of the
* Cy_CapSense_InterruptHandler() function from a MSC ISR.
*
* \param base
* The pointer to the base register address of the MSC HW block.
* This argument is kept for uniformity and backward compatibility
* and is not used. The function can be called with value NULL.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \funcusage
*
* An example of the ISR initialization:
*
* The CapSense_ISR_cfg variable should be declared by the application
* program according to the examples below:<br>
* For Core CM0+:
* \snippet capsense/snippet/main.c snippet_m0p_capsense_interrupt_source_declaration
*
* The CAPSENSE&trade; interrupt handler should be defined by the application program
* according to the example below:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_IntHandler
*
* Then, the application program configures and enables the interrupt
* for each MSC HW block between calls of the Cy_CapSense_Init() and
* Cy_CapSense_Enable() functions:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Initialization
*
*******************************************************************************/
void Cy_CapSense_InterruptHandler_V3(const MSC_Type * base, cy_stc_capsense_context_t * context)
{
    (void)base;
    context->ptrInternalContext->ptrISRCallback((void *)context);
}

#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
/*******************************************************************************
* Function Name: Cy_CapSense_SetPinState_V3
****************************************************************************//**
*
* The internal function to set the state (drive mode and HSIOM state) of the GPIO used
* by a sensor.
*
* See the Cy_CapSense_SetPinState() description for details.

* \note
* This function is available only for the fifth-generation CAPSENSE&trade; when
* Sensor Connection Method = AMUX mode.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param sensorElement
* Specifies the ID of the sensor element within the widget to change
* its pin state.
* * For the CSD widgets use the sensor ID. A macro for the
*   sensor ID within a specified widget can be found in the cycfg_capsense.h
*   file defined as CY_CAPSENSE_<WIDGET_NAME>_SNS<SENSOR_NUMBER>_ID.
* * For the CSX widgets use either Rx ID or Tx ID.
*   The first Rx in a widget corresponds to sensorElement = 0; the second
*   Rx in a widget corresponds to sensorElement = 1, and so on.
*   The last Tx in a widget corresponds to sensorElement = (RxNum + TxNum - 1).
*   A macro for the Rx ID or Tx ID can be found in the cycfg_capsense.h
*   file defined as CY_CAPSENSE_<WIDGET_NAME>_<TX/RX><TX/RX_NUMBER>_ID.
*
* \param state
* Specifies the state of the sensor to be set:
* 1. CY_CAPSENSE_GROUND          - The pin is connected to the ground.
* 2. CY_CAPSENSE_HIGHZ           - The drive mode of the pin is set to High-Z
*                                  Analog.
* 3. CY_CAPSENSE_SHIELD          - The shield signal is routed to the pin
*                                  (available only if CSD sensing method with
*                                  shield electrode is enabled).
* 4. CY_CAPSENSE_SENSOR          - The pin is connected to the scanning bus
*                                  (available only if CSD sensing method
*                                   is enabled).
* 5. CY_CAPSENSE_TX_PIN          - The Tx signal is routed to the sensor
*                                  (available only if CSX sensing method
*                                   is enabled).
* 6. CY_CAPSENSE_RX_PIN          - The pin is connected to the scanning bus
*                                  (available only if CSX sensing method
*                                   is enabled).
* 7. CY_CAPSENSE_NEGATIVE_TX_PIN - The Negative Tx signal is routed to the sensor
*                                  (available only if CSD sensing method
*                                   is enabled).
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return status
* Returns the operation status:
* - CY_CAPSENSE_STATUS_SUCCESS   - Indicates the successful electrode setting.
* - CY_CAPSENSE_STATUS_BAD_PARAM - 1) widgetID, sensorElement or state
*                                     are not valid;
*                      2) the CSD sensing method is disabled for desired
*                         CY_CAPSENSE_SHIELD or CY_CAPSENSE_SENSOR states;
*                      3) the CSX sensing method is disabled for desired
*                         CY_CAPSENSE_TX_PIN, CY_CAPSENSE_NEGATIVE_TX_PIN or
*                         CY_CAPSENSE_RX_PIN states.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SetPinState_V3(
                uint32_t widgetId,
                uint32_t sensorElement,
                uint32_t state,
                const cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t connState = CY_CAPSENSE_STATUS_BAD_PARAM;
    uint32_t eltdNum;
    uint32_t eltdIndex;
    uint32_t interruptState;
    cy_stc_capsense_pin_config_t const * ioPtr;
    cy_stc_capsense_widget_config_t const * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_electrode_config_t const * ptrSnsEltdConfig;
    if (context->ptrCommonConfig->numWd > widgetId)
    {
        /* Get number of electrodes within the widget */
        eltdNum = (uint32_t)ptrWdCfg->numCols + ptrWdCfg->numRows;

        if (eltdNum > sensorElement)
        {
            /* Set the pointer to the specified sensor element configuration structure */
            ptrSnsEltdConfig = &ptrWdCfg->ptrEltdConfig[sensorElement];
            /* Get number of electrode's pins and a pointer to the electrode's pin structure */
            eltdNum = ptrSnsEltdConfig->numPins;
            ioPtr = ptrSnsEltdConfig->ptrPin;
            /* Loop through all pins of the specified sensor electrode */
            for (eltdIndex = 0u; eltdIndex < eltdNum; eltdIndex++)
            {
                interruptState = Cy_SysLib_EnterCriticalSection();
                Cy_GPIO_Clr(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber);
                Cy_SysLib_ExitCriticalSection(interruptState);

                switch (state)
                {
                    case CY_CAPSENSE_GROUND:
                        Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_GPIO);
                        connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;

                    case CY_CAPSENSE_HIGHZ:
                        Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO);
                        connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                    case CY_CAPSENSE_SENSOR:
                            Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                    CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_CSD_SENSE);
                            connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                    case CY_CAPSENSE_SHIELD:
                            if (CY_CAPSENSE_SHIELD_ACTIVE == context->ptrCommonConfig->csdShieldMode)
                            {
                                Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                        CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD);
                            }
                            else
                            {
                                Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                        CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD);
                            }
                            connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;
                    #endif
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */


                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    case CY_CAPSENSE_TX_PIN:
                            Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                    CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_CSD_SENSE);
                            connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;

                    case CY_CAPSENSE_RX_PIN:
                            Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                    CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA);
                            connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;

                    case CY_CAPSENSE_NEGATIVE_TX_PIN:
                            Cy_CapSense_SsConfigPinRegisters(ioPtr->pcPtr, (uint32_t)ioPtr->pinNumber,
                                    CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD);
                            connState = CY_CAPSENSE_STATUS_SUCCESS;
                        break;
                #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

                    default:
                        /* No action on other sensor states */
                        break;
                }

                ioPtr++;
            }
        }
    }
    return connState;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ConnectSensor
****************************************************************************//**
*
* Connects a sensor to the specified channel MSC HW block.
*
* The function checks the widget type and performs connection of all sensor
* electrodes (sns for CSD widgets and Tx/Rx for CSX ones) including ganged.
* The sensor and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ConnectSensor(
                uint32_t chIndex,
                cy_stc_capsense_context_t * context)
{
    uint32_t snsMethod = context->ptrActiveScanSns[chIndex].currentSenseMethod;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        /* Connect all current sensor's electrodes for scan */
        if (CY_CAPSENSE_CSD_GROUP == snsMethod)
        {
            /* Connect all pins of the current sensor */
            Cy_CapSense_CsdConnectSns(chIndex, context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        if (CY_CAPSENSE_CSX_GROUP == snsMethod)
        {
            /* Connect current sensor's Tx and Rx IOs for scan
            * and set flag to indicate that IOs should be disconnected */
            Cy_CapSense_CsxConnectTx(chIndex, context);
            Cy_CapSense_CsxConnectRx(chIndex, context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

    context->ptrActiveScanSns[chIndex].connectedSnsState = CY_CAPSENSE_SNS_CONNECTED;
}


/*******************************************************************************
* Function Name: Cy_CapSense_DisconnectSensor
****************************************************************************//**
*
* Disconnects a sensor from the specified channel MSC HW block.
*
* The function checks the widget type and performs disconnection of all sensor
* electrodes (sns for CSD widgets and Tx/Rx for CSX ones) including ganged.
* The sensor and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_DisconnectSensor(
                uint32_t chIndex,
                cy_stc_capsense_context_t * context)
{
    uint32_t snsMethod = context->ptrActiveScanSns[chIndex].currentSenseMethod;

    /* Disconnect all current sensor's electrodes */
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        if (CY_CAPSENSE_CSD_GROUP == snsMethod)
        {
            /* Disconnect all pins of the current sensor */
            Cy_CapSense_CsdDisconnectSns(chIndex, context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        if (CY_CAPSENSE_CSX_GROUP == snsMethod)
        {
            /* Disconnect current sensor's Tx and Rx IOs for scan
            * and set flag to indicate that IOs should be disconnected */
            Cy_CapSense_CsxDisconnectTx(chIndex, context);
            Cy_CapSense_CsxDisconnectRx(chIndex, context);
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

    context->ptrActiveScanSns[chIndex].connectedSnsState = CY_CAPSENSE_SNS_DISCONNECTED;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CsdConnectSns
****************************************************************************//**
*
* Connects a CSD sensor to the specified channel MSC HW block.
*
* The function connects all the sensor pins including ganged.
* The sensor and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsdConnectSns(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_electrode_config_t * eltdPinCfg =
                          context->ptrActiveScanSns[chIndex].ptrEltdConfig;
    const cy_stc_capsense_pin_config_t * ptrActivePin = eltdPinCfg->ptrPin;
    uint32_t i;

    if ((chIndex + context->ptrCommonConfig->channelOffset) == eltdPinCfg->chId)
    {
        /* Connect all pins of current sensors */
        for (i = 0u; i < eltdPinCfg->numPins; i++)
        {
            Cy_CapSense_SsConfigPinRegisters(ptrActivePin->pcPtr, (uint32_t)ptrActivePin->pinNumber,
                    CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_CSD_SENSE);
            ptrActivePin++;
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_CsdDisconnectSns
****************************************************************************//**
*
* Disconnects a CSD sensor from the specified channel MSC HW block.
*
* The function disconnects all the sensor pins including ganged.
* The sensor and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsdDisconnectSns(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_electrode_config_t * eltdPinCfg =
                          context->ptrActiveScanSns[chIndex].ptrEltdConfig;
    const cy_stc_capsense_pin_config_t * ptrActivePin = eltdPinCfg->ptrPin;

    uint32_t i;

    if ((chIndex + context->ptrCommonConfig->channelOffset) == eltdPinCfg->chId)
    {
        /* Disconnect all pins of the current sensor */
        for (i = 0u; i < eltdPinCfg->numPins; i++)
        {
            Cy_CapSense_SsConfigPinRegisters(ptrActivePin->pcPtr, (uint32_t)ptrActivePin->pinNumber,
                                             context->ptrInternalContext->csdInactiveSnsDm,
                                             context->ptrInternalContext->csdInactiveSnsHsiom);
            Cy_GPIO_Clr(ptrActivePin->pcPtr, (uint32_t)ptrActivePin->pinNumber);
            ptrActivePin++;
        }
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CsxConnectTx
****************************************************************************//**
*
* Connects a CSX sensor Tx electrode to the specified channel MSC HW block.
*
* The function connects all the Tx electrode pins including ganged.
* The electrode and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsxConnectTx(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    uint32_t pinIndex;
    en_hsiom_sel_t hsiomState = CY_CAPSENSE_HSIOM_SEL_CSD_SENSE;
    const cy_stc_capsense_electrode_config_t * ptrEltd;
    const cy_stc_capsense_active_scan_sns_t * ptrActive = &context->ptrActiveScanSns[chIndex];
    const cy_stc_capsense_pin_config_t * pinPointer;

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_PHASE_TX_ENABLED)
        uint32_t i;
        uint32_t pattern;
        uint32_t eltdMptxOrder;
        cy_stc_capsense_widget_config_t const * ptrWdCfg = &context->ptrWdConfig[ptrActive->widgetIndex];
        uint32_t mptxOrd = ptrWdCfg->mpOrder;
    #endif

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_PHASE_TX_ENABLED)
        if (CY_CAPSENSE_MPTX_MIN_ORDER > mptxOrd)
        {
            ptrEltd = ptrActive->ptrTxConfig;
            if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
            {
                pinPointer = ptrActive->ptrTxConfig->ptrPin;
                for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
                {
                    Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                            CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, hsiomState);
                    pinPointer++;
                }
            }
        }
        else
        {
            /* Finds the index of the first TX electrode in the mptx group */
            eltdMptxOrder = ptrActive->sensorIndex % mptxOrd;
            i = ptrActive->sensorIndex - eltdMptxOrder;
            i = ptrWdCfg->numCols + (i % ptrWdCfg->numRows);
            ptrEltd = &ptrWdCfg->ptrEltdConfig[i];

            /* Finding the right vector / pattern for mptx operation */
            pattern = ptrWdCfg->ptrMpTable->vector;
            if (0u != eltdMptxOrder)
            {
                pattern = (pattern >> eltdMptxOrder) | (pattern << (mptxOrd - eltdMptxOrder));
            }

            if (CY_CAPSENSE_MPTX_MAX_ORDER > mptxOrd)
            {
                pattern &= (0x01uL << mptxOrd) - 1u;
            }

            /* Loop through all involved mptx TX electrodes, positive and negative */
            for (i = 0u; i < mptxOrd; i++)
            {
                if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
                {
                    hsiomState = CY_CAPSENSE_HSIOM_SEL_CSD_SENSE;
                    if (0u == (pattern & 0x01u))
                    {
                        hsiomState = CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD;
                    }
                    pinPointer = ptrEltd->ptrPin;
                    /* Loop through all pads for this electrode (ganged sensor) */
                    for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
                    {
                        Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                                CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, hsiomState);
                        pinPointer++;
                    }
                }
                pattern >>= 0x01u;
                ptrEltd++;
            }
        }
    #else
        ptrEltd = ptrActive->ptrTxConfig;
        if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
        {
            pinPointer = ptrActive->ptrTxConfig->ptrPin;
            for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
            {
                Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                        CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, hsiomState);
                pinPointer++;
            }
        }
    #endif
}


/*******************************************************************************
* Function Name: Cy_CapSense_CsxConnectRx
****************************************************************************//**
*
* Connects a CSX sensor Rx electrode to the specified channel MSC HW block.
*
* The function connects all the Rx electrode pins including ganged.
* The electrode and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsxConnectRx(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    uint32_t pinIndex;
    const cy_stc_capsense_pin_config_t * pinPointer =
                context->ptrActiveScanSns[chIndex].ptrRxConfig->ptrPin;

    if ((chIndex + context->ptrCommonConfig->channelOffset) == context->ptrActiveScanSns[chIndex].ptrRxConfig->chId)
    {
        for (pinIndex = context->ptrActiveScanSns[chIndex].ptrRxConfig->numPins;
             pinIndex-- > 0u;)
        {
            Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                    CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA);
            pinPointer++;
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_CsxDisconnectTx
****************************************************************************//**
*
* Disconnects a CSX sensor Tx electrode from the specified channel MSC HW block.
*
* The function disconnects all the Tx electrode pins including ganged.
* The electrode and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsxDisconnectTx(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    uint32_t pinIndex;
    const cy_stc_capsense_electrode_config_t * ptrEltd;
    const cy_stc_capsense_active_scan_sns_t * ptrActive = &context->ptrActiveScanSns[chIndex];
    const cy_stc_capsense_pin_config_t * pinPointer = ptrActive->ptrTxConfig->ptrPin;

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_PHASE_TX_ENABLED)
        uint32_t i;
        uint32_t eltdMptxOrder;
        cy_stc_capsense_widget_config_t const * ptrWdCfg = &context->ptrWdConfig[ptrActive->widgetIndex];
        uint32_t mptxOrd = ptrWdCfg->mpOrder;
    #endif

    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_PHASE_TX_ENABLED)
        if (CY_CAPSENSE_MPTX_MIN_ORDER > mptxOrd)
        {
            ptrEltd = ptrActive->ptrTxConfig;
            if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
            {
                for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
                {
                    Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                                                     context->ptrInternalContext->csxInactiveSnsDm,
                                                     context->ptrInternalContext->csxInactiveSnsHsiom);
                    Cy_GPIO_Clr(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber);
                    pinPointer++;
                }
            }
        }
        else
        {
            /* Finds the index of the first TX electrode in the mptx group */
            eltdMptxOrder = ptrActive->sensorIndex % mptxOrd;
            i = ptrActive->sensorIndex - eltdMptxOrder;
            i = ptrWdCfg->numCols + (i % ptrWdCfg->numRows);
            ptrEltd = &ptrWdCfg->ptrEltdConfig[i];

            /* Loop through all involved mptx TX electrodes, positive and negative */
            for (i = 0u; i < mptxOrd; i++)
            {
                if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
                {
                    pinPointer = ptrEltd->ptrPin;
                    /* Loop through all pads for this electrode (ganged sensor) */
                    for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
                    {
                        Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                                                         context->ptrInternalContext->csxInactiveSnsDm,
                                                         context->ptrInternalContext->csxInactiveSnsHsiom);
                        Cy_GPIO_Clr(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber);
                        pinPointer++;
                    }
                }
                ptrEltd++;
            }
        }
    #else
        ptrEltd = ptrActive->ptrTxConfig;
        if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrEltd->chId)
        {
            for (pinIndex = ptrEltd->numPins; pinIndex-- > 0u;)
            {
                Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                                                 context->ptrInternalContext->csxInactiveSnsDm,
                                                 context->ptrInternalContext->csxInactiveSnsHsiom);
                Cy_GPIO_Clr(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber);
                pinPointer++;
            }
        }
    #endif
}


/*******************************************************************************
* Function Name: Cy_CapSense_CsxDisconnectRx
****************************************************************************//**
*
* Disconnects a CSX sensor Rx electrode from the specified channel MSC HW block.
*
* The function disconnects all the Rx electrode pins including ganged.
* The electrode and the current slot are specified in the cy_stc_capsense_active_scan_sns_t structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_CsxDisconnectRx(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    uint32_t pinIndex;
    const cy_stc_capsense_pin_config_t * pinPointer =
                context->ptrActiveScanSns[chIndex].ptrRxConfig->ptrPin;

    if ((chIndex + context->ptrCommonConfig->channelOffset) == context->ptrActiveScanSns[chIndex].ptrRxConfig->chId)
    {
        for (pinIndex = context->ptrActiveScanSns[chIndex].ptrRxConfig->numPins;
             pinIndex-- > 0u;)
        {
            Cy_CapSense_SsConfigPinRegisters(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber,
                                             context->ptrInternalContext->csxInactiveSnsDm,
                                             context->ptrInternalContext->csxInactiveSnsHsiom);
            Cy_GPIO_Clr(pinPointer->pcPtr, (uint32_t)pinPointer->pinNumber);
            pinPointer++;
        }
    }
}
#endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */
#endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) */


/*******************************************************************************
* Function Name: Cy_CapSense_ScanISR
****************************************************************************//**
*
* This is an internal ISR function to handle the MSC sensing method operation.
*
* \param capsenseContext
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ScanISR(void * capsenseContext)
{
    uint32_t intrMask;
    uint32_t i;
    MSC_Type * ptrMscHwBase;

    #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
        uint32_t tmpRawCount;
        uint32_t slotValue;
        uint32_t wdIndex;

        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
            uint32_t prevWdIndex;
        #else
            uint32_t * ptrSensorFrame;
        #endif
    #endif

    cy_stc_capsense_context_t * cxt = (cy_stc_capsense_context_t *)capsenseContext;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = cxt->ptrCommonConfig;

    #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
        uint32_t scanSlotIndexValid;
        cy_stc_capsense_active_scan_sns_t * ptrActive;
        cy_stc_capsense_internal_context_t * ptrIntrCxt = cxt->ptrInternalContext;
        uint32_t slotStcIndex;
    #endif /* (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) */
    uint32_t curChIndex = (uint32_t)ptrCommonCfg->channelOffset + CY_CAPSENSE_TOTAL_CH_NUMBER;

    /* Define the interrupt scope */
    #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
        intrMask = MSC_INTR_MASK_FRAME_Msk;
    #else
        intrMask = MSC_INTR_MASK_SCAN_Msk;
    #endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

    /* Check which channel have fired the interrupt */
    for (i = 0u; i < CY_CAPSENSE_TOTAL_CH_NUMBER; i++)
    {
        if (0u != (ptrCommonCfg->ptrChConfig[i].ptrHwBase->INTR & intrMask))
        {
            curChIndex = i;
            break;
        }
    }

    ptrMscHwBase = ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase;

    if (curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER)
    {
        /* Clear all pending interrupts of the MSC HW block */
        ptrMscHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
        (void)ptrMscHwBase->INTR;

        #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
            slotStcIndex = ((curChIndex + ptrCommonCfg->channelOffset) * CY_CAPSENSE_SLOT_COUNT) + ptrIntrCxt->currentSlotIndex;
            /* Set the active scan pointer to the current channel */
            ptrActive = &(cxt->ptrActiveScanSns[curChIndex]);

            slotValue = cxt->ptrScanSlots[slotStcIndex].wdId;
            /* Skips raw count update for empty slots or disabled/non-working widgets */
            if ((CY_CAPSENSE_SLOT_SHIELD_ONLY > slotValue) && (0u != Cy_CapSense_IsWidgetEnabled(slotValue, cxt)))
            {
                /* Read raw counts */
                tmpRawCount = ptrMscHwBase->RESULT_FIFO_RD;
                ptrActive->ptrSnsContext->status &= (uint8_t)~CY_CAPSENSE_SNS_OVERFLOW_MASK;
                if (MSC_RESULT_FIFO_RD_OVERFLOW_Msk == (tmpRawCount & MSC_RESULT_FIFO_RD_OVERFLOW_Msk))
                {
                    ptrActive->ptrSnsContext->status |= CY_CAPSENSE_SNS_OVERFLOW_MASK;
                }
                tmpRawCount &= MSC_RESULT_FIFO_RD_RAW_COUNT_Msk;
                ptrActive->ptrSnsContext->raw = (uint16_t)tmpRawCount;
            }
            else
            {
                (void)ptrMscHwBase->RESULT_FIFO_RD;
            }

            /* Stores non-retention SNS_CTL register value for LFT mode */
            cxt->ptrInternalContext->snsCtlReg[curChIndex] = ptrMscHwBase->SNS_CTL;

            #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                if ((CY_CAPSENSE_SCAN_MULTIPLE_SLOT == cxt->ptrInternalContext->scanSingleSlot) &&
                    (CY_CAPSENSE_SLOT_SHIELD_ONLY != slotValue) &&
                    (CY_CAPSENSE_SLOT_EMPTY != slotValue))
                {
                    /* Disconnect the scanned sensor */
                    Cy_CapSense_DisconnectSensor(curChIndex, cxt);
                }
            #endif

            /* Find the next enabled slot for scanning */
            ptrActive->currentChannelSlotIndex = ptrIntrCxt->currentSlotIndex;
            while (ptrActive->currentChannelSlotIndex != ptrIntrCxt->endSlotIndex)
            {
                if (0u != Cy_CapSense_IsSlotEnabled((uint32_t)ptrActive->currentChannelSlotIndex + 1u, cxt))
                {
                    break;
                }
                /* Update active sensor structure */
                ptrActive->currentChannelSlotIndex++;
                slotStcIndex++;
            }

            /* Check for the last slot with multiple slot scan, if not - start the next slot scan */
            if (ptrActive->currentChannelSlotIndex != ptrIntrCxt->endSlotIndex)
            {
                /* Update active sensor structure */
                ptrActive->currentChannelSlotIndex++;
                slotStcIndex++;
                slotValue = cxt->ptrScanSlots[slotStcIndex].wdId;

                scanSlotIndexValid = slotStcIndex;
                if (CY_CAPSENSE_SLOT_EMPTY == slotValue)
                {
                    #if (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u)
                        if (slotStcIndex < CY_CAPSENSE_SLOT_COUNT)
                        {
                            scanSlotIndexValid += CY_CAPSENSE_SLOT_COUNT;
                        }
                        else
                        {
                            scanSlotIndexValid -= CY_CAPSENSE_SLOT_COUNT;
                        }
                    #endif /* (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u) */
                }
                else if (CY_CAPSENSE_SLOT_SHIELD_ONLY <= slotValue)
                {
                    scanSlotIndexValid = ((uint32_t)cxt->ptrScanSlots[slotStcIndex].snsId * CY_CAPSENSE_SLOT_COUNT) +
                                (slotStcIndex % CY_CAPSENSE_SLOT_COUNT);
                }
                else
                {
                    /* Do nothing */
                }

                /* Initialize next slot scan for the current channel */
                wdIndex = cxt->ptrScanSlots[scanSlotIndexValid].wdId;
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                    prevWdIndex = ptrActive->widgetIndex;
                    Cy_CapSense_ConfigureAnalogMuxResource(curChIndex, cxt->ptrWdConfig[wdIndex].senseMethod, cxt);
                #endif
                Cy_CapSense_InitActivePtr(curChIndex, wdIndex,
                            cxt->ptrScanSlots[scanSlotIndexValid].snsId, cxt);
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                    if ((slotValue != CY_CAPSENSE_SLOT_EMPTY) &&
                       (slotValue != CY_CAPSENSE_SLOT_SHIELD_ONLY))
                    {
                        /* Connect the widget first sensor electrodes */
                        Cy_CapSense_ConnectSensor(curChIndex, cxt);
                    }

                    /* Getting the sensor frame configuration */
                    Cy_CapSense_AmuxUpdateScanConfig(curChIndex, slotStcIndex, prevWdIndex, cxt);
                #else
                    ptrSensorFrame = &cxt->ptrSensorFrameContext[(((curChIndex * CY_CAPSENSE_SLOT_COUNT) +
                            ptrActive->currentChannelSlotIndex) * CY_MSC_6_SNS_REGS)];
                #endif

                #if (1u == CY_CAPSENSE_TOTAL_CH_NUMBER)
                    ptrIntrCxt->currentSlotIndex++;
                #else
                    for (i = 1u; i < CY_CAPSENSE_TOTAL_CH_NUMBER; i++)
                    {
                        if (cxt->ptrActiveScanSns[0u].currentChannelSlotIndex != cxt->ptrActiveScanSns[i].currentChannelSlotIndex)
                        {
                            break;
                        }
                        ptrIntrCxt->currentSlotIndex = cxt->ptrActiveScanSns[0u].currentChannelSlotIndex;
                    }
                #endif /* (1u == CY_CAPSENSE_TOTAL_CH_NUMBER) */

                if (NULL != ptrIntrCxt->ptrSSCallback)
                {
                    ptrIntrCxt->ptrSSCallback((cy_stc_capsense_active_scan_sns_t *)&cxt->ptrActiveScanSns[curChIndex]);
                }

                /* Start the next scan for the current channel */
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                    Cy_CapSense_AmuxStartScan(ptrMscHwBase);
                #else
                    Cy_MSC_ConfigureScan(ptrMscHwBase, CY_MSC_6_SNS_REGS, ptrSensorFrame);
                #endif
            }
            else
            {
                Cy_CapSense_ClrBusyFlags(curChIndex, cxt);
            }
        #else /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */
            /* The WHOLE FRAME scan is finished for the current channel */
            Cy_CapSense_ClrBusyFlags(curChIndex, cxt);
        #endif
    }
}


#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE) ||\
    (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE))
/*******************************************************************************
* Function Name: Cy_CapSense_AmuxUpdateCdacComp
****************************************************************************//**
*
* Updates the Compensation CDAC with the configuration of the specified sensor.
*
* \param base
* Pointer to a MSC HW block base address.
*
* \param ptrSnsContext
* Specifies the pointer to a sensor context structure.
*
*******************************************************************************/
void Cy_CapSense_AmuxUpdateCdacComp(
                MSC_Type * base,
                const cy_stc_capsense_sensor_context_t * ptrSnsContext)
{
    uint32_t tmpRegVal;

    tmpRegVal = base->SNS_CDAC_CTL;
    tmpRegVal &= ~MSC_SNS_CDAC_CTL_SEL_CO_Msk;
    tmpRegVal |= ((uint32_t)ptrSnsContext->cdacComp) << MSC_SNS_CDAC_CTL_SEL_CO_Pos;
    base->SNS_CDAC_CTL = tmpRegVal;
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_AmuxUpdateScanConfig
****************************************************************************//**
*
* Updates the MSC HW block with the configuration of the specified scan slot.
*
* \param chIndex
* The desired channel index.
*
* \param scanSlot
* Specifies the ID number of the slot.
*
* \param prevWdIndex
* Specifies the ID number of the widget scanned in scope of previous scan.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_AmuxUpdateScanConfig(
                uint32_t chIndex,
                uint32_t scanSlot,
                uint32_t prevWdIndex,
                cy_stc_capsense_context_t * context)
{
    uint32_t isSnsRow = 0u;
    MSC_Type * ptrMscHwBase = context->ptrCommonConfig->ptrChConfig[chIndex].ptrHwBase;
    cy_stc_capsense_active_scan_sns_t * ptrActive = &context->ptrActiveScanSns[chIndex];
    uint32_t sensorFrame[CY_MSC_6_SNS_REGS] = {0u, 0u, 0u, 0u, 0u, 0u};

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        if ((CY_CAPSENSE_CSD_GROUP == ptrActive->currentSenseMethod) &&
           (context->ptrWdConfig[ptrActive->widgetIndex].numCols <= ptrActive->sensorIndex))
        {
            isSnsRow = 1u;
        }
    #endif

    if ((prevWdIndex != ptrActive->widgetIndex) || (0u != isSnsRow))
    {
        (void)Cy_CapSense_GenerateSensorConfig(chIndex, scanSlot, &sensorFrame[0u], context);

        sensorFrame[CY_CAPSENSE_SNS_CTL_INDEX] &= ~MSC_SNS_CTL_START_SCAN_Msk;
        /* Configure the last slot */
        if (context->ptrInternalContext->endSlotIndex == ptrActive->currentChannelSlotIndex)
        {
            sensorFrame[CY_CAPSENSE_SNS_CTL_INDEX] |= MSC_SNS_CTL_LAST_Msk;
        }

        /* Start the next scan for the current channel */
        Cy_MSC_ConfigureScan(ptrMscHwBase, CY_MSC_6_SNS_REGS, &sensorFrame[0u]);
    }
    else
    {
        if (CY_CAPSENSE_CSD_GROUP == ptrActive->currentSenseMethod)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                Cy_CapSense_AmuxUpdateCdacComp(ptrMscHwBase, ptrActive->ptrSnsContext);
            #endif
        }
        else
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                Cy_CapSense_AmuxUpdateCdacComp(ptrMscHwBase, ptrActive->ptrSnsContext);
            #endif
        }

        /* Configure the last slot */
        if (context->ptrInternalContext->endSlotIndex == ptrActive->currentChannelSlotIndex)
        {
            ptrMscHwBase->SNS_CTL |= MSC_SNS_CTL_LAST_Msk;
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_AmuxStartScan
****************************************************************************//**
*
* Starts the scan on the specified MSC HW block.
*
* \param base
* Pointer to a MSC HW block base address.
*
*******************************************************************************/
void Cy_CapSense_AmuxStartScan(MSC_Type * base)
{
    base->SNS_CTL |= MSC_SNS_CTL_START_SCAN_Msk;
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_SetCmodInDefaultState
****************************************************************************//**
*
* Sets all available MSC Cmod pins connected via AMuxBus into a default state .
*
* Sets all external capacitors connected via AMuxBus into the default state:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - High-Z Analog, Input off drive mode.
*
* Do not call this function directly from the application program.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCmodInDefaultState(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    Cy_CapSense_SsConfigPinRegisters(
            context->ptrCommonConfig->ptrChConfig[chIndex].portCmod1,
            (uint32_t)context->ptrCommonConfig->ptrChConfig[chIndex].pinCmod1,
            CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO);
    Cy_CapSense_SsConfigPinRegisters(
            context->ptrCommonConfig->ptrChConfig[chIndex].portCmod2,
            (uint32_t)context->ptrCommonConfig->ptrChConfig[chIndex].pinCmod2,
            CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO);
}


#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
/*******************************************************************************
* Function Name: Cy_CapSense_SetCmodInAmuxModeState
****************************************************************************//**
*
* Sets all available MSC Cmod pins connected via AMuxBus into a default state.
*
* Sets all external capacitors connected via AMuxBus into the default state:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - High-Z Analog.
*
* Do not call this function directly from the application program.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCmodInAmuxModeState(
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    Cy_CapSense_SsConfigPinRegisters(
            context->ptrCommonConfig->ptrChConfig[chIndex].portCmod1,
            (uint32_t)context->ptrCommonConfig->ptrChConfig[chIndex].pinCmod1,
            CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA);

    Cy_CapSense_SsConfigPinRegisters(
            context->ptrCommonConfig->ptrChConfig[chIndex].portCmod2,
            (uint32_t)context->ptrCommonConfig->ptrChConfig[chIndex].pinCmod2,
            CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_AMUXA);
}
#endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) */


#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
/*******************************************************************************
* Function Name: Cy_CapSense_SetIOsInDesiredState
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
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetIOsInDesiredState(
                uint32_t desiredDriveMode,
                uint32_t desiredPinOutput,
                en_hsiom_sel_t desiredHsiom,
                uint32_t chIndex,
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->numPin; loopIndex++)
    {
        if (chIndex == ptrPinCfg->chId)
        {
            Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr,
                (uint32_t)ptrPinCfg->pinNumber, desiredDriveMode, desiredHsiom);

            if (0u != desiredPinOutput)
            {
                Cy_GPIO_Set(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
            }
            else
            {
                Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
            }
        }
        /* Get next electrode */
        ptrPinCfg++;
    }
}
#endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) */


/*******************************************************************************
* Function Name: Cy_CapSense_SetIOsInDefaultState
****************************************************************************//**
*
* Sets all CAPSENSE&trade; pins into a default state.
*
* Sets all the CSD/CSX IOs into a default state.
* Default state:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - High-Z Analog, Input off drive mode.
* - State   - Zero.
*
* Do not call this function directly from the application program.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetIOsInDefaultState(
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->numPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr,
                (uint32_t)ptrPinCfg->pinNumber, CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO);
        Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        ptrPinCfg++;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetShieldPinsInDefaultState
****************************************************************************//**
*
* Sets all shield pins into a default state.
*
* Do not call this function directly from the application program.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetShieldPinsInDefaultState(
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrShieldPinConfig;

    /* Loop through all shield electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->csdShieldNumPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr,
                  (uint32_t)ptrPinCfg->pinNumber, CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO);

        Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
        /* Get next electrode */
        ptrPinCfg++;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_InitActivePtr
****************************************************************************//**
*
* Initializes active scan sensor structure with all available
* pointers for further faster access to widget/sensor parameters.
*
* \param chIndex
* Specifies the channel ID number.
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
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitActivePtr(
                uint32_t chIndex,
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    Cy_CapSense_InitActivePtrWd(chIndex, widgetId, context);

    if (CY_CAPSENSE_SLOT_SHIELD_ONLY > widgetId)
    {
        Cy_CapSense_InitActivePtrSns(chIndex, sensorId, context);
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_InitActivePtrSns
****************************************************************************//**
*
* Initializes active scan sensor structure with pointers to sensor
* for further faster access to widget/sensor parameters.
*
* This function supposes that the Cy_CapSense_InitActivePtrWd() function
* is called before.
*
* \param chIndex
* The desired channel index.
*
* \param sensorId
* Specifies the ID number of the sensor within the widget. A macro for the
* sensor ID within a specified widget can be found in the cycfg_capsense.h
* file defined as CY_CAPSENSE_<WIDGET_NAME>_SNS<SENSOR_NUMBER>_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void  Cy_CapSense_InitActivePtrSns(
                uint32_t chIndex,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    #if ((CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        cy_stc_capsense_active_scan_sns_t * ptrActive = &context->ptrActiveScanSns[chIndex];
    #endif

    #if ((CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        uint32_t widgetSenseGroup = ptrActive->currentSenseMethod;
        const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[ptrActive->widgetIndex];
        ptrActive->connectedSnsState = CY_CAPSENSE_SNS_DISCONNECTED;
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            uint32_t numberRows;
            uint32_t numberCols;
        #endif
    #endif

    #if ((CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) && \
         (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_PHASE_TX_ENABLED))
        ptrActive->sensorIndex = (uint16_t)sensorId;
    #endif

    #if !((CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        (void)chIndex;
        (void)context;
    #endif

    #if ((CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        ptrActive->ptrSnsContext = &context->ptrWdConfig[ptrActive->widgetIndex].ptrSnsContext[sensorId];
    #else
        (void)sensorId;
    #endif

    #if ((CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        switch (widgetSenseGroup)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                case (uint8_t)CY_CAPSENSE_CSD_GROUP:
                    ptrActive->ptrEltdConfig = &ptrWdCfg->ptrEltdConfig[sensorId];
                    break;
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                case (uint8_t)CY_CAPSENSE_CSX_GROUP:
                    numberRows = ptrWdCfg->numRows;
                    numberCols = ptrWdCfg->numCols;
                    ptrActive->ptrRxConfig = &ptrWdCfg->ptrEltdConfig[sensorId / numberRows];
                    ptrActive->ptrTxConfig = &ptrWdCfg->ptrEltdConfig[numberCols + (sensorId % numberRows)];
                    break;
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

                default:
                    /* Widget type is not valid so nothing to do */
                    break;
        }
    #endif
}

/*******************************************************************************
* Function Name: Cy_CapSense_InitActivePtrWd
****************************************************************************//**
*
* Initializes active scan sensor structure with pointers to current widget
* for further faster access to widget/sensor parameters.
*
* This function does not update pointers to current sensor and the
* Cy_CapSense_InitActivePtrSns() function should be called after current one.
*
* \param chIndex
* The desired channel index.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitActivePtrWd(
                uint32_t chIndex,
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_active_scan_sns_t * ptrActive = &(context->ptrActiveScanSns[chIndex]);

    #if ((CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        ptrActive->widgetIndex = (uint8_t)widgetId;
    #endif

    if (CY_CAPSENSE_SLOT_SHIELD_ONLY > widgetId)
    {
        ptrActive->currentSenseMethod = context->ptrWdConfig[widgetId].senseMethod;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_SsConfigPinRegisters
****************************************************************************//**
*
* Configures drive mode and HSIOM state of a desired pin. The action
* is performed inside the critical section.
*
* \param base
* The pointer to the pin's port register base address.
*
* \param pinNum
* Position of the pin bit-field within the port register.
*
* \param dm
* Specifies drive mode of the pin.
*
* \param hsiom
* Specifies HSIOM state of the pin.
*
*******************************************************************************/
void Cy_CapSense_SsConfigPinRegisters(
                GPIO_PRT_Type * base,
                uint32_t pinNum,
                uint32_t dm,
                en_hsiom_sel_t hsiom)
{
    uint32_t interruptState = Cy_SysLib_EnterCriticalSection();
    if (CY_CAPSENSE_HSIOM_SEL_GPIO == hsiom)
    {
        Cy_GPIO_SetHSIOM(base, pinNum, hsiom);
        Cy_GPIO_SetDrivemode(base, pinNum, dm);
    }
    else
    {
        Cy_GPIO_SetDrivemode(base, pinNum, dm);
        Cy_GPIO_SetHSIOM(base, pinNum, hsiom);
    }
    Cy_SysLib_ExitCriticalSection(interruptState);
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
     (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD))
/*******************************************************************************
* Function Name: Cy_CapSense_EnableShieldElectrodes
****************************************************************************//**
*
* This internal function initializes Shield Electrodes.
*
* The function sets the bit in the HSIOM register which enables the shield electrode
* functionality on the pin. The port and pin configurations are stored in
* the cy_capsense_shieldIoList structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_EnableShieldElectrodes(uint32_t chIndex, cy_stc_capsense_context_t * context)
{
    uint32_t shieldIndex;
    const cy_stc_capsense_pin_config_t * ptrShieldPin = context->ptrShieldPinConfig;
    uint32_t * ptrSwSelCfg = &context->ptrBaseFrameContext[chIndex].swSelGpio;

    *ptrSwSelCfg &= ~MSC_SW_SEL_GPIO_SW_CSD_SHIELD_Msk;
    if (CY_CAPSENSE_SHIELD_ACTIVE == context->ptrCommonConfig->csdShieldMode)
    {
        *ptrSwSelCfg |= CY_CAPSENSE_FW_SHIELD_ACTIVE_AMUX_REG_SW_CSD_SHIELD_VALUE;
    }
    else
    {
        *ptrSwSelCfg |= CY_CAPSENSE_FW_SHIELD_PASSIVE_AMUX_REG_SW_CSD_SHIELD_VALUE;
    }

    context->ptrCommonConfig->ptrChConfig[chIndex].ptrHwBase->SW_SEL_GPIO = *ptrSwSelCfg;

    for (shieldIndex = 0u; shieldIndex < context->ptrCommonConfig->csdShieldNumPin; shieldIndex++)
    {
        if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrShieldPin->chId)
        {
            if (CY_CAPSENSE_SHIELD_ACTIVE == context->ptrCommonConfig->csdShieldMode)
              {
                  Cy_CapSense_SsConfigPinRegisters(ptrShieldPin->pcPtr, (uint32_t)ptrShieldPin->pinNumber,
                          CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD);
              }
              else
              {
                  Cy_CapSense_SsConfigPinRegisters(ptrShieldPin->pcPtr, (uint32_t)ptrShieldPin->pinNumber,
                          CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF, CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD);
              }
        }
        ptrShieldPin++;
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_DisableShieldElectrodes
****************************************************************************//**
*
* This internal function disables Shield Electrodes.
*
* The function resets the bit in the HSIOM register which disables the shield
* electrode functionality on the pin. The port and pin configurations are
* stored in the cy_capsense_shieldIoList structure.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_DisableShieldElectrodes(uint32_t chIndex, cy_stc_capsense_context_t * context)
{
    uint32_t shieldIndex;
    const cy_stc_capsense_pin_config_t * ptrShieldPin = context->ptrShieldPinConfig;

    context->ptrBaseFrameContext[chIndex].swSelGpio &= ~MSC_SW_SEL_GPIO_SW_CSD_SHIELD_Msk;
    /* Reverts the .swSelGpio register state to default AMUX+CSX+MPTX */
    context->ptrBaseFrameContext[chIndex].swSelGpio |= CY_CAPSENSE_FW_NEGATIVE_TX_AMUX_REG_SW_CSD_SHIELD_VALUE;

    context->ptrCommonConfig->ptrChConfig[chIndex].ptrHwBase->SW_SEL_GPIO =
                    context->ptrBaseFrameContext[chIndex].swSelGpio;

    for (shieldIndex = 0u; shieldIndex < context->ptrCommonConfig->csdShieldNumPin; shieldIndex++)
    {
        if ((chIndex + context->ptrCommonConfig->channelOffset) == ptrShieldPin->chId)
        {
            Cy_CapSense_SsConfigPinRegisters(ptrShieldPin->pcPtr, (uint32_t)ptrShieldPin->pinNumber,
                                             context->ptrInternalContext->csxInactiveSnsDm,
                                             context->ptrInternalContext->csxInactiveSnsHsiom);
            Cy_GPIO_Clr(ptrShieldPin->pcPtr, (uint32_t)ptrShieldPin->pinNumber);
        }
        ptrShieldPin++;
    }
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN) &&\
           (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)) */


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAbort_V3
****************************************************************************//**
*
* This function sets the sequencer to the idle state by resetting the hardware,
* it can be used to abort current scan.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade;.
*
* \note
* If this function is called from ISR during initialization or
* auto-calibration the operation of these functions will be corrupted.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAbort_V3(cy_stc_capsense_context_t * context)
{
    uint32_t interruptState;
    uint32_t curChIndex;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    const cy_stc_capsense_channel_config_t * ptrMscChan = ptrCommonCfg->ptrChConfig;

    interruptState = Cy_SysLib_EnterCriticalSection();

    /* Clear ENABLE bit for all channels */
    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        ptrMscChan[curChIndex].ptrHwBase->CTL &= ~MSC_CTL_ENABLED_Msk;
    }

    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        /* Wait until ENABLE bit is cleared for all channels*/
        while (0u != (MSC_CTL_ENABLED_Msk & ptrMscChan[curChIndex].ptrHwBase->CTL))
        {}
        /* Set ENABLE bit for all channels */
        ptrMscChan[curChIndex].ptrHwBase->CTL |= MSC_CTL_ENABLED_Msk;
        /* Clear all pending interrupts of the MSC HW block */
        ptrMscChan[curChIndex].ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
        (void)ptrMscChan[curChIndex].ptrHwBase->INTR;
    }

    Cy_SysLib_ExitCriticalSection(interruptState);

    (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_UNDEFINED, context);

    context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;

    /* Wait for initialization */
    Cy_SysLib_DelayUs(ptrCommonCfg->analogWakeupDelay);

    return CY_CAPSENSE_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name: Cy_CapSense_MwState_V3
****************************************************************************//**
*
* This function returns a detailed state of the CAPSENSE&trade; middleware and MSC
* hardware in Single- or Multi-channel mode. This feature is useful in
* multi-thread applications or in ISR.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
* If the middleware is busy, a new scan, setup widgets, any kind of reconfiguration, 
* or parameter change should not be initiated.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the state of the middleware as a sum of the state and status masks:
*
* - CY_CAPSENSE_BUSY_CH_MASK                     - The set [x] bit of the result
*                                                  means that the previously initiated
*                                                  scan for the [x] channel is in progress.
*                                                  The next scan frame cannot be started.
* - CY_CAPSENSE_BUSY                             - The previously initiated scan is
*                                                  in progress.
* - CY_CAPSENSE_MW_STATE_BIST_MASK               - The BIST is in progress.
*                                                  The next scan frame cannot be started.
* - CY_CAPSENSE_MW_STATE_CALIBRATION_MASK        - The auto-calibration is in progress.
*                                                  The next scan frame cannot be started.
* - CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK         - The smart sensing algorithm is
*                                                  in progress.
*                                                  The next scan frame cannot be started.
* - CY_CAPSENSE_MW_STATE_INITIALIZATION_MASK     - Middleware initialization is
*                                                  in progress and the next scan frame
*                                                  cannot be initiated.
* - CY_CAPSENSE_MW_STATE_SCAN_SLOT_MASK[x]       - The set [x] number of the result
*                                                  means that the previously initiated
*                                                  scan for the [x] slot is completed
*                                                  or in progress. In CS-DMA mode, this
*                                                  field is set only for the first
*                                                  scanned slot.
*
*******************************************************************************/
cy_capsense_mw_state_t Cy_CapSense_MwState_V3(const cy_stc_capsense_context_t * context)
{
    return ((context->ptrCommonContext->status & ~CY_CAPSENSE_MW_STATE_SCAN_SLOT_MASK) |
            ((uint32_t)context->ptrInternalContext->currentSlotIndex << CY_CAPSENSE_MW_STATE_SCAN_SLOT_POS));
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetBusyFlags
****************************************************************************//**
*
* Sets BUSY flags of the cy_capsense_context.status register specified
* by the flags parameter.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* This is an internal function. Do not call this function directly from
* the application program.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetBusyFlags(
                uint32_t chIndex,
                cy_stc_capsense_context_t * context)
{
    context->ptrCommonContext->status |= ((((uint32_t)CY_CAPSENSE_BUSY_CH_MASK) << chIndex) |
                                          CY_CAPSENSE_BUSY);
}


/*******************************************************************************
* Function Name: Cy_CapSense_ClrBusyFlags
****************************************************************************//**
*
* Clears BUSY flags of the cy_capsense_context.status register specified
* by the flags parameter.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* This is an internal function. Do not call this function directly from
* the application program.
*
* \param chIndex
* The desired channel index.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ClrBusyFlags(
                uint32_t chIndex,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        /* Clear Last flag inside sensor frame structure for the current channel */
        uint32_t snsPos = context->ptrInternalContext->endSlotIndex + (context->ptrCommonConfig->numSlots * chIndex);
        context->ptrSensorFrameContext[(snsPos * CY_MSC_6_SNS_REGS) + CY_CAPSENSE_SNS_CTL_INDEX] &= ~MSC_SNS_CTL_LAST_Msk;
    #endif

    /* Clear busy flag for the current channel */
    context->ptrCommonContext->status &= (uint32_t)(~(((uint32_t)CY_CAPSENSE_BUSY_CH_MASK) << chIndex));
    if (0u == (context->ptrCommonContext->status & CY_CAPSENSE_BUSY_ALL_CH_MASK))
    {
        context->ptrCommonContext->status &= ~CY_CAPSENSE_BUSY;
    }

    if (0u == context->ptrCommonContext->status)
    {
        /* Mark completion of scan cycle */
        context->ptrCommonContext->scanCounter++;
        /* Check for EndOfScan callback */
        if (NULL != context->ptrInternalContext->ptrEOSCallback)
        {
            #if (CY_CAPSENSE_SCAN_MODE_INT_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                context->ptrInternalContext->ptrEOSCallback((cy_stc_capsense_active_scan_sns_t *)&context->ptrActiveScanSns[0u]);
            #else
                context->ptrInternalContext->ptrEOSCallback(NULL);
            #endif
        }
    }
}

#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureAnalogMuxResource
****************************************************************************//**
*
* Configures the Analog Mux connections for a specified channel. This function
* should be called before call of the Cy_CapSense_InitActivePtr() function.
*
* \param chIndex
* The desired channel index.
*
* \param snsMethod
* Desired sensing method the Analog Mux resources should be configured to.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ConfigureAnalogMuxResource(
                uint32_t chIndex,
                uint32_t snsMethod,
                cy_stc_capsense_context_t * context)
{
    uint32_t * ptrSwSelCfg;
    uint32_t prevSnsMethod = (uint32_t)context->ptrActiveScanSns[chIndex].currentSenseMethod;

    if (snsMethod != prevSnsMethod)
    {
        ptrSwSelCfg = &context->ptrBaseFrameContext[chIndex].swSelGpio;

        *ptrSwSelCfg &= ~CY_CAPSENSE_CMOD_AMUX_MSK;
        context->ptrCommonConfig->ptrChConfig[chIndex].ptrHwBase->SW_SEL_GPIO = *ptrSwSelCfg;

        /* Enable the specified mode */
        if (CY_CAPSENSE_CSD_GROUP == snsMethod)
        {
            *ptrSwSelCfg |= CY_CAPSENSE_FW_CMOD_AMUX_CSD_REG_SW_SEL_GPIO_VALUE;
            Cy_CapSense_SetIOsInDesiredState(context->ptrInternalContext->csdInactiveSnsDm, 0u,
                                             context->ptrInternalContext->csdInactiveSnsHsiom, chIndex, context);
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                Cy_CapSense_EnableShieldElectrodes(chIndex, context);
            #endif
        }
        else
        {
            *ptrSwSelCfg |= CY_CAPSENSE_FW_CMOD_AMUX_CSX_REG_SW_SEL_GPIO_VALUE;
            Cy_CapSense_SetIOsInDesiredState(context->ptrInternalContext->csxInactiveSnsDm, 0u,
                                             context->ptrInternalContext->csxInactiveSnsHsiom, chIndex, context);
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                Cy_CapSense_DisableShieldElectrodes(chIndex, context);
            #endif
        }

        context->ptrCommonConfig->ptrChConfig[chIndex].ptrHwBase->SW_SEL_GPIO = *ptrSwSelCfg;
    }
}
#endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) */


#if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
/*******************************************************************************
* Function Name: Cy_CapSense_InitializeDmaResource
****************************************************************************//**
*
* Initializes the DMAC channels in CS-DMA scanning mode.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of operation.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_InitializeDmaResource(
                cy_stc_capsense_context_t * context)
{
    const cy_stc_capsense_channel_config_t * ptrCurMscChCfg = context->ptrCommonConfig->ptrChConfig;
    uint32_t curChIndex;
    uint8_t wrIdx;
    uint8_t rdIdx;
    uint8_t wrChIdx;
    uint8_t rdChIdx;
    DMAC_Type * dmacBase;
    MSC_Type * mscBase;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context->ptrCommonConfig->ptrDmacBase)
    {
        dmacBase = context->ptrCommonConfig->ptrDmacBase;
        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
        {
            ptrCurMscChCfg = &(context->ptrCommonConfig->ptrChConfig[curChIndex]);
            mscBase = ptrCurMscChCfg->ptrHwBase;
            wrIdx = ptrCurMscChCfg->dmaWrChIndex;
            wrChIdx = ptrCurMscChCfg->dmaChainWrChIndex;
            rdIdx = ptrCurMscChCfg->dmaRdChIndex;
            rdChIdx = ptrCurMscChCfg->dmaChainRdChIndex;

            /* Perform the initial configuration of the WR channel of DMAC */
            Cy_DMAC_Descriptor_DeInit(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetSrcIncrement(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Descriptor_SetDstIncrement(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Descriptor_SetDataSize(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_WORD);
            Cy_DMAC_Descriptor_SetSrcTransferSize(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetDstTransferSize(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetRetrigger(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_RETRIG_IM);
            Cy_DMAC_Descriptor_SetFlipping(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetTriggerType(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_SINGLE_DESCR);
            Cy_DMAC_Descriptor_SetCpltState(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetState(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Channel_SetPriority(dmacBase, wrIdx, 3u);
            Cy_DMAC_Descriptor_SetPreemptable(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Channel_SetCurrentDescriptor(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetDstAddress(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING, (void volatile *)&mscBase->SNS_SW_SEL_CSW_MASK2);
            Cy_DMAC_Descriptor_SetDataCount(dmacBase, wrIdx, CY_DMAC_DESCRIPTOR_PING,  CY_MSC_6_SNS_REGS);
            Cy_DMAC_Channel_Enable(dmacBase, wrIdx);

            /* Perform the initial configuration of the Chain WR channel of DMAC */
            Cy_DMAC_Descriptor_DeInit(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetSrcIncrement(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Descriptor_SetDstIncrement(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetDataSize(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_WORD);
            Cy_DMAC_Descriptor_SetSrcTransferSize(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetDstTransferSize(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetRetrigger(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_RETRIG_IM);
            Cy_DMAC_Descriptor_SetFlipping(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetTriggerType(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_SINGLE_ELEMENT);
            Cy_DMAC_Descriptor_SetCpltState(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetState(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Channel_SetPriority(dmacBase, wrChIdx, 3u);
            Cy_DMAC_Descriptor_SetPreemptable(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Channel_SetCurrentDescriptor(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetDstAddress(dmacBase, wrChIdx, CY_DMAC_DESCRIPTOR_PING, (void volatile *)&dmacBase->DESCR[wrIdx].PING_SRC);
            Cy_DMAC_Channel_Enable(dmacBase, wrChIdx);

            /* Perform the initial configuration of the RD channel of DMAC */
            Cy_DMAC_Descriptor_DeInit(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetSrcIncrement(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetDstIncrement(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetDataSize(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_HALFWORD);
            Cy_DMAC_Descriptor_SetSrcTransferSize(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetDstTransferSize(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_DATA);
            Cy_DMAC_Descriptor_SetRetrigger(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_RETRIG_IM);
            Cy_DMAC_Descriptor_SetFlipping(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetTriggerType(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_SINGLE_DESCR);
            Cy_DMAC_Descriptor_SetCpltState(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetState(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Channel_SetPriority(dmacBase, rdIdx, 3u);
            Cy_DMAC_Descriptor_SetPreemptable(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Channel_SetCurrentDescriptor(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetSrcAddress(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, (void volatile const *)&(mscBase->RESULT_FIFO_RD));
            Cy_DMAC_Descriptor_SetDataCount(dmacBase, rdIdx, CY_DMAC_DESCRIPTOR_PING, 1u);
            Cy_DMAC_Channel_Enable(dmacBase, rdIdx);

            /* Perform the initial configuration of the Chain Read channel of DMAC */
            Cy_DMAC_Descriptor_DeInit(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetSrcIncrement(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Descriptor_SetDstIncrement(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetDataSize(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_WORD);
            Cy_DMAC_Descriptor_SetSrcTransferSize(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetDstTransferSize(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_TRANSFER_SIZE_WORD);
            Cy_DMAC_Descriptor_SetRetrigger(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_RETRIG_IM);
            Cy_DMAC_Descriptor_SetFlipping(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetTriggerType(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, CY_DMAC_SINGLE_ELEMENT);
            Cy_DMAC_Descriptor_SetCpltState(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Descriptor_SetState(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, true);
            Cy_DMAC_Channel_SetPriority(dmacBase, rdChIdx, 3u);
            Cy_DMAC_Descriptor_SetPreemptable(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, false);
            Cy_DMAC_Channel_SetCurrentDescriptor(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING);
            Cy_DMAC_Descriptor_SetDstAddress(dmacBase, rdChIdx, CY_DMAC_DESCRIPTOR_PING, (void volatile *) &dmacBase->DESCR[rdIdx].PING_DST);
            Cy_DMAC_Enable(dmacBase);
        }

        capStatus = CY_CAPSENSE_STATUS_SUCCESS;

    }

    return capStatus;
}
#endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

#if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureDmaResource
****************************************************************************//**
*
* Configures DMAC channels prior to the scan.
*
* \param mscChIndex
* Index of the MSC channel to configure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of operation.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ConfigureDmaResource(
                uint32_t mscChIndex,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_channel_config_t * ptrCurMscChCfg = &(context->ptrCommonConfig->ptrChConfig[mscChIndex]);
    DMAC_Type * dmacBase = context->ptrCommonConfig->ptrDmacBase;
    uint8_t chainWrChIndex = ptrCurMscChCfg->dmaChainWrChIndex;
    uint8_t chainRdChIndex = ptrCurMscChCfg->dmaChainRdChIndex;
    uint32_t currentSlotIndex = context->ptrInternalContext->currentSlotIndex;
    uint32_t endSlotIndex = context->ptrInternalContext->endSlotIndex;
    uint32_t currSlotPos = (mscChIndex + context->ptrCommonConfig->channelOffset) * CY_CAPSENSE_SLOT_COUNT;

    /* Disable DMAC channels before configuring */
    Cy_DMAC_Channel_Disable(dmacBase, chainRdChIndex);

    /* Configure the destination and data count for the Chain Write channel of DMAC */
    Cy_DMAC_Descriptor_SetSrcAddress(dmacBase, chainWrChIndex, CY_DMAC_DESCRIPTOR_PING, &context->ptrCommonConfig->ptrDmaWrChSnsCfgAddrLocal[currSlotPos + currentSlotIndex]);
    Cy_DMAC_Descriptor_SetDataCount(dmacBase, chainWrChIndex, CY_DMAC_DESCRIPTOR_PING, (endSlotIndex - currentSlotIndex + 1u));

    /* Configure the destination and data count for the Chain Read channel of DMAC */
    Cy_DMAC_Descriptor_SetSrcAddress(dmacBase, chainRdChIndex, CY_DMAC_DESCRIPTOR_PING, &context->ptrCommonConfig->ptrDmaRdChSnsCfgAddrLocal[currSlotPos + context->ptrInternalContext->firstValidSlot]);
    Cy_DMAC_Descriptor_SetDataCount(dmacBase, chainRdChIndex, CY_DMAC_DESCRIPTOR_PING, context->ptrInternalContext->numValidSlots);

    /* Enable DMAC channels to perform scanning */
    Cy_DMAC_Channel_Enable(dmacBase, chainRdChIndex);

    return capStatus;
}
#endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */


#if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
/*******************************************************************************
* Function Name: Cy_CapSense_InitializeDmaArrays
****************************************************************************//**
*
* Initializes DMA arrays used for scanning.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of operation.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_InitializeDmaArrays(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t slotIndex;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;

    for (slotIndex = 0u; slotIndex < (CY_CAPSENSE_SLOT_COUNT * CY_CAPSENSE_TOTAL_CH_NUMBER); slotIndex++)
    {
        ptrCommonCfg->ptrDmaWrChSnsCfgAddrLocal[slotIndex] = ptrCommonCfg->ptrDmaWrChSnsCfgAddr[slotIndex];
        ptrCommonCfg->ptrDmaRdChSnsCfgAddrLocal[slotIndex] = ptrCommonCfg->ptrDmaRdChSnsCfgAddr[slotIndex];
    }

    context->ptrInternalContext->numValidSlots = CY_CAPSENSE_SLOT_COUNT_MAX_VALUE;

    return capStatus;
}
#endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */


#if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureDmaArrays
****************************************************************************//**
*
* Configures DMA arrays used for scanning considering widget enable/disable
* and working/non-working statuses.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of operation.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ConfigureDmaArrays(
                cy_stc_capsense_context_t * context)
{
    uint32_t chIndex;
    uint32_t slotOffset;
    uint32_t slotIndex;
    uint32_t slotStatus;
    uint32_t wdIndex;
    uint32_t rawPtrIndex = 0u;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    cy_stc_capsense_internal_context_t * ptrIntrCxt = context->ptrInternalContext;
    uint32_t * ptrSnsFrmCxt = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CTL_INDEX];

    ptrIntrCxt->numValidSlots = CY_CAPSENSE_SLOT_COUNT_MAX_VALUE;

    /* Loops across all slots within one enabled channel */
    for (slotIndex = 0u; slotIndex < CY_CAPSENSE_SLOT_COUNT; slotIndex++)
    {
        /* Gets status of a slot considering all channels */
        slotStatus = Cy_CapSense_IsSlotEnabled(slotIndex, context);
        /* Loops across all configured channels */
        for (chIndex = 0u; chIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; chIndex++)
        {
            slotOffset = chIndex * (CY_CAPSENSE_SLOT_COUNT * CY_MSC_6_SNS_REGS);
            /* Disable sensor */
            ptrSnsFrmCxt[slotOffset] &= (uint32_t)~MSC_SNS_CTL_VALID_Msk;
            /* Mark sensors valid and store pointer to rawcount address */
            if (0u != slotStatus)
            {
                ptrSnsFrmCxt[slotOffset] |= MSC_SNS_CTL_VALID_Msk;
                wdIndex = context->ptrScanSlots[(chIndex * CY_CAPSENSE_SLOT_COUNT) + slotIndex].wdId;
                if (0u != Cy_CapSense_IsWidgetEnabled(wdIndex, context))
                {
                    ptrCommonCfg->ptrDmaRdChSnsCfgAddrLocal[(chIndex * CY_CAPSENSE_SLOT_COUNT) + rawPtrIndex] =
                            ptrCommonCfg->ptrDmaRdChSnsCfgAddr[(chIndex * CY_CAPSENSE_SLOT_COUNT) + slotIndex];
                }
                else
                {
                    ptrCommonCfg->ptrDmaRdChSnsCfgAddrLocal[(chIndex * CY_CAPSENSE_SLOT_COUNT) + rawPtrIndex] =
                            ptrCommonCfg->ptrEmptyRawCount;
                }
            }
        }
        if (0u != slotStatus)
        {
            rawPtrIndex++;
        }
        ptrSnsFrmCxt = &ptrSnsFrmCxt[CY_MSC_6_SNS_REGS];
    }

    return capStatus;
}
#endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */


/*******************************************************************************
* Function Name: Cy_CapSense_SetModClkDivider
****************************************************************************//**
*
* Sets the modulator clock and then starts it.
*
* Do not call this function directly from the application program.
*
* \param dividerValue
* The MOD_CLK divider to be set for the PERI clock.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetModClkDivider(
                uint32_t dividerValue,
                const cy_stc_capsense_context_t * context)
{
    uint32_t dividerIndex = (uint32_t)context->ptrCommonConfig->periDividerIndex;
    cy_en_sysclk_divider_types_t dividerType;
    uint32_t dividerValueLocal = dividerValue;

    if (0u == dividerValueLocal)
    {
        dividerValueLocal = 1u;
    }
    dividerValueLocal--;

    switch (context->ptrCommonConfig->periDividerType)
    {
        case (uint8_t)CY_SYSCLK_DIV_8_BIT:
            dividerType = CY_SYSCLK_DIV_8_BIT;
            break;
        case (uint8_t)CY_SYSCLK_DIV_16_BIT:
            dividerType = CY_SYSCLK_DIV_16_BIT;
            break;
        case (uint8_t)CY_SYSCLK_DIV_16_5_BIT:
            dividerType = CY_SYSCLK_DIV_16_5_BIT;
            break;
        default:
            dividerType = CY_SYSCLK_DIV_24_5_BIT;
            break;
    }

    (void)Cy_SysClk_PeriphDisableDivider(dividerType, dividerIndex);
    if ((CY_SYSCLK_DIV_8_BIT == dividerType) || (CY_SYSCLK_DIV_16_BIT == dividerType))
    {
        (void)Cy_SysClk_PeriphSetDivider(dividerType, dividerIndex, dividerValueLocal);
    }
    else
    {
        (void)Cy_SysClk_PeriphSetFracDivider(dividerType, dividerIndex, dividerValueLocal, 0u);
    }
    (void)Cy_SysClk_PeriphEnableDivider(dividerType, dividerIndex);
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateSlot
****************************************************************************//**
*
* Calibrates one of the CapDACs for selected slot. The calibration targets
* are specified in the cy_stc_capsense_internal_context_t structure
* as csdCalibrTarget and csxCalibrTarget fields.
*
* \param scanSlotId
* The slot ID calibration scans will be done for.
*
* \param autoCalibrMode
* Specifies the slot auto-calibration mode:
* - CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR  - The successive approximation
*     (8-9 scans) of RefCDAC with disabled CompCDAC
* - CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR - The successive approximation
*     (8-9 scans) of CompCDAC with fixed RefCDAC
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_CALIBRATION_FAIL - The calibration failed if software
*                                         watchdog timeout occurred
*                                         during any calibration scan,
*                                         the scan was not completed, or
*                                         resulted raw counts
*                                         are outside the limits.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateSlot(
                uint32_t scanSlotId,
                uint32_t autoCalibrMode,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_SUCCESS;

    uint32_t target;
    uint32_t snsIndex;
    uint32_t wdIndex;
    uint32_t chIndex;
    uint32_t maxRawTmp;
    uint32_t calMaskNext = CY_CAPSENSE_CAL_MIDDLE_VALUE;
    uint32_t calMaskPast = 0u;
    uint32_t calibrationBitEn = 0u;
    uint32_t sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
    uint8_t * ptrCdac;
    uint32_t * ptrRawTarget;
    uint32_t rawTarget[CY_CAPSENSE_MAX_CH_NUM];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    cy_stc_capsense_sensor_context_t * ptrSnsCxtArray[CY_CAPSENSE_MAX_CH_NUM];
    const cy_stc_capsense_widget_config_t * ptrWdCfg;

    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        uint32_t * ptrSnsFrmCxt;
        uint32_t * ptrSnsFrmCxtArray[CY_CAPSENSE_MAX_CH_NUM];
    #endif

    if (0u != Cy_CapSense_IsSlotEnabled(scanSlotId, context))
    {
        /* Checks a slot for auto-calibration */
        for (chIndex = 0u; chIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; chIndex++)
        {
            #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
                if (context->ptrInternalContext->hwConfigState != CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING)
                {
                    /* Generates sensor frame configuration */
                    Cy_CapSense_GenerateAllSensorConfig(chIndex,
                                                    &context->ptrSensorFrameContext[(CY_CAPSENSE_SLOT_COUNT *
                                                    (chIndex + context->ptrCommonConfig->channelOffset)) *
                                                    CY_MSC_6_SNS_REGS], context);
                }
            #endif

            wdIndex = context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * chIndex)].wdId;
            if (CY_CAPSENSE_SLOT_SHIELD_ONLY > wdIndex)
            {
                /* Finds a sensing method for existing slot */
                sensingGroup = context->ptrWdConfig[wdIndex].senseMethod;
                /* Checks if the auto-calibration is enabled for the current sense method */
                #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CALIBRATION_EN)
                    if (CY_CAPSENSE_CSX_GROUP == sensingGroup)
                    {
                        sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                    }
                #else

                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
                        if ((CY_CAPSENSE_CSX_GROUP == sensingGroup) && (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode))
                        {
                            sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                        }
                    #endif

                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                        if ((CY_CAPSENSE_CSX_GROUP == sensingGroup) && (CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR == autoCalibrMode))
                        {
                            sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                        }
                    #endif
                #endif

                #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CALIBRATION_EN)
                    if (CY_CAPSENSE_CSD_GROUP == sensingGroup)
                    {
                        sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                    }
                #else

                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
                        if ((CY_CAPSENSE_CSD_GROUP == sensingGroup) && (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode))
                        {
                            sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                        }
                    #endif

                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                        if ((CY_CAPSENSE_CSD_GROUP == sensingGroup) && (CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR == autoCalibrMode))
                        {
                            sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                        }
                    #endif
                #endif

                /* Different sensing methods not allowed in a single slot */
                break;
            }
        }
    }

    /* Skips non-auto-calibrated slots */
    if (CY_CAPSENSE_UNDEFINED_GROUP != sensingGroup)
    {
        for (chIndex = 0u; chIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; chIndex++)
        {
            /* Populates all sensors in widget */
            wdIndex = context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * chIndex)].wdId;
            if (CY_CAPSENSE_SLOT_SHIELD_ONLY > wdIndex)
            {
                calibrationBitEn |= (uint32_t)(0x01uL << chIndex);

                snsIndex = context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * chIndex)].snsId;
                ptrWdCfg = &context->ptrWdConfig[wdIndex];

                /* Gets target in percentage */
                target = context->ptrInternalContext->intrCsdRawTarget;

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    if (CY_CAPSENSE_CSX_GROUP == sensingGroup)
                    {
                        target = context->ptrInternalContext->intrCsxRawTarget;
                    }
                #endif

                maxRawTmp = context->ptrWdContext[wdIndex].maxRawCount;
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                    if ((CY_CAPSENSE_CSD_GROUP == sensingGroup) && (ptrWdCfg->numCols <= snsIndex))
                    {
                        maxRawTmp = context->ptrWdContext[wdIndex].maxRawCountRow;
                    }
                #endif

                rawTarget[chIndex] = (uint16_t)((maxRawTmp * target) / CY_CAPSENSE_PERCENTAGE_100);
                ptrSnsCxtArray[chIndex] = &ptrWdCfg->ptrSnsContext[snsIndex];
                ptrSnsCxtArray[chIndex]->cdacComp = 0u;

                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
                    ptrSnsFrmCxtArray[chIndex] = &context->ptrSensorFrameContext[(CY_MSC_6_SNS_REGS *
                                                ((context->ptrCommonConfig->numSlots * chIndex) + scanSlotId)) + CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
                #endif
            }
        }

        /* Perform repeated continuous scan of a slot and tune CDAC */
        do
        {
            ptrRawTarget = &rawTarget[0u];
            for (chIndex = 0u; chIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; chIndex++)
            {
                if (0u != (calibrationBitEn & (0x01uL << chIndex)))
                {
                    ptrSnsCxt = ptrSnsCxtArray[chIndex];

                    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
                        ptrSnsFrmCxt = ptrSnsFrmCxtArray[chIndex];
                    #endif
                    /* Update CDACs based on scan result */
                    ptrCdac = &ptrSnsCxt->cdacComp;
                    if (CY_CAPSENSE_CSD_GROUP == sensingGroup)
                    {
                        if ((uint32_t)ptrSnsCxt->raw < *ptrRawTarget)
                        {
                            *ptrCdac &= ~(uint8_t)calMaskPast;
                        }
                    }
                    else
                    {
                        if ((uint32_t)ptrSnsCxt->raw >= *ptrRawTarget)
                        {
                            *ptrCdac &= ~(uint8_t)calMaskPast;
                        }
                    }
                    *ptrCdac |= (uint8_t)calMaskNext;
                    if (0u == *ptrCdac)
                    {
                        (*ptrCdac)++;
                    }

                    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
                        if (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode)
                        {
                            *ptrSnsFrmCxt &= (uint32_t)~(MSC_SNS_CDAC_CTL_SEL_RE_Msk | MSC_SNS_CDAC_CTL_SEL_CO_Msk);
                            *ptrSnsFrmCxt |= (((uint32_t)(*ptrCdac)) << MSC_SNS_CDAC_CTL_SEL_RE_Pos);
                        }
                        else
                        {
                            *ptrSnsFrmCxt &= (uint32_t)~MSC_SNS_CDAC_CTL_SEL_CO_Msk;
                            *ptrSnsFrmCxt |= (((uint32_t)(*ptrCdac)) << MSC_SNS_CDAC_CTL_SEL_CO_Pos);
                        }
                    #else
                        (void)autoCalibrMode;
                    #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */
                }
                ptrRawTarget++;
            }

            /* CTRL_MUX + CSX */
            #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                /* Dummy scan */
                if (CY_CAPSENSE_CSX_GROUP == sensingGroup)
                {
                    calibStatus |= Cy_CapSense_ScanSlots(scanSlotId, 1u, context);
                    calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);
                }
            #endif

            /* Scan all sensors in slot */
            calibStatus |= Cy_CapSense_ScanSlots(scanSlotId, 1u, context);
            calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);

            for (chIndex = 0u; chIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; chIndex++)
            {
                wdIndex = context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * chIndex)].wdId;
                if (CY_CAPSENSE_SLOT_SHIELD_ONLY > wdIndex)
                {
                    snsIndex = context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * chIndex)].snsId;
                    Cy_CapSense_PreProcessSensor(wdIndex, snsIndex, context);
                }
            }

            /* Switches to the next cycle */
            calMaskPast = calMaskNext;
            calMaskNext >>= 1u;

        }
        while (calMaskPast != 0u);
    }

    return calibStatus;
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_NormalizeCdac
****************************************************************************//**
*
* Finds maximum reference CDAC value within a widget and stores it to the
* proper location in widget structure.
*
* The function supposes to be called after CDAC calibration in single CDAC mode
* when compensation CDAC is disabled and the CompCDAC field is used to store
* reference CDAC.
*
* Additionally, the function reduces reference CDAC value twice if final user's
* auto-calibration has enabled compensation CDAC.
*
* \param widgetId
* Specifies the desired widget ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_NormalizeCdac(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint8_t maxRefCdac = 0u;
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        uint8_t maxRowRefCdac = 0u;
    #endif
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdCfg->ptrSnsContext;
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        uint32_t * ptrSnsFrmCxt;
        uint32_t cdacTmp;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            for (i = 0u; i < ptrWdCfg->numCols; i++)
            {
                if (ptrSnsCxt->cdacComp > maxRefCdac)
                {
                    maxRefCdac = ptrSnsCxt->cdacComp;
                }
                ptrSnsCxt->cdacComp = 0u;
                ptrSnsCxt++;
            }
            for (i = ptrWdCfg->numCols; i < ptrWdCfg->numSns; i++)
            {
                if (ptrSnsCxt->cdacComp > maxRowRefCdac)
                {
                    maxRowRefCdac = ptrSnsCxt->cdacComp;
                }
                ptrSnsCxt->cdacComp = 0u;
                ptrSnsCxt++;
            }
            /* Aligns rows and cols RefCDAC if enabled */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_ROW_COL_ALIGN_EN)
                if (maxRefCdac < maxRowRefCdac)
                {
                    maxRefCdac = maxRowRefCdac;
                }
                maxRowRefCdac = maxRefCdac;
            #endif

            /* Normalizes RefCDAC if CompCDAC is enabled */
            #if (0u != CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                /* Skip CDAC splitting for SmartSense as not needed */
                if (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK))
                {
                    maxRefCdac = (maxRefCdac + 1u) >> 1u;
                    maxRowRefCdac = (maxRowRefCdac + 1u) >> 1u;
                }
            #endif
            ptrWdCfg->ptrWdContext->cdacRef = maxRefCdac;
            ptrWdCfg->ptrWdContext->rowCdacRef = maxRowRefCdac;
        }
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
        {
            /* CSX Widget */
            for (i = 0u; i < ptrWdCfg->numSns; i++)
            {
                if (ptrSnsCxt->cdacComp > maxRefCdac)
                {
                    maxRefCdac = ptrSnsCxt->cdacComp;
                }
                ptrSnsCxt->cdacComp = 0u;
                ptrSnsCxt++;
            }
            /* Normalizes RefCDAC if CompCDAC is enabled */
            #if (0u != CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                maxRefCdac = (maxRefCdac + 1u) >> 1u;
            #endif
            ptrWdCfg->ptrWdContext->cdacRef = maxRefCdac;
        }
    #endif

    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        ptrSnsFrmCxt = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
        for (i = 0u; i < (CY_CAPSENSE_SLOT_COUNT * CY_CAPSENSE_TOTAL_CH_NUMBER); i++)
        {
            if (widgetId == context->ptrScanSlots[i].wdId)
            {
                cdacTmp = ptrWdCfg->ptrWdContext->cdacRef;
                if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) &&
                    (ptrWdCfg->numCols <= context->ptrScanSlots[i].snsId))
                {
                    cdacTmp = ptrWdCfg->ptrWdContext->rowCdacRef;
                }
                *ptrSnsFrmCxt &= (uint32_t)~(MSC_SNS_CDAC_CTL_SEL_RE_Msk | MSC_SNS_CDAC_CTL_SEL_CO_Msk);
                *ptrSnsFrmCxt |= (uint32_t)(cdacTmp << MSC_SNS_CDAC_CTL_SEL_RE_Pos);
            }
            ptrSnsFrmCxt = &ptrSnsFrmCxt[CY_MSC_6_SNS_REGS];
        }
    #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */

    return CY_CAPSENSE_STATUS_SUCCESS;
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateCompDivider
****************************************************************************//**
*
* This function adjusts the compensation CDAC divider for the specified widget.
*
* The function scans all widget sensors with fixed reference CDAC and maximum
* compensation CDAC values starting from the compensation CDAC divider value
* equal to the snsClk divider (Kcomp = snsClkDiv / CompDiv = 1). If raw counts
* are over the raw target, the function decreases compensation CDAC divider
* value providing Kcomp as integer until raw counts are less than the raw
* target.
*
* The function does not consider multi-frequency feature. I.e. only the first
* (base) frequency channel is used.
*
* \param widgetId
* Specifies the desired widget ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*                                      The compensation CDAC divider is adjusted
*                                      for the specified widget.
* - CY_CAPSENSE_STATUS_CALIBRATION_FAIL - The compensation CDAC divider can not be
*                                         adjusted for the specified widget.
* - CY_CAPSENSE_STATUS_BAD_PARAM     - Invalid parameter(s).
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateCompDivider(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_SUCCESS;

    uint32_t curChIndex;
    uint32_t curSlotIndex;
    uint32_t skipSlotFlag;
    uint32_t snsOverflow = 0u;
    uint32_t snsIndex;
    uint32_t rawTemp;
    uint32_t target;
    uint32_t j;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    uint32_t compDivDefault = ptrWdCfg->ptrWdContext->snsClk;

    /* Sets Max CompCDAC Code within specified widget */
    for (j = 0u; j < ptrWdCfg->numSns; j++)
    {
        ptrWdCfg->ptrSnsContext[j].cdacComp = CY_CAPSENSE_CDAC_MAX_CODE;
    }
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        Cy_CapSense_SetMaxCompCdac(widgetId, context);
    #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */

    /* Defines initial value for Compensation Divider including rows and cols */
    #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
        (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) &&
            (((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdCfg->wdType) ||
             ((uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E == ptrWdCfg->wdType)))
        {
            if (compDivDefault > ptrWdCfg->ptrWdContext->rowSnsClk)
            {
                compDivDefault = ptrWdCfg->ptrWdContext->rowSnsClk;
            }
        }
        else
        {
            /* Initializes unused rowSnsClk for further optimization */
            ptrWdCfg->ptrWdContext->rowSnsClk = ptrWdCfg->ptrWdContext->snsClk;
        }
    #else
        /* Initializes unused rowSnsClk for further optimization */
        ptrWdCfg->ptrWdContext->rowSnsClk = ptrWdCfg->ptrWdContext->snsClk;
    #endif

    target = context->ptrInternalContext->intrCsdRawTarget;
    if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
    {
        target = context->ptrInternalContext->intrCsxRawTarget;
    }
    /* Recalculate maxRawCount accordingly to frequency channel */
    target = (target * ptrWdCfg->ptrWdContext->maxRawCount) / CY_CAPSENSE_PERCENTAGE_100;

    compDivDefault++;

    /*
     * Loop through all Compensation Divider values in descending order.
     *
     * Loop ends when CompDiv value equal 1 or Calibration fails.
     * Switches to the next iteration is done when kComp is not an integer
     * or calibration overflows.
     */
    do
    {
        /* Switches to the next possible divider */
        compDivDefault--;
        /* Checks if divider is valid */
        if (((ptrWdCfg->ptrWdContext->snsClk % compDivDefault) != 0u) ||
            ((ptrWdCfg->ptrWdContext->rowSnsClk % compDivDefault) != 0u))
        {
            continue;
        }

        ptrWdCfg->ptrWdContext->cdacCompDivider = (uint16_t)compDivDefault;
        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
            Cy_CapSense_SetCompDivider(widgetId, context);
        #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */

        snsOverflow = 0u;
        for (curSlotIndex = 0u; curSlotIndex < CY_CAPSENSE_SLOT_COUNT; curSlotIndex++)
        {
            skipSlotFlag = 1u;
            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
            {
                /* Skip slots without specified widget ID */
                if (widgetId == context->ptrScanSlots[curSlotIndex + (curChIndex * CY_CAPSENSE_SLOT_COUNT)].wdId)
                {
                    skipSlotFlag = 0u;
                    break;
                }
            }

            if (0u == skipSlotFlag)
            {
                /* CTRL_MUX + CSX */
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    /* Dummy scan */
                    if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                    {
                        calibStatus |= Cy_CapSense_ScanSlots(curSlotIndex, 1u, context);
                        calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);
                    }
                #endif
                calibStatus |= Cy_CapSense_ScanSlots(curSlotIndex, 1u, context);
                calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);

                /* Checks only the specified widget in the current slot */
                for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                {
                    if (widgetId == context->ptrScanSlots[curSlotIndex + (curChIndex * CY_CAPSENSE_SLOT_COUNT)].wdId)
                    {
                        snsIndex = context->ptrScanSlots[curSlotIndex + (curChIndex * CY_CAPSENSE_SLOT_COUNT)].snsId;

                        Cy_CapSense_PreProcessSensor(widgetId, snsIndex, context);

                        rawTemp = ptrWdCfg->ptrSnsContext[snsIndex].raw;

                        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                        {
                            if (rawTemp > target)
                            {
                                snsOverflow = 1u;
                            }
                        }
                        else
                        {
                            if (rawTemp <= target)
                            {
                                snsOverflow = 1u;
                            }
                        }
                    }
                }
                if (0u != snsOverflow)
                {
                    /* If at least one sensor of the specified widget has
                     * Raw Count too high (no enough compensation level)
                     * then switch to the next compensation divider value.
                     */
                    break;
                }
            }
        }
        /* Finish the searching if no overflow across all sensors of the specified widget */
        if (0u == snsOverflow)
        {
            break;
        }
    } while ((compDivDefault > CY_CAPSENSE_CDAC_COMP_DIV_MIN_MSCV3) &&
             (CY_CAPSENSE_STATUS_SUCCESS == calibStatus));

    return calibStatus;
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) ||\
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_VerifyCalibration
****************************************************************************//**
*
* Verifies that the calibrated widgets meets the configured conditions.
*
* This function checks whether raw count of each sensor of the specified widgets
* is within the raw count range defined by raw count target and +/- calibration
* error.
*
* \param widgetId
* Specifies the desired widget ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_VerifyCalibration(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint32_t snsIndex;
    uint32_t temp;
    uint32_t lowerLimit;
    uint32_t upperLimit;
    uint32_t target;
    uint32_t calibrationError;
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    const cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdCfg->ptrSnsContext;

    /* CTRL_MUX + CSX */
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        /* Dummy scan */
        if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
        {
            calibStatus |= Cy_CapSense_ScanSlots(ptrWdCfg->firstSlotId, 1u, context);
            calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);
        }
    #endif

    calibStatus |= Cy_CapSense_ScanSlots(ptrWdCfg->firstSlotId, ptrWdCfg->numSlots, context);
    calibStatus |= Cy_CapSense_WaitEndScan(CY_CAPSENSE_CALIBRATION_TIMEOUT, context);
    Cy_CapSense_PreProcessWidget(widgetId, context);

    if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
    {
        /* Calculate acceptable raw count range based on the resolution, target and error */
        lowerLimit = 0u;
        calibrationError = context->ptrCommonConfig->csxCalibrationError;
        target = context->ptrInternalContext->intrCsxRawTarget;
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            calibrationError = context->ptrCommonConfig->csdCalibrationError;
            target = context->ptrInternalContext->intrCsdRawTarget;
        }
        if (target > calibrationError)
        {
            lowerLimit = target - calibrationError;
        }
        upperLimit = target + calibrationError;
        if (upperLimit > CY_CAPSENSE_PERCENTAGE_100)
        {
            upperLimit = CY_CAPSENSE_PERCENTAGE_100;
        }

        /* Finds max raw count value */
        temp = context->ptrWdContext[widgetId].maxRawCount;

        /* Checks raw count of each sensor */
        ptrSnsCxt = &ptrWdCfg->ptrSnsContext[0u];
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            for (snsIndex = 0u; snsIndex < ptrWdCfg->numCols; snsIndex++)
            {
                if ((ptrSnsCxt->raw < ((temp * lowerLimit) / CY_CAPSENSE_PERCENTAGE_100)) ||
                    (ptrSnsCxt->raw > ((temp * upperLimit) / CY_CAPSENSE_PERCENTAGE_100)))
                {
                    calibStatus |= CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL;
                    break;
                }
                ptrSnsCxt++;
            }
            temp = context->ptrWdContext[widgetId].maxRawCountRow;

            for (snsIndex = ptrWdCfg->numCols; snsIndex < ptrWdCfg->numSns; snsIndex++)
            {
                if ((ptrSnsCxt->raw < ((temp * lowerLimit) / CY_CAPSENSE_PERCENTAGE_100)) ||
                    (ptrSnsCxt->raw > ((temp * upperLimit) / CY_CAPSENSE_PERCENTAGE_100)))
                {
                    calibStatus |= CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL;
                    break;
                }
                ptrSnsCxt++;
            }
        }
        else
        {
            for (snsIndex = 0u; snsIndex < ptrWdCfg->numSns; snsIndex++)
            {
                if ((ptrSnsCxt->raw < ((temp * lowerLimit) / CY_CAPSENSE_PERCENTAGE_100)) ||
                    (ptrSnsCxt->raw > ((temp * upperLimit) / CY_CAPSENSE_PERCENTAGE_100)))
                {
                    calibStatus |= CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL;
                    break;
                }
                ptrSnsCxt++;
            }
        }
    }

    return calibStatus;
}
#endif


#if ((CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) &&\
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_SetCompDivider
****************************************************************************//**
*
* Sets the desired Compensation divider into sensor frames.
*
* This function takes Compensation Divider value from widget structure and
* applies it to all widget's sensors in sensor frame structures.
* This function must not be called when AMUX mode is configured.
*
* \param widgetId
* Specifies widget ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCompDivider(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint32_t * currentFramePtr;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots = context->ptrScanSlots;
    uint32_t compensationCdacDivider = context->ptrWdContext[widgetId].cdacCompDivider;

    if (0u != compensationCdacDivider)
    {
        compensationCdacDivider--;
    }

    currentFramePtr = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_SCAN_CTL_INDEX];
    for (i = 0u; i < (CY_CAPSENSE_SLOT_COUNT * CY_CAPSENSE_TOTAL_CH_NUMBER); i++)
    {
        if (ptrScanSlots->wdId == widgetId)
        {
            *currentFramePtr &= ~MSC_SNS_SCAN_CTL_COMP_DIV_Msk;
            *currentFramePtr |= (uint32_t)(compensationCdacDivider << MSC_SNS_SCAN_CTL_COMP_DIV_Pos);
        }
        ptrScanSlots++;
        currentFramePtr = &currentFramePtr[CY_MSC_6_SNS_REGS];
    }
}
#endif


#if ((CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) &&\
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_SetMaxCompCdac
****************************************************************************//**
*
* Sets the maximum Compensation CDAC code into sensor frames.
*
* This function applies maximum Compensation CDAC code for all
* sensors within specified widget in sensor frame structures.
* This function must not be called when AMUX mode is configured.
*
* \param widgetId
* Specifies widget ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
*
*******************************************************************************/
void Cy_CapSense_SetMaxCompCdac(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint32_t * currentFramePtr;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots = context->ptrScanSlots;

    currentFramePtr = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
    for (i = 0u; i < (CY_CAPSENSE_SLOT_COUNT * CY_CAPSENSE_TOTAL_CH_NUMBER); i++)
    {
        if (ptrScanSlots->wdId == widgetId)
        {
            *currentFramePtr |= MSC_SNS_CDAC_CTL_SEL_CO_Msk;
        }
        ptrScanSlots++;
        currentFramePtr = &currentFramePtr[CY_MSC_6_SNS_REGS];
    }
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_WaitEndScan
****************************************************************************//**
*
* Waits till end of scan or till the provided timeout.
*
* \param timeout
* Watchdog timeout in microseconds.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_TIMEOUT       - The software watchdog timeout occurred
*                                      during the scan, the scan was not completed.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_WaitEndScan(
                uint32_t timeout,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_BAD_PARAM;

    /* Approximate duration of Wait For Scan */
    uint32_t isBusyLoopDuration = 5uL;

    uint32_t cpuFreqMHz;
    uint32_t watchdogCounter;

    if (NULL != context)
    {
        status = CY_CAPSENSE_STATUS_SUCCESS;
        cpuFreqMHz = context->ptrCommonConfig->cpuClkHz / CY_CAPSENSE_CONVERSION_MEGA;

        /* Init Watchdog Counter to prevent a hang */
        watchdogCounter = Cy_CapSense_WatchdogCyclesNum(timeout, cpuFreqMHz, isBusyLoopDuration);
        while (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(context))
        {
            if (0uL == watchdogCounter)
            {
                status = CY_CAPSENSE_STATUS_TIMEOUT;
                break;
            }
            watchdogCounter--;
        }
    }

    return status;
}


#if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
/*******************************************************************************
* Function Name: Cy_CapSense_SlotPinState_V3
****************************************************************************//**
*
* The internal function that configures the specified electrode to the desired state
* in the specified Active slot by updating the CAPSENSE&trade; configuration.
*
* See the Cy_CapSense_SlotPinState() description for details.
*
* \note
* This function is available for the fifth-generation (only
* for CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) and the fifth-generation
* low power CAPSENSE&trade;. For the fourth-generation CAPSENSE&trade; and
* the fifth-generation CAPSENSE&trade; with
* CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD use Cy_CapSense_SetPinState().
* For Low Power slots for the fifth-generation low power CAPSENSE&trade;
* use Cy_CapSense_LpSlotPinState().
*
* \param slotId
* The slot ID to change the pin state for (Active slot ID for
* the fifth-generation low power CAPSENSE&trade;).
*
* \param ptrEltdCfg
* The pointer to an electrode the all pins states of which will be configured
* as pinState parameter.
*
* \param pinState
* The parameter have different values for the fifth- and the fifth-generation
* low power CAPSENSE&trade;.
*
* The desired pins state for CSX widget electrodes can be:
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_RX - Rx electrode.
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_TX - Tx electrode.
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_GND - Grounded.
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_TX_NEGATIVE - Negative Tx electrode
*     (for multi-phase TX method).
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z - Unconnected (high-z).
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2 - Connected to VDDA/2.
*
* The desired pins state for CSD widget electrodes can be:
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS - Self-cap sensor.
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z - Unconnected (high-z).
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_GND - Grounded.
* * CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD - Shield is routed to the pin.
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
* \funcusage
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_SlotPinState
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SlotPinState_V3(
                uint32_t slotId,
                const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                uint32_t pinState,
                cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint32_t mask;
    uint32_t * ptrSnsFrm;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_CONFIG;
    
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        uint32_t idCounter;
        uint32_t wdIndex;
    #endif
    
    mask = 0u;
    ptrSnsFrm = &context->ptrSensorFrameContext[(((uint32_t)ptrEltdCfg->chId * CY_CAPSENSE_SLOT_COUNT) + slotId) *
                                                CY_MSC_6_SNS_REGS];
    capStatus = CY_CAPSENSE_STATUS_SUCCESS;

    for (i = 0u; i < ptrEltdCfg->numPins; i++)
    {
        mask |= 0x01uL << ptrEltdCfg->ptrPin[i].padNumber;
    }

    for (i = 0u; i < CY_CAPSENSE_CTRLMUX_PIN_STATE_MASK_NUMBER; i++)
    {
        ptrSnsFrm[i] &= ~mask;
        if (0u != (pinState & (1uL << (CY_CAPSENSE_CTRLMUX_PIN_STATE_MASK_NUMBER - 1u - i))))
        {
            ptrSnsFrm[i] |= mask;
        }
    }

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        wdIndex = context->ptrScanSlots[slotId].wdId;
        if (CY_CAPSENSE_CSX_GROUP == context->ptrWdConfig[wdIndex].senseMethod)
        {
            if (CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2 == pinState)
            {
                idCounter = context->ptrInternalContext->mapSenseMethod[CY_CAPSENSE_CSX_RM_SENSING_METHOD_INDEX];
                if (CY_CAPSENSE_ENABLE == context->ptrWdContext[wdIndex].cdacDitherEn)
                {
                    /* Shift sense mode to cdac dither enabled modes */
                    idCounter = context->ptrInternalContext->mapSenseMethod[CY_CAPSENSE_CSX_RM_SENSING_METHOD_INDEX + CY_CAPSENSE_REG_MODE_DITHERING];
                }        
                /* Close the reference to filter switch */
                context->ptrBaseFrameContext->mode[idCounter].swSelTop |= MSC_MODE_SW_SEL_TOP_RMF_Msk;
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                    context->ptrBaseFrameContext->mode[idCounter].swSelSh = CY_CAPSENSE_FW_CSX_VDDA2_AMUX_MODE_SW_SEL_SH_VALUE;
                #else
                    context->ptrBaseFrameContext->mode[idCounter].swSelSh = CY_CAPSENSE_FW_CSX_VDDA2_CTLMUX_MODE_SW_SEL_SH_VALUE;
                #endif
                (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_UNDEFINED, context);
            }
        }
    #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

    /* Set undefined value to initiate frame re-configuration in Cy_CapSense_ScanSlots() function */
    context->ptrInternalContext->currentSlotIndex = CY_CAPSENSE_SLOT_COUNT_MAX_VALUE;

    return capStatus;
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_SwitchHwConfiguration
****************************************************************************//**
*
* Configures CAPSENSE&trade;-related hardware to the specified state.
*
* \param configuration
* The state of CAPSENSE&trade;-related hardware:
* * CY_CAPSENSE_HW_CONFIG_UNDEFINED.
* * CY_CAPSENSE_HW_CONFIG_CAPTURED_DEFAULT.
* * CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS      - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM    - At least one of input parameters is not valid.
* - CY_CAPSENSE_STATUS_HW_BUSY      - The MSC HW block is busy and cannot be
*                                     configured.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SwitchHwConfiguration(
                uint32_t configuration,
                cy_stc_capsense_context_t * context)
{
    uint32_t curChIndex;
    cy_en_msc_status_t mscStatus;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;

    if (context->ptrInternalContext->hwConfigState != configuration)
    {
        /* Turn off the previous HW configuration */
        switch (context->ptrInternalContext->hwConfigState)
        {
            case CY_CAPSENSE_HW_CONFIG_UNDEFINED:
                break;
            case CY_CAPSENSE_HW_CONFIG_CAPTURED_DEFAULT:
                break;
            case CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING:
                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                    if (CY_CAPSENSE_SCAN_SNGL_SLOT == context->ptrInternalContext->scanSingleSlot)
                    {
                        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                        {
                            /* Disconnect the scanned sensor */
                            Cy_CapSense_DisconnectSensor(curChIndex, context);
                        }
                    }
                #endif
                break;
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
                case CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY:
                    Cy_CapSense_BistDisableMode(context);
                    break;
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */
            case CY_CAPSENSE_HW_CONFIG_CHANNEL_SATURATION:
                break;
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN)
                case CY_CAPSENSE_HW_CONFIG_AUTO_DITHERING:
                    break;
            #endif
            default:
                capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
                break;
        }
        if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
        {
            /* Turn on the desired HW configuration */
            switch (configuration)
            {
                case CY_CAPSENSE_HW_CONFIG_UNDEFINED:
                    break;
                case CY_CAPSENSE_HW_CONFIG_CAPTURED_DEFAULT:
                    break;
                case CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING:
                    Cy_CapSense_SetModClkDivider(context->ptrInternalContext->modClk, context);
                    Cy_CapSense_SetIOsInDefaultState(context);
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                        Cy_CapSense_SetShieldPinsInDefaultState(context);
                    #endif

                    #if (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE)
                        (void)Cy_CapSense_InitializeDmaResource(context);
                        /* Set undefined value to initiate DMA configuration in Cy_CapSense_ScanSlots() function */
                        context->ptrInternalContext->currentSlotIndex = CY_CAPSENSE_SLOT_COUNT_MAX_VALUE;
                    #endif /* (CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN == CY_CAPSENSE_SCAN_MODE) */

                    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                    {
                        context->ptrActiveScanSns[curChIndex].currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;

                        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                            Cy_CapSense_SetCmodInAmuxModeState(curChIndex, context);
                        #else
                            Cy_CapSense_SetCmodInDefaultState(curChIndex, context);
                        #endif

                        /* MSCv3 IP Block Base Register Configuration */
                        mscStatus = Cy_MSC_Configure(ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase,
                                                    &context->ptrBaseFrameContext[curChIndex],
                                                    CY_MSC_CAPSENSE_KEY,
                                                    ptrCommonCfg->ptrChConfig[curChIndex].ptrHwContext);
                        if (CY_MSC_SUCCESS != mscStatus)
                        {
                            capStatus = CY_CAPSENSE_STATUS_HW_BUSY;
                            break;
                        }

                        /* Clear all pending interrupts of the MSC HW block */
                        ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
                        (void)ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR;
                    }
                    break;
                case CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY:
                    Cy_CapSense_SetIOsInDefaultState(context);

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                        Cy_CapSense_SetShieldPinsInDefaultState(context);
                    #endif

                    for (curChIndex = 0u; curChIndex < context->ptrCommonConfig->numChannels; curChIndex++)
                    {
                        context->ptrActiveScanSns[curChIndex].currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
                        Cy_CapSense_SetCmodInDefaultState(curChIndex, context);
                        /* Clear all pending interrupts of the MSC HW block */
                        ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
                        (void)ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR;
                    }
                    break;
                case CY_CAPSENSE_HW_CONFIG_CHANNEL_SATURATION:
                    capStatus = Cy_CapSense_ConfigureSaturationMode(context);
                    break;
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN)
                    case CY_CAPSENSE_HW_CONFIG_AUTO_DITHERING:
                        capStatus = Cy_CapSense_ConfigureAutoDitherMode(context);
                        break;
                #endif
                default:
                    capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
                    break;
            }
        }
        if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
        {
            context->ptrInternalContext->hwConfigState = (uint8_t)configuration;
        }
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_InitializeSourceSenseClk
****************************************************************************//**
*
* Performs the Sense Clock source and the LFSR auto-selection functionality.
* Validates the Sense Clock configuration.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG - Sense Clock Divider is out of the valid
*                                         range for the specified Clock source
*                                         configuration.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_InitializeSourceSenseClk(const cy_stc_capsense_context_t * context)
{
    uint32_t wdIndex;
    uint32_t snsClkSrc;
    uint32_t prsConfigured = 0u;
    uint32_t snsClkDivMin;
    uint32_t lfsrRange;

    uint32_t retVal = CY_CAPSENSE_STATUS_SUCCESS;

    cy_stc_capsense_widget_context_t * ptrWdCxt;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = context->ptrWdConfig;

    for (wdIndex = 0u; wdIndex < context->ptrCommonConfig->numWd; wdIndex++)
    {
        ptrWdCxt = ptrWdCfg->ptrWdContext;
        snsClkSrc = ptrWdCxt->snsClkSource;

        if ((0u != (snsClkSrc & CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK)) ||
           (CY_CAPSENSE_CLK_SOURCE_SSC == (snsClkSrc & CY_CAPSENSE_CLK_SOURCE_MASK)))
        {
            if (0u != (ptrWdCxt->lfsrBits & CY_CAPSENSE_LFSR_BITS_AUTO_MASK))
            {
                /*
                * Execute the LFSR range auto-selection functionality when the
                * Sense Clock source parameter is configured with the SSC or
                * SSC-auto option, and the LFSR range parameter is configured with
                * the auto option.
                */
                ptrWdCxt->lfsrBits = (uint8_t)Cy_CapSense_GetLfsrBitsAuto(ptrWdCfg, context);
            }

            if (0u != (snsClkSrc & CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK))
            {
                /*
                * Execute SSC auto-selection functionality when the
                * Sense Clock source parameter is configured with the SSC-Auto
                * option. This routine performs checking if the SSC clock source
                * is valid for the widget, specified by the input parameter.
                */
                ptrWdCxt->snsClkSource = (uint8_t)Cy_CapSense_GetClkSrcSSCAuto(ptrWdCfg, context);
            }
        }
        else if (0u != (snsClkSrc & CY_CAPSENSE_CLK_SOURCE_PRS_AUTO_MASK))
        {
            /*
            * Execute PRS auto-selection functionality when the
            * Sense Clock source parameter is configured with the PRS-Auto.
            * This routine performs checking if the PRS clock source is valid
            * for the widget, specified by the input parameter.
            */
            ptrWdCxt->snsClkSource = (uint8_t)Cy_CapSense_GetClkSrcPRSAuto(ptrWdCxt, context);
        }
        else
        {
            /*
            * No action required. The Sense Clock source and the LFSR range
            * parameter are configured with the fixed option (Direct, PRS or
            * SSC - for the Sense Clock source parameter and [-2; 1], [-4; 3],
            * [-8; 7] and [-16; 15] - for the LFSR range parameter.)
            */
        }

        /* Determine the MIN valid Sense Clock divider for the used sensing method. */
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            snsClkDivMin = CY_CAPSENSE_4PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
        }
        else
        {
            snsClkDivMin = CY_CAPSENSE_2PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
        }

        /*
        * Update the MIN valid Sense Clock divider with the used clock dithering
        * range if the Sense Clock source parameter is configured with the SSC
        * option.
        */
        if (CY_CAPSENSE_CLK_SOURCE_SSC == (snsClkSrc & CY_CAPSENSE_CLK_SOURCE_MASK))
        {
            lfsrRange = ptrWdCxt->lfsrBits;
            lfsrRange &= (uint32_t)(~((uint32_t)CY_CAPSENSE_LFSR_BITS_AUTO_MASK));
            snsClkDivMin += Cy_CapSense_GetLfsrDitherVal(lfsrRange, context->ptrInternalContext->lfsrScale);
        }

        /* Perform validation if the Sense Clock divider is within the valid range. */
        if ((ptrWdCxt->snsClk < snsClkDivMin) ||
           (ptrWdCxt->snsClk > CY_CAPSENSE_DIRECT_SNS_CLOCK_DIVIDER_MAX))
        {
            retVal = CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG;
        }

        #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
            (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
            if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) &&
                (((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdCfg->wdType) ||
                ((uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E == ptrWdCfg->wdType)))
            {
                if ((ptrWdCxt->rowSnsClk < snsClkDivMin) ||
                   (ptrWdCxt->rowSnsClk > CY_CAPSENSE_DIRECT_SNS_CLOCK_DIVIDER_MAX))

                {
                    retVal = CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG;
                }
            }
        #endif

        if (CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG == retVal)
        {
            break;
        }

        if (0u != (CY_CAPSENSE_CLK_SOURCE_PRS & ptrWdCxt->snsClkSource))
        {
            prsConfigured++;
        }
        ptrWdCfg++;
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == retVal)
    {
        /* Set pro wait and epi cycle number to max in case of PRS is configured at least for one widget */
        if (0u != prsConfigured)
        {
            context->ptrInternalContext->numEpiCycles = CY_CAPSENSE_MAX_PRO_EPI_PRS_CYCLE_NUMBER;
            context->ptrInternalContext->numFineInitWaitCycles = CY_CAPSENSE_MAX_PRO_EPI_PRS_CYCLE_NUMBER;
        }
        else
        {
            context->ptrInternalContext->numEpiCycles = context->ptrCommonConfig->numEpiCycles;
            context->ptrInternalContext->numFineInitWaitCycles = CY_CAPSENSE_NUM_FINE_INIT_CYCLES;
        }
    }

    return retVal;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetLfsrBitsAuto
****************************************************************************//**
*
* Selects the number of LSB bits to use from the LSFR to provide the clock
* dithering variation on the base period if the snsClkSource member of the
* cy_stc_capsense_widget_context_t is configured with the
* CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK mask or with the or the
* CY_CAPSENSE_CLK_SOURCE_SSC value.
*
* \param ptrWdConfig
* Specifies the pointer to a widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetLfsrBitsAuto(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context)
{
    uint32_t lfsrScale;
    uint32_t lfsrRange;
    uint32_t ditherLimitPercents;
    uint32_t snsClkDividerMin;
    uint32_t snsClkDividerTmp;

    const cy_stc_capsense_widget_context_t * ptrWdCxt;

    ptrWdCxt = ptrWdConfig->ptrWdContext;
    snsClkDividerTmp = ptrWdCxt->snsClk;
    lfsrScale = context->ptrInternalContext->lfsrScale;
    ditherLimitPercents = ptrWdConfig->lfsrDitherLimit;

    #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
        (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod) &&
           (((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdConfig->wdType) ||
            ((uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E == ptrWdConfig->wdType)))
        {
            /*
            * Obtain the MIN Sense Clock divider within the two-dimension CSD widget.
            * The LFSR range should be determined, basing on the MIN Sense Clock
            * divider in order to be sufficient for rows and columns.
            */
            if (snsClkDividerTmp > ptrWdCxt->rowSnsClk)
            {
                snsClkDividerTmp = ptrWdCxt->rowSnsClk;
            }
        }
    #endif

    /* Determine the MIN valid Sense Clock divider for the used sensing method. */
    if (CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod)
    {
        snsClkDividerMin = CY_CAPSENSE_4PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
    }
    else
    {
        snsClkDividerMin = CY_CAPSENSE_2PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
    }

    /*
    * Execute the LFSR range selection routine. This routine determines
    * the LFSR_BITS option, required to provide the max clock dithering variation
    * for the specified sense Clock divider, LFSR dither (in percents) and
    * LFSR scale values.
    */
    lfsrRange = Cy_CapSense_GetLfsrBitsNumber(snsClkDividerTmp, snsClkDividerMin,
                                              ditherLimitPercents, lfsrScale);

    return (lfsrRange | CY_CAPSENSE_LFSR_BITS_AUTO_MASK);
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetClkSrcSSCAuto
****************************************************************************//**
*
* Performs checking if the SSC clock source is valid for the widget, specified
* by the input parameter.
*
* \param ptrWdConfig
* Specifies the pointer to a widget configuration structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the valid clock source:
* - SSC    -if SSC is valid for the specified widget configuration.
* - Direct -if SSC is not valid for the specified widget configuration.
* The CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK bit is always set in the return
* value.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetClkSrcSSCAuto(
                const cy_stc_capsense_widget_config_t * ptrWdConfig,
                const cy_stc_capsense_context_t * context)
{
    uint32_t lfsrRange;
    uint32_t lfsrScale;
    uint32_t snsClkSrc;
    uint32_t autoSelMode;
    uint32_t ditherLimitPercents;
    uint32_t lfsrDitherCycles;
    uint32_t ditherLimitCycles;
    uint32_t lfsrPolySize;
    uint32_t snsClkDivMin;
    uint32_t subConvNumber;
    const cy_stc_capsense_widget_context_t * ptrWdCxt;
    #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
        (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        uint32_t snsClkSrcRow;
    #endif

    ptrWdCxt = ptrWdConfig->ptrWdContext;

    /*
    * Determine the whole number of sub-conversion per scan (includes Dummy and
    * Measurement conversions).
    */
    subConvNumber = ptrWdCxt->numSubConversions;
    subConvNumber += context->ptrInternalContext->numFineInitCycles;
    if (CY_CAPSENSE_16_BIT_MASK < subConvNumber)
    {
        subConvNumber = CY_CAPSENSE_16_BIT_MASK;
    }
    lfsrRange = ((uint32_t)ptrWdCxt->lfsrBits & (uint32_t)(~((uint32_t)CY_CAPSENSE_LFSR_BITS_AUTO_MASK)));
    lfsrScale = context->ptrInternalContext->lfsrScale;
    autoSelMode = ptrWdConfig->snsClkSourceAutoSelMode;
    ditherLimitPercents = ptrWdConfig->lfsrDitherLimit;

    /*
    * Determine the polynomial duration in clock cycles, e.g. the duration of 8-bit
    * polynomial is 255 clock cycles.
    */
    lfsrPolySize = Cy_CapSense_GetPolySize(context->ptrInternalContext->lfsrPoly);

    /*
    * Determine the clock dithering variation for the specifies LFSR range,
    * e.g for the [-2; 1] range the clock variation will be 2 cycles.
    */
    lfsrDitherCycles = Cy_CapSense_GetLfsrDitherVal(lfsrRange, lfsrScale);

    /* Determine the MIN valid Sense Clock divider for the used sensing method. */
    if (CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod)
    {
        snsClkDivMin = CY_CAPSENSE_4PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
    }
    else
    {
        snsClkDivMin = CY_CAPSENSE_2PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
    }

    /*
    * Determines the max allowed clock dithering variation for the specified
    * Sense Clock divider, LFSR dither (in percents) and LFSR scale values.
    */
    ditherLimitCycles = Cy_CapSense_GetLfsrDitherLimit(ptrWdCxt->snsClk,
                                    snsClkDivMin, ditherLimitPercents, lfsrScale);

    /*
    * Execute the SSC-auto selection routine. This routine validates all the
    * criteria, that are required for good SSC performance are met.
    * The CY_CAPSENSE_CLK_SOURCE_SSC will be returned if all the criteria are met,
    * the CY_CAPSENSE_CLK_SOURCE_DIRECT will be returned if not.
    */
    snsClkSrc = Cy_CapSense_RunSSCAuto(autoSelMode, lfsrDitherCycles,
                                    ditherLimitCycles, lfsrPolySize, subConvNumber);

    #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
        (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        /* Repeat the SSC-auto selection routine for the row of two-dimension CSD
         * widget.
         */
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod) &&
           (((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdConfig->wdType) ||
            ((uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E == ptrWdConfig->wdType)))
        {
            if ((uint8_t)CY_CAPSENSE_CLK_SOURCE_DIRECT != snsClkSrc)
            {
                ditherLimitCycles = Cy_CapSense_GetLfsrDitherLimit(ptrWdCxt->rowSnsClk,
                                                snsClkDivMin, ditherLimitPercents, lfsrScale);
                snsClkSrcRow = Cy_CapSense_RunSSCAuto(autoSelMode, lfsrDitherCycles,
                                                ditherLimitCycles, lfsrPolySize, (uint16_t)subConvNumber);

                if (snsClkSrc != snsClkSrcRow)
                {
                    /*
                    * If the automatically determined Sense Clock sources for
                    * rows and columns are different, the widget Sense Clock
                    * source should be set to Direct.
                    */
                    snsClkSrc = CY_CAPSENSE_CLK_SOURCE_DIRECT;
                }
            }
        }
    #endif

    return (snsClkSrc | CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK);
}


/*******************************************************************************
* Function Name: Cy_CapSense_RunSSCAuto
****************************************************************************//**
*
* Implements checking the following rules:
* 1. An LFSR value should be selected so that the max clock dither in percents
*    is limited with the value, specified by the lfsrDitherLimit parameter.
* 2. At least one full spread spectrum polynomial should pass during the scan
*    time.
* 3. The value of the number of conversions should be an integer multiple of the
*    repeat period of the polynomial, that is specified by the
*    Sense Clock LFSR Polynomial parameter.
*
* \param autoSelMode
*  Defines set of rules, to be checked in order to decide if the SSC clock
*  source is applicable for the specified parameters.
*
* \param lfsrDitherCycles
*  Dintering variation in the number of Mod Clock cycles.
*
* \param ditherLimitCycles
*  Max dither in percentage, the actual clock dithering variation should
*  be limited with.
*
* \param lfsrPolySize
*  Number of Sense Clock cycles, required to fully pass the configured
*  polynomial.
*
* \param subConvNumber
*  Number of sub-conversions(including Dummy).
*
* \return
* Returns the valid clock source:
* - CY_CAPSENSE_CLK_SOURCE_SSC    -if SSC is valid for the specified widget
*                                  configuration.
* - CY_CAPSENSE_CLK_SOURCE_DIRECT -if SSC is not valid for the specified widget
*                                  configuration.
*
*******************************************************************************/
uint32_t Cy_CapSense_RunSSCAuto(
                uint32_t autoSelMode,
                uint32_t lfsrDitherCycles,
                uint32_t ditherLimitCycles,
                uint32_t lfsrPolySize,
                uint32_t subConvNumber)
{
    uint32_t snsClkSrc = CY_CAPSENSE_CLK_SOURCE_SSC;

    /* Check if the Sense Clock variation is lower than MAX allowed. */
    if (lfsrDitherCycles > ditherLimitCycles)
    {
        snsClkSrc = CY_CAPSENSE_CLK_SOURCE_DIRECT;
    }

    /*
    * Check if at least one whole polynomial period will be performed
    * during the scan. This part of code is executed if the Clock Source
    * auto selection mode is set to Medium or Strong.
    */
    if (autoSelMode != CY_CAPSENSE_SNS_CLK_SOURCE_AUTO_SEL_MODE_WEAK)
    {
        if (subConvNumber < lfsrPolySize)
        {
            snsClkSrc = CY_CAPSENSE_CLK_SOURCE_DIRECT;
        }
    }

    /*
    * Check if the integer number of the polynomial periods will be performed
    * during the scan. This part of code is executed if the Clock Source
    * auto selection mode is set to Strong.
    */
    if (autoSelMode == CY_CAPSENSE_SNS_CLK_SOURCE_AUTO_SEL_MODE_STRONG)
    {
        if (0u != (subConvNumber % lfsrPolySize))
        {
            snsClkSrc = CY_CAPSENSE_CLK_SOURCE_DIRECT;
        }
    }

    return snsClkSrc;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetClkSrcPRSAuto
****************************************************************************//**
*
* Performs checking if the PRS clock source is valid for the widget, specified
* by the input parameter.
*
* \param ptrWdContext
*  Specifies the pointer to a widget context structure.
*
* \param context
*  The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
*  Returns the valid clock source:
* - PRS    -if SSC is valid for the specified widget configuration.
* - Direct -if SSC is not valid for the specified widget configuration.
* The CY_CAPSENSE_CLK_SOURCE_PRS_AUTO_MASK bit is always set in the return
* value.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetClkSrcPRSAuto(
                const cy_stc_capsense_widget_context_t * ptrWdContext,
                const cy_stc_capsense_context_t * context)
{
    uint32_t snsClkSrc;
    uint32_t lfsrPolySize;
    uint32_t risingEdgesNum;
    uint32_t subConvNumber;

    snsClkSrc = CY_CAPSENSE_CLK_SOURCE_PRS;

    /*
    * Determine the whole number of sub-conversion per scan (includes Dummy and
    * Measurement conversions).
    */
    subConvNumber = ptrWdContext->numSubConversions;
    subConvNumber += context->ptrInternalContext->numFineInitCycles;

    /*
    * Determine the polynomial duration in clock cycles, e.g. the duration of 8-bit
    * polynomial is 255 clock cycles.
    */
    lfsrPolySize = Cy_CapSense_GetPolySize(context->ptrInternalContext->lfsrPoly);

    /*
    * The number of rising edges per one PRS period is (polyLength + 1) / 4.
    * polyLength = 2^N - 1, where N is size of the shift register (8BIT, 12BIT, etc).
    */
    risingEdgesNum = (lfsrPolySize + 1u) >> 2u;

    /*
    * Check if at least one whole polynomial period will be performed
    * during the scan.
    */
    if (subConvNumber < risingEdgesNum)
    {
        snsClkSrc = CY_CAPSENSE_CLK_SOURCE_DIRECT;
    }

    return (snsClkSrc | CY_CAPSENSE_CLK_SOURCE_PRS_AUTO_MASK);
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetPolySize
****************************************************************************//**
*
* Calculates the number of Sense Clock cycles, required to fully pass the
* specified polynomial.
*
* \param lfsrPoly
*  Specifies the polynomial the calculation should be performed for.
*
* \return
*  Number of Sense Clock cycles, required to fully pass the specified
*  polynomial.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetPolySize(uint32_t lfsrPoly)
{
    /* Initialization the msbIndex variable with the MAX possible size of polynomial */
    uint32_t polySize = 0xFFFFu;
    uint32_t polyMsbMask = 0x8000u;

    while ((1u < polySize) && (0u == (lfsrPoly & polyMsbMask)))
    {
        polySize >>= 1u;
        polyMsbMask >>= 1u;
    }

    return polySize;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetLfsrBitsNumber
****************************************************************************//**
*
* Determines SNS_CTL.LFSR_BITS option, required to provide the max clock
* dithering variation for the specified parameters.
*
* \param snsClkDivider
* The divider value for the sense clock.
*
* \param snsClkDividerMin
* The minimal valid value of the sense clock divider.
*
* \param ditherLimitPercents
*  Max dither in percentage, the actual clock dither should be limited with.
*
* \param lfsrScale
*  Number of bits to shift the LFSR output left before adding to SENSE_DIV.
*
* \return
*  The SNS_CTL.LFSR_BITS option:
*   - CY_CAPSENSE_LFSR_BITS_RANGE_0 - Use 2 bits range: [-2,1]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_1 - Use 3 bits range: [-4,3]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_2 - Use 4 bits range: [-8,7]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_3 - Use 5 bits range: [-16,15]
*
*******************************************************************************/
uint32_t Cy_CapSense_GetLfsrBitsNumber(
                uint32_t snsClkDivider,
                uint32_t snsClkDividerMin,
                uint32_t ditherLimitPercents,
                uint32_t lfsrScale)
{
    uint32_t retVal;
    uint32_t ditherLimitCycles;

    /*
    * Determine the max allowed clock dithering variation for the specified
    * Sense Clock divider, LFSR dither (in percents) and LFSR scale values.
    */
    ditherLimitCycles = Cy_CapSense_GetLfsrDitherLimit(snsClkDivider, snsClkDividerMin,
                                                       ditherLimitPercents, lfsrScale);

    /* Based in the MAX allowed clock dithering, Determines the LFSR_BITS option. */
    if (CY_CAPSENSE_LFSR_RANGE_3_DITHER_MAX <= ditherLimitCycles)
    {
        retVal = CY_CAPSENSE_LFSR_BITS_RANGE_3;
    }
    else if (CY_CAPSENSE_LFSR_RANGE_2_DITHER_MAX <= ditherLimitCycles)
    {
        retVal = CY_CAPSENSE_LFSR_BITS_RANGE_2;
    }
    else if (CY_CAPSENSE_LFSR_RANGE_1_DITHER_MAX <= ditherLimitCycles)
    {
        retVal = CY_CAPSENSE_LFSR_BITS_RANGE_1;
    }
    else
    {
        retVal = CY_CAPSENSE_LFSR_BITS_RANGE_0;
    }

    return retVal;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetLfsrDitherVal
****************************************************************************//**
*
* Determines the clock dithering variation for the specified parameters.
*
* \param lfsrBits
*  The supported SNS_CTL.LFSR_BITS option:
*   - CY_CAPSENSE_LFSR_BITS_RANGE_0 - Use 2 bits range: [-2,1]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_1 - Use 3 bits range: [-4,3]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_2 - Use 4 bits range: [-8,7]
*   - CY_CAPSENSE_LFSR_BITS_RANGE_3 - Use 5 bits range: [-16,15]
*
* \param lfsrScale
*  Number of bits to shift the LFSR output left before adding to SENSE_DIV.
*
* \return
*  Dintering variation in the number of Mod Clock cycles.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetLfsrDitherVal(
                uint32_t lfsrBits,
                uint32_t lfsrScale)
{
    return (uint32_t)(1uL << (((lfsrBits & CY_CAPSENSE_LFSR_BITS_RANGE_MASK) + 1u) + lfsrScale));
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetLfsrDitherLimit
****************************************************************************//**
*
* Determines the max allowed clock dithering variation for the specified
* parameters.
*
* \param snsClkDivider
* The divider value for the sense clock.
*
* \param snsClkDividerMin
* The minimal valid value of the sense clock divider.
*
* \param ditherLimitPercents
* Max dither in percentage, the actual clock dither should be limited with.
*
* \param lfsrScale
*  Number of bits to shift the LFSR output left before adding to SENSE_DIV.
*
* \return
*  Dintering variation in the number of Mod Clock cycles.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetLfsrDitherLimit(
                uint32_t snsClkDivider,
                uint32_t snsClkDividerMin,
                uint32_t ditherLimitPercents,
                uint32_t lfsrScale)
{
    uint32_t ditherLimitCycles;
    uint32_t ditherLimitCyclesMax;

    /*
    * Determine the dither limit in clock cycles for the specified Dither limit
    * in percents.
    */
    ditherLimitCycles = ((ditherLimitPercents * snsClkDivider) / CY_CAPSENSE_PERCENTAGE_100);
    ditherLimitCycles >>= lfsrScale;

    /* Determine MAX allowed dither in cycles for the specified Sense Clock divider. */
    ditherLimitCyclesMax = (snsClkDivider - snsClkDividerMin) / (1u << lfsrScale);

    /*
    * Determine the minimal value to eliminate violation of allowed Sense Clock
    * divider range.
    */
    if (ditherLimitCycles > ditherLimitCyclesMax)
    {
        ditherLimitCycles = ditherLimitCyclesMax;
    }

    return ditherLimitCycles;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_SsAutoTune
****************************************************************************//**
*
* Performs the parameters auto-tuning for the optimal CAPSENSE&trade; operation when
* smart sensing algorithm is enabled.
*
* This function performs the following tasks:
* - Tune the Sense Clock optimal value.
* - Calculate the number of sub-conversions for the optimal finger capacitance.
*
* \note
* The function supposes final auto-calibration is called right after this
* function.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - Zero     - All the widgets are auto-tuned successfully.
* - Non-zero - Auto-tuning failed for any widget.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SsAutoTune(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t autoTuneStatus = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t i;
    uint32_t j;
    uint32_t watchdogCounter;
    MSC_Type * mscBase;
    uint32_t curChIndex;
    uint32_t rawCountMax;
    uint32_t numberSubConv;
    cy_stc_capsense_hw_smartsense_config_t autoTuneConfig;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    cy_stc_capsense_widget_context_t * ptrWdCxt;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    cy_stc_capsense_common_context_t * ptrCommonCxt = context->ptrCommonContext;
    #if (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u)
        uint32_t loopFlag;
        uint64_t wdIdMask;
        uint32_t maxDivider;
        uint32_t maxSubconversions;
        uint32_t wdIndex[CY_CAPSENSE_MAX_CH_NUM];
    #endif /* (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u) */

    ptrCommonCxt->status |= CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK;

    /* Step #0: Check if configuration is valid */
    if ((ptrCommonCfg->counterMode != CY_CAPSENSE_COUNTER_MODE_SATURATE) ||
        (ptrCommonCfg->numWd > CY_CAPSENSE_SMARTSENSE_WD_MAX_NUMBER))
    {
        autoTuneStatus |= CY_CAPSENSE_STATUS_BAD_CONFIG;
    }

    /* Step #1: Sets the default parameters for preliminary scans */
    ptrWdCfg = context->ptrWdConfig;
    for (i = 0u; i < ptrCommonCfg->numWd; i++)
    {
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            /* Initializes SmartSense scan configuration */
            ptrWdCxt = ptrWdCfg->ptrWdContext;
            ptrWdCxt->snsClkSource <<= CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_POS;
            ptrWdCxt->numSubConversions = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_NSUB;
            ptrWdCxt->cdacRef = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_REF_CDAC;
            ptrWdCxt->rowCdacRef = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_REF_CDAC;
            ptrWdCxt->snsClk = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK;
            ptrWdCxt->rowSnsClk = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK;
            ptrSnsCxt = ptrWdCfg->ptrSnsContext;
            for (j = 0u; j < ptrWdCfg->numSns; j++)
            {
                ptrSnsCxt->cdacComp = 0u;
                ptrSnsCxt++;
            }
        }
        ptrWdCfg++;
    }
    autoTuneStatus |= Cy_CapSense_SsInitialize(context);

    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        mscBase = ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase;

        mscBase->SCAN_CTL2 &= (uint32_t)~((uint32_t)MSC_SCAN_CTL2_NUM_EPI_CYCLES_Msk);
        mscBase->SCAN_CTL2 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                                   MSC_SCAN_CTL2_NUM_EPI_CYCLES_Pos);

        mscBase->INIT_CTL4 &= (uint32_t)~((uint32_t)MSC_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Msk);
        mscBase->INIT_CTL4 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                                   MSC_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Pos);
    }

    /* Step #2: Execute preliminary scan */
    autoTuneStatus |= Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
    autoTuneStatus |= Cy_CapSense_ScanAllSlots(context);

    /* Initialize the watchdog counter to prevent a hang */
    watchdogCounter = Cy_CapSense_WatchdogCyclesNum(CY_CAPSENSE_CALIBRATION_TIMEOUT,
                                                    (context->ptrCommonConfig->cpuClkHz / CY_CAPSENSE_CONVERSION_MEGA),
                                                    CY_CAPSENSE_LOOP_DURATION_CYCLES);

    /* Wait till the end of preliminary scans for the calculated timeout */
    autoTuneStatus |= Cy_CapSense_WaitEndScan(watchdogCounter, context);

    /*
     * Step #3: Calculate sensor capacitances, sense clock dividers, and sub-conversion numbers depending on the configured
     * serial resistances, and finger capacitances for each sensor
     */
    autoTuneConfig.modClock = ptrCommonCfg->periClkHz / context->ptrInternalContext->modClk;
    autoTuneConfig.snsResistance = CY_CAPSENSE_SMARTSENSE_CSD_RESISTANCE_CONST;
    autoTuneConfig.correctionCoeff = CY_CAPSENSE_SENSOR_CONNECTION_MODE;

    ptrWdCfg = context->ptrWdConfig;
    for (i = 0u; i < CY_CAPSENSE_WIDGET_COUNT; i++)
    {
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) && (0u != Cy_CapSense_IsWidgetEnabled(i, context)))
        {
            ptrWdCxt = ptrWdCfg->ptrWdContext;
            autoTuneConfig.kRef0 = ptrWdCxt->snsClk;
            autoTuneConfig.nSub0 = ptrWdCxt->numSubConversions;
            autoTuneConfig.refCdac = ptrWdCxt->cdacRef;
            autoTuneConfig.fingerCap = ptrWdCxt->fingerCap;

            rawCountMax = 0u;
            for (j = 0u; j < ptrWdCfg->numSns; j++)
            {
                if (rawCountMax < ptrWdCfg->ptrSnsContext[j].raw)
                {
                    rawCountMax = ptrWdCfg->ptrSnsContext[j].raw;
                }
            }

            autoTuneConfig.raw = (uint16_t)rawCountMax;
            (void)Cy_CapSense_GetSmartSenseCapacitance(&autoTuneConfig);
            autoTuneConfig.snsCapacitance /= ptrWdCfg->numChopCycles;
            (void)Cy_CapSense_GetSmartSenseFrequencyDivider(&autoTuneConfig);

            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_MULTI_FREQUENCY_WIDGET_EN)
                /* Check for sub-widget */
                if (0u != (ptrWdCfg->mfsConfig & CY_CAPSENSE_MFS_EN_MASK))
                {
                    if (0u != (ptrWdCfg->mfsConfig & CY_CAPSENSE_MFS_WIDGET_FREQ_CH_1_MASK))
                    {
                        autoTuneConfig.kRef1 += CY_CAPSENSE_CSD_MFS_DIVIDER_OFFSET_F1;
                    }

                    if (0u != (ptrWdCfg->mfsConfig & CY_CAPSENSE_MFS_WIDGET_FREQ_CH_2_MASK))
                    {
                        autoTuneConfig.kRef1 += CY_CAPSENSE_CSD_MFS_DIVIDER_OFFSET_F2;
                    }
                }
            #endif

            /* Configure widget sense clock divider by the calculated value */
            ptrWdCxt->snsClk = autoTuneConfig.kRef1;
            ptrWdCxt->rowSnsClk = autoTuneConfig.kRef1;

            (void)Cy_CapSense_GetSmartSenseNumSubconversions(&autoTuneConfig);

            /* Calculates max possible Nsub value */
            numberSubConv = MSC_RESULT_FIFO_RD_RAW_COUNT_Msk >> MSC_RESULT_FIFO_RD_RAW_COUNT_Pos;
            numberSubConv /= autoTuneConfig.kRef1;

            /* Limits Nsub to avoid RawCount overflow */
            if ((uint16_t)numberSubConv > autoTuneConfig.nSub1)
            {
                numberSubConv = autoTuneConfig.nSub1;
            }

            ptrWdCxt->numSubConversions = (uint16_t)numberSubConv / ptrWdCfg->numChopCycles;
            /* Decrease Nsub when CompCDAC is enabled as it increases the sensitivity approximately twice */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                ptrWdCxt->numSubConversions >>= 1u;
                numberSubConv = (uint32_t)ptrWdCxt->numSubConversions << 1u;
            #endif
            numberSubConv *= ptrWdCfg->numChopCycles;

            /*
             * numberSubConv contains real value including chopping and compensation.
             * autoTuneConfig.nSub1 contains initially calculated value.
             * So, correcting the initially calculated sensitivity.
             */
            autoTuneConfig.sigPFC = (uint16_t)((numberSubConv * autoTuneConfig.sigPFC) / autoTuneConfig.nSub1);
            ptrWdCxt->sigPFC = autoTuneConfig.sigPFC;
        }
        ptrWdCfg++;
    }

    /* Step #4: Align SnsClkDiv and Nsub for multi-channel solution */
    #if (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u)
        /* Implemented faster algorithm to align frequency across all slots */
        ptrWdCfg = context->ptrWdConfig;
        /* Reset the widget group to initial state, no widgets in group */
        wdIdMask = 0u;
        maxDivider = 1u;
        maxSubconversions = 1u;
        /* A single loop through all slots */
        for (i = 0u; i < CY_CAPSENSE_SLOT_COUNT; i++)
        {
            /* Save widget IDs for the current slot */
            for (j = 0u; j < CY_CAPSENSE_TOTAL_CH_NUMBER; j++)
            {
                wdIndex[j] = context->ptrScanSlots[i + (j * CY_CAPSENSE_SLOT_COUNT)].wdId;
            }

            /* If CSX then, update the current group, reset the widget group and switch to the next slot */
            loopFlag = 0u;
            for (j = 0u; j < CY_CAPSENSE_TOTAL_CH_NUMBER; j++)
            {
                if (wdIndex[j] < CY_CAPSENSE_SLOT_SHIELD_ONLY)
                {
                    if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg[wdIndex[j]].senseMethod)
                    {
                        /* Updates the current group */
                        Cy_CapSense_SsAutoTuneScanDurationAlignment(wdIdMask, maxDivider, maxSubconversions, context);
                        /* Reset the widget group to initial state, no widgets in group */
                        wdIdMask = 0u;
                        maxDivider = 1u;
                        maxSubconversions = 1u;
                        /* Sets a flag to switch to the new slot */
                        loopFlag = 1u;
                        break;
                    }
                }
            }
            if (0u != loopFlag)
            {
                continue;
            }

            /* Check if new slot has overlapping with widget group */
            loopFlag = 0u;
            for (j = 0u; j < CY_CAPSENSE_TOTAL_CH_NUMBER; j++)
            {
                if (wdIndex[j] < CY_CAPSENSE_TOTAL_WIDGET_COUNT)
                {
                    if (0u != (wdIdMask & (1uLL << wdIndex[j])))
                    {
                        /* It has, so continue the same group */
                        loopFlag = 1u;
                        break;
                    }
                }
            }

            if (0u == loopFlag)
            {
                /* Updates the current group */
                Cy_CapSense_SsAutoTuneScanDurationAlignment(wdIdMask, maxDivider, maxSubconversions, context);
                /* Reset the widget group to initial state, no widgets in group */
                wdIdMask = 0u;
                maxDivider = 1u;
                maxSubconversions = 1u;
            }

            /* Process new slot with the widget group */
            for (j = 0u; j < CY_CAPSENSE_TOTAL_CH_NUMBER; j++)
            {
                if (wdIndex[j] < CY_CAPSENSE_TOTAL_WIDGET_COUNT)
                {
                    /* Adds widget to a group */
                    wdIdMask |= 1uLL << wdIndex[j];
                    /* Stores max divider for a group */
                    if (maxDivider < ptrWdCfg[wdIndex[j]].ptrWdContext->snsClk)
                    {
                        maxDivider = ptrWdCfg[wdIndex[j]].ptrWdContext->snsClk;
                    }
                    /* Stores max number of sub-conversions for a group */
                    if (maxSubconversions < ptrWdCfg[wdIndex[j]].ptrWdContext->numSubConversions)
                    {
                        maxSubconversions = ptrWdCfg[wdIndex[j]].ptrWdContext->numSubConversions;
                    }
                }
            }
        }
        /* If no more slots, the current widget group still need to be updated */
        Cy_CapSense_SsAutoTuneScanDurationAlignment(wdIdMask, maxDivider, maxSubconversions, context);
    #endif /* (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u) */

    /* Step #5: Restore project parameters */
    ptrWdCxt = context->ptrWdContext;
    for (i = 0u; i < CY_CAPSENSE_WIDGET_COUNT; i++)
    {
        if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) && (0u != Cy_CapSense_IsWidgetEnabled(i, context)))
        {
            ptrWdCxt = ptrWdCfg->ptrWdContext;
            /* Reverting back original customer's settings */
            ptrWdCxt->snsClkSource = (uint8_t)(((uint32_t)ptrWdCxt->snsClkSource >> CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_POS) &
                                     ~CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_MASK);
        }
        ptrWdCfg++;
    }

    autoTuneStatus |= Cy_CapSense_SsInitialize(context);

    ptrCommonCxt->status &= ~CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK;

    return autoTuneStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SsAutoTuneScanDurationAlignment
****************************************************************************//**
*
* Updates specified widgets with provided SnsClk/TxClk and Nsub.
*
* \param widgetMask
* A mask of widgets. The first bit correspond to a widget with ID = 0. The
* second bit corresponds to widget with ID = 1. And so on.
*
* \param clockDivider
* Specifies a divider should be configured.
*
* \param subConversion
* Specifies a number of sub-conversions should be configured.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SsAutoTuneScanDurationAlignment(
                uint64_t widgetMask,
                uint32_t clockDivider,
                uint32_t subConversion,
                cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_widget_context_t * ptrWdCxt = context->ptrWdContext;
    uint64_t widgetMaskLocal = widgetMask;

    while (widgetMaskLocal != 0u)
    {
        if (0u != (widgetMaskLocal & 0x01u))
        {
            ptrWdCxt->snsClk = (uint16_t)clockDivider;
            ptrWdCxt->rowSnsClk = (uint16_t)clockDivider;
            ptrWdCxt->numSubConversions = (uint16_t)subConversion;
        }
        widgetMaskLocal >>= 1u;
        ptrWdCxt++;
    }
}
#endif


#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CIC2_FILTER_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_GetCIC2SamplesMax
****************************************************************************//**
*
* This internal function determines the MAX number of CIC2 samples for specified
* decimation rate that can be accumulated in the internal CIC2 HW register
* with no overflow.
*
* \param cic2Rate
* CIC2 decimation rate.
*
* \return
* The MAX allowed number of CIC2 samples.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetCIC2SamplesMax(
                uint32_t cic2Rate)
{
    uint32_t retVal;

    retVal = CY_CAPSENSE_CIC2_ACC_MAX_VAL / (cic2Rate * cic2Rate);

    return retVal;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetCIC2HwDivider
****************************************************************************//**
*
* This internal function determines the value of the divider that will be
* applied to the data, accumulated by the CIC2 HW for the specified number of
* samples.
*
* \param cic2Samples
* The number of CIC2 samples for the specified sensing parameters. This value
* can be obtained by using the Cy_CapSense_GetCIC2SamplesNum function.
*
* \return
* The CIC2 HW divider value.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetCIC2HwDivider(
                uint32_t cic2Samples)
{
    uint32_t cic2Divider;

    if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_8)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_16;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_4)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_8;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_2)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_4;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_1)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_2;
    }
    else
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_1;
    }

    return cic2Divider;
}
#endif /* #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CIC2_FILTER_EN) */


/*******************************************************************************
* Function Name: Cy_CapSense_ExecuteSaturatedScan
****************************************************************************//**
*
* This internal function executes a scan with the saturated channel to obtain
* the MAX raw count or the scan duration for the specified scan slot.
*
* \param ptrMaxRaw
* Specifies the pointer to store the resulting MAX raw count value.
*
* \param scanSlotId
* The slot ID scans will be done for.
*
* \param mode
* Saturated scan execution mode:
* - CY_CAPSENSE_SATURATED_MAX_COUNT   - Used to obtain the MAX raw count.
* - CY_CAPSENSE_SATURATED_SCAN_TIME   - Used to obtain the scan duration in terms of Mod Clock cycles.
*
* \param context
* The pointer to the context structure allocated by the user or middleware.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS      - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_TIMEOUT      - A timeout reached during the scan time measurement.
* - CY_CAPSENSE_STATUS_BAD_PARAM    - Not valid input parameter.
* - CY_CAPSENSE_STATUS_HW_BUSY      - The MSCLP HW block is busy and cannot be
*                                     switched to another mode.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ExecuteSaturatedScan(
                uint32_t * ptrMaxRaw,
                uint32_t scanSlotId,
                uint32_t mode,
                cy_stc_capsense_context_t * context)
{
    uint32_t tmpVal;
    uint32_t watchdog;
    uint32_t scanConfigTmp[CY_MSC_6_SNS_REGS];
    cy_capsense_status_t status;
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        uint32_t * ptrSnsFrame;
    #endif

    const uint32_t cpuCyclesPerLoop = 5u;
    MSC_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig[0u].ptrHwBase;

    (void)mode;

    /* Configures the MSC HW block for operation with the permanently saturated channel. */
    status = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_CHANNEL_SATURATION, context);

    /* The MSC0 HW block will be used to perform measurements. Rest of MSC HW blocks will be disabled. */
    for (tmpVal = 1u; tmpVal < CY_CAPSENSE_TOTAL_CH_NUMBER; tmpVal++)
    {
        ptrHwBase = context->ptrCommonConfig->ptrChConfig[tmpVal].ptrHwBase;
        ptrHwBase->CTL &= (~MSC_CTL_ENABLED_Msk);
    }

    ptrHwBase = context->ptrCommonConfig->ptrChConfig[0u].ptrHwBase;

    /* Disable CIC2 filtering to measure the scan duration in in terms of Mod Clock cycles */
    ptrHwBase->FILTER_CTL &= (~MSC_FILTER_CTL_FILTER_MODE_Msk);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CIC2_FILTER_EN)
        if (CY_CAPSENSE_SATURATED_MAX_COUNT == mode)
        {
            /* Keep CIC2 filtering enabled if it is used in the current MW configuration
             * to measure the MAX raw count.
             */
            ptrHwBase->FILTER_CTL |= (MSC_FILTER_CTL_FILTER_MODE_Msk);
        }
    #endif

    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        /* Obtain the scan parameters offset in the cy_capsense_sensorFrameContext[]
         * array for the slots that are part of Active widgets.
         */
        tmpVal = scanSlotId * CY_MSC_6_SNS_REGS;
        /* The code below sets up configuration of the sense control registers. */
        ptrSnsFrame = &context->ptrSensorFrameContext[tmpVal];
        scanConfigTmp[3u] = ptrSnsFrame[3u]; /* Set up the SNS_SCAN_CTL register configuration. */
        scanConfigTmp[4u] = ptrSnsFrame[4u]; /* Set up the SNS_CDAC_CTL register configuration. */
        scanConfigTmp[5u] = ptrSnsFrame[5u]; /* Set up the SNS_CTL register configuration. */
    #else
        for (tmpVal = 0u; tmpVal < CY_CAPSENSE_TOTAL_CH_NUMBER; tmpVal++)
        {
            context->ptrActiveScanSns[tmpVal].currentChannelSlotIndex = (uint16_t)scanSlotId;
        }
        (void)Cy_CapSense_GenerateSensorConfig(0u, scanSlotId, &scanConfigTmp[0u], context);
    #endif

    /* The measurement with a saturated channel will be performed with all the sensors disconnected.
     * The code below sets up the configuration of the MASK registers with 0x00000000u.
     * The SW_SEL_CSW_FUNC[0u] is configured in the scope of the Cy_CapSense_ConfigureSaturationMode()
     * routine with all the switches open.
     */
    scanConfigTmp[0u] = 0x00u; /* Set up the SNS_SW_SEL_CSW_MASK2 register configuration. */
    scanConfigTmp[1u] = 0x00u; /* Set up the SNS_SW_SEL_CSW_MASK1 register configuration. */
    scanConfigTmp[2u] = 0x00u; /* Set up the SNS_SW_SEL_CSW_MASK0 register configuration. */

    scanConfigTmp[3u] &= (uint32_t)(~MSC_SNS_SCAN_CTL_NUM_CONV_Msk); /* Disable System Chopping */

    if (status == CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* Initiate the scan in the CPU operating mode */
        Cy_MSC_ConfigureScan(ptrHwBase, CY_MSC_6_SNS_REGS, &scanConfigTmp[0u]);

        /* Disable clock sync and external frame start */
        ptrHwBase->CTL &= ~(MSC_CTL_CLK_SYNC_EN_Msk | MSC_CTL_EXT_FRAME_START_EN_Msk);

        /* Clear all pending interrupts */
        ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
        (void)ptrHwBase->INTR;

        /* Set the multi-channel mode configuration to OFF */
        ptrHwBase->SNS_CTL &= (~MSC_SNS_CTL_MULTI_CH_MODE_Msk);

        /* Configure the last slot */
        ptrHwBase->SNS_CTL |= MSC_SNS_CTL_LAST_Msk | MSC_SNS_CTL_VALID_Msk;

        /* Start scanning with START_SCAN */
        ptrHwBase->SNS_CTL |= MSC_SNS_CTL_START_SCAN_Msk;

        /* Start FSM with START_FRAME */
        ptrHwBase->FRAME_CMD = MSC_FRAME_CMD_START_FRAME_Msk;

        watchdog = Cy_CapSense_GetScanWatchdogTime(scanSlotId, context);
        tmpVal = context->ptrCommonConfig->cpuClkHz / CY_CAPSENSE_CONVERSION_MEGA;
        watchdog = Cy_CapSense_WatchdogCyclesNum(watchdog, tmpVal, cpuCyclesPerLoop);

        while ((ptrHwBase->INTR & MSC_INTR_MASK_SCAN_Msk) == 0u)
        {
            if (0uL == watchdog)
            {
                break;
            }
            watchdog--;
        }

        tmpVal = (uint16_t)ptrHwBase->RESULT_FIFO_RD;

        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CIC2_FILTER_EN)
            tmpVal -= ((uint32_t)context->ptrInternalContext->numEpiCycles - 1u);
            tmpVal *= context->ptrWdConfig->numChopCycles;
        #endif

        *ptrMaxRaw = CY_CAPSENSE_16_BIT_MASK;
        if (tmpVal <= CY_CAPSENSE_16_BIT_MASK)
        {
            *ptrMaxRaw = (uint16_t)tmpVal;
        }

        /* Clear all pending interrupts */
        ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
        (void)ptrHwBase->INTR;

        /* Restore clock sync and external frame start settings */
        ptrHwBase->CTL |= (uint32_t)(((uint32_t)context->ptrCommonConfig->syncFrameStartEn << MSC_CTL_EXT_FRAME_START_EN_Pos) |
                                     ((uint32_t)context->ptrCommonConfig->syncClockEn << MSC_CTL_CLK_SYNC_EN_Pos));

        if (0u == watchdog)
        {
            status |= CY_CAPSENSE_STATUS_TIMEOUT;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureSaturationMode
****************************************************************************//**
*
* Configures the MSC HW block for operation with the permanently saturated
* channel.
* The raw data counter is continuously enabled from the beginning to the end of
* the scan. The scan is performed in the CPU operating mode.
*
* \param context
* The pointer to the context structure allocated by the user or middleware.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The MSCLP HW block is busy and cannot be
*                                         configured.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ConfigureSaturationMode(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    cy_en_msc_status_t mscStatus;
    uint32_t curChIndex;
    uint32_t i;
    MSC_Type * ptrHwBase;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;

    /* Initializes all enabled MSC channels for scan */
    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        ptrHwBase = ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase;

        /* Generates base frame configurations for all enabled MSC channels */
        status |= Cy_CapSense_GenerateBaseConfig(curChIndex, context);

        if (CY_CAPSENSE_STATUS_SUCCESS != status)
        {
            break;
        }

        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
            /* Generates sensor frame configuration */
            Cy_CapSense_GenerateAllSensorConfig(curChIndex,
                                            &context->ptrSensorFrameContext[(CY_CAPSENSE_SLOT_COUNT *
                                            (curChIndex + context->ptrCommonConfig->channelOffset)) *
                                            CY_MSC_6_SNS_REGS], context);
        #endif

        /* MSCv3 IP Block Base Register Configuration */
        mscStatus = Cy_MSC_Configure(ptrHwBase,
                                    &context->ptrBaseFrameContext[curChIndex],
                                    CY_MSC_CAPSENSE_KEY,
                                    ptrCommonCfg->ptrChConfig[curChIndex].ptrHwContext);

        if (CY_MSC_SUCCESS != mscStatus)
        {
            status = CY_CAPSENSE_STATUS_HW_BUSY;
        }
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        ptrHwBase = context->ptrCommonConfig->ptrChConfig[0u].ptrHwBase;

        /* Enables HW IP to allow a scan frame start */
        ptrHwBase->CTL |= MSC_CTL_ENABLED_Msk;

        /* Disables all interrupts */
        ptrHwBase->INTR_MASK = 0x00u;
        (void)ptrHwBase->INTR_MASK;

        /* The 0x00 value of the OPERATING_MODE field switches the HW to the CPU
        * configures it to generate VDDA/2 voltage at the output.
        */
        CY_REG32_CLR_SET(ptrHwBase->CTL, MSC_CTL_OPERATING_MODE, 0u);

        /* Configures the Switch Control Global Function #0 to have all switches open.
        * All the sensing electrodes will be disconnected during the MAX raw count measurement.
        */
        ptrHwBase->SW_SEL_CSW_FUNC[0u] = 0x00u;

        /* Iterates through all the MODE structures and update them with the configuration,
        * required to saturate the measurement channel.
        */
        for (i = 0u; i < MSC0_SENSE_MODE_NR; i++)
        {
            /* The 0x01 value of the RMF field enables the internal voltage divider and
            * configures it to generate VDDA/2 voltage at the output.
            */
            ptrHwBase->MODE[i].SW_SEL_TOP  = _VAL2FLD(MSC_MODE_SW_SEL_TOP_RMF, 0x01u);

            /* Sets the CPF and the CMG bits to get the positive comparator input connected
            * to the VDDA/2 voltage level and the negative comparator input to the GND.
            */
            ptrHwBase->MODE[i].SW_SEL_COMP = _VAL2FLD(MSC_MODE_SW_SEL_COMP_CPF, 0x01u) |
                                            _VAL2FLD(MSC_MODE_SW_SEL_COMP_CMG, 0x01u);
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: Cy_CapSense_GetScanWatchdogTime
****************************************************************************//**
*
* This internal function calculates the watchdog time in microseconds for the
* specified slot. The calculated value is the duration of one sample with the
* five-time margin.
*
* WATCHDOG_TIME = (SCAN_CYCLES / MOD_CLK_FREQ) * 5;
*
* where:
*
* SCAN_CYCLES = INIT_CYCLES + (PRO_CYCLES + CONV_CYCLES + EPI_CYCLES) * NUM_CONV;
* INIT_CYCLES = INIT_CMOD_RAIL_CYCLES + INIT_CMOD_SHORT_CYCLES;
* PRO_CYCLES = PRO_OFFSET_CYCLES + PRO_WAIT_KREF_DELAY;
* CONV_CYCLES = (PRO_DUMMY_SUB_CONVS + NUM_SUB_CONVS) * SENSE_DIV;
* EPI_CYCLES = EPI_KREF_DELAY;
*
* The corresponding parameters are listed below:
* - EPI_KREF_DELAY - The duration of EPILOGUE defined in relation to Kref.
*   The value is interpreted as number of clk_mod cycles.
* - INIT_CMOD_RAIL_CYCLES - Duration of the coarse initialization phase when
*   Cmod1 is connected to VDDA and Cmod2 is connected to VSSA. The parameter is
*   defined in terms of Mod Clock cycles.
* - INIT_CMOD_SHORT_CYCLES - Duration of the coarse short phase when
*   Cmod1 and Cmod2 are shorted together. The parameter is defined in terms of
*   Mod Clock cycles.
* - PRO_OFFSET_CYCLES - Maximum number of Mod Clock cycles to be assigned for
*   the PRO_OFFSET state. If NUM_PRO_OFFSET_TRIPS are observed before this
*   timeout, exit at that point.
* - PRO_DUMMY_SUB_CONVS - Number of sub-conversions (dummy cycles) to be run
*   during PRO_DUMMY.
* - PRO_WAIT_KREF_DELAY - The duration of PRO_WAIT defined in relation to Kref.
*   The value is interpreted as SENSE_DIV increments.
* - SENSE_DIV - The length of one sub-conversion in terms of Mod Clock cycles.
* - NUM_SUB_CONVS - Number of sub-conversions.
* - NUM_CONV - Number of chop cycles.
* - MOD_CLK_FREQ - Modulation clock frequency.
*
* \param scanSlotId
* The slot ID the watchdog time calculation will be done for.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns watch-dog time interval in microseconds.
*
*******************************************************************************/
uint32_t Cy_CapSense_GetScanWatchdogTime(
                uint32_t scanSlotId,
                cy_stc_capsense_context_t * context)
{
    uint32_t tmpVal;
    uint32_t snsClkDivider;
    uint32_t modClkFreqMhz;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;

    /* Define the MARGIN_KOEFF = 5 */
    const uint32_t watchdogTimeMarginCoeff = 5u;

    modClkFreqMhz  = context->ptrCommonConfig->periClkHz;
    modClkFreqMhz /= context->ptrInternalContext->modClk;
    modClkFreqMhz /= CY_CAPSENSE_CONVERSION_MEGA;

    /* Initialize pointers with the configuration address for the specified scan slot and MSC channel #0 */
    ptrScanSlots = &context->ptrScanSlots[scanSlotId];
    ptrWdCfg = &context->ptrWdConfig[ptrScanSlots->wdId];

    /* For the multi-channel mode, iterate through enabled channels and find the sensing channel that
     * drives the sensor (not Shield only, Tx only, or Empty).
     */
    #if (1u < CY_CAPSENSE_TOTAL_CH_NUMBER)
        for (tmpVal = 0u; tmpVal < CY_CAPSENSE_TOTAL_CH_NUMBER; tmpVal++)
        {
            ptrScanSlots = &context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * tmpVal)];
            if (CY_CAPSENSE_SLOT_SHIELD_ONLY > ptrScanSlots->wdId)
            {
                ptrWdCfg = &context->ptrWdConfig[ptrScanSlots->wdId];
                break;
            }
        }
    #endif

    /* Initialize the .snsClkDivider field with the value for the slot that is
    *  specified by the scanSlotId parameter of the function.
    */
    snsClkDivider = ptrWdCfg->ptrWdContext->snsClk;
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_TOUCHPAD_EN) ||\
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_MATRIX_EN))
                if (ptrWdCfg->numCols <= ptrScanSlots->snsId)
                {
                    snsClkDivider = ptrWdCfg->ptrWdContext->rowSnsClk;
                }
            #endif /* CY_CAPSENSE_CSD_TOUCHPAD_EN || CY_CAPSENSE_CSD_MATRIX_EN */
                    }
    #endif /* CY_CAPSENSE_CSD_EN */

    /* Calculate the CONV_CYCLES number. */
    tmpVal = ((uint32_t)context->ptrInternalContext->numFineInitCycles + ptrWdCfg->ptrWdContext->numSubConversions);
    tmpVal *= snsClkDivider;

    if (CY_CAPSENSE_CLK_SOURCE_PRS == (ptrWdCfg->ptrWdContext->snsClkSource & CY_CAPSENSE_CLK_SOURCE_MASK))
    {
        /* Taking into account correction that is implemented in the Cy_CapSense_AdjustSnsClkDivider() function,
         * the scan duration is two time longer if the PRS is used as the sense clock source.
         */
        tmpVal <<= 1u;
    }

    /* Add the PRO_CYCLES number. */
    tmpVal += ((uint32_t)context->ptrInternalContext->numProOffsetCycles + context->ptrInternalContext->numFineInitWaitCycles);

    /* Add the EPI_CYCLES. The .numEpiCycles parameter is defined in terms of
     * Mod Clock cycles for the fifth-generation devices.
    */
    tmpVal += (uint32_t)context->ptrInternalContext->numEpiCycles;

    /* Calculate the SCAN_CYCLES number by multiplying the Mod Clock cycles number per one chop cycle by
    *  number of chop cycles.
    */
    tmpVal *= ptrWdCfg->numChopCycles;

    /* Add the INIT_CYCLES number. */
    tmpVal += ((uint32_t)context->ptrInternalContext->numCoarseInitChargeCycles + context->ptrInternalContext->numCoarseInitSettleCycles);

    /* Convert the calculated duration in terms of clk_mod cycles to the microseconds. */
    tmpVal /= modClkFreqMhz;

    /* Multiply the calculated value by the MARGIN_KOEFF value. */
    tmpVal *= watchdogTimeMarginCoeff;

    return tmpVal;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_AUTO_DITHER_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureAutoDitherMode
****************************************************************************//**
*
* Configures the MSC HW block for operation in the auto-dithering mode.
*
* \param context
* The pointer to the context structure allocated by the user or middleware.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_CONFIG_OVERFLOW  - The number of pin function exceeds the limit.
* - CY_CAPSENSE_STATUS_BAD_CONFIG       - The function does not suppose to be
*                                         called with the current CAPSENSE&trade;
*                                         configuration.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The MSCLP HW block is busy and cannot be
*                                         configured.
*
*******************************************************************************/
static cy_capsense_status_t Cy_CapSense_ConfigureAutoDitherMode(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    cy_en_msc_status_t mscStatus;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    uint32_t curChIndex;
    MSC_Type * ptrHwBase;

    Cy_CapSense_SetModClkDivider(context->ptrInternalContext->modClk, context);
    Cy_CapSense_SetIOsInDefaultState(context);
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
        Cy_CapSense_SetShieldPinsInDefaultState(context);
    #endif

    /* Initializes all enabled MSC channels for scan */
    for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
    {
        context->ptrActiveScanSns[curChIndex].currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;

        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
            Cy_CapSense_SetCmodInAmuxModeState(curChIndex, context);
        #else
            Cy_CapSense_SetCmodInDefaultState(curChIndex, context);
        #endif

        ptrHwBase = ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase;

        /* Generates base frame configurations for all enabled MSC channels */
        status |= Cy_CapSense_GenerateBaseConfig(curChIndex, context);

        if (CY_CAPSENSE_STATUS_SUCCESS != status)
        {
            break;
        }

        #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
            /* Generates sensor frame configuration */
            Cy_CapSense_GenerateAllSensorConfig(curChIndex,
                                            &context->ptrSensorFrameContext[(CY_CAPSENSE_SLOT_COUNT *
                                            (curChIndex + context->ptrCommonConfig->channelOffset)) *
                                            CY_MSC_6_SNS_REGS], context);
        #endif

        /* MSCv3 IP Block Base Register Configuration */
        mscStatus = Cy_MSC_Configure(ptrHwBase,
                                    &context->ptrBaseFrameContext[curChIndex],
                                    CY_MSC_CAPSENSE_KEY,
                                    ptrCommonCfg->ptrChConfig[curChIndex].ptrHwContext);

        if (CY_MSC_SUCCESS != mscStatus)
        {
            status = CY_CAPSENSE_STATUS_HW_BUSY;
            break;
        }

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CIC2_FILTER_EN)
            /* Sets FILTER_MODE to CIC1 */
            ptrHwBase->FILTER_CTL &= ~MSC_FILTER_CTL_FILTER_MODE_Msk;
        #endif /* CY_CAPSENSE_CIC2_FILTER_EN */

        /* Clears all pending interrupts of the current MSC HW block */
        ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
        (void)ptrCommonCfg->ptrChConfig[curChIndex].ptrHwBase->INTR;

        /* Disables all interrupts */
        ptrHwBase->INTR_MASK = 0x00u;
        (void)ptrHwBase->INTR_MASK;

        /* Enables HW IP to allow a scan frame start */
        ptrHwBase->CTL |= MSC_CTL_ENABLED_Msk;

        ptrHwBase->CTL &= ~MSC_CTL_OPERATING_MODE_Msk;
    }

    return status;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CdacDitherScaleCalc
****************************************************************************//**
*
* This internal function calculates the dither scale value based on highest
* raw count value in AUTO dithering mode.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return status
* Returns the status of the operation (different statuses can be combined):
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed
*                                         successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - At least one of input parameters is
*                                         not valid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
static cy_capsense_status_t Cy_CapSense_CdacDitherScaleCalc(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    uint32_t watchdog;
    uint32_t wdIndex;
    uint32_t slotId;
    uint32_t rawCountMax;
    uint32_t curChIndex;
    uint32_t slotValue;
    uint32_t scanSlotIndexValid;
    uint32_t scanFlag;
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
        uint32_t snsIndex;
    #endif
    uint32_t slotIndex;
    MSC_Type * ptrHwBase;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;

    /* Forces HW configuration update on the next scan */
    status = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_AUTO_DITHERING, context);

    /* Performs scans for active slots */
    for (slotId = 0u; slotId < CY_CAPSENSE_SLOT_COUNT; slotId++)
    {
        scanFlag = 0u;

        for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
        {
            context->ptrActiveScanSns[curChIndex].currentChannelSlotIndex = (uint16_t)slotId;
            slotIndex = slotId + (curChIndex * CY_CAPSENSE_SLOT_COUNT);
            ptrScanSlots = &context->ptrScanSlots[slotIndex];
            slotValue = ptrScanSlots->wdId;
            scanSlotIndexValid = slotIndex;

            if (CY_CAPSENSE_SLOT_EMPTY == slotValue)
            {
                #if (CY_CAPSENSE_TOTAL_CH_NUMBER > 1u)
                    if (slotIndex < CY_CAPSENSE_SLOT_COUNT)
                    {
                        scanSlotIndexValid += CY_CAPSENSE_SLOT_COUNT;
                    }
                    else
                    {
                        scanSlotIndexValid -= CY_CAPSENSE_SLOT_COUNT;
                    }
                #endif
            }
            else if (CY_CAPSENSE_SLOT_SHIELD_ONLY <= slotValue)
            {
                scanSlotIndexValid = ((uint32_t)ptrScanSlots->snsId * CY_CAPSENSE_SLOT_COUNT) + slotId;
            }
            else
            {
                /* Do nothing */
            }

            /* Initializes for each channel the active sensor structure for the current sensor */
            wdIndex = context->ptrScanSlots[scanSlotIndexValid].wdId;
            ptrWdCfg = &context->ptrWdConfig[wdIndex];

            #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                snsIndex = context->ptrScanSlots[scanSlotIndexValid].snsId;
                (void)Cy_CapSense_ConfigureAnalogMuxResource(curChIndex, context->ptrWdConfig[wdIndex].senseMethod, context);
                Cy_CapSense_InitActivePtr(curChIndex, wdIndex, snsIndex, context);

                if ((slotValue != CY_CAPSENSE_SLOT_EMPTY) &&
                    (slotValue != CY_CAPSENSE_SLOT_SHIELD_ONLY))
                    {
                        /* Connects the widget first sensor electrodes */
                        Cy_CapSense_ConnectSensor(curChIndex, context);
                    }
            #endif

            if (CY_CAPSENSE_CDAC_DITHERING_MODE_AUTO == ptrWdCfg->cdacDitherScaleMode)
            {
                /* Handles only CSD and CSX widgets */
                if (CY_CAPSENSE_ISX_GROUP != ptrWdCfg->senseMethod)
                {
                    /* Sets a flag to perform scan */
                    scanFlag = 1u;
                }
            }
        }

        if (0u != scanFlag)
        {
            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
            {
                ptrHwBase = context->ptrCommonConfig->ptrChConfig[curChIndex].ptrHwBase;
                ptrHwBase->FRAME_CMD = MSC_FRAME_CMD_START_FRAME_Msk;
            }
            
            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
            {
                ptrHwBase = context->ptrCommonConfig->ptrChConfig[curChIndex].ptrHwBase;
                slotIndex = slotId + (curChIndex * CY_CAPSENSE_SLOT_COUNT);
                status = Cy_CapSense_CdacDitherConfigScan(ptrHwBase, curChIndex, ((curChIndex * CY_CAPSENSE_SLOT_COUNT) + slotId), slotIndex * CY_MSC_6_SNS_REGS, context);
            }
            watchdog = CY_CAPSENSE_CDAC_DITHERING_MOD_CLK_PERIODS;
            ptrHwBase = context->ptrCommonConfig->ptrChConfig[0u].ptrHwBase;

            /* Waits end of scan */
            while ((ptrHwBase->INTR & MSC_INTR_MASK_SCAN_Msk) == 0u)
            {
                if (0uL == watchdog)
                {
                    break;
                }
                watchdog--;
            }

            if (0u == watchdog)
            {
                status |= CY_CAPSENSE_STATUS_TIMEOUT;
            }
            else
            {
                for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                {
                    ptrHwBase = context->ptrCommonConfig->ptrChConfig[curChIndex].ptrHwBase;
                    ptrScanSlots = &context->ptrScanSlots[slotId + (CY_CAPSENSE_SLOT_COUNT * curChIndex)];

                    if (CY_CAPSENSE_SLOT_SHIELD_ONLY > ptrScanSlots->wdId)
                    {
                        ptrWdCfg = &context->ptrWdConfig[context->ptrScanSlots[slotId + (curChIndex * CY_CAPSENSE_SLOT_COUNT)].wdId];

                        if (CY_CAPSENSE_CDAC_DITHERING_MODE_AUTO == ptrWdCfg->cdacDitherScaleMode)
                        {
                            /* Handles only CSD and CSX widgets */
                            if (CY_CAPSENSE_ISX_GROUP != ptrWdCfg->senseMethod)
                            {
                                ptrSnsCxt = &ptrWdCfg->ptrSnsContext[context->ptrScanSlots[slotId + (curChIndex * CY_CAPSENSE_SLOT_COUNT)].snsId];

                                /* Reads raw counts */
                                ptrSnsCxt->raw = (uint16_t)(ptrHwBase->RESULT_FIFO_RD & MSC_RESULT_FIFO_RD_RAW_COUNT_Msk);
                            }
                        }
                    }
                    /* Clears all pending interrupts */
                    ptrHwBase->INTR = CY_CAPSENSE_MSC_INTR_ALL_MSK;
                    (void)ptrHwBase->INTR;

                    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)
                        if (CY_CAPSENSE_SNS_CONNECTED == context->ptrActiveScanSns[curChIndex].connectedSnsState)
                        {
                            /* Disconnects the scanned sensor */
                            Cy_CapSense_DisconnectSensor(curChIndex, context);
                        }
                    #endif
                }
            }
        }
    }

    if (status == CY_CAPSENSE_STATUS_SUCCESS)
    {
        for (wdIndex = 0u; wdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; wdIndex++)
        {
            ptrWdCfg = &context->ptrWdConfig[wdIndex];
            if (CY_CAPSENSE_CDAC_DITHERING_MODE_AUTO == ptrWdCfg->cdacDitherScaleMode)
            {
                rawCountMax = 0u;
                ptrSnsCxt = ptrWdCfg->ptrSnsContext;

                for (slotId = 0u; slotId < ptrWdCfg->numSns; slotId++)
                {
                    /* Finds the biggest raw count value in widget */
                    if (ptrSnsCxt->raw > rawCountMax)
                    {
                        rawCountMax = ptrSnsCxt->raw;
                    }
                    ptrSnsCxt++;
                }

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    /* Divides the result by 2 for the FW CSX scanning method */
                    if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                    {
                        rawCountMax >>= 1u;
                    }
                #endif

                /* Sets dither value according to measured widget raw counts */
                if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_300_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 6u;
                } else if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_500_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 5u;
                } else if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_1000_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 4u;
                } else if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_3000_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 3u;
                } else if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_5000_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 2u;
                } else if (rawCountMax < CY_CAPSENSE_DITHERING_SNS_CAP_10000_FEMTO_RAW)
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 1u;
                } else
                {
                    ptrWdCfg->ptrWdContext->cdacDitherValue = 0u;
                }
            }
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CdacDitherConfigScan
****************************************************************************//**
*
* This internal function configures and performs scanning for a slot.
*
* \param ptrHwBase
* The pointer to HW base pointer for current channel.
*
* \param chId
* Specifies the ID number of the channel
*
* \param slotId
* Specifies the ID number of the slot.
*
* \param snsFrameIdx
* The index of sensor frame context structure.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return status
* Returns the status of the operation (different statuses can be combined):
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed
*                                      successfully.
* - CY_CAPSENSE_STATUS_TIMEOUT       - The software watchdog timeout occurred
*                                      during the scan, the scan was not completed.
*
*******************************************************************************/
static cy_capsense_status_t Cy_CapSense_CdacDitherConfigScan(
                MSC_Type * ptrHwBase,
                uint32_t chId,
                uint32_t slotId,
                uint32_t snsFrameIdx,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    uint32_t scanConfigTmp[CY_MSC_6_SNS_REGS] = {0u, 0u, 0u, 0u, 0u, 0u};
    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        (void)snsFrameIdx;
        uint32_t index;
    #endif

    #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
        /* Inherits sensor frame to local array */
        for (index = 0; index < CY_MSC_6_SNS_REGS; index++)
        {
            scanConfigTmp[index] = context->ptrSensorFrameContext[snsFrameIdx + index];
        }
    #else /* CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD */
        /* Getting the sensor frame configuration */
        (void)Cy_CapSense_GenerateSensorConfig(chId, slotId, &scanConfigTmp[0u], context);
    #endif

    /* Configures Sensor Clock, Clock Sources, set the multi-channel mode configuration to OFF */
    scanConfigTmp[CY_CAPSENSE_SNS_CTL_INDEX] &= (uint32_t)~(MSC_SNS_CTL_START_SCAN_Msk | MSC_SNS_CTL_MULTI_CH_MODE_Msk |
                                                            MSC_SNS_CTL_SENSE_DIV_Msk | MSC_SNS_CTL_LFSR_MODE_Msk);
    scanConfigTmp[CY_CAPSENSE_SNS_CTL_INDEX] |= (((CY_CAPSENSE_CDAC_DITHERING_KREF - 1u) << MSC_SNS_CTL_SENSE_DIV_Pos) |
                                                 MSC_SNS_CTL_LAST_Msk | MSC_SNS_CTL_VALID_Msk | MSC_SNS_CTL_START_SCAN_Msk);

    if (1u < CY_CAPSENSE_TOTAL_CH_NUMBER)
    {
        scanConfigTmp[CY_CAPSENSE_SNS_CTL_INDEX] |= (CY_CAPSENSE_SYNC_INTERNAL << MSC_SNS_CTL_MULTI_CH_MODE_Pos);
    }

    /* Disables chopping. Configures Number of sub-conversions, Compensation divider */
    scanConfigTmp[CY_CAPSENSE_SNS_SCAN_CTL_INDEX] &= (uint32_t)~(MSC_SNS_SCAN_CTL_NUM_CONV_Msk |
                                                                 MSC_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk |
                                                                 MSC_SNS_SCAN_CTL_COMP_DIV_Msk);
    scanConfigTmp[CY_CAPSENSE_SNS_SCAN_CTL_INDEX] |= (((CY_CAPSENSE_CDAC_DITHERING_NUM_SUBCONV - 1u) << MSC_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos) |
                                                     ((CY_CAPSENSE_CDAC_DITHERING_COMP_DIV - 1u) << MSC_SNS_SCAN_CTL_COMP_DIV_Pos));

    /* Configures Cap DACs and Clock reference rate*/
    scanConfigTmp[CY_CAPSENSE_SNS_CDAC_CTL_INDEX] = 0u;
    scanConfigTmp[CY_CAPSENSE_SNS_CDAC_CTL_INDEX] |= ((CY_CAPSENSE_CDAC_DITHERING_CDAC_REF << MSC_SNS_CDAC_CTL_SEL_RE_Pos) |
                                                     (CY_CAPSENSE_CDAC_DITHERING_CLOCK_REF_RATE << MSC_SNS_CDAC_CTL_CLOCK_REF_RATE_Pos));

    /* Initiates the scan in the CPU operating mode */
    Cy_MSC_ConfigureScan(ptrHwBase, CY_MSC_6_SNS_REGS, &scanConfigTmp[0u]);

    return status;
}
#endif /* CY_CAPSENSE_CDAC_AUTO_DITHER_EN */


#endif /* CY_IP_M0S8MSCV3 */


/* [] END OF FILE */
