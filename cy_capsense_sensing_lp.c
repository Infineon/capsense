/***************************************************************************//**
* \file cy_capsense_sensing_lp.c
* \version 4.0
*
* \brief
* This file contains the source of functions common for different scanning
* modes.
*
********************************************************************************
* \copyright
* Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company)
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
#include "cycfg_capsense.h"
#include "cycfg_capsense_defines.h"
#include "cycfg_peripherals.h"
#include "cy_capsense_sensing.h"
#include "cy_capsense_sensing_lp.h"
#include "cy_capsense_generator_lp.h"
#include "cy_capsense_common.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_processing.h"
#include "cy_capsense_lib.h"
#include "cy_capsense_selftest.h"
#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #include "cy_msclp.h"
#endif

#if (defined(CY_IP_M0S8MSCV3LP))

/*******************************************************************************
* Local definition
*******************************************************************************/
#define CY_CAPSENSE_CSD_CCOMP_CALC_DIV                  (4u * CY_CAPSENSE_PERCENTAGE_100)
#define CY_CAPSENSE_CSX_CCOMP_CALC_DIV                  (8u * CY_CAPSENSE_PERCENTAGE_100)

#define CY_CAPSENSE_EXT_FRAME_START_MODE_SCAN           (1u)
#define CY_CAPSENSE_EXT_FRAME_START_MODE_LP_SCAN        (2u)

#define CY_CAPSENSE_CTL_OPERATING_MODE_CPU              (0u)
#define CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS            (2u)
#define CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS           (3u)

#define CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS              (0u)
#define CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN               (1u)

#define CY_CAPSENSE_SCAN_ST_BSLN_RESET_MSK              (0x01u)
#define CY_CAPSENSE_SCAN_ST_BSLN_VALID_MSK              (0x02u)

#define CY_CAPSENSE_CTL_CFG_OFFSET_VAL                  ((CY_MSCLP_REG_OFFSET_SNS_SW_SEL_CSW_LO_MASK2 - \
                                                          CY_MSCLP_REG_OFFSET_SNS_LP_AOS_SNS_CTL0) / \
                                                         CY_CAPSENSE_BYTE_IN_32_BIT)
#define CY_CAPSENSE_CTL_SNS_CTL_OFFSET_VAL              (CY_MSCLP_6_SNS_REGS - 1u)
#define CY_CAPSENSE_CTL_LP_SNS_CTL_OFFSET_VAL           (CY_MSCLP_11_SNS_REGS - 1u)
#define CY_CAPSENSE_LP_AOS_SNS_CFG_REGS_NUM             (5u)

#define CY_CAPSENSE_MULTIPLIER_TWO                      (2u)

/* CIC2 Filter Divider */
#define CY_CAPSENSE_CIC2_DIVIDER_1             (1u)
#define CY_CAPSENSE_CIC2_DIVIDER_2             (2u)
#define CY_CAPSENSE_CIC2_DIVIDER_4             (4u)
#define CY_CAPSENSE_CIC2_DIVIDER_8             (8u)
#define CY_CAPSENSE_CIC2_DIVIDER_16            (16u)
#define CY_CAPSENSE_CIC2_DIVIDER_32            (32u)
#define CY_CAPSENSE_CIC2_DIVIDER_64            (64u)
#define CY_CAPSENSE_CIC2_DIVIDER_128           (128u)
#define CY_CAPSENSE_CIC2_DIVIDER_256           (256u)

/* CIC2 Accumulator parameters */
#define CY_CAPSENSE_CIC2_ACC_BIT_NUM        (24u)
#define CY_CAPSENSE_CIC2_ACC_MAX_VAL        ((1uL << CY_CAPSENSE_CIC2_ACC_BIT_NUM) - 1u)


/*******************************************************************************
* Constants
*******************************************************************************/
#if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_USE_CAPTURE)
    const cy_stc_msclp_base_config_t cy_capsense_msclpCfg = CY_CAPSENSE_MSC_CONFIG_DEFAULT;
#endif

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal *//** \{ */
/******************************************************************************/
cy_capsense_status_t Cy_CapSense_SlotPinStateInternal(
                uint32_t * ptrSnsFrm,
                const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                uint32_t pinState,
                cy_stc_capsense_context_t * context);
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

#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_FULL_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_HW_EN))
void Cy_CapSense_SsAutoTuneScanDurationAlignment(
                uint64_t widgetMask,
                uint32_t clockDivider,
                uint32_t subConversion,
                cy_stc_capsense_context_t * context);
#endif
#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
void Cy_CapSense_SetCompDivider(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
void Cy_CapSense_SetMaxCompCdac(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_CalibrateSlotsInternal(
                uint32_t snsFrameType,
                uint32_t autoCalibrMode,
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_CalibrateAllSlotsInternal(
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context);
cy_capsense_status_t Cy_CapSense_ScanSlotInternalCPU(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context);
#endif

void Cy_CapSense_TransferRawCounts(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context);

void Cy_CapSense_TransferLpRawCounts(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context);

cy_capsense_status_t Cy_CapSense_WaitMrssStatusChange(
                uint32_t timeout,
                uint32_t mrssStatus,
                cy_stc_capsense_context_t * context);

/** \} \endcond */


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSlotsInternal
****************************************************************************//**
*
* Transfers sensor frame to Sensor Data RAM and initiates the non-blocking scan
* of specified slots. If the specified slots configuration does not fit into the
* HW SNS RAM then only the CY_CAPSENSE_FIFO_SNS_MAX_NUM number of sensors will
* be scanned. The remaining sensors will be scanned in the next sub-frame(s).
*
* This is an internal function. Do not call this function directly from
* the application program.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
*******************************************************************************/
void Cy_CapSense_ScanSlotsInternal(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    uint32_t index;
    uint32_t * ptrSensorFrame;
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    uint32_t numberSlotsLocal = (CY_CAPSENSE_FIFO_SNS_MAX_NUM > numberSlots) ? numberSlots : CY_CAPSENSE_FIFO_SNS_MAX_NUM;
    context->ptrInternalContext->numSlots = (uint16_t)numberSlotsLocal;
    const uint32_t cfgOffsetVal = CY_CAPSENSE_CTL_CFG_OFFSET_VAL;

    if(0u != numberSlotsLocal)
    {
        /* Initialize frame data pointer */
        ptrSensorFrame = &context->ptrSensorFrameContext[startSlotId * CY_MSCLP_6_SNS_REGS];

        /* Disable HW IP to allow MRSS operations */
        ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

        /* Check if IMO is running */
        if (0u == (context->ptrCommonConfig->ptrChConfig->ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
        {
            /* Enable MRSS to provide access to the SNS_STC registers */
            ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_START_Msk;
            (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX * CY_MSCLP_MRSS_TIMEOUT_FULL,
                                                   CY_CAPSENSE_MRSS_TURN_ON, context);
        }

        /* Copy sensor frame to Sensor Data RAM */
        for(index = 0u; index < (numberSlotsLocal * CY_MSCLP_6_SNS_REGS); index++)
        {
            ptrHwBase->SNS.SENSOR_DATA[index] = ptrSensorFrame[index];
        }

        /* Configure the last slot */
        ptrHwBase->SNS.SENSOR_DATA[((numberSlotsLocal - 1u) * CY_MSCLP_6_SNS_REGS) +
                                   CY_CAPSENSE_CTL_SNS_CTL_OFFSET_VAL] |= MSCLP_SNS_SNS_CTL_LAST_Msk;

        /* Reset FIFO */
        ptrHwBase->SNS.FIFO_CMD |= MSCLP_SNS_FIFO_CMD_FIFO_RESET_Msk;

        /* Configure Sensor Data RAM start and slot size */
        CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL2, MSCLP_SCAN_CTL2_FRAME_CFG_START_ADDR, 0u);
        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, cfgOffsetVal);

        /* Configure FIFO */
        CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL1, MSCLP_SCAN_CTL1_FRAME_RES_START_ADDR,
                                               MSCLP_SNS_SRAM_WORD_SIZE - numberSlotsLocal);

        if ((NULL != context->ptrInternalContext->ptrSSCallback) &&
            (CY_CAPSENSE_ENABLE == context->ptrInternalContext->firstActSubFrame))
        {
            context->ptrInternalContext->ptrSSCallback(context->ptrActiveScanSns);
        }

        /* Stop MRSS only for the first sub-frame in LP_AOS mode after the callback as it can operate with SNS registers */
        if ((CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS == context->ptrInternalContext->operatingMode) &&
            (CY_CAPSENSE_ENABLE == context->ptrInternalContext->firstActSubFrame))
        {
            if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
            {
                /* Stop the MRSS to save power */
                ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
                (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, context);
            }
        }

        /* Enable HW IP to allow a scan frame start */
        ptrHwBase->CTL |= MSCLP_CTL_ENABLED_Msk;

        /* Provide a delay for the HW settling before the scan start in the AS-MS operating mode */
        if (CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS == context->ptrInternalContext->operatingMode)
        {
            Cy_SysLib_DelayUs(CY_CAPSENSE_MSCLP_HW_SETTLING_TIME_US);
        }

        /* Check if an external start is enabled */
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_EXT_FRM_START_EN)
            if (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_INITIALIZATION_MASK))
            {
                if (CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS == context->ptrInternalContext->operatingMode)
                {
                    CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_EXT_FRAME_START_MODE,
                                     CY_CAPSENSE_EXT_FRAME_START_MODE_LP_SCAN);
                }
                else
                {
                    CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_EXT_FRAME_START_MODE,
                                     CY_CAPSENSE_EXT_FRAME_START_MODE_SCAN);
                }
            }
            else
            {
                /* Start scan */
                if (CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS == context->ptrInternalContext->operatingMode)
                 {
                    ptrHwBase->WAKEUP_CMD = MSCLP_WAKEUP_CMD_START_FRAME_AOS_Msk;
                }
                else
                {
                    ptrHwBase->SNS.FRAME_CMD = MSCLP_SNS_FRAME_CMD_START_FRAME_Msk;
                }
            }
        #else
            /* Start scan */
            if (CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS == context->ptrInternalContext->operatingMode)
            {
                ptrHwBase->WAKEUP_CMD = MSCLP_WAKEUP_CMD_START_FRAME_AOS_Msk;
            }
            else
            {
                ptrHwBase->SNS.FRAME_CMD = MSCLP_SNS_FRAME_CMD_START_FRAME_Msk;
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_EXT_FRM_START_EN) */
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSlots_V3Lp
****************************************************************************//**
*
* Initiates the non-blocking scan of specified slots. Scanning is
* initiated only if no scan is in progress. Scan finishing can be
* checked by the Cy_CapSense_IsBusy() function. The transition into system DEEP SLEEP
* mode is allowed after the scan is started.
*
* This function initiates a scan only for the first number of slots that could
* be fit into the Sensor Data RAM and then exits. Scans for the remaining slots
* started in the interrupt service routine (part of middleware), which triggered
* at the end of the scan completion.
*
* If the /ref activeWakeupTimer parameter set by the Cy_CapSense_ConfigureMsclpTimer()
* function is more than 0u, the correspondent time interval will be inserted before
* the frame scan start to provide a desired refresh rate.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
* - CY_CAPSENSE_STATUS_HW_LOCKED        - The MSCLP HW block is captured by another middleware.
* - CY_CAPSENSE_STATUS_TIMEOUT          - A timeout reached during the MRSS start.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanSlots_V3Lp(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
    uint32_t lastSlot = startSlotId + numberSlots - 1u;
    cy_stc_capsense_internal_context_t * ptrIntrCxt;
    MSCLP_Type * ptrHwBase;

    #if (CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS == CY_CAPSENSE_ACTIVE_SCAN_HW_MODE)
        const uint32_t cfgOffsetVal = CY_CAPSENSE_CTL_CFG_OFFSET_VAL;
    #endif

    if ((NULL != context) && (0u != numberSlots))
    {
        if (CY_CAPSENSE_SLOT_COUNT > lastSlot)
        {
            if (CY_CAPSENSE_BUSY == Cy_CapSense_IsBusy(context))
            {
                /* Previous scan is not completed. Return the BUSY status */
                capStatus = CY_CAPSENSE_STATUS_HW_BUSY;
            }
            else
            {
                capStatus = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
                if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
                {
                    Cy_CapSense_SetBusyFlags(context);

                    /* Set Active widget scan status */
                    context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_WD_SCAN_MASK);
                    context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_ACTIVE_WD_SCAN_MASK;

                    ptrIntrCxt = context->ptrInternalContext;
                    ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;

                    /* Check for needed MSCv3LP operating mode */
                    #if (CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS == CY_CAPSENSE_ACTIVE_SCAN_HW_MODE)
                        if (0u == context->ptrInternalContext->activeWakeupTimer)
                        {
                            ptrIntrCxt->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS;
                            CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE,
                                                             CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS);
                            CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, cfgOffsetVal);
                        }
                        else
                        {
                            ptrIntrCxt->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS;
                            CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE,
                                                             CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS);
                            CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, 0uL);

                            /* Set AOS_CTL register with enabled MRSS power cycle */
                            ptrHwBase->AOS_CTL = ((uint16_t)(context->ptrInternalContext->activeWakeupTimer) |
                                                  (_VAL2FLD(MSCLP_AOS_CTL_FR_TIMEOUT_INTERVAL, 1u)));

                            /* Disable HW processing */
                            ptrHwBase->CE_CTL = 0uL;
                        }
                    #else /* CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS or CY_CAPSENSE_CTL_OPERATING_MODE_CPU */
                        ptrIntrCxt->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS;
                        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE,
                                                         CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS);
                        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, 0uL);

                        /* Set AOS_CTL register with enabled MRSS power cycle */
                        ptrHwBase->AOS_CTL = ((uint16_t)(context->ptrInternalContext->activeWakeupTimer) |
                                              (_VAL2FLD(MSCLP_AOS_CTL_FR_TIMEOUT_INTERVAL, 1u)));

                        /* Disable HW processing */
                        ptrHwBase->CE_CTL = 0uL;
                    #endif /* (CY_CAPSENSE_CTL_OPERATING_MODE_AS_MS == CY_CAPSENSE_ACTIVE_SCAN_HW_MODE) */

                    /* Unmask FRAME interrupt for ACTIVE and ALR modes (AS_MS and LP_AOS HW modes) */
                    ptrHwBase->INTR_LP_MASK = MSCLP_INTR_LP_FRAME_Msk;

                    /* Check for system VDDA value and enable PUMP if VDDA is less then threshold */
                    #if (CY_MSCLP_VDDA_PUMP_TRESHOLD > CY_CAPSENSE_VDDA_MV)
                        ptrHwBase->PUMP_CTL = MSCLP_PUMP_CTL_PUMP_MODE_Msk;
                    #else
                        ptrHwBase->PUMP_CTL = 0u;
                    #endif

                    /* Initialize internal context */
                    ptrIntrCxt->currentSlotIndex = (uint16_t)startSlotId;
                    ptrIntrCxt->endSlotIndex = (uint16_t)lastSlot;
                    ptrIntrCxt->firstActSubFrame = CY_CAPSENSE_ENABLE;

                    /* Initialize the scan */
                    Cy_CapSense_ScanSlotsInternal(startSlotId, numberSlots, context);
                }
            }
        }
    }
    return capStatus;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_ScanLpSlots
****************************************************************************//**
*
* Initiates a non-blocking scan for specified low power slots and exits.
* Scanning is initiated only if no scan is in progress.
* Scans initiated by this function can be performed in different system power modes
* such as Active, Sleep or Deep Sleep.
* After all specified number of frames are scanned or if a touch is
* detected during the scan, the interrupt is fired, the CPU switches
* Power mode to Active and the interrupt service routine (part of middleware)
* transfers the last frame raw counts and clears the busy status.
* If Power mode is Active during scanning, the scan complete can be checked
* by the Cy_CapSense_IsBusy() function and the application program waits until
* all scans complete prior to starting a next scan by using this function.
* The function clears the CY_CAPSENSE_MW_STATE_LP_ACTIVE_MASK status bit. It is set
* when a touch detect occurs during the low power slot scans and will remain set until the
* next Cy_CapSense_ScanLpSlots() function call.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param startLpSlotId
* The slot ID to start a scan from.
*
* \param numberLpSlots
* The number of slots to be scanned.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation completed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The hardware is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanLpSlots(
                uint32_t startLpSlotId,
                uint32_t numberLpSlots,
                cy_stc_capsense_context_t * context)
{
    uint32_t index;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
    cy_stc_capsense_internal_context_t * ptrIntrCxt;
    uint32_t * ptrSensorFrameLp;
    MSCLP_Type * ptrHwBase;
    uint32_t lastLpSlot = startLpSlotId + numberLpSlots - 1u;

    if ((NULL != context) && (0u != numberLpSlots) &&
        (CY_CAPSENSE_FIFO_SNS_LP_MAX_NUM >= numberLpSlots) && (CY_CAPSENSE_SLOT_LP_COUNT > lastLpSlot))
    {
        /* Clear the signal detect status for low power scans */
        context->ptrCommonContext->status &= (~((uint32_t)CY_CAPSENSE_MW_STATE_LP_ACTIVE_MASK));
        if (CY_CAPSENSE_BUSY == Cy_CapSense_IsBusy(context))
        {
            /* Previous scan is not completed. Return the BUSY status */
            capStatus = CY_CAPSENSE_STATUS_HW_BUSY;
        }
        else
        {
            capStatus = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
            if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
            {
                Cy_CapSense_SetBusyFlags(context);

                ptrIntrCxt = context->ptrInternalContext;
                ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;

                /* Mask all interrupts */
                ptrHwBase->INTR_MASK = 0uL;
                ptrHwBase->INTR_LP_MASK = 0uL;

                capStatus = CY_CAPSENSE_STATUS_SUCCESS;

                if (((startLpSlotId != ptrIntrCxt->currentSlotIndex) || (lastLpSlot != ptrIntrCxt->endSlotIndex)) ||
                    (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_LP_WD_SCAN_MASK)))
                {
                    /* Initialize internal context */
                    ptrIntrCxt->currentSlotIndex = (uint16_t)startLpSlotId;
                    ptrIntrCxt->endSlotIndex = (uint16_t)lastLpSlot;
                    ptrIntrCxt->numSlots = (uint16_t)numberLpSlots;

                    /* Set the LP_AOS operating mode */
                    ptrIntrCxt->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS;
                    CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE,
                                                     CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS);
                    CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, 0u);

                    /* Enable HW processing */
                    ptrHwBase->CE_CTL = MSCLP_CE_CTL_ENABLED_Msk |
                                        MSCLP_CE_CTL_BLSD_EN_Msk;

                    #if(CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_RC_IIR_FILTER_EN)
                        ptrHwBase->CE_CTL |= MSCLP_CE_CTL_RCF_EN_Msk;
                    #endif

                    /* Disable HW IP to allow MRSS operations */
                    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

                    /* Check if IMO is not running */
                    if (0u == (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
                    {
                        /* Enable only REF and IMO to provide access to the SNS_STC registers */
                        ptrHwBase->PUMP_CTL = 0u;
                        ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_START_Msk;
                        capStatus = Cy_CapSense_WaitMrssStatusChange(
                                        CY_MSCLP_CLK_LF_PERIOD_MAX * CY_MSCLP_MRSS_TIMEOUT_SMALL,
                                        CY_CAPSENSE_MRSS_TURN_ON, context);
                    }
                    else
                    {
                        /* Stop the MRSS pump to save power */
                        ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_PUMP_STOP_Msk;
                        Cy_SysLib_DelayUs(CY_MSCLP_CLK_LF_PERIOD_MAX * CY_CAPSENSE_MULTIPLIER_TWO);
                    }

                    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
                    {
                        /* Initialize frame data pointer */
                        ptrSensorFrameLp = &context->ptrSensorFrameLpContext[startLpSlotId * CY_MSCLP_11_SNS_REGS];

                        /* Copy sensor frame to Sensor Data RAM */
                        for(index = 0u; index < (numberLpSlots * CY_MSCLP_11_SNS_REGS); index++)
                        {
                            ptrHwBase->SNS.SENSOR_DATA[index] = ptrSensorFrameLp[index];
                        }

                        /* Configure the last slot */
                        ptrHwBase->SNS.SENSOR_DATA[((numberLpSlots - 1u) * CY_MSCLP_11_SNS_REGS) +
                                                   CY_CAPSENSE_CTL_LP_SNS_CTL_OFFSET_VAL] |= MSCLP_SNS_SNS_CTL_LAST_Msk;

                        /* Initialize channel engine to perform processing */
                        ptrHwBase->SNS.CE_INIT_CTL |= MSCLP_SNS_CE_INIT_CTL_SENSOR_INIT_Msk;

                        /* Reset FIFO */
                        ptrHwBase->SNS.FIFO_CMD |= MSCLP_SNS_FIFO_CMD_FIFO_RESET_Msk;

                        /* Check AHB2AHB bus response */
                        if ((ptrHwBase->SNS.BRIDGE_STATUS & MSCLP_SNS_BRIDGE_STATUS_READY_Msk) == 0u)
                        {
                            capStatus = CY_CAPSENSE_STATUS_TIMEOUT;
                        }

                        if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
                        {
                            /* Stop the MRSS to save power */
                            ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
                            (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, context);
                        }
                    }

                    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
                    {
                        /* Configure Sensor Data RAM start and slot size */
                        CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL2, MSCLP_SCAN_CTL2_FRAME_CFG_START_ADDR, 0u);
                        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_CFG_OFFSET, 0u);

                        /* Set AOS_CTL register */
                        ptrHwBase->AOS_CTL = MSCLP_AOS_CTL_MRSS_PWR_CYCLE_EN_Msk;
                        ptrHwBase->AOS_CTL |= (uint16_t)((context->ptrCommonContext->wotScanInterval *
                                                         (CY_SYSCLK_ILO_FREQ / CY_CAPSENSE_CONVERSION_KILO)) - 1u);
                        ptrHwBase->AOS_CTL |= _VAL2FLD(MSCLP_AOS_CTL_FR_TIMEOUT_INTERVAL,
                                                            context->ptrCommonContext->wotTimeout);

                        /* Check for system VDDA value and enable PUMP if VDDA is less then threshold */
                        #if (CY_MSCLP_VDDA_PUMP_TRESHOLD > CY_CAPSENSE_VDDA_MV)
                            ptrHwBase->PUMP_CTL = MSCLP_PUMP_CTL_PUMP_MODE_Msk;
                        #else
                            ptrHwBase->PUMP_CTL = 0uL;
                        #endif

                        /* Configure the start address of FIFO considering fractional part discarding after integer division */
                        CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL1, MSCLP_SCAN_CTL1_FRAME_RES_START_ADDR,
                                         (MSCLP_SNS_SRAM_WORD_SIZE -
                                       (((MSCLP_SNS_SRAM_WORD_SIZE / numberLpSlots) - CY_MSCLP_11_SNS_REGS) * numberLpSlots)));
                    }
                }
                else
                {
                    if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
                    {
                        /* Disable HW IP to allow MRSS operations */
                        ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

                        /* Stop the MRSS to save power */
                        ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
                        (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, context);
                    }
                }

                /* Set Low Power widget scan status */
                context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_WD_SCAN_MASK);
                context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_LP_WD_SCAN_MASK;

                if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
                {
                    /* Start of scan callback */
                    if (NULL != ptrIntrCxt->ptrSSCallback)
                    {
                        ptrIntrCxt->ptrSSCallback(context->ptrActiveScanSns);
                    }

                    /* Enable HW IP to allow scans */
                    ptrHwBase->CTL |= MSCLP_CTL_ENABLED_Msk;

                    /* Un-mask FR_TIMEOUT and SIG_DETECT interrupts in LP-AOS mode */
                    ptrHwBase->INTR_LP_MASK = MSCLP_INTR_LP_MASK_FR_TIMEOUT_Msk | MSCLP_INTR_LP_MASK_SIG_DET_Msk;

                    /* Check if an external start is disabled */
                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_EXT_FRM_START_EN)
                        /* Start scan */
                        ptrHwBase->WAKEUP_CMD |= MSCLP_WAKEUP_CMD_START_FRAME_AOS_Msk;
                    #else
                        /* Set external frame start */
                        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_EXT_FRAME_START_MODE,
                                         CY_CAPSENSE_EXT_FRAME_START_MODE_LP_SCAN);
                    #endif /* (CY_CAPSENSE_DISABLE == CY_CAPSENSE_EXT_FRM_START_EN) */
                }
            }
        }
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllLpWidgets
****************************************************************************//**
*
* Initiates a non-blocking scan for all low power widgets/sensors.
* Scanning is initiated only if no scan is in progress. The scan complete can be
* checked by the Cy_CapSense_IsBusy() function.
*
* The function is the wrapper for the Cy_CapSense_ScanAllLpSlots() function.
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
* - CY_CAPSENSE_STATUS_HW_BUSY          - The hardware is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAllLpWidgets(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        capStatus = Cy_CapSense_ScanLpSlots(0u, CY_CAPSENSE_SLOT_LP_COUNT, context);
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllLpSlots
****************************************************************************//**
*
* Initiates a non-blocking scan of all low power slots and then exits.
* Scanning is initiated only if no scan is in progress.
* A scan initiated by this function can be performed in different power modes
* such as Active, Sleep or Deep Sleep.
* After all specified  number of frames are scanned or if a touch is
* detected during the scan, the interrupt is fired, the CPU switches
* Power mode to Active and the interrupt service routine (part of middleware)
* updates the busy status.
* If Power mode is Active during the scan, the scan complete can be checked
* by the Cy_CapSense_IsBusy() function and the application program waits until
* all scans complete prior to starting a next scan by using this function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation completed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - The input parameter is invalid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The hardware is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAllLpSlots(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        capStatus = Cy_CapSense_ScanLpSlots(0u, CY_CAPSENSE_SLOT_LP_COUNT, context);
    }

    return capStatus;
}
#endif /* CY_CAPSENSE_LP_EN */

/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllWidgets_V3Lp
****************************************************************************//**
*
* Initiates the non-blocking scan for all widgets/sensors. Scanning is
* initiated only if no scan is in progress. Scan finishing can be
* checked by the Cy_CapSense_IsBusy() function.
*
* The function is the wrapper for the Cy_CapSense_ScanAllSlots() function
* to provide the backward compatibility.
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
* - CY_CAPSENSE_STATUS_HW_BUSY          - The HW is busy with the previous scan.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAllWidgets_V3Lp(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        capStatus = Cy_CapSense_ScanSlots(0u, CY_CAPSENSE_SLOT_COUNT, context);
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanWidget_V3Lp
****************************************************************************//**
*
* Initiates the scanning of all sensors in the widget. Scanning is
* initiated only if no scan is in progress. Scan finishing can be
* checked by the Cy_CapSense_IsBusy() function. If the widget is of a low power type
* it is scanned with the LP_AoS scanning mode (i.e. with the configured maximum
* number of frames and scan refresh interval).
*
* The function uses Cy_CapSense_ScanSlots() or Cy_CapSense_ScanLpSlots() function
* with the parameters of
* startSlotId and numberSlots retrieved from the firstSlotId and numSlots
* fields of the cy_stc_capsense_widget_config_t structure.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_ScanWidget_V3Lp(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        if (widgetId < CY_CAPSENSE_TOTAL_WIDGET_COUNT)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == context->ptrWdConfig[widgetId].wdType)
                {
                    capStatus = Cy_CapSense_ScanLpSlots(context->ptrWdConfig[widgetId].firstSlotId,
                                                        context->ptrWdConfig[widgetId].numSlots, context);
                }
                else
                {
                    capStatus = Cy_CapSense_ScanSlots(context->ptrWdConfig[widgetId].firstSlotId,
                                                      context->ptrWdConfig[widgetId].numSlots, context);
                }
            #else
            {
                capStatus = Cy_CapSense_ScanSlots(context->ptrWdConfig[widgetId].firstSlotId,
                                                  context->ptrWdConfig[widgetId].numSlots, context);
            }
            #endif
        }
    }
    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSensor_V3Lp
****************************************************************************//**
*
* Initiates the scanning of the selected sensor in the widget. If the widget is
* of a low power type its sensor is scanned with the LP_AoS scanning mode
* (i.e. with the configured maximum number of frames and scan refresh interval).
* Scanning is
* initiated only if no scan is in progress. Scan finishing can be
* checked by the Cy_CapSense_IsBusy() function. The function uses
* Cy_CapSense_ScanSlots() or Cy_CapSense_ScanLpSlots() function.
*
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_ScanSensor_V3Lp(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if ((NULL != context) &&
        (widgetId < CY_CAPSENSE_TOTAL_WIDGET_COUNT) &&
        (sensorId < context->ptrWdConfig[widgetId].numSns))
    {
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
            if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == context->ptrWdConfig[widgetId].wdType)
            {
                capStatus = Cy_CapSense_ScanLpSlots((sensorId + context->ptrWdConfig[widgetId].firstSlotId), 1u,
                                                    context);
            }
            else
            {
                capStatus = Cy_CapSense_ScanSlots((sensorId + context->ptrWdConfig[widgetId].firstSlotId), 1u, context);
            }
        #else
        {
            capStatus = Cy_CapSense_ScanSlots((sensorId + context->ptrWdConfig[widgetId].firstSlotId), 1u, context);
        }
        #endif
    }

    return capStatus;
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllSlots_V3Lp
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
* * 40% of the maximum raw count for ISX widgets.
*
* To change calibration targets use the Cy_CapSense_SetCalibrTarget() function.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllSlots_V3Lp(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        calibStatus = Cy_CapSense_CalibrateAllSlotsInternal(
                            CY_CAPSENSE_SNS_FRAME_ACTIVE, context);
    }

    return calibStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetCalibrationTarget_V3Lp
****************************************************************************//**
*
* Sets the CapDAC auto-calibration raw count targets for CSD, CSX and/or ISX
* widgets.
*
* The function sets the specified raw count targets if CSD, CSX and/or
* ISX widgets are in the project and the auto-calibration is enabled for them.
* These targets will be used instead the configured ones by
* Cy_CapSense_CalibrateAllSlots(), Cy_CapSense_CalibrateAllWidgets() and
* Cy_CapSense_CalibrateWidget() functions.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param calibrTarget
* The raw counts target in percentage for the specified sensing method.
* It should be more than 0u and less than 100u. If the specified target is
* outside the range, then it will not be updated and
* the CY_CAPSENSE_STATUS_BAD_PARAM status will be returned.
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
cy_capsense_status_t Cy_CapSense_SetCalibrationTarget_V3Lp(
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


/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllWidgets_V3Lp
****************************************************************************//**
*
* Calibrates CapDACs for all widgets.
*
* The function is the wrapper for the Cy_CapSense_CalibrateAllSlots() function
* to provide the backward compatibility.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllWidgets_V3Lp(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        calibStatus = Cy_CapSense_CalibrateAllSlotsInternal(
                            CY_CAPSENSE_SNS_FRAME_ACTIVE, context);
    }

    return calibStatus;
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllLpSlots
****************************************************************************//**
*
* Executes CapDAC auto-calibration for all relevant low power widgets if enabled.
*
* Having enabled the CDAC auto-calibration algorithm is the most common use
* case. It helps to tune your system considering board-to-board variation,
* temperature drift, etc. CDAC auto-calibration is enabled by default.
*
* The function performs searching of Reference CDAC code, Compensation CDAC
* code, Compensation Divider (whichever is enabled) by using a successive
* approximation method to make the sensor's raw count closest to the
* defined targets. The auto-calibration target values are defined
* (by default) as:
* * 85% of the maximum raw count for CSD widgets
* * 40% of the maximum raw count for CSX widgets.
* * 40% of the maximum raw count for ISX widgets.
*
* To change calibration targets use the Cy_CapSense_SetCalibrTarget() function.
*
* \note
* This function is available only for the fifth-generation
* low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllLpSlots(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        calibStatus = Cy_CapSense_CalibrateAllSlotsInternal(
                            CY_CAPSENSE_SNS_FRAME_LOW_POWER, context);
    }

    return calibStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllLpWidgets
****************************************************************************//**
*
* Calibrates CapDACs for all Low power widgets.
*
* The function is the wrapper for the Cy_CapSense_CalibrateAllLpSlots() function
* to provide the backward compatibility.
*
* \note
* This function is available only for the fifth-generation
* low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllLpWidgets(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        calibStatus = Cy_CapSense_CalibrateAllSlotsInternal(
                            CY_CAPSENSE_SNS_FRAME_LOW_POWER, context);
    }

    return calibStatus;
}
#endif
#endif

/*******************************************************************************
* Function Name: Cy_CapSense_ConfigureMsclpTimer
****************************************************************************//**
*
* Configures the wakeup timer value for ACTIVE mode. The wakeup time is
* introduced before each scan frame. Set it to 0 to reach the maximum refresh
* rate. The maximum wakeup time value is defined
* by the CY_CAPSENSE_MAX_WAKEUP_TIMER_MS macro.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param wakeupTimer
* The desired wakeup timer value in milliseconds. The real wakeup time interval
* depends on ILO frequency which have a big tolerance (above +/- 50 %), see device
* datasheets.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_BAD_PARAM     - The input parameter is invalid.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ConfigureMsclpTimer(
                uint32_t wakeupTimer,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_BAD_PARAM;

    if ((NULL != context) && (CY_CAPSENSE_MAX_WAKEUP_TIMER_MS >= wakeupTimer))
    {
        context->ptrInternalContext->activeWakeupTimer = (uint16_t) (wakeupTimer *
                                                             (CY_SYSCLK_ILO_FREQ / CY_CAPSENSE_CONVERSION_KILO));
        result = CY_CAPSENSE_STATUS_SUCCESS;
    }

    return result;
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateWidget_V3Lp
****************************************************************************//**
*
* Executes the CapDAC calibration for all the sensors in the specified widget
* to the default target value.
*
* This function performs exactly the same tasks as
* Cy_CapSense_CalibrateAllWidgets(), but only for a specified widget.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateWidget_V3Lp(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibrationStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
    cy_stc_capsense_widget_config_t const * ptrWdCfg;
    MSCLP_Type * ptrHwBase;
    uint32_t snsFrameType;

    if ((NULL != context) && (CY_CAPSENSE_TOTAL_WIDGET_COUNT > widgetId))
    {
        context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;
        ptrWdCfg = &context->ptrWdConfig[widgetId];
        snsFrameType = ((ptrWdCfg->wdType == (uint8_t)CY_CAPSENSE_WD_LOW_POWER_E) ?
                                                          ((uint32_t)CY_CAPSENSE_SNS_FRAME_LOW_POWER) :
                                                          ((uint32_t)CY_CAPSENSE_SNS_FRAME_ACTIVE));
        ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;

        /* Disable HW IP to allow MRSS operations */
        ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

        /* Check for system VDDA value and enable PUMP if VDDA is less then threshold */
        #if (CY_MSCLP_VDDA_PUMP_TRESHOLD > CY_CAPSENSE_VDDA_MV)
            ptrHwBase->PUMP_CTL = MSCLP_PUMP_CTL_PUMP_MODE_Msk;
        #else
            ptrHwBase->PUMP_CTL = 0u;
        #endif

        /* Check if IMO is running for the calibration operation */
        if (0u == (context->ptrCommonConfig->ptrChConfig->ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
        {
            /* Enable only REF and IMO to provide access to the SNS_STC registers */
            ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_START_Msk;
            (void)Cy_CapSense_WaitMrssStatusChange(
                            CY_MSCLP_CLK_LF_PERIOD_MAX * CY_MSCLP_MRSS_TIMEOUT_SMALL,
                            CY_CAPSENSE_MRSS_TURN_ON, context);
        }

        /* Enable HW IP to allow a scan frame start */
        ptrHwBase->CTL |= MSCLP_CTL_ENABLED_Msk;

        /* Disable HW processing */
        ptrHwBase->CE_CTL = 0uL;

        /* Disable all interrupts */
        ptrHwBase->INTR_MASK = 0uL;
        ptrHwBase->INTR_LP_MASK = 0uL;

        /* Clear all pending interrupts */
        ptrHwBase->INTR_LP = CY_CAPSENSE_MSCLP_INTR_LP_ALL_MSK;
        ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;

        /* Set CPU operating mode */
        context->ptrInternalContext->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_CPU;
        CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE, CY_CAPSENSE_CTL_OPERATING_MODE_CPU);

        /* Set raw counts to store to FIFO */
        CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL1, MSCLP_SCAN_CTL1_RC_STORE_EN, CY_CAPSENSE_ENABLE);

        switch (ptrWdCfg->senseMethod)
        {
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && (0u != CY_CAPSENSE_CSD_CALIBRATION_EN))
                case CY_CAPSENSE_CSD_GROUP:

                /* Calibrate RefCDAC with disabled CompCDAC */
                #if (0u != CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
                    context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
                    calibrationStatus = Cy_CapSense_CalibrateSlotsInternal(
                                                    snsFrameType, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR,
                                                    (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                    context);
                    calibrationStatus |= Cy_CapSense_NormalizeCdac(widgetId, context);
                    context->ptrCommonContext->status &=
                            (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
                #endif

                /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                #if (0u != CY_CAPSENSE_CSD_CDAC_COMP_DIV_AUTO_USAGE)
                    calibrationStatus |= Cy_CapSense_CalibrateCompDivider(widgetId, context);
                #endif

                /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3 and #7) */
                #if (0u != CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                    calibrationStatus |= Cy_CapSense_CalibrateSlotsInternal(
                                                    snsFrameType, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR,
                                                    (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                    context);
                #endif

                break;

            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) && (0u != CY_CAPSENSE_CSX_CALIBRATION_EN))
                case CY_CAPSENSE_CSX_GROUP:

                    /* Calibrate RefCDAC with disabled CompCDAC */
                    #if (0u != CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
                        context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
                        calibrationStatus = Cy_CapSense_CalibrateSlotsInternal(
                                                        snsFrameType, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR,
                                                        (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                        context);
                        calibrationStatus |= Cy_CapSense_NormalizeCdac(widgetId, context);
                        context->ptrCommonContext->status &=
                                (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
                    #endif

                    /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                    #if (0u != CY_CAPSENSE_CSX_CDAC_COMP_DIV_AUTO_USAGE)
                        calibrationStatus |= Cy_CapSense_CalibrateCompDivider(widgetId, context);
                    #endif

                    /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3 and #7) */
                    #if (0u != CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                        calibrationStatus |= Cy_CapSense_CalibrateSlotsInternal(
                                                        snsFrameType, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR,
                                                        (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                        context);
                    #endif

                    break;

            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) && (0u != CY_CAPSENSE_ISX_CALIBRATION_EN))
                case CY_CAPSENSE_ISX_GROUP:

                /* Calibrate RefCDAC with disabled CompCDAC */
                #if (0u != CY_CAPSENSE_ISX_CDAC_REF_AUTO_USAGE)
                    context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
                    calibrationStatus = Cy_CapSense_CalibrateSlotsInternal(
                                                    snsFrameType, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR,
                                                    (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                    context);
                    calibrationStatus |= Cy_CapSense_NormalizeCdac(widgetId, context);
                    context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
                #endif

                /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
                #if (0u != CY_CAPSENSE_ISX_CDAC_COMP_DIV_AUTO_USAGE)
                    calibrationStatus |= Cy_CapSense_CalibrateCompDivider(widgetId, context);
                #endif

                /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3 and #7) */
                #if (0u != CY_CAPSENSE_ISX_CDAC_COMP_USAGE)
                    calibrationStatus |= Cy_CapSense_CalibrateSlotsInternal(
                                                    snsFrameType, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR,
                                                    (uint32_t)ptrWdCfg->firstSlotId, (uint32_t)ptrWdCfg->numSlots,
                                                    context);
                #endif

                break;

            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */

            default:
                calibrationStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
                break;
        }

        /* Check calibration result */
        calibrationStatus |= Cy_CapSense_VerifyCalibration(widgetId, context);

        /* Disable HW IP to allow MRSS operations */
        ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

        if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
        {
            /* Stop the MRSS to save power */
            ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
            (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, context);
        }

        /* Update CRC if BIST is enabled */
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) &&\
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_WDGT_CRC_EN))
            Cy_CapSense_UpdateCrcWidget(widgetId, context);
        #endif

        if (CY_CAPSENSE_STATUS_SUCCESS != calibrationStatus)
        {
            calibrationStatus |= CY_CAPSENSE_STATUS_CALIBRATION_FAIL;
        }

        context->ptrCommonContext->status &= ~(uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;
    }

    return calibrationStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllSlotsInternal
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
* * 40% of the maximum raw count for ISX widgets.
*
* To change calibration targets use the Cy_CapSense_SetCalibrTarget() function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllSlotsInternal(
                uint32_t snsFrameType,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;
    uint32_t curWdIndex;
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_REF_AUTO_USAGE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_COMP_DIV_AUTO_USAGE))
        const cy_stc_capsense_widget_config_t * ptrWdCfg;
    #else
        (void)snsFrameType;
    #endif

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_REF_AUTO_USAGE) || \
         (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_COMP_USAGE))
        uint32_t slotCount = ((snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE) ? (CY_CAPSENSE_SLOT_COUNT) : (CY_CAPSENSE_SLOT_LP_COUNT));
    #endif

    context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;
    calibStatus = CY_CAPSENSE_STATUS_SUCCESS;

    /* Disable HW IP to allow MRSS operations */
    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

    /* Check for system VDDA value and enable PUMP if VDDA is less then threshold */
    #if (CY_MSCLP_VDDA_PUMP_TRESHOLD > CY_CAPSENSE_VDDA_MV)
        ptrHwBase->PUMP_CTL = MSCLP_PUMP_CTL_PUMP_MODE_Msk;
    #else
        ptrHwBase->PUMP_CTL = 0u;
    #endif

    /* Check if IMO is running for the calibration operation */
    if (0u == (context->ptrCommonConfig->ptrChConfig->ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
    {
        /* Enable only REF and IMO to provide access to the SNS_STC registers */
        ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_START_Msk;
        (void)Cy_CapSense_WaitMrssStatusChange(
                        CY_MSCLP_CLK_LF_PERIOD_MAX * CY_MSCLP_MRSS_TIMEOUT_SMALL,
                        CY_CAPSENSE_MRSS_TURN_ON, context);
    }

    /* Enable HW IP to allow a scan frame start */
    ptrHwBase->CTL |= MSCLP_CTL_ENABLED_Msk;

    /* Disable HW processing */
    ptrHwBase->CE_CTL = 0uL;

    /* Disable all interrupts */
    ptrHwBase->INTR_MASK = 0uL;
    ptrHwBase->INTR_LP_MASK = 0uL;

    /* Clear all pending interrupts */
    ptrHwBase->INTR_LP = CY_CAPSENSE_MSCLP_INTR_LP_ALL_MSK;
    ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;

    /* Set CPU operating mode */
    context->ptrInternalContext->operatingMode = CY_CAPSENSE_CTL_OPERATING_MODE_CPU;
    CY_REG32_CLR_SET(ptrHwBase->CTL, MSCLP_CTL_OPERATING_MODE,
                                     CY_CAPSENSE_CTL_OPERATING_MODE_CPU);

    /* Set raw counts to store to FIFO */
    CY_REG32_CLR_SET(ptrHwBase->SCAN_CTL1, MSCLP_SCAN_CTL1_RC_STORE_EN, CY_CAPSENSE_ENABLE);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_REF_AUTO_USAGE)
        context->ptrCommonContext->status |= CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK;
        /* RefCDAC auto-calibration in single CDAC mode */
        calibStatus |= Cy_CapSense_CalibrateSlotsInternal(snsFrameType, CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR,
                                                          0u, slotCount, context);

        /* Finds max RefCDAC and normalizes RefCDAC */
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
            for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
            {
                ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                {
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                        if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                        {
                            if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                        else
                        {
                            if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                    #else
                        calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                    #endif
                }
            }
        #endif
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
            for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
            {
                ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                {
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                        if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                        {
                            if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                        else
                        {
                            if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                    #else
                        calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                    #endif
                }
            }
        #endif
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CDAC_REF_AUTO_USAGE)
            for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
            {
                ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                if (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod)
                {
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                        if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                        {
                            if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                        else
                        {
                            if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                            {
                                calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                            }
                        }
                    #else
                        calibStatus |= Cy_CapSense_NormalizeCdac(curWdIndex, context);
                    #endif
                }
            }
        #endif
        context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK);
    #endif

    /* Skip rest of the stages for SmartSense as not needed */
    if (0u == (context->ptrCommonContext->status & CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK))
    {
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CDAC_COMP_USAGE)
            /* Find CompDivider with fixed Ref CDAC and maximum CompCDAC (for case #8 or #4) */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CDAC_COMP_DIV_AUTO_USAGE)
                for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
                {
                    ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                    if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
                    {
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                            if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                            {
                                if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                            else
                            {
                                if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                        #else
                            calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                        #endif
                    }
                }
            #endif
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CDAC_COMP_DIV_AUTO_USAGE)
                for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
                {
                    ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                    if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                    {
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                            if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                            {
                                if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                            else
                            {
                                if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                        #else
                            calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                        #endif
                    }
                }
            #endif
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CDAC_COMP_DIV_AUTO_USAGE)
                for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
                {
                    ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                    if (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod)
                    {
                        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                            if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                            {
                                if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                            else
                            {
                                if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                                {
                                    calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                                }
                            }
                        #else
                            calibStatus |= Cy_CapSense_CalibrateCompDivider(curWdIndex, context);
                        #endif
                    }
                }
            #endif

            /* Calibrate CompCDAC with fixed Ref CDAC and fixed CompDivider (case #3, #4, #7, #8) */
            if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
            {
                calibStatus |= Cy_CapSense_CalibrateSlotsInternal(snsFrameType, CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR,
                                                                  0u, slotCount, context);

            }
        #endif

        /* Check calibration result */
        if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
        {
            for (curWdIndex = 0u; curWdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; curWdIndex++)
            {
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                    ptrWdCfg = &context->ptrWdConfig[curWdIndex];
                    if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType))
                    {
                        if(snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                        {
                            calibStatus |= Cy_CapSense_VerifyCalibration(curWdIndex, context);
                        }
                    }
                    else
                    {
                        if (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE)
                        {
                            calibStatus |= Cy_CapSense_VerifyCalibration(curWdIndex, context);
                        }
                    }
                #else
                    calibStatus |= Cy_CapSense_VerifyCalibration(curWdIndex, context);
                #endif
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

    /* Disable HW IP to allow MRSS operations */
    ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

    /* Disable PUMP */
    ptrHwBase->PUMP_CTL = 0u;

    if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
    {
        /* Stop the MRSS to save power */
        ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
        (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, context);
    }

    context->ptrCommonContext->status &= ~(uint32_t)CY_CAPSENSE_MW_STATE_CALIBRATION_MASK;

    return calibStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateSlotsInternal
****************************************************************************//**
*
* Executes CapDAC auto-calibration for all specified slots if enabled.
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
* * 40% of the maximum raw count for ISX widgets.
*
* To change calibration targets use the Cy_CapSense_SetCalibrTarget() function.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param autoCalibrMode
* Specifies the slot auto-calibration mode:
* - CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR  - The successive approximation
*     (8-9 scans) of RefCDAC with disabled CompCDAC
* - CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR - The successive approximation
*     (8-9 scans) of CompCDAC with fixed RefCDAC
* - CY_CAPSENSE_CAL_MODE_COMP_CDAC_MAX_CODE - One scan with maximum
*     CompCDAC and fixed RefCDAC
*
* \param startSlotId
* The slot ID calibration will be started from.
*
* \param numberSlots
* The number of slots will be calibrated.
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
cy_capsense_status_t Cy_CapSense_CalibrateSlotsInternal(
                uint32_t snsFrameType,
                uint32_t autoCalibrMode,
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t target;
    uint32_t snsIndex;
    uint32_t maxRawTmp;
    uint32_t calMaskNext;
    uint32_t wdIndex;
    uint32_t calMaskPast;
    uint32_t sensingGroup;
    uint32_t scanSlotId;
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        uint32_t dummyScanFlagEn;
    #endif

    uint8_t * ptrCdac;
    uint32_t rawTarget;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    uint32_t * ptrSnsFrmCxt;
    uint32_t snsFrameIndex;
    #if (CY_CAPSENSE_ENABLE != CY_CAPSENSE_LP_EN)
        ptrScanSlots = context->ptrScanSlots;
    #endif

    calibStatus = Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING, context);
    if (CY_CAPSENSE_STATUS_SUCCESS == calibStatus)
    {
        for (scanSlotId = startSlotId; scanSlotId <= (startSlotId + numberSlots - 1u); scanSlotId++)
        {
            calMaskNext = CY_CAPSENSE_CAL_MIDDLE_VALUE;

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                ptrScanSlots = (snsFrameType == CY_CAPSENSE_SNS_FRAME_ACTIVE) ? (context->ptrScanSlots) : (context->ptrLpScanSlots);
            #endif

            snsIndex = ptrScanSlots[scanSlotId].snsId;
            wdIndex = ptrScanSlots[scanSlotId].wdId;

            calMaskPast = 0u;
            sensingGroup = context->ptrWdConfig[wdIndex].senseMethod;
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                dummyScanFlagEn = 1u;
            #endif

            /* Checks if the auto-calibration is enabled for the current sense method */
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                if (CY_CAPSENSE_CSX_GROUP == sensingGroup)
                {
                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CALIBRATION_EN)
                        sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                    #else
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CDAC_REF_AUTO_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSX_CDAC_COMP_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                    #endif
                }
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                if (CY_CAPSENSE_CSD_GROUP == sensingGroup)
                {
                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CALIBRATION_EN)
                        sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                    #else
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CDAC_REF_AUTO_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_CSD_CDAC_COMP_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                    #endif
                }
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
                if (CY_CAPSENSE_ISX_GROUP == sensingGroup)
                {
                    #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_ISX_CALIBRATION_EN)
                        sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                    #else
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_ISX_CDAC_REF_AUTO_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                        #if (CY_CAPSENSE_DISABLE == CY_CAPSENSE_ISX_CDAC_COMP_USAGE)
                            if (CY_CAPSENSE_CAL_MODE_COMP_CDAC_SUC_APPR == autoCalibrMode)
                            {
                                sensingGroup = CY_CAPSENSE_UNDEFINED_GROUP;
                            }
                        #endif
                    #endif
                }
            #endif

            /* Skips non-auto-calibrated slots */
            if (CY_CAPSENSE_UNDEFINED_GROUP != sensingGroup)
            {
                ptrWdCfg = &context->ptrWdConfig[wdIndex];

                /* Gets target in percentage */
                switch (sensingGroup)
                {
                    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && \
                        (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN))
                        case (uint8_t)CY_CAPSENSE_CSD_GROUP:
                            target = context->ptrInternalContext->intrCsdRawTarget;
                            break;
                    #endif

                    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) && \
                        (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
                        case (uint8_t)CY_CAPSENSE_CSX_GROUP:
                            target = context->ptrInternalContext->intrCsxRawTarget;
                            break;
                    #endif

                    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) && \
                        (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
                        case (uint8_t)CY_CAPSENSE_ISX_GROUP:
                            target = context->ptrInternalContext->intrIsxRawTarget;
                            break;
                    #endif

                    default:
                        /* Widget type is not valid */
                        target = 0u;
                        break;
                }

                maxRawTmp = context->ptrWdContext[wdIndex].maxRawCount;
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                    if ((CY_CAPSENSE_CSD_GROUP == sensingGroup) && (ptrWdCfg->numCols <= snsIndex))
                    {
                        maxRawTmp = context->ptrWdContext[wdIndex].maxRawCountRow;
                    }
                #endif

                rawTarget = (uint16_t)((maxRawTmp * target) / CY_CAPSENSE_PERCENTAGE_100);
                ptrSnsCxt = &ptrWdCfg->ptrSnsContext[snsIndex];
                ptrSnsCxt->cdacComp = 0u;

                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
                    if (snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
                    {
                        snsFrameIndex = (CY_MSCLP_11_SNS_REGS * scanSlotId) + CY_CAPSENSE_FRM_LP_SNS_CDAC_CTL_INDEX;
                        ptrSnsFrmCxt = &context->ptrSensorFrameLpContext[snsFrameIndex];
                    }
                    else
                    {
                        snsFrameIndex = (CY_MSCLP_6_SNS_REGS * scanSlotId) + CY_CAPSENSE_SNS_CDAC_CTL_INDEX;
                        ptrSnsFrmCxt = &context->ptrSensorFrameContext[snsFrameIndex];
                    }
                #else
                    snsFrameIndex = (CY_MSCLP_6_SNS_REGS * scanSlotId) + CY_CAPSENSE_SNS_CDAC_CTL_INDEX;
                    ptrSnsFrmCxt = &context->ptrSensorFrameContext[snsFrameIndex];
                #endif

                /* Perform repeated continuous scan of a slot and tune CDAC */
                do
                {
                    /* Update CDACs based on scan result */
                    ptrCdac = &ptrSnsCxt->cdacComp;
                    if (CY_CAPSENSE_CSD_GROUP == sensingGroup)
                    {
                        if ((uint32_t)ptrSnsCxt->raw < rawTarget)
                        {
                            *ptrCdac &= ~(uint8_t)calMaskPast;
                        }
                    }
                    else
                    {
                        if ((uint32_t)ptrSnsCxt->raw >= rawTarget)
                        {
                            *ptrCdac &= ~(uint8_t)calMaskPast;
                        }
                    }

                    *ptrCdac |= (uint8_t)calMaskNext;

                    if (0u == *ptrCdac)
                    {
                        (*ptrCdac)++;
                    }

                    if (CY_CAPSENSE_CAL_MODE_REF_CDAC_SUC_APPR == autoCalibrMode)
                    {
                        *ptrSnsFrmCxt &= (uint32_t)~(MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk | MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk);
                        *ptrSnsFrmCxt |= ((uint32_t)*ptrCdac << MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos);
                    }
                    else
                    {
                        *ptrSnsFrmCxt &= (uint32_t)~MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk;
                        *ptrSnsFrmCxt |= ((uint32_t)*ptrCdac << MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Pos);
                    }

                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                        /* Dummy scan */
                        if ((0u != dummyScanFlagEn) && (CY_CAPSENSE_CSX_GROUP == sensingGroup))
                        {
                            calibStatus |= Cy_CapSense_ScanSlotInternalCPU(snsFrameType, scanSlotId, context);

                            dummyScanFlagEn = 0u;
                        }
                    #endif

                    /* Scan all sensors in slot */
                    calibStatus |= Cy_CapSense_ScanSlotInternalCPU(snsFrameType, scanSlotId, context);

                    Cy_CapSense_PreProcessSensor(wdIndex, snsIndex, context);

                    /* Switches to the next cycle */
                    calMaskPast = calMaskNext;
                    calMaskNext >>= 1u;
                }
                while (calMaskPast != 0u);
            }
        }
    }

    return calibStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSlotInternalCPU
****************************************************************************//**
*
* Initiates the blocking scan of specified slot in CPU mode.
*
* This function initiates a scan of specified slot in the CPU driven scan
* mode, waits for operation completion and then exits.
* Use of this function is limited to the CapDAC auto-calibration.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
*
* \param snsFrameType
* The Sensor Frame type:
* * CY_CAPSENSE_SNS_FRAME_ACTIVE     - Sensor frame for Active slot
* * CY_CAPSENSE_SNS_FRAME_LOW_POWER  - Sensor frame for Low Power slot
*
* \param scanSlotId
* The slot ID scans will be done for.
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
cy_capsense_status_t Cy_CapSense_ScanSlotInternalCPU(
                uint32_t snsFrameType,
                uint32_t scanSlotId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_SUCCESS;
    cy_stc_capsense_internal_context_t * ptrIntrCxt = context->ptrInternalContext;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    MSCLP_Type * ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    uint32_t * ptrSensorFrame;
    uint32_t tmpRawCount;
    uint32_t wdIndex;
    uint32_t snsIndex;
    uint32_t cpuFreqMHz;
    uint32_t watchdogCounter;
    uint32_t isBusyLoopDuration;
    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN) && \
            ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)))
        const cy_stc_capsense_widget_config_t * ptrWdCfg;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        if (snsFrameType == CY_CAPSENSE_SNS_FRAME_LOW_POWER)
        {
            ptrSensorFrame = &context->ptrSensorFrameLpContext[(scanSlotId * CY_MSCLP_11_SNS_REGS) + CY_CAPSENSE_FRM_LP_SNS_SW_SEL_CSW_LO_MASK2_INDEX];
            ptrScanSlots = context->ptrLpScanSlots;
        }
        else
        {
            ptrSensorFrame = &context->ptrSensorFrameContext[scanSlotId * CY_MSCLP_6_SNS_REGS];
            ptrScanSlots = context->ptrScanSlots;
        }
    #else
        ptrSensorFrame = &context->ptrSensorFrameContext[scanSlotId * CY_MSCLP_6_SNS_REGS];
        ptrScanSlots = context->ptrScanSlots;
        (void) snsFrameType;
    #endif

    /* Initialize internal context */
    ptrIntrCxt->currentSlotIndex = (uint16_t)scanSlotId;
    ptrIntrCxt->endSlotIndex = (uint16_t)scanSlotId;

    /* Set sensor config registers (frame) */
    Cy_MSCLP_ConfigureScan(ptrHwBase, CY_MSCLP_6_SNS_REGS, ptrSensorFrame);

    /* Start FSM with START_FRAME */
    ptrHwBase->SNS.FRAME_CMD = MSCLP_SNS_FRAME_CMD_START_FRAME_Msk;

    /* Configure the last slot */
    ptrHwBase->SNS.SNS_CTL |= MSCLP_SNS_SNS_CTL_LAST_Msk | MSCLP_SNS_SNS_CTL_VALID_Msk;

    /* Start scanning with START_SCAN */
    ptrHwBase->SNS.SNS_CTL |= MSCLP_SNS_SNS_CTL_START_SCAN_Msk;

    /* Approximate duration of Wait For Scan */
    isBusyLoopDuration = 5uL;
    cpuFreqMHz = context->ptrCommonConfig->cpuClkHz / CY_CAPSENSE_CONVERSION_MEGA;
    /* Init Watchdog Counter to prevent a hang */
    watchdogCounter = Cy_CapSense_WatchdogCyclesNum(CY_CAPSENSE_CALIBRATION_TIMEOUT, cpuFreqMHz, isBusyLoopDuration);

    while ((ptrHwBase->INTR & MSCLP_INTR_MASK_SCAN_Msk) == 0u)
    {
        if (0uL == watchdogCounter)
        {
            result = CY_CAPSENSE_STATUS_CALIBRATION_FAIL;
            break;
        }
        watchdogCounter--;
    }

    if (result == CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* Read raw counts */
        tmpRawCount = ptrHwBase->SNS.RESULT_FIFO_RD;

        /* Clear all pending interrupts */
        ptrHwBase->INTR_LP = CY_CAPSENSE_MSCLP_INTR_LP_ALL_MSK;
        ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;

        snsIndex = ptrScanSlots[scanSlotId].snsId;
        wdIndex = ptrScanSlots[scanSlotId].wdId;

        ptrSnsCxt = &context->ptrWdConfig[wdIndex].ptrSnsContext[snsIndex];
        ptrSnsCxt->status &= (uint8_t)~CY_CAPSENSE_SNS_OVERFLOW_MASK;

        if (MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk == (tmpRawCount & MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk))
        {
            ptrSnsCxt->status |= CY_CAPSENSE_SNS_OVERFLOW_MASK;
        }
        tmpRawCount &= MSCLP_SNS_RESULT_FIFO_RD_RAW_COUNT_Msk;

        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN) && \
             ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)))
            ptrWdCfg = &context->ptrWdConfig[wdIndex];
            if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType)
            {
                if ((CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod) ||
                    (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod))
                {
                    /* Limit raw counts */
                    if (tmpRawCount < ptrWdCfg->ptrWdContext->maxRawCount)
                    {
                        /* Invert raw counts for CSX/ISX widgets */
                        tmpRawCount = (ptrWdCfg->ptrWdContext->maxRawCount - tmpRawCount);
                    }
                    else
                    {
                        tmpRawCount = 0u;
                    }
                }
            }
        #endif
        ptrSnsCxt->raw = (uint16_t)tmpRawCount;
    }

    return result;
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
           (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN)) */


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
* Returns the status of the operation (different statuses can be combined):
* - CY_CAPSENSE_STATUS_SUCCESS          - The operation is performed
*                                         successfully.
* - CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG - Sense Clock Divider is out
*                                         of the valid range
*                                         for the specified Clock source
*                                         configuration.
* - CY_CAPSENSE_STATUS_BAD_PARAM        - At least one of input parameters is
*                                         not valid.
* - CY_CAPSENSE_STATUS_HW_BUSY          - The MSCLP HW block is busy and cannot be
*                                         configured.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SsInitialize(cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    uint32_t i;

    capStatus |= Cy_CapSense_InitializeSourceSenseClk(context);

    /* Initialize the MSCLP channel for scan */
    /* Reset the sense method */
    context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
    context->ptrInternalContext->scanSingleSlot = CY_CAPSENSE_SCAN_MULTIPLE_SLOT;

    /* Configure inactive sensor states */
    context->ptrInternalContext->intrCsdInactSnsConn = ptrCommonCfg->csdInactiveSnsConnection;
    context->ptrInternalContext->intrCsxInactSnsConn = ptrCommonCfg->csxInactiveSnsConnection;
    context->ptrInternalContext->intrIsxInactSnsConn = ptrCommonCfg->isxInactiveSnsConnection;

    /* Generate base frame configurations for all enabled MSCLP channels */
    capStatus |= Cy_CapSense_GenerateBaseConfig(context);

    /* Generates sensor frame configuration */
    Cy_CapSense_GenerateAllSensorConfig(CY_CAPSENSE_SNS_FRAME_ACTIVE, context);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        Cy_CapSense_GenerateAllSensorConfig(CY_CAPSENSE_SNS_FRAME_LOW_POWER, context);
    #endif /* CY_CAPSENSE_LP_EN */

    /* Assign the ISR for scan */
    context->ptrInternalContext->ptrISRCallback = &Cy_CapSense_ScanISR;

    /* Find maximum raw count for each widget */
    ptrWdCfg = &context->ptrWdConfig[0u];
    for (i = 0u; i < CY_CAPSENSE_TOTAL_WIDGET_COUNT; i++)
    {
        capStatus |= Cy_CapSense_InitializeMaxRaw(ptrWdCfg, context);
        ptrWdCfg++;
    }

    /* Check if an external start is enabled */
    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_EXT_FRM_START_EN)
        Cy_GPIO_Pin_FastInit(CY_CAPSENSE_EXT_FRM_START_PORT, CY_CAPSENSE_EXT_FRM_START_PIN,
                             CY_GPIO_DM_HIGHZ, 0u, HSIOM_SEL_DS_1);
    #endif /* (CY_CAPSENSE_DISABLE != CY_CAPSENSE_EXT_FRM_START_EN) */

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

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_InterruptHandler_V3Lp
****************************************************************************//**
*
* Implements the interrupt service routine (ISR) for the Fifth generation low power
* CAPSENSE&trade;.
*
* At the end of the whole frame scan, the MSCLP HW block generates interrupts
* (for the frame while regular widget scanning and for the time-out number of frames
* or for signal detection while low power widget scanning). The ISR distinguishes
* these interrupts and reads raw counts into the Data structure respectively, and
* then exits if no other slots to scan left,
* Otherwise - starts a scan for the next set of regular sensors. The HW MRSS is turned
* off after the last regular widget scan.
*
* The the CAPSENSE&trade; middleware uses this interrupt to implement the
* non-blocking sensor scan method, in which only the first frame scan is
* initiated in the interrupt service routine as soon as the current frame scan
* completes. The interrupt service routine is implemented
* as a part of the CAPSENSE&trade; middleware.
*
* The CAPSENSE&trade; middleware does not initialize or modify the priority
* of interrupts - instead, the application program
* configures an MSCLP interrupt and assigns the interrupt vector to
* the Cy_CapSense_InterruptHandler() function. For details, refer to the function
* usage example.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;.
*
* The call of the End Of Scan callback (for details, see the \ref group_capsense_callbacks
* section) is the part of the Cy_CapSense_InterruptHandler() routine
* and that call lengthens its execution. Such a callback lengthens the MSCLP ISR execution
* in the case of a direct call of the Cy_CapSense_InterruptHandler() function
* from a MSCLP ISR.
*
* \param base
* The pointer to the base register address of the MSCLP HW block.
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
* for each MSCLP HW block between calls of the Cy_CapSense_Init() and
* Cy_CapSense_Enable() functions:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Initialization
*
*******************************************************************************/
void Cy_CapSense_InterruptHandler_V3Lp(const MSCLP_Type * base, cy_stc_capsense_context_t * context)
{
    (void)base;
    context->ptrInternalContext->ptrISRCallback((void *)context);
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanISR
****************************************************************************//**
*
* This is an internal ISR function to handle the MSCLP sensing
* in AS_MS and LP_AoS operating modes.
*
* \param capsenseContext
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ScanISR(void * capsenseContext)
{
    cy_stc_capsense_context_t * cxt = (cy_stc_capsense_context_t *)capsenseContext;
    MSCLP_Type * ptrHwBase = cxt->ptrCommonConfig->ptrChConfig->ptrHwBase;
    cy_stc_capsense_internal_context_t * ptrIntrCxt = cxt->ptrInternalContext;
    uint32_t numScanSlots;

    /* Check for interrupt source: MSCLP_INTR_LP_FRAME_Msk for ACTIVE widget scan */
    if (MSCLP_INTR_LP_FRAME_Msk == (ptrHwBase->INTR_LP & ptrHwBase->INTR_LP_MASK & MSCLP_INTR_LP_FRAME_Msk))
    {
        /* Clear all pending interrupts of the MSCLP HW block */
        ptrHwBase->INTR_LP = CY_CAPSENSE_MSCLP_INTR_LP_ALL_MSK;
        ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;

        /* Read the last ACTIVE frame raw counts */
        Cy_CapSense_TransferRawCounts(ptrIntrCxt->currentSlotIndex, ptrIntrCxt->numSlots, cxt);

        ptrIntrCxt->currentSlotIndex += ptrIntrCxt->numSlots;

        /* Check for the last slot with multiple slot scan, if not - start the next slot scan */
        if (ptrIntrCxt->currentSlotIndex <= ptrIntrCxt->endSlotIndex)
        {
            numScanSlots = (uint32_t)ptrIntrCxt->endSlotIndex - (uint32_t)ptrIntrCxt->currentSlotIndex + 1uL;

            /* Disable delay before next scan */
            if (CY_CAPSENSE_CTL_OPERATING_MODE_LP_AOS == ptrIntrCxt->operatingMode)
            {
                ptrHwBase->AOS_CTL &= ~MSCLP_AOS_CTL_WAKEUP_TIMER_Msk;
            }

            /* Clear the first sub-frame flag */
            ptrIntrCxt->firstActSubFrame = CY_CAPSENSE_DISABLE;

            Cy_CapSense_ScanSlotsInternal(ptrIntrCxt->currentSlotIndex, numScanSlots, cxt);
        }
        else
        {
            /* Disable HW IP to allow MRSS operations */
            ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

            if (CY_CAPSENSE_MRSS_TURN_OFF == ptrIntrCxt->mrssStateAfterScan)
            {
                if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
                {
                    /* Stop the MRSS to save power */
                    ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
                    (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, cxt);
                }
            }

            /* Mark completion of active scan cycle */
            cxt->ptrCommonContext->scanCounter++;

            Cy_CapSense_ClrBusyFlags(cxt);
        }
    }
    else
    {
        /* It was the low power scan - check if a signal detection had occurred */
        if (MSCLP_INTR_LP_SIG_DET_Msk == (ptrHwBase->INTR_LP & MSCLP_INTR_LP_SIG_DET_Msk))
        {
            cxt->ptrCommonContext->status |= (uint32_t)CY_CAPSENSE_MW_STATE_LP_ACTIVE_MASK;
        }

        /* Clear all pending interrupts of the MSCLP HW block */
        ptrHwBase->INTR_LP = CY_CAPSENSE_MSCLP_INTR_LP_ALL_MSK;
        ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;

        /* Read the last low power frame raw counts */
        if((cxt->ptrCommonContext->lpDataSt & CY_CAPSENSE_LP_PROCESS_ENABLED_MASK) != 0u)
        {
            Cy_CapSense_TransferLpRawCounts(ptrIntrCxt->currentSlotIndex, ptrIntrCxt->numSlots, cxt);

            /* Mark completion of low power scan cycle */
            cxt->ptrCommonContext->lpScanCounter++;
        }

        /* Disable HW IP to allow MRSS operations */
        ptrHwBase->CTL &= (~MSCLP_CTL_ENABLED_Msk);

        if (CY_CAPSENSE_MRSS_TURN_OFF == ptrIntrCxt->mrssStateAfterScan)
        {
            if (0u != (ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
            {
                /* Stop the MRSS to save power */
                ptrHwBase->MRSS_CMD = MSCLP_MRSS_CMD_MRSS_STOP_Msk;
                (void)Cy_CapSense_WaitMrssStatusChange(CY_MSCLP_CLK_LF_PERIOD_MAX, CY_CAPSENSE_MRSS_TURN_OFF, cxt);
            }
        }

        Cy_CapSense_ClrBusyFlags(cxt);
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetCmodInDesiredState
****************************************************************************//**
*
* Sets all available MSCLP Cmod pins into a desired state.
*
The default state to provide the fifth generation low power CAPSENSE&trade; operation is:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - High-Z Analog, Input off drive mode.
* - MSC_ANA - Enabled.
*
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param desiredMscCtrl
* Specifies the desired (MSC_ANA) control status of the IO port analog signaling by the MSC.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCmodInDesiredState(
                uint32_t desiredDriveMode,
                en_hsiom_sel_t desiredHsiom,
                uint32_t desiredMscCtrl,
                const cy_stc_capsense_context_t * context)
{
    Cy_CapSense_SsConfigPinRegisters(
    context->ptrCommonConfig->ptrChConfig->portCmod1,
    (uint32_t)context->ptrCommonConfig->ptrChConfig->pinCmod1,
    desiredDriveMode, desiredHsiom, desiredMscCtrl);

    Cy_CapSense_SsConfigPinRegisters(
    context->ptrCommonConfig->ptrChConfig->portCmod2,
    (uint32_t)context->ptrCommonConfig->ptrChConfig->pinCmod2,
    desiredDriveMode, desiredHsiom, desiredMscCtrl);
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetIOsInDesiredState
****************************************************************************//**
*
* Sets all CAPSENSE&trade; pins into a desired state.
*
* The default state to provide the fifth generation low power CAPSENSE&trade; operation is:
* - HSIOM   - Disconnected, the GPIO mode.
* - DM      - Strong drive.
* - MSC_ANA - Enabled.
* - State   - Zero.
*
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param desiredPinOutput
* Specifies the desired pin output data register (DR) state.
*
* \param desiredMscCtrl
* Specifies the desired (MSC_ANA) control status of the IO port analog signaling by the MSC.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetIOsInDesiredState(
                uint32_t desiredDriveMode,
                en_hsiom_sel_t desiredHsiom,
                uint32_t desiredPinOutput,
                uint32_t desiredMscCtrl,
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrPinConfig;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->numPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber,
            desiredDriveMode, desiredHsiom, desiredMscCtrl);

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


/*******************************************************************************
* Function Name: Cy_CapSense_SetShieldPinsInDesiredState
****************************************************************************//**
*
* Sets all shield pins into a default state.
*
* Do not call this function directly from the application program.
*
* \param desiredDriveMode
* Specifies the desired pin control port (PC) configuration.
*
* \param desiredHsiom
* Specifies the desired pin HSIOM state.
*
* \param desiredMscCtrl
* Specifies the desired (MSC_ANA) control status of the IO port analog signaling by the MSC.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetShieldPinsInDesiredState(
                uint32_t desiredDriveMode,
                en_hsiom_sel_t desiredHsiom,
                uint32_t desiredMscCtrl,
                const cy_stc_capsense_context_t * context)
{
    uint32_t loopIndex;
    const cy_stc_capsense_pin_config_t * ptrPinCfg = context->ptrShieldPinConfig;

    /* Loop through all shield electrodes */
    for (loopIndex = 0u; loopIndex < context->ptrCommonConfig->csdShieldNumPin; loopIndex++)
    {
        Cy_CapSense_SsConfigPinRegisters(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber,
            desiredDriveMode, desiredHsiom, desiredMscCtrl);

        Cy_GPIO_Clr(ptrPinCfg->pcPtr, (uint32_t)ptrPinCfg->pinNumber);
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
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    Cy_CapSense_InitActivePtrWd(widgetId, context);
    Cy_CapSense_InitActivePtrSns(sensorId, context);
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
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
        cy_stc_capsense_active_scan_sns_t * ptrActive = context->ptrActiveScanSns;
    #endif

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
        uint32_t widgetSenseGroup = ptrActive->currentSenseMethod;
        const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[ptrActive->widgetIndex];
        ptrActive->connectedSnsState = CY_CAPSENSE_SNS_DISCONNECTED;
        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            uint32_t numberRows;
            uint32_t numberCols;
        #endif
    #endif

    #if !(CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
        (void)context;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
        ptrActive->ptrSnsContext = &context->ptrWdConfig[ptrActive->widgetIndex].ptrSnsContext[sensorId];
    #else
        (void)sensorId;
    #endif

    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
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
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_InitActivePtrWd(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_active_scan_sns_t * ptrActive = context->ptrActiveScanSns;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN)
        ptrActive->widgetIndex = (uint8_t)widgetId;
    #endif

    ptrActive->currentSenseMethod = context->ptrWdConfig[widgetId].senseMethod;
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
* *
* \param mscCtrl
* Specifies MSC_ANA control state of the IO port analog signaling by the MSC.
*
*******************************************************************************/
void Cy_CapSense_SsConfigPinRegisters(
                GPIO_PRT_Type * base,
                uint32_t pinNum,
                uint32_t dm,
                en_hsiom_sel_t hsiom,
                uint32_t mscCtrl)
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

    if (CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS != mscCtrl)
    {
        Cy_GPIO_MscControlEnable(base, pinNum);
    }
    else
    {
        Cy_GPIO_MscControlDisable(base, pinNum);
    }
    Cy_SysLib_ExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAbort_V3Lp
****************************************************************************//**
*
* This function sets the sequencer to the idle state by resetting the hardware,
* it can be used to abort current scan.
*
* \note
* This function is available only for the fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_ScanAbort_V3Lp(cy_stc_capsense_context_t * context)
{
    uint32_t interruptState;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    const cy_stc_capsense_channel_config_t * ptrChannelCfg = ptrCommonCfg->ptrChConfig;

    interruptState = Cy_SysLib_EnterCriticalSection();

    /* Clear ENABLE bit */
    ptrChannelCfg->ptrHwBase->CTL &= ~MSCLP_CTL_ENABLED_Msk;
    /* Wait until ENABLE bit is cleared for all channels*/
    while (0uL != (MSCLP_CTL_ENABLED_Msk & ptrChannelCfg->ptrHwBase->CTL)) {}
    /* Set ENABLE bit for all channels */
    ptrChannelCfg->ptrHwBase->CTL |= MSCLP_CTL_ENABLED_Msk;
    /* Clear all pending interrupts of the MSCLP HW block */
    ptrChannelCfg->ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;
    (void)ptrChannelCfg->ptrHwBase->INTR;

    Cy_SysLib_ExitCriticalSection(interruptState);

    (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_UNDEFINED, context);

    context->ptrCommonContext->status = CY_CAPSENSE_NOT_BUSY;

    /* Wait for initialization */
    Cy_SysLib_DelayUs(ptrCommonCfg->analogWakeupDelay);

    return CY_CAPSENSE_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name: Cy_CapSense_MwState_V3Lp
****************************************************************************//**
*
* This function returns a detailed state of the CAPSENSE&trade; middleware and MSC
* hardware in Single- or Multi-channel mode. This feature is useful in
* multi-thread applications or in ISR.
* Use the Cy_CapSense_IsBusy() function to verify if HW is busy at a particular
* moment.
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
* - CY_CAPSENSE_MW_STATE_ACTIVE_WD_SCAN_MASK     - The last or currently scanned widget
*                                                  type is Active widget.
* - CY_CAPSENSE_MW_STATE_LP_WD_SCAN_MASK         - The last or currently scanned widget
*                                                  type is Low Power widget
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
*                                                  or in progress.
*
*******************************************************************************/
cy_capsense_mw_state_t Cy_CapSense_MwState_V3Lp(const cy_stc_capsense_context_t * context)
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
* This is an internal function. Do not call this function directly from
* the application program.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetBusyFlags(cy_stc_capsense_context_t * context)
{
    context->ptrCommonContext->status |= (((uint32_t)CY_CAPSENSE_BUSY_CH_MASK) | CY_CAPSENSE_BUSY);
}


/*******************************************************************************
* Function Name: Cy_CapSense_ClrBusyFlags
****************************************************************************//**
*
* Clears BUSY flags of the cy_capsense_context.status register specified
* by the flags parameter.
*
* This is an internal function. Do not call this function directly from
* the application program.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_ClrBusyFlags(cy_stc_capsense_context_t * context)
{
    /* Clear busy flag for the current channel */
    context->ptrCommonContext->status &= (uint32_t)(~((uint32_t)CY_CAPSENSE_BUSY_CH_MASK));
    context->ptrCommonContext->status &= ~CY_CAPSENSE_BUSY;

    /* Check for EndOfScan callback */
    if (NULL != context->ptrInternalContext->ptrEOSCallback)
    {
        context->ptrInternalContext->ptrEOSCallback(NULL);
    }
}


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
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
* - CY_CAPSENSE_CAL_MODE_COMP_CDAC_MAX_CODE - One scan with maximum
*     CompCDAC and fixed RefCDAC
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
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        calibStatus = Cy_CapSense_CalibrateSlotsInternal(
                        CY_CAPSENSE_SNS_FRAME_ACTIVE, autoCalibrMode, scanSlotId, 1u, context);
    }

    return calibStatus;
}


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
    uint32_t numSlots;
    uint8_t maxRefCdac = 0u;
    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
        uint8_t maxRowRefCdac = 0u;
    #endif
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdCfg->ptrSnsContext;
    uint32_t * ptrSnsFrmCxt;
    uint32_t cdacTmp;

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
                    /* Division with rounding up */
                    maxRefCdac = (maxRefCdac + 1u) >> 1u;
                    maxRowRefCdac = (maxRowRefCdac + 1u) >> 1u;
                }
            #endif
            ptrWdCfg->ptrWdContext->cdacRef = maxRefCdac;
            ptrWdCfg->ptrWdContext->rowCdacRef = maxRowRefCdac;
        }
    #endif
    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
        if ((CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod) ||
            (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod))
        {
            /* CSX or ISX Widget */
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
                if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
                {
                    /* Division with rounding up */
                    maxRefCdac = (maxRefCdac + 1u) >> 1u;
                }
            #endif

            #if (0u != CY_CAPSENSE_ISX_CDAC_COMP_USAGE)
                if (CY_CAPSENSE_ISX_GROUP == ptrWdCfg->senseMethod)
                {
                    /* Division with rounding up */
                    maxRefCdac = (maxRefCdac + 1u) >> 1u;
                }
            #endif
            ptrWdCfg->ptrWdContext->cdacRef = maxRefCdac;
        }
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType)
        {
            ptrSnsFrmCxt = &context->ptrSensorFrameLpContext[CY_CAPSENSE_FRM_LP_SNS_CDAC_CTL_INDEX];
            numSlots = CY_CAPSENSE_SLOT_LP_COUNT;
            ptrScanSlots = context->ptrLpScanSlots;
        }
        else
        {
            ptrSnsFrmCxt = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
            numSlots = CY_CAPSENSE_SLOT_COUNT;
            ptrScanSlots = context->ptrScanSlots;
        }
    #else
        ptrSnsFrmCxt = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
        numSlots = CY_CAPSENSE_SLOT_COUNT;
        ptrScanSlots = context->ptrScanSlots;
    #endif

    for (i = 0u; i < numSlots; i++)
    {
        if (widgetId == ptrScanSlots[i].wdId)
        {
            cdacTmp = ptrWdCfg->ptrWdContext->cdacRef;
            if ((CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod) &&
                (ptrWdCfg->numCols <= ptrScanSlots[i].snsId))
            {
                cdacTmp = ptrWdCfg->ptrWdContext->rowCdacRef;
            }
            *ptrSnsFrmCxt &= (uint32_t)~(MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk | MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk);
            *ptrSnsFrmCxt |= (uint32_t)(cdacTmp << MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos);
        }

        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType)
        {
            ptrSnsFrmCxt = &ptrSnsFrmCxt[CY_MSCLP_11_SNS_REGS];
        }
        else
        {
            ptrSnsFrmCxt = &ptrSnsFrmCxt[CY_MSCLP_6_SNS_REGS];
        }
    }

    return CY_CAPSENSE_STATUS_SUCCESS;
}


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
    uint32_t curSlotIndex;
    uint32_t numSlots;
    uint32_t snsOverflow = 0u;
    uint32_t snsIndex;
    uint32_t rawTemp;
    uint32_t target;
    uint32_t frameType;
    uint32_t j;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        uint32_t dummyScanFlagEn = 1u;
    #endif

    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    uint32_t compDivDefault = ptrWdCfg->ptrWdContext->snsClk;
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType)
        {
            numSlots = CY_CAPSENSE_SLOT_LP_COUNT;
            frameType = CY_CAPSENSE_SNS_FRAME_LOW_POWER;
            ptrScanSlots = context->ptrLpScanSlots;
        }
        else
        {
            numSlots = CY_CAPSENSE_SLOT_COUNT;
            frameType = CY_CAPSENSE_SNS_FRAME_ACTIVE;
            ptrScanSlots = context->ptrScanSlots;
        }
    #else
        numSlots = CY_CAPSENSE_SLOT_COUNT;
        frameType = CY_CAPSENSE_SNS_FRAME_ACTIVE;
        ptrScanSlots = context->ptrScanSlots;
    #endif

    /* Sets Max CompCDAC Code within specified widget */
    for (j = 0u; j < ptrWdCfg->numSns; j++)
    {
        ptrWdCfg->ptrSnsContext[j].cdacComp = CY_CAPSENSE_CDAC_MAX_CODE;
    }
    Cy_CapSense_SetMaxCompCdac(widgetId, context);

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
        Cy_CapSense_SetCompDivider(widgetId, context);

        snsOverflow = 0u;
        for (curSlotIndex = 0u; curSlotIndex < numSlots; curSlotIndex++)
        {
            /* Checks only the specified widget in the current slot */
            if (widgetId == ptrScanSlots[curSlotIndex].wdId)
            {
                #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                    /* Dummy scan */
                    if ((0u != dummyScanFlagEn) && (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod))
                    {
                        calibStatus |= Cy_CapSense_ScanSlotInternalCPU(frameType , curSlotIndex, context);
                        dummyScanFlagEn = 0u;
                    }
                #endif

                calibStatus |= Cy_CapSense_ScanSlotInternalCPU(frameType , curSlotIndex, context);

                snsIndex = ptrScanSlots[curSlotIndex].snsId;
                Cy_CapSense_PreProcessSensor(widgetId, snsIndex, context);
                rawTemp = ptrWdCfg->ptrSnsContext[snsIndex].raw;

                switch (ptrWdCfg->senseMethod)
                {
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                        case CY_CAPSENSE_CSD_GROUP:
                            if (rawTemp > target)
                            {
                                snsOverflow = 1u;
                            }
                            break;
                    #endif

                    #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || \
                            (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
                        case CY_CAPSENSE_CSX_GROUP:
                        case CY_CAPSENSE_ISX_GROUP:
                            if (rawTemp <= target)
                            {
                                snsOverflow = 1u;
                            }
                            break;
                    #endif

                    default:
                        /* No action for other methods */
                        break;
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
    } while ((compDivDefault > 1u) && (CY_CAPSENSE_STATUS_SUCCESS == calibStatus));

    return calibStatus;
}


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
    uint32_t currSlot;
    uint32_t frameType;
    cy_capsense_status_t calibStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    const cy_stc_capsense_sensor_context_t * ptrSnsCxt = ptrWdCfg->ptrSnsContext;

    frameType = ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == ptrWdCfg->wdType) ?
                    (CY_CAPSENSE_SNS_FRAME_LOW_POWER) : (CY_CAPSENSE_SNS_FRAME_ACTIVE);

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
        /* Dummy scan */
        if (CY_CAPSENSE_CSX_GROUP == ptrWdCfg->senseMethod)
        {
            calibStatus |= Cy_CapSense_ScanSlotInternalCPU(frameType, ptrWdCfg->firstSlotId, context);
        }
    #endif

    for (currSlot = ptrWdCfg->firstSlotId; currSlot < ((uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots); currSlot++)
    {
        calibStatus |= Cy_CapSense_ScanSlotInternalCPU(frameType, currSlot, context);
    }

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
        switch (ptrWdCfg->senseMethod)
        {
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
                case CY_CAPSENSE_CSD_GROUP:
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
                    break;
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || \
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
                case CY_CAPSENSE_CSX_GROUP:
                case CY_CAPSENSE_ISX_GROUP:
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
                    break;
            #endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) || \
                       (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)) */

            default:
                calibStatus |= CY_CAPSENSE_STATUS_BAD_PARAM;
                /* No action */
                break;
        }
    }

    return calibStatus;
}


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
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    uint32_t compensationCdacDivider = context->ptrWdContext[widgetId].cdacCompDivider;

    if (0uL != compensationCdacDivider)
    {
        compensationCdacDivider--;
    }

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
    if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == context->ptrWdConfig[widgetId].wdType))
    {
        ptrScanSlots = context->ptrLpScanSlots;
        currentFramePtr = &context->ptrSensorFrameLpContext[CY_CAPSENSE_FRM_LP_SNS_SCAN_CTL_INDEX];

        for (i = 0u; i < CY_CAPSENSE_SLOT_LP_COUNT; i++)
        {
            if (ptrScanSlots->wdId == widgetId)
            {
                *currentFramePtr &= ~MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Msk;
                *currentFramePtr |= (uint32_t)(compensationCdacDivider << MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Pos);
            }
            ptrScanSlots++;
            currentFramePtr = &currentFramePtr[CY_MSCLP_11_SNS_REGS];
        }
    }
    else
    #endif
    {
        ptrScanSlots = context->ptrScanSlots;
        currentFramePtr = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_SCAN_CTL_INDEX];

        for (i = 0u; i < CY_CAPSENSE_SLOT_COUNT; i++)
        {
            if (ptrScanSlots->wdId == widgetId)
            {
                *currentFramePtr &= ~MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Msk;
                *currentFramePtr |= (uint32_t)(compensationCdacDivider << MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Pos);
            }
            ptrScanSlots++;
            currentFramePtr = &currentFramePtr[CY_MSCLP_6_SNS_REGS];
        }
    }
}


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
    const cy_stc_capsense_scan_slot_t * ptrScanSlots;

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
    if (((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E == context->ptrWdConfig[widgetId].wdType))
    {
        ptrScanSlots = context->ptrLpScanSlots;
        currentFramePtr = &context->ptrSensorFrameLpContext[CY_CAPSENSE_FRM_LP_SNS_CDAC_CTL_INDEX];

        for (i = 0u; i < CY_CAPSENSE_SLOT_LP_COUNT; i++)
        {
            if (ptrScanSlots->wdId == widgetId)
            {
                *currentFramePtr |= MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk;
            }
            ptrScanSlots++;
            currentFramePtr = &currentFramePtr[CY_MSCLP_11_SNS_REGS];
        }
    }
    else
    #endif
    {
        ptrScanSlots = context->ptrScanSlots;
        currentFramePtr = &context->ptrSensorFrameContext[CY_CAPSENSE_SNS_CDAC_CTL_INDEX];

        for (i = 0u; i < CY_CAPSENSE_SLOT_COUNT; i++)
        {
            if (ptrScanSlots->wdId == widgetId)
            {
                *currentFramePtr |= MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk;
            }
            ptrScanSlots++;
            currentFramePtr = &currentFramePtr[CY_MSCLP_6_SNS_REGS];
        }
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


/*******************************************************************************
* Function Name: Cy_CapSense_SlotPinStateInternal
****************************************************************************//**
*
* Configures the desired electrode to the specified state by
* updating the CAPSENSE&trade; configuration.
*
* See detailed descriptions for the  Cy_CapSense_SlotPinState() and
* Cy_CapSense_LpSlotPinState() functions.
*
* \param ptrSnsFrm
* The pointer for sensor frame (Active or Low Power) context structure.
*
* \param ptrEltdCfg
* The pointer to an electrode the all pins states of which will be configured
* as pinState parameter.
*
* \param pinState
* The desired pins state for CSX widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_TX     - Tx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_NEG_TX - Negative Tx electrode
*     (for multi-phase TX method).
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_VDDA2  - Connected to VDDA/2.
* The desired pins state for CSD widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_CSD_SNS    - Self-cap sensor.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_ACT_SHIELD - Active shield is routed to the pin.
* * CY_CAPSENSE_PIN_STATE_IDX_PAS_SHIELD - Passive shield is routed to the pin.
* The desired pins state for ISX widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_LX     - Lx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
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
cy_capsense_status_t Cy_CapSense_SlotPinStateInternal(
                uint32_t * ptrSnsFrm,
                const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                uint32_t pinState,
                cy_stc_capsense_context_t * context)
{
    uint32_t i;
    uint32_t mask = 0u;
    uint32_t * ptrCswFunc = &context->ptrBaseFrameContext->swSelCswFunc[0u];
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_CONFIG;
    const uint32_t pinStates[CY_CAPSENSE_CTRLMUX_PIN_STATE_NUMBER] = CY_CAPSENSE_PIN_STATES_ARR;

    /* Check if the desired pin state is assigned already */
    if ((context->ptrInternalContext->mapPinState[pinState] == CY_CAPSENSE_SCW_FUNC_PIN_STATE_IDX_UNDEFINED) &&
        (MSCLP_CSW_GLOBAL_FUNC_NR > context->ptrInternalContext->numFunc))
    {
        context->ptrInternalContext->mapPinState[pinState] = context->ptrInternalContext->numFunc;
        ptrCswFunc[context->ptrInternalContext->numFunc] = pinStates[pinState];
        context->ptrCommonConfig->ptrChConfig->ptrHwBase->SW_SEL_CSW_FUNC[context->ptrInternalContext->numFunc] =
                                                           pinStates[pinState];
        context->ptrInternalContext->numFunc += 1u;
        capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    }
    else if (context->ptrInternalContext->mapPinState[pinState] != CY_CAPSENSE_SCW_FUNC_PIN_STATE_IDX_UNDEFINED)
    {
        capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    }
    else
    {
        /* Do nothing */
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
    {
        for (i = 0u; i < ptrEltdCfg->numPins; i++)
        {
            mask |= 0x01uL << ptrEltdCfg->ptrPin[i].padNumber;
        }

        for (i = 0u; i < CY_CAPSENSE_CTRLMUX_PIN_STATE_MASK_NUMBER; i++)
        {
            ptrSnsFrm[i] &= ~mask;
            if (0u != (context->ptrInternalContext->mapPinState[pinState] &
                       ((uint8_t)(1u << (CY_CAPSENSE_CTRLMUX_PIN_STATE_MASK_NUMBER - 1u - i)))))
            {
                ptrSnsFrm[i] |= mask;
            }
        }
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_SlotPinState
****************************************************************************//**
*
* Configures the specified electrode to the desired state in the specified
* slot (Active slot for fifth-generation low power
* CAPSENSE&trade;) by updating the CAPSENSE&trade; configuration.
*
* This function changes / overwrites configuration of an electrode (several
* pins if the electrode is ganged to more pins) with a state specified
* by the pinState parameter. The function does this only for the specified slot ID
* (Active slotID for the fifth-generation low power CAPSENSE&trade;).
* If the electrode should have the desired state during scans in another
* slots, the function should be called multiple times for each desired slot
* (Active slot for fifth-generation low power CAPSENSE&trade;).
*
* The function call changes the pin states permanently and all further scans of
* the slot will have the electrode state as specified by the pinState parameter.
* Call the function again to change the electrode state to a new desired one or
* reinitialize CAPSENSE&trade; middleware by using the Cy_CapSense_Enable() function.
*
* The re-configuration for the fifth-generation
* CAPSENSE&trade; is possible only when
* the parameter Sensor connection method = CTRLMUX.
* If the parameter Sensor connection method = AMUXBUS, then use
* the Cy_CapSense_SetPinState() function.
*
* The function changes the configuration of an electrode without storing
* the previous state. The user is responsible to keep the previous state to
* revert to the default settings if needed. Also, the default settings
* can be configured again by calling Cy_CapSense_Enable() function that
* leads to repeating CAPSENSE&trade; Data Structure initialization,
* DAC auto-calibration, and baseline initialization.
*
* Using the function is recommended only for advanced users for specific use cases.
* For instance, to change the CAPSENSE&trade; default electrode configuration,
* the function could be called by using a CAPSENSE&trade; Data Structure
* Initialization callback ptrEODsInitCallback. For details of how to register
* the callback see the \ref group_capsense_callbacks section. That avoids repeating of
* DAC auto-calibration and baseline initialization since the callback is called after
* CAPSENSE&trade; Data Structure initialization but before the first initialization
* scan.
*
* The function is a low-level function and does not perform an input parameter
* verification (like slotID, pointers, etc.). For example, the
* CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD pin state is not available if a shield is not
* configured in the project, but the function will set the pin state and the HW block
* behavior is unpredictable.
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
* For the fifth-generation CAPSENSE&trade;:
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
* For the fifth-generation low power CAPSENSE&trade;:
*
* The desired pins state for CSX widget electrodes can be:
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_TX     - Tx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_NEG_TX - Negative Tx electrode
*     (for multi-phase TX method).
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_VDDA2  - Connected to VDDA/2.
*
* The desired pins state for CSD widget electrodes can be:
* * CY_CAPSENSE_PIN_STATE_IDX_CSD_SNS    - Self-cap sensor.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_ACT_SHIELD - Active shield is routed to the pin.
* * CY_CAPSENSE_PIN_STATE_IDX_PAS_SHIELD - Passive shield is routed to the pin.
*
* The desired pins state for ISX widget electrodes can be:
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_LX     - Lx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
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
cy_capsense_status_t Cy_CapSense_SlotPinState(
                uint32_t slotId,
                const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                uint32_t pinState,
                cy_stc_capsense_context_t * context)
{
    uint32_t * ptrSnsFrm = &context->ptrSensorFrameContext[slotId * CY_MSCLP_6_SNS_REGS];

    return (Cy_CapSense_SlotPinStateInternal(ptrSnsFrm, ptrEltdCfg, pinState, context));
}


#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_LP_EN)
/*******************************************************************************
* Function Name: Cy_CapSense_LpSlotPinState
****************************************************************************//**
*
* Configures the specified electrode to the desired state in the specified Low Power
* slot by updating the CAPSENSE&trade; configuration.
*
* This function changes / overwrites configuration of an electrode (several
* pins in case the electrode is ganged to more pins) with a state specified
* by the pinState parameter. The function does this only for the specified Low Power
* slot ID. If the electrode should have the desired state during scans in another Low
* Power slots, the function should be called multiple times for each desired Low Power
* slot.
*
* The function call changes the pin states permanently and all further scans of
* the slot will have the electrode state as specified by the pinState parameter.
* Call the function again to change the electrode state to a new desired one or
* reinitialize CAPSENSE&trade; middleware by using the Cy_CapSense_Enable() function.
*
* The function changes the configuration of an electrode without storing
* the previous state. A user is responsible to keep the previous state to
* revert to the default settings if needed. Also, the default settings
* can be configured again by calling Cy_CapSense_Enable() function that
* leads to repeating CAPSENSE&trade; Data Structure initialization,
* DAC auto-calibration, and baseline initialization.
*
* Using the function is recommended only for advanced users for specific use cases.
* For instance, to change the CAPSENSE&trade; default electrode configuration,
* the function could be called by using a CAPSENSE&trade; Data Structure
* Initialization callback ptrEODsInitCallback. For details of how to register
* the callback see the \ref group_capsense_callbacks section. That avoids repeating of
* DAC auto-calibration and baseline initialization since the callback is called after
* CAPSENSE&trade; Data Structure initialization but before the first initialization
* scan.
*
* The function is a low-level function and does not perform an input parameter
* verification (like Low Power slot ID, pointers, etc.). For example, the
* CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD pin state is not available if a shield is not
* configured in the project, but the function will set the pin state and the HW block
* behavior is unpredictable.
*
* \note
* This function is available only for the fifth-generation low power
* CAPSENSE&trade;. For the fifth-generation (only for
* CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) and fifth-generation low power
* CAPSENSE&trade; Active slots use Cy_CapSense_SlotPinState(). For fourth-generation
* CAPSENSE&trade and fifth-generation CAPSENSE&trade with
* CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD use Cy_CapSense_SetPinState().
*
* \param lpSlotId
* The desired Low Power slot ID.
*
* \param ptrEltdCfg
* The pointer to an electrode the all pins states of which will be configured
* as pinState parameter.
*
* \param pinState
* The desired pins state for CSX widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_TX     - Tx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_NEG_TX - Negative Tx electrode
*     (for multi-phase TX method).
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_CSX_VDDA2  - Connected to VDDA/2.
* The desired pins state for CSD widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_CSD_SNS    - Self-cap sensor.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
* * CY_CAPSENSE_PIN_STATE_IDX_ACT_SHIELD - Active shield is routed to the pin.
* * CY_CAPSENSE_PIN_STATE_IDX_PAS_SHIELD - Passive shield is routed to the pin.
* The desired pins state for ISX widget electrodes:
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_LX     - Lx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_ISX_RX     - Rx electrode.
* * CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z     - Unconnected (high-z).
* * CY_CAPSENSE_PIN_STATE_IDX_GND        - Grounded.
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
* \funcusage
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_LpSlotPinState
*
*******************************************************************************/
    cy_capsense_status_t Cy_CapSense_LpSlotPinState(
                    uint32_t lpSlotId,
                    const cy_stc_capsense_electrode_config_t * ptrEltdCfg,
                    uint32_t pinState,
                    cy_stc_capsense_context_t * context)
    {
        uint32_t * ptrSnsFrm = &context->ptrSensorFrameLpContext[(lpSlotId * CY_MSCLP_11_SNS_REGS) + CY_MSCLP_5_SNS_REGS];

        return (Cy_CapSense_SlotPinStateInternal(ptrSnsFrm, ptrEltdCfg, pinState, context));
    }
#endif /* CY_CAPSENSE_LP_EN */


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
* - CY_CAPSENSE_STATUS_BAD_PARAM    - At least one of input parameters is
*                                     not valid.
* - CY_CAPSENSE_STATUS_HW_BUSY      - The MSCLP HW block is busy and cannot be
*                                     configured.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SwitchHwConfiguration(
                uint32_t configuration,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    cy_en_msclp_status_t msclpStatus;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    MSCLP_Type * ptrHwBase = ptrCommonCfg->ptrChConfig->ptrHwBase;

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
                break;
            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN)
                case CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY:
                    Cy_CapSense_BistDisableMode(context);
                    break;
            #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_TST_HW_GROUP_EN) */
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
                    Cy_CapSense_SetIOsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO, 0u,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
                        Cy_CapSense_SetShieldPinsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                    #endif

                    /* Reset last scanned widget type */
                    context->ptrCommonContext->status &= (uint32_t)~((uint32_t)CY_CAPSENSE_MW_STATE_WD_SCAN_MASK);
                    context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;

                    Cy_CapSense_SetCmodInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);

                    msclpStatus = Cy_MSCLP_Configure(ptrHwBase,
                                                     context->ptrBaseFrameContext,
                                                     CY_MSCLP_CAPSENSE_KEY,
                                                     ptrCommonCfg->ptrChConfig->ptrHwContext);
                    if (CY_MSCLP_SUCCESS != msclpStatus)
                    {
                        capStatus = CY_CAPSENSE_STATUS_HW_BUSY;
                        break;
                    }

                    /* Clear all pending interrupts of the MSCLP HW block */
                    ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;
                    (void)ptrHwBase->INTR;

                    /* The time interval is required for settling analog part of the HW block. */
                    Cy_SysLib_DelayUs(CY_CAPSENSE_ANALOG_SETTLING_TIME_US);

                    break;
                case CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY:
                    Cy_CapSense_SetIOsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO, 0u,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                    Cy_CapSense_SetShieldPinsInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                    context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
                    Cy_CapSense_SetCmodInDesiredState(CY_CAPSENSE_DM_GPIO_ANALOG, CY_CAPSENSE_HSIOM_SEL_GPIO,
                        CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN, context);
                    /* Clear all pending interrupts of the MSCLP HW block */
                    ptrHwBase->INTR = CY_CAPSENSE_MSCLP_INTR_ALL_MSK;
                    (void)ptrHwBase->INTR;
                    break;
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

    return (capStatus);
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
    uint32_t snsClkDivMin = 0u;
    uint32_t lfsrRange;

    uint32_t retVal = CY_CAPSENSE_STATUS_SUCCESS;

    cy_stc_capsense_widget_context_t * ptrWdCxt;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = context->ptrWdConfig;

    for (wdIndex = 0u; wdIndex < CY_CAPSENSE_TOTAL_WIDGET_COUNT; wdIndex++)
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
                * SSC-Auto option, and the LFSR range parameter is configured with
                * the Auto option.
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
        switch (ptrWdCfg->senseMethod)
        {
            #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) ||\
                 (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
                case CY_CAPSENSE_CSD_GROUP:
                case CY_CAPSENSE_ISX_GROUP:
                    snsClkDivMin = CY_CAPSENSE_4PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
                    break;
            #endif

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
                case CY_CAPSENSE_CSX_GROUP:
                    snsClkDivMin = CY_CAPSENSE_2PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
                    break;
            #endif

            default:
                retVal = CY_CAPSENSE_STATUS_BAD_PARAM;
                /* No action */
                break;
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
            snsClkDivMin += Cy_CapSense_GetLfsrDitherVal(lfsrRange, context->ptrCommonContext->lfsrScale);
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

        ptrWdCfg++;
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
    lfsrScale = context->ptrCommonContext->lfsrScale;
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
    switch (ptrWdConfig->senseMethod)
    {
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
            case CY_CAPSENSE_CSD_GROUP:
            case CY_CAPSENSE_ISX_GROUP:
                snsClkDividerMin = CY_CAPSENSE_4PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
                break;
        #endif

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            case CY_CAPSENSE_CSX_GROUP:
                snsClkDividerMin = CY_CAPSENSE_2PH_DIRECT_SNS_CLOCK_DIVIDER_MIN;
                break;
        #endif

        default:
            snsClkDividerMin = 0u;
            break;
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
    subConvNumber += context->ptrCommonContext->numFineInitCycles;
    if (CY_CAPSENSE_16_BIT_MASK < subConvNumber)
    {
        subConvNumber = CY_CAPSENSE_16_BIT_MASK;
    }
    lfsrRange = ((uint32_t)ptrWdCxt->lfsrBits & (uint32_t)(~((uint32_t)CY_CAPSENSE_LFSR_BITS_AUTO_MASK)));
    lfsrScale = context->ptrCommonContext->lfsrScale;
    autoSelMode = ptrWdConfig->snsClkSourceAutoSelMode;
    ditherLimitPercents = ptrWdConfig->lfsrDitherLimit;

    /*
    * Determine the polynomial duration in clock cycles, e.g. the duration of 8-bit
    * polynomial is 255 clock cycles.
    */
    lfsrPolySize = Cy_CapSense_GetPolySize(context->ptrCommonContext->lfsrPoly);

    /*
    * Determine the clock dithering variation for the specifies LFSR range,
    * e.g for the [-2; 1] range the clock variation will be 2 cycles.
    */
    lfsrDitherCycles = Cy_CapSense_GetLfsrDitherVal(lfsrRange, lfsrScale);

    /* Determine the MIN valid Sense Clock divider for the used sensing method. */
    if ((CY_CAPSENSE_CSD_GROUP == ptrWdConfig->senseMethod) ||\
        (CY_CAPSENSE_ISX_GROUP == ptrWdConfig->senseMethod))
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
    * Execute the SSC-Auto selection routine. This routine validates all the
    * criteria, that are required for good SSC performance are met.
    * The CY_CAPSENSE_CLK_SOURCE_SSC will be returned if all the criteria are met,
    * the CY_CAPSENSE_CLK_SOURCE_DIRECT will be returned if not.
    */
    snsClkSrc = Cy_CapSense_RunSSCAuto(autoSelMode, lfsrDitherCycles,
                                    ditherLimitCycles, lfsrPolySize, subConvNumber);

    #if ((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) ||\
         (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        /* Repeat the SSC-Auto selection routine for the row of two-dimension CSD
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
    uint32_t lfsrPolySize;
    uint32_t risingEdgesNum;
    uint32_t subConvNumber;
    uint32_t snsClkSrc = CY_CAPSENSE_CLK_SOURCE_PRS;

    /*
    * Determine the whole number of sub-conversion per scan (includes Dummy and
    * Measurement conversions).
    */
    subConvNumber = ptrWdContext->numSubConversions;
    subConvNumber += context->ptrCommonContext->numFineInitCycles;

    /*
    * Determine the polynomial duration in clock cycles, e.g. the duration of 8-bit
    * polynomial is 255 clock cycles.
    */
    lfsrPolySize = Cy_CapSense_GetPolySize(context->ptrCommonContext->lfsrPoly);

    /*
    * The number of rising edges per one PRS period is (polyLength + 1) / 4.
    * polyLength = 2^N - 1, where N is size of the shift register (8BIT, 12BIT, etc).
    */
    risingEdgesNum = (lfsrPolySize + 1u) >> CY_CAPSENSE_4PH_PRS_SNS_CLOCK_DIVIDER_SHIFT;

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


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_FULL_EN) || \
     (CY_CAPSENSE_ENABLE == CY_CAPSENSE_SMARTSENSE_HW_EN))
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
* - Calibrate Reference and Compensation CDACs.
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

    /*
     * SmartSense functionality is not available yet for the
     * fifth-generation low power CAPSENSE&trade; middleware
     * and will be added later
     */
    #if (true)

        (void)context;

    #else

    uint32_t i;
    uint32_t j;
    uint32_t range;
    MSCLP_Type * msclpBase;
    uint32_t rawCountMax;
    cy_stc_capsense_hw_smartsense_config_t autoTuneConfig;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    cy_stc_capsense_widget_context_t * ptrWdCxt;
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    const cy_stc_capsense_common_config_t * ptrCommonCfg = context->ptrCommonConfig;
    cy_stc_capsense_common_context_t * ptrCommonCxt = context->ptrCommonContext;

    ptrCommonCxt->status |= CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK;

    /* Step #0: Check if configuration is valid */
    if ((ptrCommonCfg->counterMode != CY_CAPSENSE_COUNTER_MODE_SATURATE) ||
    #if (CY_CAPSENSE_CSD_CDAC_COMP_EN == CY_CAPSENSE_ENABLE)
        (ptrCommonCfg->csdCdacCompDivAutoEn != CY_CAPSENSE_ENABLE) ||
    #endif /* CY_CAPSENSE_CSD_CDAC_COMP_EN */
        (ptrCommonCfg->csdRefCdacAutoEn != CY_CAPSENSE_ENABLE))
    {
        autoTuneStatus |= CY_CAPSENSE_STATUS_BAD_CONFIG;
    }

    /* Step #1: Sets the default parameters for the preliminary scanning */
    ptrWdCfg = context->ptrWdConfig;
    for (i = 0u; i < CY_CAPSENSE_TOTAL_WIDGET_COUNT; i++)
    {
        if ((uint8_t)CY_CAPSENSE_WD_LOW_POWER_E != ptrWdCfg->wdType)
        {
            /*
            * Set the Sense Clock source set to Direct.
            * Since the Number of Epilogue cycles is a common parameter, it is required
            * to configure the Sense Clock source to Direct for all widgets (including CSX)
            * in order to have correct number of Epilogue cycles for operation with
            * the Direct clock.The setup of the Epilogue cycles number is the part of
            * the Cy_CapSense_InitializeSourceSenseClk() routine.
            */
            ptrWdCxt = ptrWdCfg->ptrWdContext;
            ptrWdCxt->snsClkSource <<= CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_POS;

            if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
            {
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
        }
        ptrWdCfg++;
    }
    autoTuneStatus |= Cy_CapSense_SsInitialize(context);

    msclpBase = ptrCommonCfg->ptrChConfig->ptrHwBase;

    msclpBase->SCAN_CTL2 &= (uint32_t)~((uint32_t)MSCLP_SCAN_CTL2_NUM_EPI_CYCLES_Msk);
    msclpBase->SCAN_CTL2 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                               MSCLP_SCAN_CTL2_NUM_EPI_CYCLES_Pos);

    msclpBase->INIT_CTL4 &= (uint32_t)~((uint32_t)MSCLP_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Msk);
    msclpBase->INIT_CTL4 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                               MSCLP_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Pos);

    /* Step #2: Execute preliminary scan */
    autoTuneStatus |= Cy_CapSense_ScanAllSlots(context);
    while (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(context)) {}

    /* Step #3: Update settings for the widgets / sensors with low capacitance */
    ptrWdCfg = context->ptrWdConfig;
    for (i = 0u; i < CY_CAPSENSE_TOTAL_WIDGET_COUNT; i++)
    {
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
            ptrWdCxt = ptrWdCfg->ptrWdContext;
            range = 0u;
            rawCountMax = ((uint32_t)ptrCommonCfg->csdRawTarget * context->ptrWdContext[i].maxRawCount) /
                    CY_CAPSENSE_PERCENTAGE_100;
            for (j = 0u; j < ptrWdCfg->numSns; j++)
            {
                if (rawCountMax < ptrWdCfg->ptrSnsContext[j].raw)
                {
                    /* Next Widget */
                    range = 1u;
                    break;
                }
            }
            /* Lower capacitance range */
            if (range == 0u)
            {
                ptrWdCxt->numSubConversions = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_NSUB_RANGE2;
                ptrWdCxt->snsClk = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK_RANGE2;
                ptrWdCxt->rowSnsClk = CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK_RANGE2;
            }
        }
        ptrWdCfg++;
    }
    autoTuneStatus |= Cy_CapSense_SsInitialize(context);

    msclpBase = ptrCommonCfg->ptrChConfig.ptrHwBase;

    msclpBase->SCAN_CTL2 &= (uint32_t)~((uint32_t)MSCLP_SCAN_CTL2_NUM_EPI_CYCLES_Msk);
    msclpBase->SCAN_CTL2 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                               MSCLP_SCAN_CTL2_NUM_EPI_CYCLES_Pos);

    msclpBase->INIT_CTL4 &= (uint32_t)~((uint32_t)MSCLP_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Msk);
    msclpBase->INIT_CTL4 |= (uint32_t)((uint32_t)CY_CAPSENSE_SMARTSENSE_PRELIMINARY_SCAN_SNS_CLK <<
                                               MSCLP_INIT_CTL4_NUM_PRO_WAIT_CYCLES_Pos);

    /* Step #4: Execute 8-step calibration scan in single RefCDAC mode */
    (void)Cy_CapSense_CalibrateAllSlots(context);

    /* Step #5: Within each widget find a sensor with max Cp / raw count */
    autoTuneConfig.modClock = ptrCommonCfg->periClkHz / ptrCommonCxt->modClk;
    autoTuneConfig.snsResistance = ptrCommonCfg->csdRConst;
    autoTuneConfig.correctionCoeff = CY_CAPSENSE_SENSOR_CONNECTION_MODE;

    ptrWdCfg = context->ptrWdConfig;
    for (i = 0u; i < CY_CAPSENSE_TOTAL_WIDGET_COUNT; i++)
    {
        if (CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
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

            /* Step #6: Calculates capacitance, clock divider and number of sub-conversions */
            autoTuneConfig.raw = (uint16_t)rawCountMax;
            (void)Cy_CapSense_GetSmartSenseCapacitance(&autoTuneConfig);
            autoTuneConfig.snsCapacitance /= ptrWdCfg->numChopCycles;
            (void)Cy_CapSense_GetSmartSenseFrequencyDivider(&autoTuneConfig);
            (void)Cy_CapSense_GetSmartSenseNumSubconversions(&autoTuneConfig);
            ptrWdCxt->numSubConversions = autoTuneConfig.nSub1 / ptrWdCfg->numChopCycles;
            /* Perform adjusting of sense clock divider mainly for high Cp */
            autoTuneConfig.kRef1 = (autoTuneConfig.kRef1 * CY_CAPSENSE_SMARTSENSE_SCALING_DECI_VAL) /
                    CY_CAPSENSE_SMARTSENSE_CORRECTION;
            autoTuneConfig.kRef1 = (autoTuneConfig.kRef1 + CY_CAPSENSE_SMARTSENSE_ROUND_UP_2_BITS_MASK) &
                    (uint32_t)~CY_CAPSENSE_SMARTSENSE_ROUND_UP_2_BITS_MASK;
            if (CY_CAPSENSE_SMARTSENSE_MAX_KREF_VAL < autoTuneConfig.kRef1)
            {
                autoTuneConfig.kRef1 = CY_CAPSENSE_SMARTSENSE_MAX_KREF_VAL;
            }

            ptrWdCxt->snsClk = autoTuneConfig.kRef1;
            ptrWdCxt->rowSnsClk = autoTuneConfig.kRef1;
            ptrWdCxt->sigPFC = autoTuneConfig.sigPFC;
        }
        ptrWdCfg++;
    }

    /* Step #7: Assigning clock sources and restore project parameters */
    ptrWdCxt = context->ptrWdContext;
    for (i = 0u; i < CY_CAPSENSE_TOTAL_WIDGET_COUNT; i++)
    {
        /* Reverting back original customer's settings */
        ptrWdCxt->snsClkSource = (uint8_t)(((uint32_t)ptrWdCxt->snsClkSource >> CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_POS) &
                ~CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_MASK);
        ptrWdCxt++;
    }

    autoTuneStatus |= Cy_CapSense_SsInitialize(context);

    ptrCommonCxt->status &= ~CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK;

    #endif /* #if (true) */

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


/*******************************************************************************
* Function Name: Cy_CapSense_TransferRawCounts
****************************************************************************//**
*
* Transfers raw counts for specified sensors from internal MSCLP IP RAM to the
* corresponding field in the structure where the sensor data is stored.
*
* \param startSlotId
* The slot ID transfer will be started from.
*
* \param numberSlots
* The number of slots will be transferred.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_TransferRawCounts(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    cy_stc_capsense_sensor_context_t * ptrSnsCxt;
    MSCLP_Type * ptrHwBase;
    uint32_t lastSlot;
    uint32_t currSlot;
    uint32_t tmpRawCount;
    uint32_t wdIndex;
    uint32_t snsIndex;
    uint32_t fifoSize;

    ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    lastSlot = startSlotId + numberSlots - 1u;
    fifoSize = ptrHwBase->SNS.RESULT_FIFO_STATUS & MSCLP_SNS_RESULT_FIFO_STATUS_USED_Msk;

    if (fifoSize >= numberSlots)
    {
        /* Reads the raw counts for last scan */
        for (currSlot = startSlotId; currSlot <= lastSlot; currSlot++)
        {
            snsIndex = context->ptrScanSlots[currSlot].snsId;
            wdIndex = context->ptrScanSlots[currSlot].wdId;
            tmpRawCount = ptrHwBase->SNS.RESULT_FIFO_RD;
            ptrSnsCxt = &context->ptrWdConfig[wdIndex].ptrSnsContext[snsIndex];
            ptrSnsCxt->status &= (uint8_t)~CY_CAPSENSE_SNS_OVERFLOW_MASK;
            if (MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk == (tmpRawCount & MSCLP_SNS_RESULT_FIFO_RD_OVERFLOW_Msk))
            {
                ptrSnsCxt->status |= CY_CAPSENSE_SNS_OVERFLOW_MASK;
            }
            ptrSnsCxt->raw = (uint16_t)tmpRawCount;
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_TransferLpRawCounts
****************************************************************************//**
*
* Transfers raw counts for the scanned low power sensors from internal MSCLP IP
* RAM to the raw count history.
*
* \param startSlotId
* The slot ID transfer will be started from.
*
* \param numberSlots
* The number of slots will be transferred.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_TransferLpRawCounts(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    MSCLP_Type * ptrHwBase;
    uint32_t lastSlot;
    uint32_t currSlot;
    uint32_t tmpRawCount;
    uint32_t wdIndex;
    uint32_t fifoSize;
    uint32_t i;
    uint8_t wdSnsMethod;
    const cy_stc_capsense_widget_config_t * ptrWdCfg;
    cy_stc_capsense_common_context_t * ptrCommonCxt = context->ptrCommonContext;
    cy_stc_capsense_lp_historic_context_t * ptrHistoricData = context->ptrLpHistoricContext;

    ptrHwBase = context->ptrCommonConfig->ptrChConfig->ptrHwBase;
    lastSlot = startSlotId + numberSlots - 1u;
    fifoSize = ptrHwBase->SNS.RESULT_FIFO_STATUS & MSCLP_SNS_RESULT_FIFO_STATUS_USED_Msk;

    if (fifoSize >= numberSlots)
    {
        /* Reset baseline after low power scan */
        if ((ptrHwBase->SNS.RESULT_FIFO_STATUS2 & MSCLP_SNS_RESULT_FIFO_STATUS2_FIFO_OVERFLOW_Msk) != 0uL)
        {
            ptrCommonCxt->lpScanSt = CY_CAPSENSE_SCAN_ST_BSLN_RESET_MSK;
        }
        else
        {
            /* Set valid mask for baseline */
            ptrCommonCxt->lpScanSt = CY_CAPSENSE_SCAN_ST_BSLN_RESET_MSK | CY_CAPSENSE_SCAN_ST_BSLN_VALID_MSK;
        }

        ptrCommonCxt->lpFirstSnsId = (uint8_t)startSlotId;
        ptrCommonCxt->lpSnsNumber = (uint8_t)numberSlots;
        ptrCommonCxt->lpNumFrame = (uint8_t)(fifoSize / numberSlots);

        /* Reads raw counts history from MSCv3LP HW internal RAM to pop the FIFO */
        for (i = 0u; i < ptrCommonCxt->lpNumFrame; i++)
        {
            for (currSlot = startSlotId; currSlot <= lastSlot; currSlot++)
            {
                tmpRawCount = ptrHwBase->SNS.RESULT_FIFO_RD & MSCLP_SNS_RESULT_FIFO_RD_RAW_COUNT_Msk;

                wdIndex = context->ptrLpScanSlots[currSlot].wdId;
                ptrWdCfg = &context->ptrWdConfig[wdIndex];
                wdSnsMethod = ptrWdCfg->senseMethod;

                if ((CY_CAPSENSE_CSX_GROUP == wdSnsMethod) ||
                    (CY_CAPSENSE_ISX_GROUP == wdSnsMethod))
                {
                    /* Limit raw counts */
                    if (tmpRawCount < ptrWdCfg->ptrWdContext->maxRawCount)
                    {
                        /* Invert raw counts for CSX/ISX widgets */
                        tmpRawCount = (ptrWdCfg->ptrWdContext->maxRawCount - tmpRawCount);
                    }
                    else
                    {
                        tmpRawCount = 0u;
                    }
                }

                *ptrHistoricData = (cy_stc_capsense_lp_historic_context_t)tmpRawCount;
                ptrHistoricData++;
            }
        }
    }
}


/*******************************************************************************
* Function Name: Cy_CapSense_WaitMrssStatusChange
****************************************************************************//**
*
* Waits till end of MRSS status is changed or till the provided timeout.
*
* \param timeout
* Watchdog timeout in microseconds.
*
* \param mrssStatus
* MRSS status to be set
* - CY_CAPSENSE_MRSS_TURN_ON - MRSS should be turned on,
* - CY_CAPSENSE_MRSS_TURN_OFF - MRSS should be turned off)
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS       - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_TIMEOUT       - The software watchdog timeout occurred
*                                      during the wait, the MRSS status change was not
*                                      completed.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_WaitMrssStatusChange(
                uint32_t timeout,
                uint32_t mrssStatus,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;
    uint32_t waitTime = timeout;

    while (mrssStatus ==
           (context->ptrCommonConfig->ptrChConfig->ptrHwBase->MRSS_STATUS & MSCLP_MRSS_STATUS_MRSS_UP_Msk))
    {
        if (0uL == waitTime)
        {
            status = CY_CAPSENSE_STATUS_TIMEOUT;
            break;
        }
        /* Delay for 1us */
        Cy_SysLib_DelayUs(1u);
        waitTime--;
    }

    return status;
}


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
    if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_128)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_256;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_64)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_128;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_32)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_64;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_16)
    {
        cic2Divider = CY_CAPSENSE_CIC2_DIVIDER_32;
    }
    else if (cic2Samples > CY_CAPSENSE_CIC2_DIVIDER_8)
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

#endif /* CY_IP_M0S8MSCV3LP */


/* [] END OF FILE */
