/***************************************************************************//**
* \file cy_capsense_sensing.c
* \version 4.0
*
* \brief
* This file consists of common parts for different supported platforms
* like fourth and fifth generation platforms.
*
********************************************************************************
* \copyright
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_capsense_common.h"
#include "cycfg_capsense_defines.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_sensing.h"
#include "cy_gpio.h"
#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    #include "cy_capsense_sensing_v2.h"
#elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #include "cy_capsense_sensing_lp.h"
    #include "cy_capsense_generator_lp.h"
#else /* (CY_CAPSENSE_PSOC4_FIFTH_GEN) */
    #include "cy_capsense_sensing_v3.h"
    #include "cy_capsense_generator_v3.h"
#endif

#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))


/*******************************************************************************
* Function Name: Cy_CapSense_ScanWidget
****************************************************************************//**
*
* Initiates the scanning of all sensors in the widget. 

* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* For fifth-generation low power CAPSENSE&trade; the function initiates only
* Active widget scans. To initiate Low Power widget scan use
* the Cy_CapSense_ScanLpWidget() function.
*
* \note
* For the fifth-generation CAPSENSE&trade; this function is available in
* single-channel solution only. It is recommended to use
* the Cy_CapSense_ScanSlots() function instead for compatibility with
* further CAPSENSE&trade; middleware versions.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanWidget(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_ScanWidget_V3(widgetId, context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        return Cy_CapSense_ScanWidget_V3Lp(widgetId, context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        return Cy_CapSense_ScanWidget_V2(widgetId, context);
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
}

/*******************************************************************************
* Function Name: Cy_CapSense_ScanSensor
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
* For the fifth-generation CAPSENSE&trade; this function is available in
* single-channel solution. It is recommended to use
* the Cy_CapSense_ScanSlots() function instead for compatibility with
* further CAPSENSE&trade; middleware versions.
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
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanSensor(
                uint32_t widgetId,
                uint32_t sensorId,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_ScanSensor_V3(widgetId, sensorId, context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        return Cy_CapSense_ScanSensor_V3Lp(widgetId, sensorId, context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        return Cy_CapSense_ScanSensor_V2(widgetId, sensorId, context);
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
}


#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)|| (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllSlots
****************************************************************************//**
*
* Initiates the non-blocking scan of all Active slots and then exits. Scanning
* is initiated only if no scan is in progress. To initiate all Low Power slot
* scan use the Cy_CapSense_ScanAllLpSlots() function.
*
* \par
* * For fifth-generation low power CAPSENSE&trade;
* This function loads slots configurations into the internal buffer of the
* CAPSENSE&trade; hardware block and triggers scanning.
* If the scan frame doesn`t fit into the internal buffer it will be processed as
* the sequence of sub-frames. Next sub-frame will be loaded by the interrupt
* service routine(part of middleware) once scan of previous sub-frame is complete.
* The scanning process duration consists of a some
* pre-configured interval and a frame scan duration. The first slot scan starts
* after the pre-configured interval. The default duration of the interval is
* 1 ILO cycle. The interval duration can be changed
* by the Cy_CapSense_ConfigureMsclpTimer() function.
* If the "Enable external frame start" option is disabled in the CAPSENSE&trade;
* Configurator, the scanning process will be triggered immediately.
* If the "Enable external frame start" option is enabled in the CAPSENSE&trade;
* Configurator, the scanning process will be triggered by the first external
* synchronization signal after call of the function.
* The number of slots in frame in this case should not exceed the maximum
* possible slot number to fit into the internal buffer of the CAPSENSE&trade;
* hardware block.
* The External synchronization signal parameters should meet the following
* constraints:
*   * - the interval between the return of the function and the rising edge
*     of an external synchronization signal must not be less than 2 ILO cycles;
*   * - the pulse duration of an external synchronization signal
*     (i.e. signal is in High level) must not be less than 2 ILO cycles and more
*     than the full scanning process duration. The full scanning process
*     duration is the time between the rising edge of the external
*     synchronization pulse and clearing of BUSY flag. The Cy_CapSense_IsBusy()
*     function can be used to check BUSY flag;
*   * - the interval between sub-sequent pulses should not be less than full
*     scanning process duration.
* Disregarding the "Enable external frame start" option, the transition into
* system DEEP SLEEP mode is allowed after the scan process is started
* by the function.
* To decrease the start scan time when it is intended to scan the same frame,
* i.e. startSlotId and numberSlots parameters are the same, then the scan
* is performed without the MSC HW block reconfiguration.
* The number of slots in frame in this case should not exceed the maximum
* possible slot number to fit into the internal buffer of the CAPSENSE&trade;
* hardware block.
*
* \par
* * For fifth-generation CAPSENSE&trade; this function initiates a scan for
* the first slot for all channels and then exits. Scans for the remaining slots
* in the Interrupt-driven scan mode are initiated in the interrupt service
* routine (part of middleware) triggered at the end
* of each scan completion for each channel. If the syncMode field in the
* cy_stc_capsense_common_config_t structure is set to CY_CAPSENSE_SYNC_MODE_OFF,
* then the next slot scan for the channel with the fired interrupt,
* will start regardless of the another channel readiness for the next scan.
* If the syncMode field is set to CY_CAPSENSE_SYNC_INTERNAL (for single-chip projects)
* or to CY_CAPSENSE_SYNC_EXTERNAL (for multi-chip projects),
* then the next slot scan for the channel with the fired interrupt,
* will start in lockstep with another channels after they all are ready
* for the next scan (the next scan configuration is loaded into the channel MSC HW block).
* Scans for the remaining slots in CS-DMA scan mode are initiated
* by DMAC triggered at the end
* of each scan completion for each channel. The channel scan synchronization is
* performed as in Interrupt-driven scan mode. After all slots are scanned,
* the FRAME interrupt is fired and the interrupt service routine (part of middleware)
* updates the busy status. The transition into system DEEP SLEEP mode is allowed
* only when all scans are finished.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* \note
* This function is available for the fifth-generation and fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_ScanAllSlots(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    if (NULL != context)
    {
        capStatus = Cy_CapSense_ScanSlots(0u, CY_CAPSENSE_SLOT_COUNT, context);
    }

    return capStatus;
}


/*******************************************************************************
* Function Name: Cy_CapSense_ScanSlots
****************************************************************************//**
*
* Initiates the non-blocking scan of a specified regular slots set named "frame"
* and then exits.
* Scanning is initiated only if no scan is in progress.
* Check the status of the current scan using the Cy_CapSense_IsBusy()
* function. The application program should wait until the current frame scan is
* finished prior to start a next scan by using the function.
*
* \par
* * For fifth-generation low power CAPSENSE&trade;
* The function loads the frame configuration into the internal buffer of the
* CAPSENSE&trade; hardware block and triggers the scanning process.
* If the frame configuration does not fit into the internal buffer its scan will
* be processed as a sequence of sub-frame scans. A next sub-frame will be loaded
* by the interrupt service routine (part of the middleware)
* after the previous sub-frame scan is complete.
* The scanning process duration consists of a some
* pre-configured interval and a frame scan duration. The first slot scan starts
* after the pre-configured interval. The default duration of the interval is
* 1 ILO cycle. The interval duration can be changed
* by the Cy_CapSense_ConfigureMsclpTimer() function.
* If the "Enable external frame start" option is disabled in the CAPSENSE&trade;
* Configurator, the scanning process will be triggered immediately.
* If the "Enable external frame start" option is enabled in the CAPSENSE&trade;
* Configurator, the scanning process will be triggered by the first external
* synchronization signal after call of the function.
* The number of slots in frame in this case should not exceed the maximum
* possible slot number to fit into the internal buffer of the CAPSENSE&trade;
* hardware block.
* The External synchronization signal parameters should meet the following
* constraints:
*   * - the interval between the return of the function and the rising edge
*     of an external synchronization signal must not be less than 2 ILO cycles;
*   * - the pulse duration of an external synchronization signal
*     (i.e. signal is in High level) must not be less than 2 ILO cycles and more
*     than the full scanning process duration. The full scanning process
*     duration is the time between the rising edge of the external
*     synchronization pulse and clearing of BUSY flag. The Cy_CapSense_IsBusy()
*     function can be used to check BUSY flag;
*   * - the interval between sub-sequent pulses should not be less than full
*     scanning process duration.
* Disregarding the "Enable external frame start" option, the transition into
* system DEEP SLEEP mode is allowed after the scan process is started
* by the function.
* To decrease the start scan time when it is intended to scan the same frame,
* i.e. startSlotId and numberSlots parameters are the same, then the scan
* is performed without the MSC HW block reconfiguration.
* The number of slots in frame in this case should not exceed the maximum
* possible slot number to fit into the internal buffer of the CAPSENSE&trade;
* hardware block.
*
* \par
* * For fifth-generation CAPSENSE&trade; this function initiates a scan for
* the first slot for all channels and then exits. Scans for the remaining slots
* in the Interrupt-driven scan mode are initiated in the interrupt service
* routine (part of middleware) triggered at the end
* of each scan completion for each channel. If the syncMode field in the
* cy_stc_capsense_common_config_t structure is set to CY_CAPSENSE_SYNC_MODE_OFF,
* then the next slot scan for the channel with the fired interrupt,
* will start regardless of the another channel readiness for the next scan.
* If the syncMode field is set to CY_CAPSENSE_SYNC_INTERNAL (for single-chip
* projects) or to CY_CAPSENSE_SYNC_EXTERNAL (for multi-chip projects),
* then the next slot scan for the channel with the fired interrupt,
* will start in lockstep with another channels after they all are ready
* for the next scan.
* The scan for the remaining slots in CS-DMA scan mode are initiated
* by DMAC triggered at the end
* of each scan completion for each channel. The channel scan synchronization is
* performed as in Interrupt-driven scan mode. After all specified slots are
* scanned, the FRAME interrupt is fired and the interrupt service routine
* (part of middleware) updates the busy status. The transition into system
* DEEP SLEEP mode is allowed only when all specified scans are finished.
* To decrease the start scan time when it is intended to scan the same slot,
* i.e. the startSlotId parameter is the same and numberSlots = 1u, then the scan
* is performed without the MSC HW block reconfiguration. Also, in the legacy
* AMUX mode sensors are not disconnected.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
*
* \note
* This function is available for the fifth-generation and fifth-generation
* low power CAPSENSE&trade;.
*
* \param startSlotId
* The slot ID the scan of the specified frame will be started from.
*
* \param numberSlots
* The number of slots in the specified frame will be scanned.
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
cy_capsense_status_t Cy_CapSense_ScanSlots(
                uint32_t startSlotId,
                uint32_t numberSlots,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_ScanSlots_V3(startSlotId, numberSlots, context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)*/
        return Cy_CapSense_ScanSlots_V3Lp(startSlotId, numberSlots, context);
    #endif
}
#endif


/*******************************************************************************
* Function Name: Cy_CapSense_ScanAllWidgets
****************************************************************************//**
*
* Initiates scanning of all enabled widgets (and sensors) in the project.
* For fifth-generation low power CAPSENSE&trade; the function initiates only
* Active widget scans. To initiate Low Power widget scan use
* the Cy_CapSense_ScanAllLpWidgets() function.
*
* This function initiates a scan only for the first sensor in the first widget
* for the fourth-generation CAPSENSE&trade; or a scan for the first slot
* for the fifth-generation CAPSENSE&trade; and fifth-generation low power
* CAPSENSE&trade; and then exits the function. The scan
* for the remaining sensors(slots) are initiated in the interrupt-driven mode
* in the interrupt service routine (part of middleware) triggered at the end
* of each scan completion or by DMA controllers in the DMA mode for
* the fifth-generation CAPSENSE&trade;. For the fifth-generation low power
* CAPSENSE&trade; the remaining sensor scans depend on the ACTIVE sensor
* quantity, for details see the Cy_CapSense_ScanAllSlots() function description.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
* The application program should wait until the current frame scan is finished prior 
* to start a next scan by using the function.
*
* \note
* For the fifth-generation CAPSENSE&trade; and fifth-generation low power
* CAPSENSE&trade; it is recommended to use the Cy_CapSense_ScanAllSlots()
* function instead for compatibility with further CAPSENSE&trade;
* middleware versions.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ScanAllWidgets(
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_ScanAllWidgets_V3(context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        return Cy_CapSense_ScanAllWidgets_V3Lp(context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        return Cy_CapSense_ScanAllWidgets_V2(context);
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
}


#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)|| (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_MwState
****************************************************************************//**
*
* Returns a detailed state of the CAPSENSE&trade; middleware and MSC and
* MSCLP HW blocks in Single- or Multi-channel mode. This feature is useful in
* multi-thread applications or in ISR.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
* If the middleware is busy, a new scan, setup widgets, any kind of reconfiguration, 
* or parameter change should not be initiated.
*
* \note
* This function is available for the fifth-generation and fifth-generation
* low power CAPSENSE&trade;.
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
*                                                  \note This field is available only
*                                                  for the fifth-generation low power
*                                                  CAPSENSE&trade;.
* - CY_CAPSENSE_MW_STATE_LP_WD_SCAN_MASK         - The last or currently scanned widget
*                                                  type is Low Power widget.
*                                                  \note This field is available only
*                                                  for the fifth-generation low power
*                                                  CAPSENSE&trade;.
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
cy_capsense_mw_state_t Cy_CapSense_MwState(const cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_MwState_V3(context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        return Cy_CapSense_MwState_V3Lp(context);
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
}
#endif /* ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)|| (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) */


/*******************************************************************************
* Function Name: Cy_CapSense_IsBusy
****************************************************************************//**
*
* This function returns a status whether MW executes HW scanning at a particular moment.
*
* If the middleware is busy, do not initiate a new scan or set up widgets.
* Use the Cy_CapSense_IsBusy() function to check HW busyness at a particular moment.
* Use the Cy_CapSense_MwState() function to verify if MW executes any firmware 
* tasks related to initialization, scanning, and processing at a particular moment.
* If the middleware is busy, a new scan, setup widgets, any kind of reconfiguration, 
* or parameter change should not be initiated.
*
* \param context
* The pointer to the CAPSENSE&trade; context
* structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the middleware as a sum of the masks.
* - CY_CAPSENSE_NOT_BUSY                - No scan is in progress and
*                                         a next scan can be initiated.
* - CY_CAPSENSE_BUSY                    - The previously initiated scan is
*                                         in progress.
*
*******************************************************************************/
uint32_t Cy_CapSense_IsBusy(
                const cy_stc_capsense_context_t * context)
{
    return (context->ptrCommonContext->status & CY_CAPSENSE_BUSY);
}


/*******************************************************************************
* Function Name: Cy_CapSense_InterruptHandler
****************************************************************************//**
*
* Implements interrupt service routine for CAPSENSE&trade; middleware.
*
* The CAPSENSE&trade; middleware uses this interrupt to implement a
* non-blocking sensor scan method, in which only the first sensor scan is
* initiated by the application program. Subsequent sensor scans are
* initiated in the interrupt service routine as soon as the current scan
* or current sub-frame is completed.
*
* The CAPSENSE&trade; middleware does not initialize or modify the priority
* of interrupts. For the proper middleware operation, the application program
* must configure CAPSENSE&trade; block interrupt and assign the interrupt
* vector to the Cy_CapSense_InterruptHandler() function. Refer to function
* usage example for details.
*
* The calls of the Start Sample and End Of Scan callbacks
* (see the \ref group_capsense_callbacks section for details) are the part
* of the Cy_CapSense_InterruptHandler() routine and they lengthen its execution.
* These callbacks should not lengthen the ISR execution too much to prevent
* different interrupt overlaps.
*
* \param base
* The pointer to the base register address of the CAPSENSE&trade; HW block.
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
* \note MSCLP HW contains two interrupt sources.
* The CAPSENSE&trade; Middleware supports only the msclp_interrupt_<b>lp</b>_IRQn
* vector and therefore it should be used.
*
* For Core CM4:
* \snippet capsense/snippet/main.c snippet_m4_capsense_interrupt_source_declaration
*
* The CAPSENSE&trade; interrupt handler should be defined by the application program
* according to the example below:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_IntHandler
*
* Then, the application program should configure and enable the CSD block interrupt
* between calls of the Cy_CapSense_Init() and Cy_CapSense_Enable() functions:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Initialization
*
* The CapSense_HW is the pointer to the base register address of
* the CAPSENSE&trade; HW block. A macro for the pointer is in the cycfg_peripherals.h
* file defined as \<Personality_Name\>_HW. If no name is specified,
* the following default names are used:
* * csd_\<Block_Number\>_csd_\<Block_Number\>_HW - for forth-generation CAPSENSE&trade; HW.
* * msc_\<Block_Number\>_msc_\<Block_Number\>_HW - for fifth-generation CAPSENSE&trade; HW.
* * msclp_\<Block_Number\>_msclp_\<Block_Number\>_HW - for fifth-generation low power CAPSENSE&trade; HW.
*
* An example of sharing the CSD HW block (fourth-generation CAPSENSE&trade;) by the CAPSENSE&trade;
* middleware and CSDADC middleware.<br>
* Declares the CapSense_ISR_cfg variable:
* \snippet capsense/snippet/main.c snippet_m4_capsense_interrupt_source_declaration
*
* Declares the CSDADC_ISR_cfg variable:
* \snippet capsense/snippet/main.c snippet_m4_adc_interrupt_source_declaration
*
* Defines the CAPSENSE&trade; interrupt handler:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_IntHandler
*
* Defines the CSDADC interrupt handler:
* \snippet capsense/snippet/main.c snippet_CSDADC_Interrupt
*
* The part of the main.c FW flow:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_TimeMultiplex
*
*******************************************************************************/
void Cy_CapSense_InterruptHandler(
                void * base,
                cy_stc_capsense_context_t * context)
{
    (void)base;
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        Cy_CapSense_InterruptHandler_V3(NULL, context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        Cy_CapSense_InterruptHandler_V3Lp(NULL, context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        Cy_CapSense_InterruptHandler_V2(NULL, context);
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
}


/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllWidgets
****************************************************************************//**
*
* Executes CapDAC/IDAC auto-calibration for all the sensors in all widgets
* of some sensing groups (CSD, CSX, or ISX) if calibration is enabled
* for such group of widgets. The auto-calibration finds CapDAC/IDAC values
* to have raw counts close to configured target values. Use sensing and
* processing functions after auto-calibration to update all baselines and
* configured filters.
*
* Having enabled the CDAC auto-calibration algorithm is the most common use
* case. It helps to tune your system considering board-to-board variation,
* temperature drift, etc. CDAC auto-calibration is enabled by default.
*
* The default auto-calibration target values are defined as:
* * 85% of the maximum raw count for CSD widgets
* * 40% of the maximum raw count for CSX widgets.
* * 40% of the maximum raw count for ISX widgets.
* Use the Cy_CapSense_SetCalibrationTarget() function to change calibration targets .
*
* For fifth-generation low power CAPSENSE&trade; the function calibrates only
* Active widgets. For Low Power widgets use
* the Cy_CapSense_CalibrateAllLpWidgets() function.
*
* This function detects the sensing method used by each widget and performs
* a successive approximation search algorithm to find the appropriate Reference
* and Compensation CapDAC (if enabled) values for all sensors for
* the fifth-generation CAPSENSE&trade; and for all Active sensors for
* the fifth-generation low power CAPSENSE&trade;. For the fifth-generation
* low power CAPSENSE&trade; if during the calibration process it is reached
* the Reference CapDAC value lower than /ref CY_CAPSENSE_CAL_REF_CDAC_MIN_CODE
* and Fine CapDAC usage is enabled in CAPSENSE&trade; Configurator then
* the FineCDAC calibration is performed with the reference CDAC value set to 0.
*
* For the the forth-generation
* CAPSENSE&trade; the function finds appropriate modulator and compensation
* IDAC (if enabled) values for all sensors in CSD widgets
* and/or IDAC values for all sensors in CSX widgets to make sensor raw count
* to the default value level.
*
* This function could be used only if Enable auto-calibration parameter
* is enabled for CSD and/or CSX (and/or ISX for fifth-generation low power
* CAPSENSE&trade;) widgets.
*
* \note
* For the fifth-generation CAPSENSE&trade; this function is available in
* single-channel solution. It is recommended to use
* the Cy_CapSense_CalibrateAllSlots() function instead for compatibility with
* further CAPSENSE&trade; middleware versions.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateAllWidgets(
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t retVal = CY_CAPSENSE_STATUS_BAD_CONFIG;

    (void)context;
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
                retVal = Cy_CapSense_CalibrateAllWidgets_V3(context);
        #endif
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
                retVal = Cy_CapSense_CalibrateAllWidgets_V3Lp(context);
        #endif
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
                retVal = Cy_CapSense_CalibrateAllWidgets_V2(context);
        #endif
    #endif

    return retVal;
}


#if (((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) && \
     ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
      (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) || \
      (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN)))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateAllSlots
****************************************************************************//**
*
* Executes CapDAC/IDAC auto-calibration for all the sensors in all widgets
* of some sensing groups (CSD, CSX, or ISX) if calibration is enabled
* for such group of widgets. The auto-calibration finds CapDAC/IDAC values
* to have raw counts close to configured target values. Use sensing and
* processing functions after auto-calibration to update all baselines and
* configured filters.
*
* Having enabled the CDAC auto-calibration algorithm is the most common use
* case. It helps to tune your system considering board-to-board variation,
* temperature drift, etc. CDAC auto-calibration is enabled by default.
*
* For fifth-generation low power CAPSENSE&trade; the function calibrates only
* Active widgets. For Low Power widgets use
* the Cy_CapSense_CalibrateAllLpWidgets() function.
*
* The function performs searching of Reference CDAC code, Compensation CDAC
* code Compensation Divider (whichever is enabled) by using a successive
* approximation method to make the sensor's raw count closest to the
* defined targets. The auto-calibration target values are defined
* (by default) as:
* * 85% of the maximum raw count for CSD widgets
* * 40% of the maximum raw count for CSX widgets.
* * 40% of the maximum raw count for ISX widgets.
* Use the Cy_CapSense_SetCalibrationTarget() function to change calibration targets .
*
* For the fifth-generation
* low power CAPSENSE&trade; if during the calibration process it is reached
* the Reference CapDAC value lower than /ref CY_CAPSENSE_CAL_REF_CDAC_MIN_CODE
* and Fine CapDAC usage is enabled in CAPSENSE&trade; Configurator then
* the FineCDAC calibration is performed with the reference CDAC value set to 0.
*
* \note
* This function is available for the fifth-generation CAPSENSE&trade; and
* fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_CalibrateAllSlots(cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_CalibrateAllSlots_V3(context);
    #else
        return Cy_CapSense_CalibrateAllSlots_V3Lp(context);
    #endif
}


/*******************************************************************************
* Function Name: Cy_CapSense_SetCalibrationTarget
****************************************************************************//**
*
* Sets the CapDAC auto-calibration raw count target for CSD, CSX or ISX
* widgets. The function only updates the target value, use
* Cy_CapSense_CalibrateWidget() / Cy_CapSense_CalibrateAllWidgets() /
* Cy_CapSense_CalibrateAllLpWidgets() / Cy_CapSense_CalibrateAllSlots() to change
* raw counts according to the updated target.
*
* The function sets the specified raw count targets if CSD, CSX and/or ISX
* widgets are in the project and the auto-calibration is enabled for them. These
* targets will be used instead the configured ones by
* Cy_CapSense_CalibrateAllSlots(), Cy_CapSense_CalibrateAllWidgets() and
* Cy_CapSense_CalibrateWidget() functions.
*
* \note
* This function is available only for the fifth-generation CAPSENSE&trade; and
* fifth-generation low power CAPSENSE&trade;.
*
* \param calibrTarget
* The raw counts target in percentage for the specified sensing method.
* It should be in range [1..99]. If the specified target is
* outside the range, then it will not be updated and the
* CY_CAPSENSE_STATUS_BAD_PARAM status will be returned.
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
cy_capsense_status_t Cy_CapSense_SetCalibrationTarget(
                uint32_t calibrTarget,
                uint32_t snsMethod,
                cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_SetCalibrationTarget_V3(calibrTarget, snsMethod, context);
    #else
        return Cy_CapSense_SetCalibrationTarget_V3Lp(calibrTarget, snsMethod, context);
    #endif
}
#endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) */


#if (((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && (1u == CY_CAPSENSE_TOTAL_CH_NUMBER)) ||\
     (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_CalibrateWidget
****************************************************************************//**
*
* Executes the CapDAC/IDAC calibration for all the sensors in the specified
* widget to the default target value.
*
* This function performs exactly the same tasks as
* Cy_CapSense_CalibrateAllWidgets(), but only for a specified widget. Use
* sensing and processing functions after auto-calibration to update all
* baselines and configured filters.
*
* \note
* For the fifth-generation CAPSENSE&trade; this function is available in
* single-channel solution. It is recommended to use
* the Cy_CapSense_CalibrateAllSlots() function instead for compatibility with
* further CAPSENSE&trade; middleware versions.
*
* \param widgetId
* Specifies the ID number of the widget. A macro for the widget ID can be found
* in the cycfg_capsense.h file defined as CY_CAPSENSE_<WIDGET_NAME>_WDGT_ID.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_CalibrateWidget(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t retVal = CY_CAPSENSE_STATUS_BAD_CONFIG;

    (void)widgetId;
    (void)context;

    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) ||\
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
            retVal = Cy_CapSense_CalibrateWidget_V3(widgetId, context);
        #endif
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN) ||\
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_CALIBRATION_EN))
            retVal = Cy_CapSense_CalibrateWidget_V3Lp(widgetId, context);
        #endif
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        #if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_CALIBRATION_EN) || \
             (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_CALIBRATION_EN))
            retVal = Cy_CapSense_CalibrateWidget_V2(widgetId, context);
        #endif
    #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */

    return retVal;
}
#endif /* #if (((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && (1u == CY_CAPSENSE_TOTAL_CH_NUMBER)) ||\
               (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) */



#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN || \
     ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && \
      (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD)))
/*******************************************************************************
* Function Name: Cy_CapSense_SetPinState
****************************************************************************//**
*
* Sets the state (drive mode and HSIOM state) of the GPIO used by a sensor.
*
* The possible states are GND, Shield, High-Z, Tx, Negative Tx, Rx, and Sensor.
* If the sensor specified in the input parameter is a ganged sensor, then
* the state of all GPIOs associated with the ganged sensor is updated.
*
* To access a sensor of CSD of button or slider widgets, use the sensor ID.
* To access a sensor of CSD matrix button or touchpad widgets,
* use either row ID or column ID as appropriate.
* To access sensor CSX widgets, use either Rx ID or Tx ID as appropriate.
*
* This function accepts the CY_CAPSENSE_SHIELD and CY_CAPSENSE_SENSOR states
* as an input only if there is at least one CSD widget in the project.
* Similarly, this function accepts the CY_CAPSENSE_TX_PIN and
* CY_CAPSENSE_RX_PIN states as an input only if there is at least one
* CSX widget in the project.
*
* This function must not be called while the middleware is in the busy state.
* Calling this function directly from the application program is not
* recommended. This function is used to implement only the custom-specific
* use cases.
*
* Functions that perform a setup and scan of a sensor/widget automatically
* set the required pin states for a sensor as required and overwrite changes
* made by this function to a sensor that are going to be scanned. Therefore
* the Cy_CapSense_SetPinState() function should be called in StartSample
* callback (see the \ref group_capsense_callbacks section for details)
* or with low-level functions that perform a single-sensor scanning.
*
* The function is available for the fourth-generation and fifth-generation (only
* for CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD) CAPSENSE&trade;. For fifth-generation
* CAPSENSE&trade with CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD and for fifth-
* generation low power CAPSENSE&trade use Cy_CapSense_SlotPinState().
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
* \funcusage
* An example of using the Cy_CapSense_SetPinState() function to perform
* sensor state re-configuration:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_SetPinState
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SetPinState(
                uint32_t widgetId,
                uint32_t sensorElement,
                uint32_t state,
                const cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_SetPinState_V3(widgetId, sensorElement, state, context);
    #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN) */
        return Cy_CapSense_SetPinState_V2(widgetId, sensorElement, state, context);
    #endif
}
#endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN || \
           ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && \
            (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD))) */


#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP || \
     ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && \
      (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)))
/*******************************************************************************
* Function Name: Cy_CapSense_SlotPinState
****************************************************************************//**
*
* Configures the specified electrode to the desired state in the specified
* slot (Active slot for fifth-generation low power
* CAPSENSE&trade;) by updating the CAPSENSE&trade; configuration.
*
* This function changes / overwrites configuration of an electrode (several
* pins in case the electrode is ganged to more pins) with a state specified
* by the pinState parameter. The function does this only for the specified slot ID
* (Active slotID for fifth-generation low power CAPSENSE&trade;).
* If the electrode should have the desired state during scans in another
* slots, the function should be called multiple times for each desired slot
* (Active slot for fifth-generation low power CAPSENSE&trade;).
*
* The function call changes the pin states permanently and all further scans of
* the slot will have the electrode state as specified by the pinState parameter.
* Call the function again to change the electrode state to a new desired one or
* reinitialize CAPSENSE&trade; middleware by using the Cy_CapSense_Enable() function.
*
* The re-configuration is available only when
* parameter Sensor connection method = CTRLMUX.
* If parameter Sensor connection method = AMUXBUS, then the Cy_CapSense_SetPinState()
* function should be used.
*
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
* You can also use this function to change the shield electrode state - call the function
* and pass the pointer to the shield electrode configuration as an input parameter.
*
* \note
* This function is available for the fifth-generation (only
* for CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) and fifth-generation low power
* CAPSENSE&trade;. For fourth-generation CAPSENSE&trade and fifth-generation
* CAPSENSE&trade with for CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD use
* Cy_CapSense_SetPinState().
*
* \param slotId
* The desired slot ID (Active slot ID for fifth-generation low power
* CAPSENSE&trade;).
*
* \param ptrEltdCfg
* The pointer to an electrode the all pins states of which will be configured
* as pinState parameter.
*
* \param pinState
* The desired pins state for CSX widget electrodes could be:
* * CY_CAPSENSE_RX_PIN - Rx electrode
* * CY_CAPSENSE_TX_PIN - Tx electrode
* * CY_CAPSENSE_GROUND - Grounded
* * CY_CAPSENSE_NEGATIVE_TX_PIN - Negative Tx electrode
*     (for multi-phase TX method)
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_VDDA2 - Connected to VDDA/2.
* The desired pins state for CSD widget electrodes could be:
* * CY_CAPSENSE_SENSOR - Self-cap sensor
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_GROUND - Grounded
* * CY_CAPSENSE_SHIELD - Shield is routed to the pin.
* The desired pins state for ISX widget electrodes could be:
* * CY_CAPSENSE_ISX_RX_PIN - ISX Rx electrode
* * CY_CAPSENSE_ISX_LX_PIN - ISX Lx electrode
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_VDDA2 - Connected to VDDA/2.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS      - Indicates the successful electrode setting.
* - CY_CAPSENSE_STATUS_BAD_CONFIG   - The function does not suppose to be
*                                     called with the current CAPSENSE&trade;
*                                     configuration.
* - CY_CAPSENSE_STATUS_BAD_PARAM    - 1) widgetID, sensorElement or state
*                                        are not valid;
*                      2) the CSD sensing method is disabled for desired
*                         CY_CAPSENSE_SHIELD or CY_CAPSENSE_SENSOR states;
*                      3) the CSX sensing method is disabled for desired
*                         CY_CAPSENSE_TX_PIN, CY_CAPSENSE_NEGATIVE_TX_PIN or
*                         CY_CAPSENSE_RX_PIN states.
*                      4) the ISX sensing method is disabled for desired
*                         CY_CAPSENSE_ISX_RX_PIN or CY_CAPSENSE_ISX_LX_PIN
*                         states (Only for fifth-generation low power CAPSENSE&trade;).
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
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_BAD_PARAM;
    uint32_t localPinState;

    if ((NULL != context) && (NULL != ptrEltdCfg) && (CY_CAPSENSE_SLOT_COUNT > slotId))
    {
        result = Cy_CapSense_ConvertPinState(pinState, &localPinState);

        if ((uint32_t)CY_CAPSENSE_SUCCESS_E == result)
        {
            #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
                result = Cy_CapSense_SlotPinState_V3(slotId, ptrEltdCfg, localPinState, context);
            #else /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) */
                result =  Cy_CapSense_SlotPinState_V3Lp(slotId, ptrEltdCfg, localPinState, context);
            #endif
        }
    }

    return result;
}
#endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP || \
           ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && \
            (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD))) */


/*******************************************************************************
* Function Name: Cy_CapSense_SetInactiveElectrodeState
****************************************************************************//**
*
* Sets a desired state for all inactive CAPSENSE&trade;-related electrodes
* for CSD, CSX, ISX scans, or BIST measurement scans.
*
* Use the function to set/change the desired state
* of all CAPSENSE&trade;-related
* electrodes which are not scanned during regular scans or BIST capacitance
* measurements. There are separate states for the CSD
* sensing method group, the CSX sensing method group, the ISX sensing method
* group, the BIST CSD sensor
* (electrode) capacitance measurement, the BIST CSX sensor
* (electrode) capacitance measurement, and the BIST shield electrode
* capacitance measurement. For instance, it can be configured the GND state
* for all inactive sensors for CSX scanning and the High-Z state for CSD
* scanning and the Shield state for BIST CSD sensor (electrode) capacitance
* measurement.
* The function updates some corresponding parameters in the CAPSENSE&trade; Data
* Structure to provide the desired state and not changes pin state immediately.
* The desired state will be applied to all inactive electrodes
* during the CSD, CSX, ISX scans or BIST capacitance measurements.
* It is not recommended to update the Data Structure registers directly.
* Additionally (only for fifth-generation CAPSENSE&trade;), the function
* recalculates sensor frames in a case of the CTRLMUX sensor connection method.
* For fifth-generation low power CAPSENSE&trade;, the function always
* recalculates all sensor frames including low-power sensors.
*
* \note ISX sensing method is only available for fifth-generation low power CAPSENSE&trade;.
*
* \param  inactiveState
* Specifies the inactive CAPSENSE&trade; electrode state:
* - CY_CAPSENSE_SNS_CONNECTION_HIGHZ
* - CY_CAPSENSE_SNS_CONNECTION_SHIELD (only for CSD scan)
* - CY_CAPSENSE_SNS_CONNECTION_GROUND
* - CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 (only for CSX scan)
*
* \param sensingGroup
* Specifies the sensing group:
* - CY_CAPSENSE_CSD_GROUP
* - CY_CAPSENSE_CSX_GROUP
* - CY_CAPSENSE_ISX_GROUP
* - CY_CAPSENSE_BIST_CSD_GROUP
* - CY_CAPSENSE_BIST_CSX_GROUP
* - CY_CAPSENSE_BIST_SHIELD_GROUP
*
* \param context
* The pointer to the CAPSENSE&trade; context
* structure \ref cy_stc_capsense_context_t.
*
* \return
* Returns the status of the operation \ref cy_capsense_status_t.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_SetInactiveElectrodeState(
             uint32_t inactiveState,
             uint32_t sensingGroup,
             cy_stc_capsense_context_t * context)
{
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_BAD_PARAM;

    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        uint32_t curChIndex;
    #endif

    #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN)
        cy_en_capsense_bist_io_state_t inactiveStateBist;
    #endif

    if (NULL != context)
    {
        context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;
        #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
            (void)Cy_CapSense_SwitchHwConfiguration(CY_CAPSENSE_HW_CONFIG_UNDEFINED, context);
        #endif

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN)
            switch (inactiveState)
            {
                case CY_CAPSENSE_SNS_CONNECTION_GROUND:
                    inactiveStateBist = CY_CAPSENSE_BIST_IO_STRONG_E;
                    break;
                case CY_CAPSENSE_SNS_CONNECTION_HIGHZ:
                    inactiveStateBist = CY_CAPSENSE_BIST_IO_HIGHZA_E;
                    break;
                case CY_CAPSENSE_SNS_CONNECTION_SHIELD:
                    inactiveStateBist = CY_CAPSENSE_BIST_IO_SHIELD_E;
                    break;
                case CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2:
                    inactiveStateBist = CY_CAPSENSE_BIST_IO_VDDA2_E;
                    break;
                default:
                    inactiveStateBist = CY_CAPSENSE_BIST_IO_UNDEFINED_E;
                    break;
            }

            if ((CY_CAPSENSE_BIST_CSD_GROUP == sensingGroup) &&
                ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_SHIELD == inactiveState)))
            {
                capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                if (context->ptrBistContext->intrEltdCapCsdISC != inactiveStateBist)
                {
                    context->ptrBistContext->intrEltdCapCsdISC = inactiveStateBist;
                }
            }

            if ((CY_CAPSENSE_BIST_CSX_GROUP == sensingGroup) &&
                ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState)))
            {
                capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                if (context->ptrBistContext->intrEltdCapCsxISC != inactiveStateBist)
                {
                    context->ptrBistContext->intrEltdCapCsxISC = inactiveStateBist;
                }
            }

            if ((CY_CAPSENSE_BIST_SHIELD_GROUP == sensingGroup) &&
                ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_SHIELD == inactiveState)))
            {
                capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                if (context->ptrBistContext->intrEltdCapShieldISC != inactiveStateBist)
                {
                    context->ptrBistContext->intrEltdCapShieldISC = inactiveStateBist;
                }
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_BIST_EN) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
            if ((CY_CAPSENSE_CSD_GROUP == sensingGroup) &&
                ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState) ||
                 (CY_CAPSENSE_SNS_CONNECTION_SHIELD == inactiveState)))
            {
                capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                if (context->ptrInternalContext->intrCsdInactSnsConn != (uint8_t)inactiveState)
                {
                    context->ptrInternalContext->intrCsdInactSnsConn = (uint8_t)inactiveState;
                    #if (!CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
                        Cy_CapSense_SetCsdInactiveState(context);
                    #else
                        capStatus = Cy_CapSense_GeneratePinFunctionConfig(context);
                    #endif
                }
            }
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
                if ((CY_CAPSENSE_CSX_GROUP == sensingGroup) &&
                    ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                     (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState) ||
                     ((CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 == inactiveState))))
                {
                    capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                    if (context->ptrInternalContext->intrCsxInactSnsConn != (uint8_t)inactiveState)
                    {
                        /* There are MSC HW modules that should be enabled and the MSC HW internal switches
                         * that should be closed only if VDDA/2 Inactive Sensor Connection is used. These
                         * modules and switches are part of the Base configuration of the MSC HW block.
                         * The code below is intended to generate the Base MSC HW configuration depending
                         * on the Inactive Sensor Connection selection.
                        */
                        if ((CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 == context->ptrInternalContext->intrCsxInactSnsConn) ||
                           (CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 == inactiveState))
                        {
                            context->ptrInternalContext->intrCsxInactSnsConn = (uint8_t)inactiveState;

                            #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
                                for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
                                {
                                    /* Generate base frame configurations for all enabled MSC channels, depending on
                                     * the Inactive Sensor Connection selection.
                                     */
                                    capStatus = Cy_CapSense_GenerateBaseConfig(curChIndex, context);

                                    if (CY_CAPSENSE_STATUS_SUCCESS != capStatus)
                                    {
                                        break;
                                    }
                                }
                                Cy_CapSense_SetCsxInactiveState(context);
                            #else /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */
                                capStatus |= Cy_CapSense_GenerateBaseConfig(context);
                            #endif
                        }
                        else
                        {
                            context->ptrInternalContext->intrCsxInactSnsConn = (uint8_t)inactiveState;
                            #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
                                capStatus = Cy_CapSense_GeneratePinFunctionConfig(context);
                            #else
                                Cy_CapSense_SetCsxInactiveState(context);
                            #endif
                        }
                    }
                }
            #else /* CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN */
                if ((CY_CAPSENSE_CSX_GROUP == sensingGroup) &&
                    ((CY_CAPSENSE_SNS_CONNECTION_GROUND == inactiveState) ||
                     (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState)))
                 {
                    capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                    if (context->ptrInternalContext->intrCsxInactSnsConn != (uint8_t)inactiveState)
                    {
                        context->ptrInternalContext->intrCsxInactSnsConn = (uint8_t)inactiveState;
                        Cy_CapSense_SetCsxInactiveState(context);
                    }
                 }
            #endif /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN || CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) */
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

        #if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) && (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN))
            if ((CY_CAPSENSE_ISX_GROUP == sensingGroup) &&
                (CY_CAPSENSE_SNS_CONNECTION_HIGHZ == inactiveState))
            {
                capStatus = CY_CAPSENSE_STATUS_SUCCESS;
                if (context->ptrInternalContext->intrIsxInactSnsConn != (uint8_t)inactiveState)
                {
                    context->ptrInternalContext->intrIsxInactSnsConn = (uint8_t)inactiveState;
                    capStatus = Cy_CapSense_GeneratePinFunctionConfig(context);
                }
            }
        #endif

        #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
            for (curChIndex = 0u; curChIndex < CY_CAPSENSE_TOTAL_CH_NUMBER; curChIndex++)
            {
                context->ptrActiveScanSns[curChIndex].currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;

                #if (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)
                    if ((CY_CAPSENSE_STATUS_SUCCESS == capStatus) &&
                        ((CY_CAPSENSE_CSD_GROUP == sensingGroup) ||
                         (CY_CAPSENSE_CSX_GROUP == sensingGroup)))
                    {
                        Cy_CapSense_GenerateAllSensorConfig(curChIndex, &context->ptrSensorFrameContext[(CY_CAPSENSE_SLOT_COUNT *
                                    (curChIndex + context->ptrCommonConfig->channelOffset)) * CY_MSC_6_SNS_REGS], context);
                    }
                #endif /* (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) */

            }
        #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
            context->ptrActiveScanSns->currentSenseMethod = CY_CAPSENSE_UNDEFINED_GROUP;

            if (CY_CAPSENSE_STATUS_SUCCESS == capStatus)
            {
                switch (sensingGroup)
                {
                    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_EN)
                        case CY_CAPSENSE_CSD_GROUP:
                    #endif /* CY_CAPSENSE_CSD_EN */
                    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSX_EN)
                        case CY_CAPSENSE_CSX_GROUP:
                    #endif /* CY_CAPSENSE_CSX_EN */
                    #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_ISX_EN)
                        case CY_CAPSENSE_ISX_GROUP:
                    #endif /* CY_CAPSENSE_ISX_EN */

                            Cy_CapSense_GenerateAllSensorConfig(CY_CAPSENSE_SNS_FRAME_ACTIVE, context);

                            #if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_LP_EN)
                                Cy_CapSense_GenerateAllSensorConfig(CY_CAPSENSE_SNS_FRAME_LOW_POWER, context);
                            #endif /* CY_CAPSENSE_LP_EN */

                            break;

                    default:
                        /* Do nothing */
                        break;
                }
            }
        #endif
    }
    return capStatus;
}

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)|| (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_ScanAbort
****************************************************************************//**
*
* This function sets the sequencer to the idle state by resetting the hardware,
* it can be used to abort current scan.
*
* \note
* This function is available for the fifth-generation CAPSENSE&trade; and
* fifth-generation low power CAPSENSE&trade;.
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
cy_capsense_status_t Cy_CapSense_ScanAbort(cy_stc_capsense_context_t * context)
{
    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
        return Cy_CapSense_ScanAbort_V3(context);
    #elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        return Cy_CapSense_ScanAbort_V3Lp(context);
    #endif
}
#endif


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && (!CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_SetCsdInactiveState
****************************************************************************//**
*
* Sets a desired pin state for all inactive CAPSENSE&trade;-related electrodes
* for CSD scans.
*
* There is the internal function and it is not recommended to call it directly
* from the application program. The function sets the desired state of all
* CSD group CAPSENSE&trade;-related electrodes which are not scanned.
* The function updates some corresponding parameters in the CAPSENSE&trade; Data
* Structure to provide the desired state and not changes pin state immediately.
* The desired state will be applied to all inactive electrodes during the CSD scan.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCsdInactiveState(
             cy_stc_capsense_context_t * context)
{
    context->ptrInternalContext->csdInactiveSnsHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;
    switch (context->ptrInternalContext->intrCsdInactSnsConn)
    {
        case CY_CAPSENSE_SNS_CONNECTION_HIGHZ:
            context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
            break;

        case CY_CAPSENSE_SNS_CONNECTION_SHIELD:
            context->ptrInternalContext->csdInactiveSnsHsiom = CY_CAPSENSE_HSIOM_SEL_CSD_SHIELD;
            #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
                if (CY_CAPSENSE_SHIELD_ACTIVE == context->ptrCommonConfig->csdShieldMode)
                {
                    context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
                }
                else
                {
                    context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
                }
            #endif

            #if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
                context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_SHIELD;
            #endif
            break;

        case CY_CAPSENSE_SNS_CONNECTION_GROUND:
            /* CY_CAPSENSE_SNS_CONNECTION_GROUND */
            context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
            break;

        default:
            /* Set the default pin state: High-Z */
            context->ptrInternalContext->csdInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
            break;
    }
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) && (!CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) */


#if ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) && (!CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
/*******************************************************************************
* Function Name: Cy_CapSense_SetCsxInactiveState
****************************************************************************//**
*
* Sets a desired pin state for all inactive CAPSENSE&trade;-related electrodes
* for CSX scans.
*
* There is the internal function and it is not recommended to call it directly
* from the application program. The function sets the desired state of all
* CSX group CAPSENSE&trade;-related electrodes which are not scanned.
* The function updates some corresponding parameters in the CAPSENSE&trade; Data
* Structure to provide the desired state and not changes pin state immediately.
* The desired state will be applied to all inactive electrodes during the CSX scan.
*
* \param context
* The pointer to the CAPSENSE&trade; context structure \ref cy_stc_capsense_context_t.
*
*******************************************************************************/
void Cy_CapSense_SetCsxInactiveState(
             cy_stc_capsense_context_t * context)
{
    context->ptrInternalContext->csxInactiveSnsHsiom = CY_CAPSENSE_HSIOM_SEL_GPIO;
    switch (context->ptrInternalContext->intrCsxInactSnsConn)
    {
        case CY_CAPSENSE_SNS_CONNECTION_HIGHZ:
            context->ptrInternalContext->csxInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
            break;
        case CY_CAPSENSE_SNS_CONNECTION_GROUND:
            /* CY_CAPSENSE_SNS_CONNECTION_GROUND */
            context->ptrInternalContext->csxInactiveSnsDm = CY_CAPSENSE_DM_GPIO_STRONG_IN_OFF;
            break;
        case CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2:
            /* CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2 */
            context->ptrInternalContext->csxInactiveSnsHsiom = CY_CAPSENSE_HSIOM_SEL_AMUXB;
            context->ptrInternalContext->csxInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
            break;
        default:
            /* Set the default pin state: High-Z */
            context->ptrInternalContext->csxInactiveSnsDm = CY_CAPSENSE_DM_GPIO_ANALOG;
            break;
    }
}
#endif /* ((CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) && (!CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) */


#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN))
/*******************************************************************************
* Function Name: Cy_CapSense_InitializeMaxRaw
****************************************************************************//**
*
* Initializes the maxRawCount and the maxRawCountRow fields of the
* cy_stc_capsense_widget_context_t structure for the specified widget.
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
* - CY_CAPSENSE_STATUS_SUCCESS      - The operation is performed successfully.
* - CY_CAPSENSE_STATUS_TIMEOUT      - A timeout reached during the scan time measurement.
* - CY_CAPSENSE_STATUS_BAD_PARAM    - Not valid input parameter.
* - CY_CAPSENSE_STATUS_HW_BUSY      - The MSCLP HW block is busy and cannot be
*                                     switched to another mode.
*
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_InitializeMaxRaw(
                uint32_t widgetId,
                cy_stc_capsense_context_t * context)
{
    uint32_t tmpVal;
    cy_capsense_status_t capStatus = CY_CAPSENSE_STATUS_SUCCESS;
    const cy_stc_capsense_widget_config_t * ptrWdCfg = &context->ptrWdConfig[widgetId];
    #if((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        uint32_t scanSlotId;
        const cy_stc_capsense_scan_slot_t * ptrScanSlots;
    #endif

    if(0u != (CY_CAPSENSE_WD_MAXCOUNT_CALC_MASK & ptrWdCfg->ptrWdContext->status))
    {
        /* Prepare and execute the measurement to obtain the MAX raw count for the one-dimension widgets
         * or columns for two-dimension widgets.
         */
        tmpVal = ptrWdCfg->firstSlotId;
        #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
            capStatus |= Cy_CapSense_ExecuteSaturatedScan(&ptrWdCfg->ptrWdContext->maxRawCount, widgetId,
                    tmpVal, CY_CAPSENSE_SATURATED_MAX_COUNT, context);
        #else
            capStatus |= Cy_CapSense_ExecuteSaturatedScan(&ptrWdCfg->ptrWdContext->maxRawCount,
                    tmpVal, CY_CAPSENSE_SATURATED_MAX_COUNT, context);
        #endif
    }

    #if((CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_MATRIX_EN) || (CY_CAPSENSE_DISABLE != CY_CAPSENSE_CSD_TOUCHPAD_EN))
        if(CY_CAPSENSE_CSD_GROUP == ptrWdCfg->senseMethod)
        {
           if(((uint8_t)CY_CAPSENSE_WD_MATRIX_BUTTON_E == ptrWdCfg->wdType) || ((uint8_t)CY_CAPSENSE_WD_TOUCHPAD_E == ptrWdCfg->wdType))
            {
                if(0u != (CY_CAPSENSE_WD_MAXCOUNT_ROW_CALC_MASK & ptrWdCfg->ptrWdContext->status))
                {
                    /* Prepare and execute the measurement to obtain the MAX raw count for rows for two-dimension widgets. */
                    scanSlotId = (uint32_t)ptrWdCfg->firstSlotId + ptrWdCfg->numSlots - 1u;

                    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
                        capStatus |= Cy_CapSense_ExecuteSaturatedScan(&ptrWdCfg->ptrWdContext->maxRawCountRow, widgetId,
                                scanSlotId, CY_CAPSENSE_SATURATED_MAX_COUNT, context);
                    #else
                        capStatus |= Cy_CapSense_ExecuteSaturatedScan(&ptrWdCfg->ptrWdContext->maxRawCountRow,
                                scanSlotId, CY_CAPSENSE_SATURATED_MAX_COUNT, context);
                    #endif

                    /* For the multi-channel mode, iterate through enabled channels and find the sensing channel that
                     * drives the sensor (not Shield only, Tx only, or Empty).
                     */
                    for (tmpVal = 0u; tmpVal < CY_CAPSENSE_TOTAL_CH_NUMBER; tmpVal++)
                    {
                        ptrScanSlots = &context->ptrScanSlots[scanSlotId + (CY_CAPSENSE_SLOT_COUNT * tmpVal)];
                        if(CY_CAPSENSE_SLOT_SHIELD_ONLY > ptrScanSlots->wdId)
                        {
                            break;
                        }
                    }
                    if(tmpVal >= CY_CAPSENSE_TOTAL_CH_NUMBER)
                    {
                        capStatus |= CY_CAPSENSE_STATUS_BAD_CONFIG;
                    }

                    /* Swap the row and column MAX raw count values in case if rows are scanned before
                     * columns (the row's sensor is placed to the first slot of the widget).
                     */
                    if (ptrWdCfg->numRows >= ptrScanSlots->snsId)
                    {
                        tmpVal = ptrWdCfg->ptrWdContext->maxRawCount;
                        ptrWdCfg->ptrWdContext->maxRawCount = ptrWdCfg->ptrWdContext->maxRawCountRow;
                        ptrWdCfg->ptrWdContext->maxRawCountRow = (uint16_t)tmpVal;
                    }
                }
            }
        }
    #endif

    return capStatus;
}
#endif /* ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)) */


#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP || \
     ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) && \
      (CY_CAPSENSE_SENSOR_CONNECTION_MODE == CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD)))
/*******************************************************************************
* Function Name: Cy_CapSense_ConvertPinState
****************************************************************************//**
*
* The internal function that converts specified electrode state to the
* internal format, depending on the used CAPSENSE&trade; platform.
*
* \note
* This function is available for the fifth-generation (only
* for CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD) and fifth-generation low power
* CAPSENSE&trade;.
*
* \param pinState
* The desired pins state for CSX widget electrodes could be:
* * CY_CAPSENSE_RX_PIN - Rx electrode
* * CY_CAPSENSE_TX_PIN - Tx electrode
* * CY_CAPSENSE_GROUND - Grounded
* * CY_CAPSENSE_NEGATIVE_TX_PIN - Negative Tx electrode
*     (for multi-phase TX method)
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_VDDA2 - Connected to VDDA/2.
* The desired pins state for CSD widget electrodes could be:
* * CY_CAPSENSE_SENSOR - Self-cap sensor
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_GROUND - Grounded
* * CY_CAPSENSE_SHIELD - Shield is routed to the pin.
* The desired pins state for ISX widget electrodes could be:
* * CY_CAPSENSE_ISX_RX_PIN - ISX Rx electrode
* * CY_CAPSENSE_ISX_LX_PIN - ISX Lx electrode
* * CY_CAPSENSE_HIGHZ - Unconnected (High-Z)
* * CY_CAPSENSE_VDDA2 - Connected to VDDA/2.
*
* \param convertedPinState
* The pointer to the converted pins state value.
*
* \return
* Returns the status of the operation:
* - CY_CAPSENSE_STATUS_SUCCESS   - Indicates the successful electrode setting conversion.
* - CY_CAPSENSE_STATUS_BAD_PARAM - 1) pinState is not valid;
*                      2) the CSD sensing method is disabled for desired
*                         CY_CAPSENSE_SHIELD or CY_CAPSENSE_SENSOR states;
*                      3) the CSX sensing method is disabled for desired
*                         CY_CAPSENSE_TX_PIN, CY_CAPSENSE_NEGATIVE_TX_PIN or
*                         CY_CAPSENSE_RX_PIN states;
*                      4) the ISX sensing method is disabled for desired
*                         CY_CAPSENSE_ISX_RX_PIN or CY_CAPSENSE_ISX_LX_PIN
*                         states (Only for fifth-generation low power CAPSENSE&trade;).
*******************************************************************************/
cy_capsense_status_t Cy_CapSense_ConvertPinState(
                uint32_t pinState,
                uint32_t * convertedPinState)
{
    cy_capsense_status_t result = CY_CAPSENSE_STATUS_BAD_PARAM;

    #if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
        switch (pinState)
        {
            case CY_CAPSENSE_GROUND:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_GND;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_HIGHZ:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
            case CY_CAPSENSE_SENSOR:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_CSD_SNS;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
            case CY_CAPSENSE_SHIELD:
                    #if (CY_CAPSENSE_CSD_SHIELD_MODE == CY_CAPSENSE_SHIELD_ACTIVE)
                        *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_ACT_SHIELD;
                    #else
                        *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_PAS_SHIELD;
                    #endif
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;
            #endif
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */


        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            case CY_CAPSENSE_TX_PIN:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_CSX_TX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_RX_PIN:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_CSX_RX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_NEGATIVE_TX_PIN:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_CSX_NEG_TX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_VDDA2:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_CSX_VDDA2;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN)
            case CY_CAPSENSE_ISX_RX_PIN:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_ISX_RX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_ISX_LX_PIN:
                    *convertedPinState = CY_CAPSENSE_PIN_STATE_IDX_ISX_LX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_ISX_EN) */

            default:
                /* No action on other sensor states */
                break;
        }
    #else
        switch (pinState)
        {
            case CY_CAPSENSE_GROUND:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_GND;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_HIGHZ:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN)
            case CY_CAPSENSE_SENSOR:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_SHIELD_EN)
            case CY_CAPSENSE_SHIELD:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;
            #endif
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSD_EN) */

        #if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN)
            case CY_CAPSENSE_TX_PIN:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_TX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_RX_PIN:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_RX;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_NEGATIVE_TX_PIN:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_TX_NEGATIVE;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;

            case CY_CAPSENSE_VDDA2:
                    *convertedPinState = CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2;
                    result = (uint32_t)CY_CAPSENSE_SUCCESS_E;
                break;
        #endif /* (CY_CAPSENSE_ENABLE == CY_CAPSENSE_CSX_EN) */

            default:
                /* No action on other sensor states */
                break;
        }
    #endif

    return result;
}
#endif

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */
/* [] END OF FILE */
