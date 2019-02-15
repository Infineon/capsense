/***************************************************************************//**
* \file cy_capsense.h
* \version 1.1
*
* \brief
* This file includes all the header files of the CapSense middleware.
*
********************************************************************************
* \copyright
* Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
********************************************************************************
* \mainpage Cypress CapSense Middleware Library
********************************************************************************
*
* CapSense is a Cypress capacitive sensing solution. Capacitive sensing can be 
* used in a variety of applications and products where conventional mechanical 
* buttons can be replaced with sleek human interfaces to transform the way 
* users interact with electronic systems. These include home appliances, and
* automotive, IoT, and industrial applications. CapSense supports multiple 
* interfaces (widgets) using both CSX and CSD sensing methods with robust 
* performance.
* 
* CapSense has become a popular technology to replace conventional 
* mechanical- and optical-based user interfaces. There are fewer parts 
* involved, which saves cost and increases reliability with no wear-and-tear. 
* The main advantages of CapSense compared with other solutions are:  
* robust performance in harsh environmental conditions and rejection of a 
* wide range of external noise sources. 
* 
* Use CapSense for:
* * Touch and gesture detection for various interfaces
* * Proximity detection for innovative user experiences and low-power 
*   optimization
* * Contactless liquid-level sensing in a variety of applications
* * Touch-free operations in hazardous materials
* 
********************************************************************************
* \section section_capsense_general General Description
********************************************************************************
*
* The CSD HW block enables multiple sensing capabilities on PSoC devices 
* including self-cap and mutual-cap capacitive touch sensing solution, 
* a 10-bit ADC, IDAC, and Comparator. The CSD driver is a low-level 
* peripheral driver, a wrapper to manage access to the CSD HW block. 
* Each middleware access to the CSD HW block is through the CSD Driver. 
* 
* The CSD HW block can support only one function at a time. However, all  
* supported functionality (like CapSense, ADC, etc.) can be 
* time-multiplexed in a design. I.e. you can save the existing state 
* of the CapSense middleware, restore the state of the ADC middleware, perform 
* ADC measurements, and then switch back to the CapSense functionality.
* For more details and code examples refer to the description of the 
* Cy_CapSense_Save() and Cy_CapSense_Restore() functions.
* 
* \image html capsense_solution.png "CapSense Solution" width=800px
* \image latex capsense_solution.png
* 
* This section describes only CapSense middleware. Refer to the corresponding 
* sections for documentation of other middleware supported by the CSD HW block.
* 
* A CapSense solution includes: 
* * The CapSense Configurator tool, which is a configuration wizard to create 
*   and configure CapSense widgets. It could be launched in ModusToolbox 
*   from the CSD personality as well as in standalone mode. 
*   It contains a separate document about how to create and 
*   configure widgets, parameters and algorithm descriptions.
* * API to control the design from the application program. This documentation 
*   describes API with code snippets of how to use them.
* * The CapSense Tuner tool for real-time tuning, testing, and debugging,
*   for easy and smooth designing of human interfaces on customer products. 
*   The Tuner tool communicates with a device through a HW bridge and 
*   communication drivers (EzI2C, UART, etc.) and allows to monitor 
*   widget statuses, sensor signals, detected touch positions, gestures, etc.
* The application program does not need to interact with the CSD driver 
* and/or other drivers such as GPIO, SysClk directly. All of that is 
* configured and managed by middleware.
* 
* Include cy_capsense.h to get access to all functions and other declarations 
* in this library. If you are using the ModusToolbox CapSense Configurator tool, 
* you can include cycfg_capsense.h only.
* 
* \subsection subsection_capsense_features Features
*
* * Offers best-in-class signal-to-noise ratio (SNR)
* * Supports Self-Capacitance (CSD) and Mutual-Capacitance (CSX) 
*   sensing methods
* * Features SmartSense auto-tuning technology for CSD sensing to avoid 
*   complex manual tuning process
* * Supports various Widgets, such as Buttons, Matrix Buttons, Sliders, 
*   Touchpads, and Proximity Sensors
* * Provides ultra-low power consumption and liquid-tolerant capacitive 
*   sensing technology
* * Contains the integrated graphical CapSense Tuner tool for real-time tuning, 
*   testing, and debugging
* * Provides superior immunity against external noise and low-radiated 
*   emission
* * Offers best-in-class liquid tolerance
* * Supports one-finger and two-finger gestures 
*
********************************************************************************
* \section section_capsense_configuration Summary of Application Programming Interface (API)
********************************************************************************
*
* The CapSense operates on the top of the CapSense Sigma Delta (CSD) driver.
* Refer to the PDL API Reference Manual.
*
* This document provides descriptions of the functions in the CapSense 
* middleware library, and descriptions of the data structures (register map) 
* used by the middleware library.
* 
* The Application Programming Interface (API) routines allow controlling and 
* executing specific tasks using the CapSense middleware. The CapSense API 
* is described in the following sections:
* * \ref group_capsense_high_level
* * \ref group_capsense_low_level
* * \ref group_capsense_data_structure
* * \ref group_capsense_enums
* * \ref group_capsense_macros
* * \ref group_capsense_callbacks
* 
********************************************************************************
* \section section_capsense_misra MISRA-C Compliance
********************************************************************************
*
* <table class="doxtable">
*   <tr>
*     <th>MISRA Rule</th>
*     <th>Rule Class (Required/Advisory)</th>
*     <th>Rule Description</th>
*     <th>Description of Deviation(s)</th>
*   </tr>
*   <tr>
*     <td>11.4</td>
*     <td>A</td>
*     <td>A conversion should not be performed between a pointer to object 
*         and an integer type.</td>
*     <td>Such conversion is performed with CapSense context in two cases: 
*         interrupt handler and DeepSleepCallback function.
*         Both cases are verified on correct operation.</td>
*   </tr>
*   <tr>
*     <td>12.13</td>
*     <td>A</td>
*     <td>The increment (++) and decrement (--) operators should not be mixed 
*         with other operators in an expression.</td>
*     <td>These violations are reported for the GCC ARM optimized form of 
*         the "for" loop that have the following syntax:
*         for(index = COUNT; index --> 0u;)
*         It is used to improve performance.</td>
*   </tr>
*   <tr>
*     <td>1.2</td>
*     <td rowspan=2> R</td>
*     <td rowspan=2> Constant: Dereference of NULL pointer.</td>
*     <td rowspan=2> These violations are reported as a result of using of offset macros
*         of CSD Driver with corresponding documented violation 20.6. Refer 
*         to CSD Driver API Ref Guide.</td>
*   </tr>
*   <tr>
*     <td>20.3</td>
*   </tr>
* </table>
*
********************************************************************************
* \section section_capsense_more_information More Information
********************************************************************************
*
* Important information about the CapSense-technology overview, appropriate 
* Cypress device for the design, CapSense system and sensor design guidelines, 
* different interfaces and tuning guidelines necessary for a successful design 
* of a CapSense system is available in the Getting Started with CapSense
* document and the product-specific CapSense design guide. Cypress highly 
* recommends starting with these documents. They can be found on the 
* Cypress web site at www.cypress.com. For details about application notes, 
* code examples, and kits, see the References section in this datasheet.
* 
* For more information, refer to the following documents:
*
* * <a href="http://www.cypress.com/trm218176"><b>Technical Reference Manual
* (TRM)</b></a>
*
* * <a href="http://www.cypress.com/ds218787"><b>PSoC 63 with BLE Datasheet
* Programmable System-on-Chip datasheet</b></a>
*
* * <a href="http://www.cypress.com/an85951"><b>AN85951 PSoC 4 and PSoC 6
* MCU CapSense Design Guide for more details</b></a>
*
* * <a href="http://www.cypress.com/an210781"><b>AN210781 Getting Started
* with PSoC 6 MCU with Bluetooth Low Energy (BLE) Connectivity</b></a>
*
********************************************************************************
* \section section_capsense_changelog Changelog
********************************************************************************
*
* <table class="doxtable">
*   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
*   <tr>
*     <td>1.1</td>
*     <td>
*         * Marked several functions as obsolete:
*           * Cy_CapSense_CSDSetupWidget()
*           * Cy_CapSense_CSDSetupWidgetExt()
*           * Cy_CapSense_CSDScan()
*           * Cy_CapSense_CSDScanExt()
*           * Cy_CapSense_CSDCalibrateWidget()
*           * Cy_CapSense_CSXSetupWidget()
*           * Cy_CapSense_CSXSetupWidgetExt()
*           * Cy_CapSense_CSXScan()
*           * Cy_CapSense_CSXScanExt()
*           * Cy_CapSense_CSXCalibrateWidget()
*         * Added two new functions:
*           * Cy_CapSense_SetupWidgetExt()
*           * Cy_CapSense_ScanExt()
*         * Fixed the shield operation when Csh is disabled.
*         * Fixed implementation of position filtering for Radial Slider widget.
*         * Fixed Cy_CapSense_DeInit() implementation by restoring hardware to  
*           its default state.
*         * Added the possibility to enable the shield with no dedicated electrodes.
*         * Added support of protocol-agnostic tuning (UART, SPI, etc.).
*     </td>
*     <td>
*         * Improved user's experience with an API.
*         * Improved documentation.
*         * Defect fixing.
*         * Enhanced functionality.
*     </td>
*   </tr>
*   <tr>
*     <td>1.0</td>
*     <td>
*         * The initial version.
*     </td>
*     <td></td>
*   </tr>
* </table>
*
* \defgroup group_capsense_high_level               High-level Functions
*
* \defgroup group_capsense_low_level                Low-level Functions
*
* \defgroup group_capsense_data_structure           CapSense Data Structure
*
* \defgroup group_capsense_structures               CapSense Structures
* \ingroup group_capsense_data_structure
* \brief The CapSense structures.
*
* \defgroup group_capsense_gesture_structures       Gesture Structures
* \ingroup group_capsense_data_structure
* \brief The Gesture-related structures.
*
* \defgroup group_capsense_enums                    Enumerated Types
*
* \defgroup group_capsense_macros                   Macros
*
* \defgroup group_capsense_macros_general           General Macros
*   \ingroup group_capsense_macros
*   \brief General macros
* \defgroup group_capsense_macros_settings          Settings Macros
*   \ingroup group_capsense_macros
*   \brief Settings macros
* \defgroup group_capsense_macros_pin               Pin-related Macros
*   \ingroup group_capsense_macros
*   \brief Pin-related macros
* \defgroup group_capsense_macros_process           Processing Macros
*   \ingroup group_capsense_macros
*   \brief Processing macros
* \defgroup group_capsense_macros_touch             Touch-related Macros
*   \ingroup group_capsense_macros
*   \brief Touch-related macros
* \defgroup group_capsense_macros_gesture           Gesture Macros
*   \ingroup group_capsense_macros
*   \brief Gesture macros
* \defgroup group_capsense_macros_miscellaneous     Miscellaneous Macros
*   \ingroup group_capsense_macros
*   \brief Miscellaneous macros
*
* \defgroup group_capsense_callbacks                Callbacks
*
* \cond SECTION_CAPSENSE_INTERNAL
* \defgroup group_capsense_internal                 Internal Functions
* \endcond
*
**/

/******************************************************************************/
/** \addtogroup group_capsense_high_level 
* \{
*
* High-level functions represent the highest abstraction layer of the 
* CapSense middleware.
* 
* These functions perform tasks such as scanning, data processing, data 
* reporting and tuning interfaces. When performing a task, different 
* initialization is required based on a sensing method or type of 
* widgets is automatically handled by these functions. Therefore, these 
* functions are sensing methods, features, and widget type agnostics. 
* 
* All the tasks required to implement a sensing system can be fulfilled 
* by the high-level functions. But, there is a set of 
* \ref group_capsense_low_level that provides access to lower level
* and specific tasks. If a design requires access to low-level tasks, 
* these functions can be used. The functions related to a given sensing 
* methods are not available if the corresponding method is disabled.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_low_level
* \{
*
* The low-level functions represent the lower layer of abstraction in 
* support of \ref group_capsense_high_level.
* 
* These functions also enable implementation of special case designs 
* requiring performance optimization and non-typical functionalities.
* 
* The functions that contain CSD or CSX in the name are specified for 
* that sensing method appropriately and should be used only with 
* dedicated widgets having that mode.
* All other functions are general to all sensing methods. Some of the 
* functions detect the sensing method used by the widget and execute 
* tasks as appropriate.
* 
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros
* \{
*
* Specifies constants used in CapSense middleware.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_enums
* \{
*
* Documents CapSense related enumerated types.
*
* \} */

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal
* \{
*
* The section documents CapSense related internal function.
*
* These function should not be used in the application program.
*
* \} \endcond */

/******************************************************************************/
/** \addtogroup group_capsense_data_structure
* \{
*
* The CapSense Data Structure organizes configuration parameters, input, and 
* output data shared among different FW modules within the CapSense. 
* 
* The key responsibilities of the Data Structure are as follows:
* * The Data Structure is the only data container in the CapSense middleware.
* * It serves as storage for the configuration and the output data.
* * All CapSense modules use the data structure for the communication 
*   and data exchange.
* 
* The CapSense Data Structure is a composite of several smaller structures 
* (for global / common data, widget data, sensor data, and pin data). 
* Furthermore, the data is split between RAM and Flash to achieve a 
* reasonable balance between resources consumption and configuration / tuning 
* flexibility at runtime and compile time. A graphical representation of 
* the CapSense Data Structure is shown below.
* 
* Note that figure below shows a sample representation and documents the 
* high-level design of the data structure, it does not include all the 
* parameters and elements in each object.
*
* \image html capsense_ds.png "CapSense Data Structure" width=800px
* \image latex capsense_ds.png
* 
* CapSense Data Structure does not perform error checking on the data 
* written to CapSense Data Structure. It is the responsibility of application 
* program to ensure register map rule are not violated while 
* modifying the value of data field in CapSense Data Structure.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_callbacks
* \{
*
* Callbacks allow the user to execute Custom code called from the CapSense 
* middleware when an event occurs. CapSense supports two callbacks.
*
* <table>
*   <tr>
*     <th>#</th>
*     <th >Callback Name</th>
*     <th>Associated Register</th>
*     <th>Callback Function Prototype</th>
*     <th>Description</th>
*   </tr>
*   <tr>
*     <td>1</td>
*     <td>Start Sample</td>
*     <td>ptrSSCallback</td>
*     <td>\ref cy_capsense_callback_t</td>
*     <td>This is called before each sensor scan triggering. Such a callback 
*         can be used to implement user-specific use cases like changing scan 
*         parameters depending on whether a sensor is going to be scanned.</td>
*   </tr>
*   <tr>
*     <td>2</td>
*     <td>End Of Scan</td>
*     <td>ptrEOSCallback</td>
*     <td>\ref cy_capsense_callback_t</td>
*     <td>This is called after sensor scan completion and there is no other 
*         sensor in the queue to be scanned.</td>
*   </tr>
*   <tr>
*     <td>3</td>
*     <td>Tuner Send Callback</td>
*     <td>ptrTunerSendCallback</td>
*     <td>\ref cy_capsense_tuner_send_callback_t</td>
*     <td>This is called by the Cy_CapSense_RunTuner() function to establish
*         synchronus communication with the Tuner tool.</td>
*   </tr>
*   <tr>
*     <td>4</td>
*     <td>Tuner Receive Callback</td>
*     <td>ptrTunerReceiveCallback</td>
*     <td>\ref cy_capsense_tuner_receive_callback_t</td>
*     <td>This is called by the Cy_CapSense_RunTuner() function to establish
*         synchronus communication with the Tuner tool.</td>
*   </tr>
* </table>
*
* \note
* 1. Callbacks 1 and 2 are called by the Cy_CapSense_InterruptHandler() function and
*   lengthen this function execution. Usually, Cy_CapSense_InterruptHandler() is called inside the
*   CSD ISR. In such a case, the callbacks mentioned lengthen the execution of the CSD ISR as well.
*
* 2. Callbacks 2 and 3 can only be registered by direct CapSense Data Structure assignment.
*
* All callbacks can be registered by direct assignment of the function pointers to the corresponding
* CapSense Data Structure field as follows:
*   <tt>context-\>ptrCommonContext-\>\<Associated Register\> = \&CallbackFunction;</tt>
*
* Callbacks 2 and 3 can be registered / unregistered using the Cy_CapSense_RegisterCallback() and 
* Cy_CapSense_UnRegisterCallback() function.
*
* \} */


#if !defined(CY_CAPSENSE_H)
#define CY_CAPSENSE_H

#include "cy_device_headers.h"
#include "cy_capsense_common.h"
#include "cy_capsense_centroid.h"
#include "cy_capsense_control.h"
#include "cy_capsense_csd.h"
#include "cy_capsense_csx.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_lib.h"
#include "cy_capsense_gesture_lib.h"
#include "cy_capsense_processing.h"
#include "cy_capsense_sensing.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_tuner.h"

#ifndef CY_IP_MXCSDV2
    #error "The CSD driver is not supported on this device"
#endif

#endif /* CY_CAPSENSE_H */


/* [] END OF FILE */
