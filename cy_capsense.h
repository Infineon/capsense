/***************************************************************************//**
* \file cy_capsense.h
* \version 7.0
*
* \brief
* This file includes all the header files of the CAPSENSE&trade; middleware.
*
********************************************************************************
* \copyright
* Copyright 2018-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
********************************************************************************
* \mainpage
********************************************************************************
* CAPSENSE&trade; is a CYPRESS&trade; capacitive sensing solution from
* Infineon. Capacitive sensing can be used in a variety of applications
* and products where conventional mechanical buttons can be replaced with
* sleek human interfaces to transform the way users interact with electronic
* systems. These include home appliances, and automotive, IoT, and
* industrial applications. CAPSENSE&trade; supports multiple interfaces
* (widgets) using CSD, CSX, and ISX sensing methods with robust performance.
*
* CAPSENSE&trade; has become a popular technology to replace conventional
* mechanical- and optical-based user interfaces. There are fewer parts
* involved, which saves cost and increases reliability with no wear-and-tear.
* The main advantages of CAPSENSE&trade; compared with other solutions are:
* robust performance in harsh environmental conditions and rejection of a
* wide range of external noise sources.
*
* Use CAPSENSE&trade; for:
* * Touch and gesture detection for various interfaces
* * Proximity detection for innovative user experiences and low power
*   optimization
* * Contactless liquid-level sensing in a variety of applications
* * Touch-free operations in hazardous materials
*
********************************************************************************
* \section section_capsense_general General Description
********************************************************************************
*
* The CAPSENSE&trade; middleware Library supports operation with fourth-generation,
* fifth-generation and fifth-generation low power of CAPSENSE&trade; HW.
* CAPSENSE&trade; HW enables the multiple sensing
* capabilities on PSOC&trade; devices including the Self-Capacitance (CSD) and Mutual-Capacitance (CSX)
* capacitive touch sensing solutions, inductive sensing (ISX), impedance measurement, and
* other features.
*
* Middleware access available the CAPSENSE&trade; HW block through the corresponding
* peripheral Driver:
*  - CSD (CAPSENSE&trade; Sigma-Delta) driver for the forth-generation of the
*    CAPSENSE&trade; HW;
*  - MSC (Multi-Sensor Converter) driver for the fifth-generation of the
*    CAPSENSE&trade; HW;
*  - MSCLP (Multi-Sensor Converter Low Power) driver for the fifth-generation of the
*    CAPSENSE&trade; HW;
*
* The CAPSENSE&trade; peripheral driver does not provide any system-level functions.
* It only implements the interface to the CAPSENSE&trade; HW resource.
*
* The supported CAPSENSE&trade; HW blocks can perform only one function at a time.
* However, all supported functionality (like CAPSENSE&trade;, CSDADC, etc.) can be
* time-multiplexed in a design. I.e. you can save the existing state of
* the CAPSENSE&trade; middleware, restore the state of the CSDADC middleware, perform
* CSDADC measurements, and then switch back to the CAPSENSE&trade; functionality.
* For details and code examples, refer to the description of the
* Cy_CapSense_Save() and Cy_CapSense_Restore() functions.
*
* \image html CAPSENSE_SOLUTION_MSCLP.png "CAPSENSE&trade; Solution" width=800px
* \image latex CAPSENSE_SOLUTION_MSCLP.png
*
* This section describes only the CAPSENSE&trade; middleware. Refer to the corresponding
* sections of documentation for other middleware supported by the CAPSENSE&trade; HW.
*
* A CAPSENSE&trade; solution includes:
* * The CAPSENSE&trade; Configurator tool to create
*   and configure CAPSENSE&trade; widgets. It can be launched in ModusToolbox&trade;
*   from the CAPSENSE&trade; superblock personality and in Stand-alone mode.
*   It contains a separate document about how to create and
*   configure widgets, parameters and algorithm descriptions.
* * API to control the design from the application program. This documentation
*   describes API with code snippets of how to use them.
* * The CAPSENSE&trade; Tuner tool for real-time tuning, testing, and debugging,
*   for easy and smooth designing of human interfaces on customer products.
*   The Tuner tool communicates with a device through a HW bridge and
*   communication drivers (EzI2C, UART, etc.) and allows to monitor
*   widget statuses, sensor signals, detected touch positions, gestures, etc.
* The application program does not need to interact with the CSD driver
* and/or other drivers such as GPIO, SysClk directly. All of that is
* configured and managed by middleware.
*
* Include cy_capsense.h to get access to all functions and other declarations
* in this library. If you are using the ModusToolbox&trade; CAPSENSE&trade; Configurator tool,
* you can include cycfg_capsense.h only.
*
* \subsection subsection_capsense_features Features
*
* * Offers best-in-class signal-to-noise ratio (SNR)
* * Supports Self-Capacitance (CSD) and Mutual-Capacitance (CSX) sensing methods
* * Inductive sensing method (ISX) available for fifth-generation low power CAPSENSE&trade;
* * Fifth-generation low power CAPSENSE&trade; supports autonomous scanning in Deep Sleep
*   power mode
* * Supports various Widgets, such as Buttons, Matrix Buttons, Sliders,
*   Touchpads, and Proximity Sensors
* * Provides ultra-low power consumption capacitive
*   sensing technology
* * Contains the integrated graphical CAPSENSE&trade; Tuner tool for real-time tuning,
*   testing, and debugging
* * Provides superior immunity against external noise and low-radiated
*   emission
* * Offers best-in-class liquid tolerance
* * Supports one-finger and two-finger gestures
*
********************************************************************************
* \section group_capsense_quick_start Quick Start Guide
********************************************************************************
*
* The quickest way to get started with CAPSENSE&trade; is using code examples.
* Infineon Technologies continuously extends their portfolio of code examples
* at the <a href="http:/\/www.infineon.com"><b>Infineon Technologies</b></a>
* and at the <a href="https:/\/github.com/Infineon">
* <b> Infineon Technologies GitHub</b></a>. The following are the links
* to code examples for different PSOC&trade; families presented on GitHub.
*
* \subsection group_capsense_quick_start_psoc4_csd_ces Code Examples for PSOC&trade; 4 Devices with fourth-generation CAPSENSE&trade;
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-capsense-csd-button-tuning">
*   <b>CSD Button Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-capsense-csx-button-tuning">
*   <b>CSX Button Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-capsense-csd-slider-tuning">
*   <b>CSD Slider Manual Tuning</b></a>
*
* \subsection group_capsense_quick_start_psoc4_msc_ces Code Examples for PSOC&trade; 4 Devices with fifth-generation CAPSENSE&trade;
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msc-capsense-csd-button-tuning">
*   <b>CSD Button Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msc-capsense-csx-button-tuning">
*   <b>CSX Button Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msc-capsense-csd-slider-tuning">
*   <b>CSD Slider Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msc-capsense-csd-touchpad-tuning">
*   <b>CSD Touchpad Manual Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msc-capsense-csx-touchpad-tuning">
*   <b>CSX Touchpad Manual Tuning</b></a>
*
* \subsection group_capsense_quick_start_psoc4_msclp_ces Code Examples for PSOC&trade; 4 Devices with fifth-generation low power CAPSENSE&trade;
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msclp-capsense-low-power">
*   <b>MSCLP CAPSENSE&trade; low power</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msclp-low-power-csd-button">
*   <b>MSCLP low-power self-capacitance button</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msclp-low-power-csx-slider">
*   <b>MSCLP low-power mutual-capacitance slider</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msclp-liquid-tolerant-proximity">
*   <b>MSCLP CAPSENSE&trade; liquid-tolerant proximity-sensing</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc4-msclp-csx-touchpad">
*   <b>MSCLP multitouch mutual-capacitance touchpad tuning</b></a>
*
* \subsection group_capsense_quick_start_psoc6_csd_ces Code Examples for PSOC&trade; 6 Devices with fourth-generation CAPSENSE&trade;
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-capsense-buttons-slider">
*   <b>Two Buttons and Slider with CAPSENSE&trade; Tuner</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-low-power-capsense-freertos">
*   <b>low power Slider and Ganged Sensors with FreeRTOS</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-capsense-custom-scan">
*   <b>Custom Scan</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-ble-capsense-tuner-client">
*   <b>BLE Client for CAPSENSE&trade; Tuning</b></a>
*
* * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-ble-capsense-tuner-server">
*   <b>BLE Server for CAPSENSE&trade; Tuning</b></a>
*
* The CAPSENSE&trade; middleware can be used in various development environments
* such as ModusToolbox&trade;, MBED, etc. Refer to the \ref section_capsense_toolchain.
* The following are the configuration considerations for the different environments.
*
* \subsection group_capsense_quick_start_modus ModusToolbox&trade; Configuration Considerations
*
* This quick start guide assumes that the environment is configured:
* * The <a href="https:/\/github.com/Infineon/mtb-pdl-cat1">
* <b>CAT1 Peripheral Driver Library (PDL)</b></a> is included in the project
* in case if the PSOC&trade; 6 device is used.
* * The <a href="https:/\/github.com/Infineon/mtb-pdl-cat2">
* <b>CAT2 Peripheral Driver Library (PDL)</b></a> is included in the project
* in case if the PSOC&trade; 4 device is used.
* * <a href="https:/\/www.infineon.com/ModusToolboxDeviceConfig">
* <b>ModusToolbox&trade; Device Configurator Tool</b></a>,
* <a href="https:/\/www.infineon.com/ModusToolboxCapSenseConfig">
* <b>ModusToolbox&trade; CAPSENSE&trade; Configurator Tool</b></a>, and
* <a href="https:/\/www.infineon.com/ModusToolboxCapSenseTuner">
* <b>ModusToolbox&trade; CAPSENSE&trade; Tuner Tool</b></a>
* are installed on the machine.
*
* \note
* Ensure to set up the device power voltages correctly
* to the proper operation of the device power domains. The Setup is
* on the System Tab of the Device Configurator tool. Enable the Power check box
* and set up the voltages as they are red-outlined in the picture below.
*
* \image html check_power.png "Power Setup" width=800px
* \image latex check_power.png
*
* \subsection group_capsense_quick_start_mbed MBED OS Configuration Considerations
*
* You can immediately start with the following MBED OS code example available
* at the <a href="https:/\/github.com/Infineon/">
* <b> Infineon GitHub</b></a>:
* * <a href="https:/\/github.com/Infineon/mbed-os-example-capsense">
*   <b>CAPSENSE&trade; buttons and slider for PSOC&trade; 6 MCU with Mbed OS</b></a>
*
* If you are doing your own project, remember to include cycfg.h file:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Resource_Include
*
* and call the resource initialization functions in main() at the beginning:
* \snippet capsense/snippet/main.c snippet_Cy_CapSense_Resource_Initialization
*
********************************************************************************
* \section section_capsense_configuration Summary of Application Programming Interface (API)
********************************************************************************
*
* The CAPSENSE&trade; middleware operates on the top of the MSCLP/MSC/CSD HW driver.
* Refer to the PDL API Reference Manual.
*
* This document provides descriptions of the functions in the CAPSENSE&trade;
* middleware library, and descriptions of the data structures (register map)
* used by the middleware library.
*
* The Application Programming Interface (API) routines allow controlling and
* executing specific tasks using the CAPSENSE&trade; middleware. The CAPSENSE&trade; API
* is described in the following sections:
* * \ref group_capsense_high_level
* * \ref group_capsense_low_level
* * \ref group_capsense_data_structure
* * \ref group_capsense_enums
* * \ref group_capsense_macros
* * \ref group_capsense_callbacks
*
********************************************************************************
* \section section_capsense_toolchain Supported Software and Tools
********************************************************************************
*
* This version of the CAPSENSE&trade; middleware was validated for compatibility
* with the following Software and Tools:
* <table class="doxtable">
*   <tr>
*     <th>Software and Tools</th>
*     <th>Version</th>
*   </tr>
*   <tr>
*     <td>ModusToolbox&trade; Software Environment</td>
*     <td>3.5.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; Device Configurator tool</td>
*     <td>5.30</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; MSC Superblock Personality for PSOC&trade; 4 devices in the Device Configurator tool</td>
*     <td>1.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; MSCLP Personality for PSOC&trade; 4 devices in the Device Configurator tool</td>
*     <td>3.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; MSC Personality for PSOC&trade; 4 devices in the Device Configurator tool</td>
*     <td>1.1</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; CSD Personality for PSOC&trade; 4 devices in the Device Configurator tool</td>
*     <td>2.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; CSD Personality for PSOC&trade; 6 devices in the Device Configurator tool</td>
*     <td>3.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; CAPSENSE&trade; Configurator tool</td>
*     <td>8.0.0</td>
*   </tr>
*   <tr>
*     <td>- ModusToolbox&trade; CAPSENSE&trade; Tuner tool</td>
*     <td>8.0.0</td>
*   </tr>
*   <tr>
*     <td>CAT1 Peripheral Driver Library (PDL)</td>
*     <td>3.16.0</td>
*   </tr>
*   <tr>
*     <td>CAT2 Peripheral Driver Library (PDL)</td>
*     <td>2.16.0</td>
*   </tr>
*   <tr>
*     <td>GCC Compiler</td>
*     <td>11.3.1</td>
*   </tr>
*   <tr>
*     <td>IAR Compiler</td>
*     <td>9.50.2</td>
*   </tr>
*   <tr>
*     <td>Arm Compiler 6 <sup><b>(Note 1)</b></sup></td>
*     <td>6.22.0</td>
*   </tr>
*   <tr>
*     <td>MBED OS (only for PSOC&trade; 6)</td>
*     <td>5.15.8</td>
*   </tr>
* </table>
*
* \note
* The CAPSENSE&trade; middleware includes the pre-compiled libraries for
* Arm Compiler 6. They are built with the following options to be compatible
* with ModusToolbox&trade; and MBED:
* * -fshort-enums - Set the size of an enumeration type to the smallest
*                   data type that can hold all enumerator values
* * -fshort-wchar - Set the size of wchar_t to 2 bytes
* \note
* To operate in custom environments with Arm Compiler 6, apply
* the above mentioned build options.
*
********************************************************************************
* \section section_capsense_update Update to Newer Versions
********************************************************************************
* Refer to the \ref section_capsense_changelog to learn about the design impact
* of the newer version. Set up your environment in accordance with
* \ref section_capsense_toolchain.
*
* Ensure:
* * The specified version of the ModusToolbox&trade; Device Configurator tool and
*   the Personality are used to re-generate the device configuration.
* * The specified version of the ModusToolbox&trade; CAPSENSE&trade; Configurator tool is used
*   to re-generate the middleware configuration.
* * The toolchains are set up properly for your environment per the settings
*   outlined in the Supported Software and Tools.
* * The project is re-built once the toolchains are configured and the
*   configuration is completed.
*
* You might need to re-generate the configuration structures for either the
* device initialization code or the middleware initialization code.
* * Launch the ModusToolbox&trade; Device Configurator tool and perform the File->Save command
*   to re-generate the device initialization code.
* * From the ModusToolbox&trade; Device Configurator tool, launch the
*   ModusToolbox&trade; CAPSENSE&trade; Configurator tool and perform the File->Save command to
*   re-generate the middleware initialization code.
*
********************************************************************************
* \section section_capsense_memory_usage Memory Usage
********************************************************************************
*
* The CAPSENSE&trade; middleware Flash and RAM memory consumption varies:
* * marginally - depending on the compiler and device
* * significantly - depending on the project CAPSENSE&trade; configuration and
*   number of APIs called by the application program.
*
* The table below provides the middleware total memory consumption for
* specific CAPSENSE&trade; configurations. Memory consumption for any custom
* design/configuration can be determined by analyzing a *.map file
* generated by the compiler.
*
* The measurements were done with GCC compiler configured in
* the Release mode with optimization set for Size.
*
* <table class="doxtable">
*  <tr><th>Configuration:</th><th>Mode</th><th>Sensor Connection Type</th><th>Memory Type</th><th>Configuration 1</th><th>Configuration 2</th><th>Configuration 3</th><th>Configuration 4</th><th>Configuration 5</th><th>Configuration 6</th><th>Configuration 7</th><th>Configuration 8</th><th>Configuration 9</th></tr>
*  <tr align="center"><td rowspan=2>4th Gen</td><td rowspan=2>IntDrv</td><td rowspan=2>AMUX</td><td>Flash:</td><td>&lt; 6.1 kB</td><td>&lt; 8.7 kB</td><td>&lt; 11.8 kB</td><td>&lt; 5.6 kB</td><td>&lt; 6.3 kB</td><td>&lt; 8.0 kB</td><td>&lt; 9.2 kB</td><td>&lt; 9.6 kB</td><td>&lt; 19.2 kB</td></tr>
*  <tr align="center"><td>SRAM:</td><td>&lt; 0.6 kB</td><td>&lt; 0.8 kB</td><td>&lt; 0.9 kB</td><td>&lt; 0.8 kB</td><td>&lt; 0.8 kB</td><td>&lt; 1.1 kB</td><td>&lt; 1.8 kB</td><td>&lt; 0.7 kB</td><td>&lt; 2.5 kB</td></tr>
*  <tr align="center"><td rowspan=6>5th Gen</td><td rowspan=4>IntDrv</td><td rowspan=2>AMUX</td><td>Flash:</td><td>&lt; 8.2 kB</td><td>&lt; 11.0 kB</td><td>&lt; 12.4 kB</td><td>&lt; 8.2 kB</td><td>&lt; 9.6 kB</td><td>&lt; 10.7 kB</td><td>&lt; 11.8 kB</td><td>&lt; 12.6 kB</td><td>&lt; 16.3 kB</td></tr>
*  <tr align="center"><td>SRAM:</td><td>&lt; 1.1 kB</td><td>&lt; 1.2 kB</td><td>&lt; 1.3 kB</td><td>&lt; 1.3 kB</td><td>&lt; 1.3 kB</td><td>&lt; 1.7 kB</td><td>&lt; 2.3 kB</td><td>&lt; 1.7 kB</td><td>&lt; 3.3 kB</td></tr>
*  <tr align="center"><td rowspan=2>CTRL_MUX</td><td>Flash:</td><td>&lt; 7.9 kB</td><td>&lt; 10.9 kB</td><td>&lt; 12.0 kB</td><td>&lt; 7.8 kB</td><td>&lt; 9.7 kB</td><td>&lt; 10.3 kB</td><td>&lt; 11.4 kB</td><td>&lt; 12.4 kB</td><td>&lt; 16.0 kB</td></tr>
*  <tr align="center"><td>SRAM:</td><td>&lt; 1.3 kB</td><td>&lt; 1.5 kB</td><td>&lt; 1.5 kB</td><td>&lt; 1.2 kB</td><td>&lt; 1.2 kB</td><td>&lt; 2.6 kB</td><td>&lt; 3.3 kB</td><td>&lt; 2.1 kB</td><td>&lt; 5.4 kB</td></tr>
*  <tr align="center"><td rowspan=2>DMA</td><td rowspan=2>CTRL_MUX</td><td>Flash:</td><td>&lt; 8.7 kB</td><td>&lt; 11.7 kB</td><td>&lt; 13.0 kB</td><td>&lt; 8.6 kB</td><td>&lt; 10.4 kB</td><td>&lt; 11.0 kB</td><td>&lt; 12.1 kB</td><td>&lt; 13.1 kB</td><td>&lt; 16.7 kB</td></tr>
*  <tr align="center"><td>SRAM:</td><td>&lt; 1.3 kB</td><td>&lt; 1.5 kB</td><td>&lt; 1.5 kB</td><td>&lt; 2.2 kB</td><td>&lt; 2.2 kB</td><td>&lt; 2.6 kB</td><td>&lt; 3.3 kB</td><td>&lt; 2.1 kB</td><td>&lt; 5.4 kB</td></tr>
*  <tr align="center"><td rowspan=2>5th Gen Low Power</td><td rowspan=2>LP-AOS</td><td rowspan=2>AMUX</td><td>Flash:</td><td>&lt; 7.9 kB</td><td>&lt; 11.8 kB</td><td>&lt; 12.9 kB</td><td>&lt; 7.7 kB</td><td>&lt; 10.1 kB</td><td>&lt; 10.2 kB</td><td>&lt; 11.6 kB</td><td>&lt; 12.2 kB</td><td>&lt; 15.8 kB</td></tr>
*  <tr align="center"><td>SRAM:</td><td>&lt; 1.2 kB</td><td>&lt; 1.4 kB</td><td>&lt; 1.5 kB</td><td>&lt; 2.1 kB</td><td>&lt; 2.1 kB</td><td>&lt; 2.5 kB</td><td>&lt; 3.2 kB</td><td>&lt; 1.5 kB</td><td>&lt; 4.8 kB</td></tr>
*  <tr><th colspan=13>Widgets</th></tr>
*  <tr><td colspan=4>CSD Button</td><td>3(10 sensors)</td><td>3(10 sensors)</td><td>3(10 sensors)</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>1(1 sensor)</td><td>1(1 sensor)</td></tr>
*  <tr><td colspan=4>CSD Matrix Buttons</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSD Slider</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSD Touchpad</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>1(8x8)</td><td>1(8x8)</td></tr>
*  <tr><td colspan=4>CSD Proximity</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSX Button</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSX Matrix Buttons</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>1(4x8)</td><td>1(4x8)</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSX Touchpad</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>1(9x4)</td><td>1(9x4)</td><td>&nbsp;</td><td>1(8x8)</td></tr>
*  <tr><th colspan=13>Features</th></tr>
*  <tr><td colspan=4>Gesture</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Ballistic Multiplier</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Centroid Type</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Supported fingers on touchpad</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Shield</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>enabled</td></tr>
*  <tr><td colspan=4>SmartSense</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSD auto-calibration</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>CSX auto-calibration</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Self-test</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><th colspan=13>Raw Count Filters</th></tr>
*  <tr><td colspan=4>IIR</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>enabled</td><td>enabled</td></tr>
*  <tr><td colspan=4>Median</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Average</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><th colspan=13>Position Filters</th></tr>
*  <tr><td colspan=4>IIR</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Median</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Average</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Adaptive IIR</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
*  <tr><td colspan=4>Jitter</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>&nbsp;</td><td>enabled</td><td>&nbsp;</td><td>&nbsp;</td></tr>
* </table>
*
* \note
* Configuration considerations:
* * To select values for the Scan mode and Sensor connection method parameters (for the fifth-generation of the CAPSENSE&trade; HW)
* navigate to the Advanced tab in the CAPSENSE&trade; Configurator tool, and then select the General settings sub-tab.
*
* * For the forth-generation of the CAPSENSE&trade; HW, the IntDrv mode with the AMUX sensor connection type is available only.
*
********************************************************************************
* \section section_capsense_misra MISRA-C:2012 Compliance
********************************************************************************
*
* This page describes MISRA-C:2012 compliance and deviations for
* the CAPSENSE&trade; middleware.
*
* MISRA stands for Motor Industry Software Reliability Association. The MISRA
* specification covers a set of 10 mandatory rules, 110 required rules and
* 39 advisory rules that apply to firmware design and has been put together
* by the Automotive Industry to enhance the quality and robustness of
* the firmware code embedded in automotive devices.
*
* \subsection subsection_capsense_misra-mw CAPSENSE&trade; Middleware Deviation
*
* <table class="doxtable">
*   <tr>
*     <th>Rule ID</th>
*     <th>Rule Description</th>
*     <th>Description of Deviation(s)</th>
*   </tr>
*    <tr>
*     <td>Directive  4.8</td>
*     <td>If a pointer to a structure or union is never dereferenced within a translation
*         unit, then the implementation of the object should be hidden.
*     </td>
*     <td>Advisory. The middleware library consists of several modules. One of them
*         is CapSense Data Structure. All communication between the other modules
*         is performed through CapSense Data Structure.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 2.3</td>
*     <td>A project should not contain unused type declarations.
*     </td>
*     <td>Advisory. The middleware library provides API to the hardware. The type
*         is part of API, which is defined for application-level only.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 2.5</td>
*     <td>A project should not contain unused macro declarations.
*     </td>
*     <td>Advisory. The middleware library provides API to the hardware. The macro
*         is part of API, which is defined for application-level only.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 5.1</td>
*     <td>External identifiers shall be distinct.
*     </td>
*     <td>Toolchains from Supported Software and Tools documentation section
*         are verified to work with functions, names of which have similar first 31 symbols.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 5.4</td>
*     <td>Macro identifiers shall be distinct.
*     </td>
*     <td>Toolchains from Supported Software and Tools documentation section
*         are verified to work with macros, names of which have similar first 31 symbols.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 5.6</td>
*     <td>A typedef name shall be a unique identifier.
*     </td>
*     <td>During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All typedef names are unique for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 5.8</td>
*     <td>Identifiers that define objects or functions with external linkage shall be unique.
*     </td>
*     <td>During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 5.9</td>
*     <td>Identifiers that define objects or functions with internal linkage should be unique.
*     </td>
*     <td>Advisory. During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 8.3</td>
*     <td>All declarations of an object or function shall use the same names and type qualifiers.
*     </td>
*     <td>During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 8.5</td>
*     <td>An external object or function shall be declared once in one and only one file.
*     </td>
*     <td>During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 8.6</td>
*     <td>An identifier with external linkage shall have exactly one external definition.
*     </td>
*     <td>During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 8.7</td>
*     <td>Functions and objects should not be defined with external linkage if they
*         are referenced in only one translation unit.
*     </td>
*     <td>Advisory. During the code analysis, the same source files are compiled multiple times
*         with device-specific options. All object and function identifiers are unique
*         for each specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 8.13</td>
*     <td>A pointer should point to const-qualified type whenever possible.
*     </td>
*     <td>Advisory. During the code analysis, the same source files are compiled multiple times
*         with device-specific options. A function argument can be const for some specific run.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 11.4</td>
*     <td>A conversion should not be performed between a pointer to object and an integer type.
*     </td>
*     <td>Advisory. There is an issue with a SYSCLK driver (IFXID-10550), due to which integer type
*          is cast to an object pointer type. Code is manually checked and reviewed to be safe.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 11.5</td>
*     <td>A conversion should not be performed from pointer to void into pointer to object.
*     </td>
*     <td>Advisory. The cast from void pointer to an object pointer does not have any unintended effect,
*         as it is a consequence of the definition of a structure based on function pointers.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 14.2</td>
*     <td>A for loop shall be well-formed.
*     </td>
*     <td>The third clause of the for loop is empty to improve execution time.
*     </td>
*   </tr>
*    <tr>
*     <td>Rule 18.4</td>
*     <td>The +, -, += and -= operators should not be applied to an expression of pointer type.
*     </td>
*     <td>Advisory. There are several instances of pointer arithmetic in drivers. They cannot be avoided,
*         so are manually checked and reviewed to be safe.
*     </td>
*   </tr>
* </table>
*
* \subsection subsection_capsense_misra-cfg CAPSENSE&trade; Configurator Tool Generated Sources Deviation
*
* <table class="doxtable">
*   <tr>
*     <th>Rule ID</th>
*     <th>Rule Description</th>
*     <th>Description of Deviation(s)</th>
*   </tr>
*   <tr>
*     <td>Rule 3.1</td>
*     <td>The character sequences / * and / / shall not be used within a comment.</td>
*     <td>Using of the special comment symbols is need for Doxygen comment
*         support, it does not have any impact on functionality. </td>
*   </tr>
*   <tr>
*     <td>Rule 8.4</td>
*     <td>A compatible declaration shall be visible when an object or function with external linkage is defined.</td>
*     <td>The CAPSENSE&trade; middleware library consists of several modules. One of them is CAPSENSE&trade; Built-in Self Test (BIST).
*         All the BIST variables are used in other data structures and accessed using pointers.</td>
*   </tr>
*   <tr>
*     <td>Rule 11.4</td>
*     <td>A conversion should not be performed between a pointer to object and an integer type.</td>
*     <td>Advisory. There are several instances of pointer conversions in Generated Source.
*         They are manually checked and reviewed to be safe.</td>
*   </tr>
* </table>
*
********************************************************************************
* \section section_capsense_errata Errata
********************************************************************************
*
* This section lists the known problems with the CAPSENSE&trade; middleware.
*
* <table class="doxtable">
*   <tr><th>ID</th><th>Known Issue</th><th>Workaround</th></tr>
*   <tr>
*     <td>319100</td>
*     <td>
*         GPIO simultaneous operation with unrestricted strength and frequency
*         creates noise that can affect CAPSENSE&trade; operation. This issue
*         is applicable for the fourth CAPSENSE&trade; generation devices.
*     </td>
*     <td>
*         For detail, refer to the errata section of the device datasheet.
*     </td>
*   </tr>
*   <tr>
*     <td>3159</td>
*     <td>
*         Scanning a sensor with low capacitance (about 8pF and less) with low
*         frequency (around 300kHz and less) might lead to raw count variation
*         from scan to scan. This issue is applicable for the fourth CAPSENSE&trade;
*         generation devices.
*     </td>
*     <td>
*         There are several possible workarounds:
*         1. Increase the Scan resolution.
*         2. Increase the Sense clock frequency. For the best results, perform
*            scanning with as high as possible Sense clock frequency.
*         3. If shield is required for a design, enable the shield tank (Csh)
*            capacitor.
*         4. Increase the sensor capacitance by changing its layout or introduce
*            extra capacitor between the sensor pin and ground.
*         5. Increase number of Fine initialization cycles. Open the cycfg_capsense.c
*            file and modify the .csdFineInitTime field of the cy_capsense_commonConfig
*            structure.
*         6. Increase the CSD init switch resistance. Open the cycfg_capsense.c file
*            and update the .csdInitSwRes field of the cy_capsense_commonConfig structure
*            with the CY_CAPSENSE_INIT_SW_RES_HIGH value.
*     </td>
*   </tr>
*   <tr>
*     <td>17158</td>
*     <td>
*         ISX scan followed by BIST measurement or with other sensing methods
*         CSD & CSX introduces drift in raw count. This impacts the
*         performance and accuracy of the ISX scanning.
*     </td>
*     <td>
*         There are several possible workarounds:
*         1. Use ISX in standalone mode (without other sensing methods)
*         2. Use ISX without built-in self-test (BIST)
*         3. Configure Inactive sensor connection parameter to High-Z
*         4. Introduce a dummy scan before ISX.
*     </td>
*   </tr>
* </table>
*
* \note Refer to the "Errata" section of the device datasheet to get information
* about the known problems related to the CAPSENSE&trade; HW IP block.
*
********************************************************************************
* \section section_capsense_changelog Changelog
********************************************************************************
*
* <table class="doxtable">
*   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
*   <tr>
*     <td rowspan="9">7.0.0</td>
*     <td colspan="2"><i><b>
*       This version is compatible to the v6.10.0,
*       however due to switching to MTB 3.5, new Device-DB, PDL, etc.
*       a migration of projects created with past version might be
*       not possible. In this case a project should be newly created.
*       Also, if you use the CAPSENSE&trade; middleware v7.0.0, you must use
*       the CAPSENSE&trade; Configurator v8.0.0 tool or later. This MW version
*       is not compatible with the previous Configurator tool versions.
*     </b></i>
*   </tr>
*   <tr>
*     <td> Added support of PSOC&trade; 4100T Plus device family.
*     </td>
*     <td>
*         New device support.
*     </td>
*   </tr>
*   <tr>
*     <td> Added scaling to CDAC codes in the manual mode for the CSD sensing
*       method by using trim registers.
*     </td>
*     <td>
*         New feature support, improved manufacturing tolerance.
*     </td>
*   </tr>
*   <tr>
*     <td> Added scaling to measured sensor, electrode and shield capacitances
*       in built-in self-test library.
*     </td>
*     <td>
*         New feature support, improved manufacturing tolerance.
*     </td>
*   </tr>
*   <tr>
*     <td> Redefined CDAC LSB from 8.86fF to 8.87fF.
*     </td>
*     <td>
*         Improved accuracy.
*     </td>
*   </tr>
*   <tr>
*     <td> Fixed Shield electrode capacitance measurement inside built-in self-test
*       library. Wrong measurement happens when the first sensor in the configuration
*       does not belong to CSD sensing method.
*     </td>
*     <td>
*         Defect fixing.
*     </td>
*   </tr>
*   <tr>
*     <td> Removed intermediate verification of CDAC auto-calibration status.
*     </td>
*     <td>
*         Supporting customer specific use cases.
*     </td>
*   </tr>
*   <tr>
*     <td> Fixed CDAC auto-calibration corner case when auto and manual modes are combined for the fifth-generation low power CAPSENSE&trade;.
*     </td>
*     <td>
*         Defect fixing.
*     </td>
*   </tr>
*   <tr>
*     <td> Fixed HW configuration for inactive sensor connection set to active shield in built-in self-test library for sensor capacitance measurement for fifth-generation CAPSENSE&trade;.
*     </td>
*     <td>
*         Defect fixing.
*     </td>
*   </tr>
*   <tr>
*     <td rowspan="9">6.10.0</td>
*     <td colspan="2"><i><b>
*       This version is compatible to the v5.0, 
*       however due to switching to MTB 3.4, new Device-DB, PDL,
*       compiler versions, etc. a migration of projects created with 
*       past version might be not possible. In this case a project should be newly created.
*       Also, if you use the CAPSENSE&trade; middleware v6.10.0, you must use
*       the CAPSENSE&trade; Configurator v7.10.0 tool or later. This MW version is not
*       compatible with the previous Configurator tool versions.
*     </b></i>
*   </tr>
*   <tr>
*     <td> Added Liquid level sensing method for fifth-generation low power device.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   </tr>
*   <tr>
*     <td> Added Inductive sensing method for fifth-generation low power device.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td> Added auto-tuning for Inductive sensing widgets. The ISX auto-tuning
*       feature is at the Beta readiness level. It is not fully validated and
*       might not work as expected. Use it to start working on the ISX tuning.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td> Added Common mode filter.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td> Added Multi-pin Selection option for fifth-generation low power device.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td> 
*       Increased supported sensor capacitance range to 275pF and total 
*       Shield electrode capacitance range to 1160pF (CAPSENSE&trade; fifth-generation low power device).
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Fixed some non-functional MISRA violations.
*     </td>
*     <td>
*         Defect fixing.
*     </td>
*   </tr>
*   <tr>
*     <td> Fixed documentation.
*     </td>
*     <td>
*         User experience improvement.
*     </td>
*   </tr>
*   <tr>
*     <td>6.0.0</td>
*     <td colspan="2"><i><b>
*       This release is for internal purpose.
*     </b></i>
*     </td>
*   </tr>
*   <tr>
*     <td rowspan="13">5.0</td>
*     <td colspan="2"><i><b>
*       This version is not backward compatible with the
*       previous version due to re-designed CDAC auto-calibration.
*       We tried to keep the established API, but your design may need to be
*       updated to operate with CAPSENSE&trade; middleware v5.0.
*       Also, if you use the CAPSENSE&trade; middleware v5.0, you must use
*       the CAPSENSE&trade; Configurator v6.20 tool or later. This MW version is not
*       compatible with the previous Configurator tool versions.
*     </b></i>
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Re-designed CDAC auto-calibration algorithm for the fifth-generation
*         LP devices
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added auto-selection mode of CIC2 decimation rate for the
*         fifth-generation low power devices
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added auto-selection mode of CDAC Dither scale value for the
*         fifth-generation and fifth-generation low power devices
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added the HW IIR Filter initialization function
*         Cy_CapSense_ScanInitializeHwIirAllSlots() and
*         Cy_CapSense_ScanInitializeHwIirSlots() for fifth-generation low power devices
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Corrected the low power widget scanning flow by excluding waking up at a
*         signal detection - only the end of the frame interrupt is kept.
*         Condition: multiple low power sensors. This will avoid multiple interrupt
*         generation and in some cases prevent the device from hanging.
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Corrected description of the cy_stc_capsense_position_t structure
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Removed Epilogue cycles from the maximum Raw Counts definition
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Hidden a Multi-phase Self feature for fifth-generation low power devices
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Disabled the External Frame Start in the Cy_CapSense_InitializeMaxRaw()
*         function for the fifth-generation devices
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Corrected description of the BIST measurement function
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Corrected description of the cy_stc_capsense_touch_t structure
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Updated description of the software watchdog time calculation for
*         the fifth-generation devices
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td rowspan="17" >4.0</td>
*     <td colspan="2"><i><b>
*       This version is not backward compatible with the
*       previous version due to fifth-generation low power devices support
*       implementation and defect fixes.
*       We tried to keep the established API, but your design may need to be
*       updated to operate with CAPSENSE&trade; middleware v4.0.
*     </b></i>
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added CAPSENSE&trade; fifth-generation low power device support
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added the Inductive sensing method support (ISX)
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added the feature of enabling/disabling widgets and built-in
*         self-test working/non-working widgets
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added Multi-phase Self support for fifth-generation low power devices
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Updated the CIC2 functionality for the fifth-generation devices
*         to be consistent with the fifth-generation low power devices.
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added saturated scan during CAPSENSE&trade; initialization to measure
*         maximum rawcounts precisely instead of using equations.
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Changed the meaning of bit[2] of the widget status register in the cy_stc_capsense_widget_context_t:
*         logical 1 means the widget is enabled. Replaced CY_CAPSENSE_WD_DISABLE_MASK with
*         CY_CAPSENSE_WD_ENABLE_MASK for consistency
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Swapped the position of bit[0] and bit[1] of the sensor status register in the cy_stc_capsense_sensor_context_t:
*         bit[0] reports the Proximity sensor status, bit[1] - the regular sensor status.
*         Updated values of CY_CAPSENSE_SNS_PROX_STATUS_MASK and CY_CAPSENSE_SNS_TOUCH_STATUS_MASK macros.
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Renamed cy_stc_active_scan_sns_t to cy_stc_capsense_active_scan_sns_t.
*     </td>
*     <td>
*         Following naming convention. Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Renamed cy_stc_msc_channel_config_t to cy_stc_capsense_channel_config_t, updated field names
*     </td>
*     <td>
*         Following naming convention. Updated for consistency between fifth-generation and fifth-generation low power devices
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Renamed Cy_CapSense_ProcessWidgetMptxDeconvolution() to Cy_CapSense_ProcessWidgetMpDeconvolution()
*     </td>
*     <td>
*         Updated function name to better reflect its purpose
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Renamed Cy_CapSense_SetCalibrationTargets() to Cy_CapSense_SetCalibrationTarget(),
*         updated function interface and implementation to cover all sensing methods.
*     </td>
*     <td>
*         Feature enhancement.
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Fixed incorrect shield pins configuration for Cy_CapSense_MeasureCapacitanceSensor()
*         function for fourth-generation devices
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Updater measurement error handling for Cy_CapSense_MeasureCapacitanceShieldElectrode(),
*         Cy_CapSense_MeasureCapacitanceSlotSensors() and Cy_CapSense_MeasureCapacitanceSensorElectrode()
*         functions for fifth-generation devices
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Optimized cy_stc_capsense_common_config_t structure (renamed fields, removed unused fields)
*     </td>
*     <td>
*         User experience improvement
*     </td>
*   </tr>
*   <tr>
*     <td>
*         Added Long Press gesture
*     </td>
*     <td>
*         Feature enhancement
*     </td>
*   </tr>
*   <tr>
*     <td>3.0.1</td>
*     <td>
*       Fixed calls of Cy_DMAC_Descriptor_SetDstAddress() and Cy_DMAC_Descriptor_SetSrcAddress()
*       accordingly to changes in DMAC v1.20 mtb-pdl-cat2 driver.
*     </td>
*     <td>
*         Defect fixing
*     </td>
*   </tr>
*   <tr>
*     <td rowspan="12">3.0</td>
*     <td colspan="2"><i><b>
*       This version is not backward compatible with the
*       previous version due to implemented memory consumption optimization.
*       We tried to keep the established API, but your design may need to be
*       updated to operate with CAPSENSE&trade; middleware v3.0.
*
*       Also, if you use the CAPSENSE&trade; middleware v3.0 version then
*       you must use the CAPSENSE&trade; Configurator v4.0 tool or later.
*       This MW version is not compatible with the previous Configurator tool
*       versions.
*     </b></i></td>
*   </tr>
*   <tr>
*     <td>
*     Added fifth-generation CAPSENSE&trade; device support which includes
*     but not limited:
*     * Scanning mode: DMA and Interrupt Driven
*     * Sensor connection method: Analog mux bus and Control mux bus
*     * Re-designed multi-frequency scan feature
*     * Multi-phase TX feature
*     * Improved signal-to-noise ratio
*     * Improved refresh rate
*     </td>
*     <td>New CAPSENSE&trade; MSC platform covering</td>
*   </tr>
*   <tr>
*     <td colspan="2"><i><b>The following rows show changes related to fourth-generation CAPSENSE&trade;</b></i></td>
*   </tr>
*   <tr>
*     <td>Removed usage of deprecated return error and status codes,
*         cy_status enum was replaced with the cy_capsense_status_t variable</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Removed usage of deprecated types, such as uint32 and uint16</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Changed the default IDAC gain index for the CSX auto-calibration to the configured one</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>The following functions were made obsolete:
*         * Cy_CapSense_CSDConnectSns()
*         * Cy_CapSense_CSDDisconnectSns()
*         * Cy_CapSense_CSXConnectRx()
*         * Cy_CapSense_CSXConnectTx()
*         * Cy_CapSense_CSXDisconnectRx()
*         * Cy_CapSense_CSXDisconnectTx()
*         * Cy_CapSense_CalibrateAllCsdWidgets()
*         * Cy_CapSense_CalibrateAllCsxWidgets()
*     </td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>For fourth-generation CAPSENSE&trade; devices callbacks were moved from ptrCommonContext
*         to ptrInternalContext structure</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>Updated the description of the Cy_CapSense_RunSelfTest() function </td>
*     <td>Documentation defect fixing</td>
*   </tr>
*   <tr>
*     <td>Specified measurement units in the following function descriptions:
*         * Cy_CapSense_MeasureCapacitanceSensor()
*         * Cy_CapSense_MeasureCapacitanceShield()
*         * Cy_CapSense_MeasureCapacitanceCap()
*     </td>
*     <td>Documentation defect fixing</td>
*   <tr>
*     <td>Added call of the Cy_CapSense_Wakeup() function to the registered
*     callback in the Cy_CapSense_DeepSleepCallback() function</td>
*     <td>Improved robustness of a sensing module</td>
*   </tr>
*   <tr>
*     <td>Added a possibility to enable/disable independent features of
*     BIST module</td>
*     <td>Expanded flexibility</td>
*   </tr>
*   <tr>
*     <td rowspan="9">2.10</td>
*     <td>Added Built-in Self-test (BIST) library</td>
*     <td>Support Class B (IEC-60730), safety integrity-level compliant design</td>
*   </tr>
*   <tr>
*     <td>Improved the Csh and Cmod coarse initialization functionality.</td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>Improved the shield performance when Csh is enabled</td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>Fixed Cy_CapSense_ScanExt() operation</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Fixed the bug in the Cy_CapSense_SetPinState() function</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Optimized software watch-dog values used in monitoring CAPSENSE&trade;
*         scanning duration</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>Improved IDAC auto-calibration</td>
*     <td>Operation accuracy increasing</td>
*   </tr>
*   <tr>
*     <td>Added the following functions:
*         * Cy_CapSense_GetParam()
*         * Cy_CapSense_SetParam()
*         * Cy_CapSense_GetCRC()
*     </td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>Changed the type of context argument to const in the following
*         functions:
*         * Cy_CapSense_CSDConnectSns()
*         * Cy_CapSense_CSXConnectRx()
*         * Cy_CapSense_CSXConnectTx()
*         * Cy_CapSense_CSXDisconnectRx()
*         * Cy_CapSense_CSXDisconnectTx()
*         * Cy_CapSense_SetPinState()
*     </td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td rowspan="6">2.0</td>
*     <td>Added memory usage section to the CAPSENSE&trade; API Ref Guide</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>Updated documentation</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>Added the errata section to the CAPSENSE&trade; API Ref Guide</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>CAPSENSE&trade; MW sources are enclosed with the conditional compilation to
*         ensure a successful compilation for non-CAPSENSE&trade;-capable devices</td>
*     <td>Fixing a compilation error for non CAPSENSE&trade;-capable devices</td>
*   </tr>
*   <tr>
*     <td>Optimized flash memory consumption based on user's configuration</td>
*     <td>Flash foot-print optimization</td>
*   </tr>
*   <tr>
*     <td>Renamed function Cy_CapSense_CheckCommandIntegrity() to
*         Cy_CapSense_CheckTunerCmdIntegrity()</td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td rowspan="2">1.20</td>
*     <td>Added Arm Compiler 6 support</td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>Changed the hierarchy of the binary files folders</td>
*     <td>MBED OS compatibility</td>
*   </tr>
*   <tr>
*     <td rowspan="6">1.1</td>
*     <td>
*         The following functions made obsolete:
*         * Cy_CapSense_CSDSetupWidget()
*         * Cy_CapSense_CSDSetupWidgetExt()
*         * Cy_CapSense_CSDScan()
*         * Cy_CapSense_CSDScanExt()
*         * Cy_CapSense_CSDCalibrateWidget()
*         * Cy_CapSense_CSXSetupWidget()
*         * Cy_CapSense_CSXSetupWidgetExt()
*         * Cy_CapSense_CSXScan()
*         * Cy_CapSense_CSXScanExt()
*         * Cy_CapSense_CSXCalibrateWidget()
*
*         Two simple functions introduced to replace the listed above functions:
*         * Cy_CapSense_SetupWidgetExt()
*         * Cy_CapSense_ScanExt()
*     </td>
*     <td>User experience improvement</td>
*   </tr>
*   <tr>
*     <td>Fixed the shield operation when Csh is disabled</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Fixed the implementation of the position filtering for the Radial Slider widget</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Added restoring hardware to its default state in the Cy_CapSense_DeInit() implementation</td>
*     <td>Defect fixing</td>
*   </tr>
*   <tr>
*     <td>Added the capability to enable the shield electrode without dedicated electrodes</td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>Added support of a protocol-agnostic tuner interface (UART, SPI, etc.)</td>
*     <td>Feature enhancement</td>
*   </tr>
*   <tr>
*     <td>1.0</td>
*     <td>
*         The initial version
*     </td>
*     <td></td>
*   </tr>
* </table>
*
********************************************************************************
* \section section_capsense_more_information More Information
********************************************************************************
*
* Important information about the CAPSENSE&trade;-technology overview, appropriate
* CYPRESS&trade; device from Infineon for the design, CAPSENSE&trade; system and sensor
* design guidelines, different interfaces and tuning guidelines necessary
* for a successful design of a CAPSENSE&trade; system is available in the Getting
* Started with CAPSENSE&trade; document and the product-specific CAPSENSE&trade; design guide.
* Infineon Technologies highly recommends starting with these documents. They can be
* found on the Infineon Technologies web site at www.infineon.com.
*
* For more information, refer to the following documents:
*
* * CAPSENSE&trade; Overview:
*
*   * <a href="https:/\/www.infineon.com/ModusToolboxCapSenseConfig"><b>ModusToolbox&trade;
*   CAPSENSE&trade; Configurator Tool Guide</b></a>
*
*   * <a href="https:/\/www.infineon.com/ModusToolboxCapSenseTuner"><b>ModusToolbox&trade;
*   CAPSENSE&trade; Tuner Tool Guide</b></a>
*
*   * <a href="https:/\/www.infineon.com/dgdl/Infineon-AN85951_PSOC_4_and_PSOC_6_MCU_CapSense_Design_Guide-ApplicationNotes-v30_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0723535d4661">
*   <b>CAPSENSE&trade; Design Guide</b></a>
*
* * ModusToolbox&trade; Overview:
*
*   * <a href="https:/\/www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software">
*   <b>ModusToolbox&trade; Software Environment, Quick Start Guide, Documentation,
*   and Videos</b></a>
*
*   * <a href="https:/\/www.infineon.com/ModusToolboxDeviceConfig"><b>ModusToolbox&trade;
*   Device Configurator Tool Guide</b></a>
*
* * Infineon Technologies Kits and Code Examples:
*
*   * <a href="https:/\/github.com/Infineon/mbed-os-example-capsense">
*   <b>CAPSENSE&trade; buttons and slider for PSOC&trade; 6 MCU with Mbed OS</b></a>
*
*   * <a href="https:/\/github.com/Infineon/mtb-example-psoc6-capsense-buttons-slider-freertos">
*   <b>CAPSENSE&trade; Middleware Code Example for FreeRTOS</b></a>
*
*   * <a href="https:/\/www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-145-40xx">
*   <b>CY8CKIT-145-40XX PSOC&trade; 4000S CAPSENSE&trade; Prototyping Kit</b></a>
*
*   * <a href="https:/\/www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-149">
*   <b>CY8CKIT-149 PSOC&trade; 4100S Plus Prototyping Kit</b></a>
*
* * General Information:
*
*   * <a href="https:/\/infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html">
*   <b>CAT1 PDL API Reference</b></a>
*
*   * <a href="https:/\/infineon.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html">
*   <b>CAT2 PDL API Reference</b></a>
*
*   * <a href="http:/\/www.infineon.com/cms/en/search.html#!term=AN210781&view=downloads"><b>AN210781 Getting Started with
*   PSOC&trade; 6 MCU with Bluetooth Low Energy (BLE) Connectivity</b></a>
*
*   * <a href="http:/\/www.infineon.com/cms/en/search.html#!term=AN215671&view=downloads"><b>AN215671 PSOC&trade; 6 MCU
*   firmware design for BLE applications</b></a>
*
*   * <a href="https:/\/www.infineon.com/cms/en/search.html#!term=psoc%206%20TRM&view=downloads">
*   <b>PSOC&trade; 6 Technical Reference Manual</b></a>
*
*   * <a href="https:/\/www.infineon.com/cms/en/search.html#!term=psoc%204000S%20TRM&view=downloads">
*   <b>PSOC&trade; 4000S Family: PSOC&trade; 4 Architecture Technical Reference Manual (TRM)</b></a>
*
*   * <a href="https:/\/www.infineon.com/cms/en/search.html#!term=psoc%204100S%20TRM&view=downloads">
*   <b>PSOC&trade; 4100S and PSOC&trade; 4100S Plus: PSOC&trade; 4 Architecture Technical Reference Manual (TRM)</b></a>
*
*   * <a href="http:/\/www.infineon.com/cms/en/search.html#!term=PSOC%2063%20datasheet&view=downloads">
*   <b>PSOC&trade; 63 with BLE Datasheet Programmable System-on-Chip datasheet</b></a>
*
*   * <a href="https:/\/github.com/Infineon"><b> Infineon Technologies GitHub</b></a>
*
*   * <a href="http:/\/www.infineon.com"><b>Infineon Technologies</b></a>
*
* \note
* The links to another software component's documentation (middleware and PDL)
* point to GitHub to the latest available version of the software.
* To get documentation of the specified version, download from GitHub and unzip
* the component archive. The documentation is available in the <i>docs</i> folder.
*
* \defgroup group_capsense_high_level               High-level Functions
*
* \defgroup group_capsense_low_level                Low-level Functions
*
* \defgroup group_capsense_data_structure           Data Structure
*
* \defgroup group_capsense_structures               Structures
* \ingroup group_capsense_data_structure
* \brief The CAPSENSE&trade; structures.
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
* \defgroup group_capsense_macros_mw_state          Middleware State Macros
*   \ingroup group_capsense_macros
*   \brief Middleware state macros
* \defgroup group_capsense_macros_status            Status Macros
*   \ingroup group_capsense_macros
*   \brief Status macros
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
* \defgroup group_capsense_macros_bist              Built-in Self-test Macros
*   \ingroup group_capsense_macros
*   \brief Built-in Self-test macros
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
* High-level functions represent the highest abstraction layer of
* the CAPSENSE&trade; middleware.
*
* These functions perform tasks such as scanning, data processing, data
* reporting and tuning interfaces. When performing a task, different
* initialization is required based on a sensing method or type of
* widgets is automatically handled by these functions. Therefore, these
* functions are sensing methods, features, and widget type agnostics.
* The only exception is the functions for low power widget scanning,
* they are available only for the fifth-generation low power CAPSENSE&trade;.
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
* The Low-level functions represent the lower layer of abstraction in
* support of \ref group_capsense_high_level.
*
* These functions also enable implementation of special case designs
* requiring performance optimization and non-typical functionalities.
*
* All functions are general to all sensing methods. Some of the
* functions detect the sensing method used by the widget and execute
* tasks as appropriate.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros
* \{
*
* Specifies constants used in the CAPSENSE&trade; middleware.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_enums
* \{
*
* Documents the CAPSENSE&trade; middleware related enumerated types.
*
* \} */

/******************************************************************************/
/** \cond SECTION_CAPSENSE_INTERNAL */
/** \addtogroup group_capsense_internal
* \{
*
* The section documents the CAPSENSE&trade; middleware related internal function.
*
* These function should not be used in the application program.
*
* \} \endcond */

/******************************************************************************/
/** \addtogroup group_capsense_data_structure
* \{
*
* The CAPSENSE&trade; Data Structure organizes configuration parameters, input, and
* output data shared among different FW modules within the CAPSENSE&trade;.
*
* The key responsibilities of the Data Structure are as follows:
* * The Data Structure is the only data container in the CAPSENSE&trade; middleware.
* * It serves as storage for the configuration and the output data.
* * All CAPSENSE&trade; modules use the data structure for the communication
*   and data exchange.
*
* The CAPSENSE&trade; Data Structure is a composite of several smaller structures
* (for global / common data, widget data, sensor data, and pin data).
* Furthermore, the data is split between RAM and Flash to achieve a
* reasonable balance between resources consumption and configuration / tuning
* flexibility at runtime and compile time. A graphical representation of
* the CAPSENSE&trade; Data Structure is shown below.
*
* Note that figure below shows a sample representation and documents the
* high-level design of the data structure, it does not include all the
* parameters and elements in each object.
*
* \image html capsense_ds.png "CAPSENSE&trade; Data Structure" width=800px
* \image latex capsense_ds.png
*
* CAPSENSE&trade; Data Structure does not perform error checking on the data
* written to CAPSENSE&trade; Data Structure. It is the responsibility of application
* program to ensure register map rule are not violated while
* modifying the value of data field in CAPSENSE&trade; Data Structure.
*
* Note: If the middleware configuration from the application program is changed
* by writing registers to the data structure, call the Cy_CapSense_Enable() function
* to repeat the initialization process.
*
* \} */

/******************************************************************************/
/** \addtogroup group_capsense_callbacks
* \{
*
* Callbacks allow the user to execute Custom code called from the CAPSENSE&trade;
* middleware when an event occurs. CAPSENSE&trade; supports several callbacks.
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
*         parameters depending on whether a sensor is going to be scanned.
*       \par
*       * For fifth-generation low power CAPSENSE&trade; this callback is called
*         by CapSense MW before scan triggering. If a timeout to postpone a scan
*         is configured (using Cy_CapSense_ConfigureMsclpTimer() function) then callback
*         will be called before this timeout. The StartSample callback is called
*         only once before entire frame scan in Cy_CapSense_ScanSlots()
*         and Cy_CapSense_ScanLpSlots() functions.
*       \par
*       * For the fifth-generation CAPSENSE&trade; in CS-DMA mode this callback is called
*         only once in Cy_CapSense_ScanSlots() function with NULL passed as a parameter.
*         In INT driven mode it is called in Cy_CapSense_ScanSlots() and
*         in Cy_CapSense_ScanISR() function before HW starting the scan. </td>
*   </tr>
*   <tr>
*     <td>2</td>
*     <td>End Of Scan</td>
*     <td>ptrEOSCallback</td>
*     <td>\ref cy_capsense_callback_t</td>
*     <td>This is called after sensor scan completion and there is no other
*         sensor in the queue to be scanned.
*       \par
*       * For the fifth-generation low power CAPSENSE&trade; this callback is called
*         with NULL passed as a parameter.</td>
*       \par
*       * For the fifth-generation CAPSENSE&trade; in CS-DMA mode this callback is called
*         with NULL passed as a parameter.</td>
*   </tr>
*   <tr>
*     <td>3</td>
*     <td>Tuner Send Callback</td>
*     <td>ptrTunerSendCallback</td>
*     <td>\ref cy_capsense_tuner_send_callback_t</td>
*     <td>This is called by the Cy_CapSense_RunTuner() function to establish
*         synchronous communication with the Tuner tool.
*       \par
*       * For the fifth-generation low power CAPSENSE&trade; the callback is called only once for each new scan.
*       \par
*       * For the previous CAPSENSE&trade; generations the callback is called once per scan cycle or periodically if device is in suspended mode.</td>
*   </tr>
*   <tr>
*     <td>4</td>
*     <td>Tuner Receive Callback</td>
*     <td>ptrTunerReceiveCallback</td>
*     <td>\ref cy_capsense_tuner_receive_callback_t</td>
*     <td>This is called by the Cy_CapSense_RunTuner() function to establish
*         synchronous communication with the Tuner tool.
*         </td>
*   </tr>
*   <tr>
*     <td>5</td>
*     <td>CAPSENSE&trade; Data Structure Initialization Callback</td>
*     <td>ptrEODsInitCallback</td>
*     <td>\ref cy_capsense_ds_init_callback_t</td>
*     <td>This is called by the Cy_CapSense_Enable() function after CAPSENSE&trade;
*         Data Structure initialization complete and before launching
*         the first initialization scan. Using this callback is not
*         recommended. It is used only to implement only user's
*         specific use cases (while changing the CAPSENSE&trade;
*         default configuration).
*       \par
*       * The callback is available for Fifth Generation CAPSENSE&trade; and
*         Fifth Generation low power CAPSENSE&trade; devices.</td>
*   </tr>
* </table>
*
* \note
* Callbacks 1 and 2 are called by the Cy_CapSense_InterruptHandler()
* function and lengthen this function execution. Usually,
* Cy_CapSense_InterruptHandler() is called inside the ISR.
* Then these callbacks also lengthen the ISR execution.
*
* All callbacks can be registered by direct assignment of the function
* pointers to the corresponding CAPSENSE&trade; Data Structure field after call of
* the Cy_CapSense_Init() function as follows:
*  *
* <tt>context-\>ptrInternalContext-\>\<Associated Register\> = \&CallbackFunction;</tt>
*
* Callbacks 1 and 2 can be registered / unregistered using
* the Cy_CapSense_RegisterCallback() and Cy_CapSense_UnRegisterCallback()
* functions.
*
* \} */


#if !defined(CY_CAPSENSE_H)
#define CY_CAPSENSE_H

#include "cy_device_headers.h"
#include "cy_capsense_common.h"
#include "cy_capsense_centroid.h"
#include "cy_capsense_control.h"
#include "cy_capsense_filter.h"
#include "cy_capsense_lib.h"
#include "cy_capsense_gesture_lib.h"
#include "cy_capsense_processing.h"
#include "cy_capsense_structure.h"
#include "cy_capsense_tuner.h"
#include "cy_capsense_sensing.h"
#include "cy_capsense_selftest.h"
#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    #include "cy_capsense_csd_v2.h"
    #include "cy_capsense_csx_v2.h"
    #include "cy_capsense_sensing_v2.h"
    #include "cy_capsense_selftest_v2.h"
#elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
    #include "cy_capsense_generator_v3.h"
    #include "cy_capsense_sensing_v3.h"
    #include "cy_capsense_selftest_v3.h"
    #include "cy_capsense_sm_base_full_wave_v3.h"
#elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #include "cy_capsense_generator_lp.h"
    #include "cy_capsense_sensing_lp.h"
    #include "cy_capsense_selftest_lp.h"
    #include "cy_capsense_sm_base_full_wave_lp.h"
#else
    /* Supported platform not found */
#endif

#endif /* CY_CAPSENSE_H */


/* [] END OF FILE */
