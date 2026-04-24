# CAPSENSE™ Middleware Library

### Overview
CAPSENSE™ is a capacitive sensing solution from Infineon Technologies AG. Capacitive sensing can be used in a variety of applications and products where 
conventional mechanical buttons can be replaced with sleek human interfaces to transform the way users interact with electronic systems. 
These include home appliances, automotive, IoT, and industrial applications. CAPSENSE™ supports multiple interfaces (widgets) using the Self-Capacitance (CSD), 
Mutual-Capacitance (CSX), and Inductive sensing (ISX) sensing methods with robust performance.

CAPSENSE™ has become a popular technology to replace conventional mechanical- and optical-based user interfaces. There are fewer parts involved, 
which saves cost and increases the reliability with no wear-and-tear. The main advantages of CAPSENSE™ compared with other solutions are: 
robust performance in harsh environmental conditions and rejection of a wide range of external noise sources.

Use CAPSENSE™ for:
* Touch and gesture detection for various interfaces
* Proximity detection for innovative user experiences and low-power optimization
* Touch-free operations in hazardous materials

### Features
* Offers best-in-class signal-to-noise ratio (SNR)
* Supports the Self-Capacitance (CSD), Mutual-Capacitance (CSX), and Inductive (ISX) sensing methods
* Features smart sensing algorithm auto-tuning technology for CSD sensing to avoid complex manual tuning process
* Supports various Widgets, such as Buttons, Matrix Buttons, Sliders, Touchpads, and Proximity Sensors
* Provides ultra-low power consumption and liquid-tolerant capacitive sensing technology
* Contains integrated graphical CAPSENSE™ Tuner for real-time tuning, testing, and debugging
* Provides superior immunity against external noise and low-radiated emission
* Offers best-in-class liquid tolerance
* Supports one-finger and two-finger gestures

### Quick start
The [ModusToolbox™ CAPSENSE™ Configurator User Guide](https://www.infineon.com/ModusToolboxCapSenseConfig) instructs step-by-step  
how to configure and launch the CAPSENSE™ Configurator in [ModusToolbox™](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software). 
The CAPSENSE™ Configurator can be launched in the ModusToolbox™ IDE from the CSD, MSC or MSCLP personality and as a stand-alone tool. Refer to [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/).

Use code examples to quickly start the CAPSENSE™ Configurator. Infineon Technologies AG continuously extends their portfolio of code examples at the <a href="http:/\/www.infineon.com"><b>Infineon Technologies</b></a> and at the <a href="https:/\/github.com/Infineon"><b> Infineon Technologies GitHub</b></a>.

### More information
For more information, refer to:
* CAPSENSE™ overview:
  * [CAPSENSE™ Middleware RELEASE.md](./RELEASE.md)
  * [CAPSENSE™ Middleware API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html)
  * [ModusToolbox™ CAPSENSE™ Configurator Tool Guide](https://www.infineon.com/ModusToolboxCapSenseConfig)
  * [ModusToolbox™ CAPSENSE™ Tuner Tool Guide](https://www.infineon.com/ModusToolboxCapSenseTuner)
  * [CAPSENSE™ Design Guide](https://www.infineon.com/row/public/documents/30/42/infineon-an85951-psoc-4-psoc-6-capsense-design-guide-applicationnotes-en.pdf)
  * [CSDADC Middleware API Reference Guide](https://infineon.github.io/csdadc/csdadc_api_reference_manual/html/index.html)
  * [CSDIDAC Middleware API Reference Guide](https://infineon.github.io/csdidac/csdidac_api_reference_manual/html/index.html)

* ModusToolbox™ Overview:
  * [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
  * [ModusToolbox™ Device Configurator Tool Guide](https://www.infineon.com/ModusToolboxDeviceConfig)

* Infineon Technologies AG Kits and Code Examples:
  * [CAPSENSE™ Middleware Code Example for MBED OS](https://github.com/Infineon/mbed-os-example-capsense)
  * [CAPSENSE™ Middleware Code Example for FreeRTOS](https://github.com/Infineon/mtb-example-psoc6-emwin-eink-freertos)
  * [CY8CKIT-145-40XX PSOC™ 4000S CAPSENSE™ Prototyping Kit](https://www.infineon.com/evaluation-board/CY8CKIT-145-40XX)
  * [CY8CKIT-149 PSOC™ 4100S Plus Prototyping Kit](https://www.infineon.com/evaluation-board/CY8CKIT-149)
  * [CY8CKIT-041-40XX PSOC™ 4 S-Series Pioneer Kit](https://www.infineon.com/dgdl/Infineon-CY8CKIT-041-40XX_PSoC_4_S-Series_Pioneer_Kit_Quick_Start_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0efc44781263)
  * [CY8CKIT-041-41XX PSOC™ 4100S CAPSENSE™ Pioneer Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-041-41xx/)
  * [CY8CKIT-040T PSOC™ 4000T CAPSENSE™ Evaluation Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-040t/)

* General Information:
  * [AN210781 Getting Started with PSOC™ 6 MCU with Bluetooth Low Energy (BLE) Connectivity](https://www.infineon.com/row/public/documents/30/42/infineon-an210781-getting-started-with-psoc-6-mcu-with-bluetooth-low-energy-ble-connectivity-applicationnotes-en.pdf)
  * [AN215671 PSOC&trade; 6 MCU firmware design for BLE applications](https://www.infineon.com/row/public/documents/30/42/infineon-an215671-psoc-6-mcu-firmware-design-for-ble-applications-applicationnotes-en.pdf)
  * [PSOC™ 6 Technical Reference Manual](https://www.infineon.com/dgdl/Infineon-PSoC_6_MCU_PSoC_63_with_BLE_Architecture_Technical_Reference_Manual-AdditionalTechnicalInformation-v11_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f946fea01ca)
  * [PSOC™ 63 with BLE Datasheet Programmable System-on-Chip datasheet](https://www.infineon.com/dgdl/Infineon-PSoC_6_MCU_PSoC_63_with_BLE_Datasheet_Programmable_System-on-Chip_(PSoC)-DataSheet-v16_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0ee4efe46c37)
  * [CAT1 PDL API Reference](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html)
  * [CAT2 PDL API Reference](https://infineon.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html)
  * [PSOC™ 4000S Family: PSOC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/assets/row/public/documents/30/57/infineon-psoc4000s-architecture-trm-additionaltechnicalinformation-en.pdf?fileId=8ac78c8c7d0d8da4017d0f915c737eb7)
  * [PSOC™ 4100S and PSOC™ 4100S Plus: PSOC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/row/public/documents/30/57/infineon-psoc-4100s-and-psoc-4100s-plus-architecture-trm-additionaltechnicalinformation-en.pdf)
  * [Infineon Technologies GitHub](https://github.com/Infineon)
  * [Infineon Technologies](https://www.infineon.com)

---
Infineon Technologies AG, 2019-2026.
