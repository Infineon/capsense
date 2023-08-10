# CAPSENSE™ Middleware Library

### Overview
CAPSENSE™ is CYPRESS™ capacitive sensing solution from Infineon Technologies AG. Capacitive sensing can be used in a variety of applications and products where 
conventional mechanical buttons can be replaced with sleek human interfaces to transform the way users interact with electronic systems. 
These include home appliances, automotive, IoT, and industrial applications. CAPSENSE™ supports multiple interfaces (widgets) using the Self-Capacitance (CSD), 
Mutual-Capacitance (CSX), and Inductive sensing (ISX) sensing methods with robust performance.

CAPSENSE™ has become a popular technology to replace conventional mechanical- and optical-based user interfaces. There are fewer parts involved, 
which saves cost and increases the reliability with no wear-and-tear. The main advantages of CAPSENSE™ compared with other solutions are: 
robust performance in harsh environmental conditions and rejection of a wide range of external noise sources.

Use CAPSENSE™ for:
* Touch and gesture detection for various interfaces
* Proximity detection for innovative user experiences and low-power optimization
* Contactless liquid-level sensing in a variety of applications
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
The [ModusToolbox™ CAPSENSE™ Configurator User Guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_CAPSENSE_Configurator_5.0_User_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c8386267f0183a960b36a598c) instructs step-by-step  
how to configure and launch the CAPSENSE™ Configurator in [ModusToolbox™](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software). 
The CAPSENSE™ Configurator can be launched in the ModusToolbox™ IDE from the CSD, MSC or MSCLP personality and as a stand-alone tool. Refer to [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/).

Use code examples to quickly start the CAPSENSE™ Configurator. Infineon Technologies AG continuously extends their portfolio of code examples at the <a href="http:/\/www.infineon.com"><b>Infineon Technologies</b></a> and at the <a href="https:/\/github.com/Infineon"><b> Infineon Technologies GitHub</b></a>.

### More information
For more information, refer to:
* CAPSENSE™ overview:
  * [CAPSENSE™ Middleware RELEASE.md](./RELEASE.md)
  * [CAPSENSE™ Middleware API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html)
  * [ModusToolbox™ CAPSENSE™ Configurator Tool Guide](www.cypress.com/ModusToolboxCapSenseConfig)
  * [ModusToolbox™ CAPSENSE™ Tuner Tool Guide](www.cypress.com/ModusToolboxCapSenseTuner)
  * [CAPSENSE™ Design Guide](https://www.infineon.com/dgdl/Infineon-AN85951_PSoC_4_and_PSoC_6_MCU_CapSense_Design_Guide-ApplicationNotes-v27_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0723535d4661)
  * [CSDADC Middleware API Reference Guide](https://infineon.github.io/csdadc/csdadc_api_reference_manual/html/index.html)
  * [CSDIDAC Middleware API Reference Guide](https://infineon.github.io/csdidac/csdidac_api_reference_manual/html/index.html)

* ModusToolbox™ Overview:
  * [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
  * [ModusToolbox™ Device Configurator Tool Guide](https://www.cypress.com/ModusToolboxDeviceConfig)

* Infineon Technologies AG Kits and Code Examples:
  * [CAPSENSE™ Middleware Code Example for MBED OS](https://github.com/Infineon/mbed-os-example-capsense)
  * [CAPSENSE™ Middleware Code Example for FreeRTOS](https://github.com/Infineon/mtb-example-psoc6-emwin-eink-freertos)
  * [CY8CKIT-145-40XX PSoC™ 4000S CAPSENSE™ Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-145-40xx/)
  * [CY8CKIT-149 PSoC™ 4100S Plus Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-149/)
  * [CY8CKIT-041-40XX PSoC™ 4 S-Series Pioneer Kit](https://www.infineon.com/dgdl/Infineon-CY8CKIT-041-40XX_PSoC_4_S-Series_Pioneer_Kit_Quick_Start_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0efc44781263)
  * [CY8CKIT-041-41XX PSoC™ 4100S CAPSENSE™ Pioneer Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-041-41xx/)

* General Information:
  * [AN210781 Getting Started with PSoC™ 6 MCU with Bluetooth Low Energy (BLE) Connectivity](https://www.infineon.com/dgdl/Infineon-AN210781_Getting_Started_with_PSoC_6_MCU_with_Bluetooth_Low_Energy_(BLE)_Connectivity_on_PSoC_Creator-ApplicationNotes-v05_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d311f536528)
  * [PSoC™ 6 Technical Reference Manual](https://www.infineon.com/dgdl/Infineon-PSoC_6_MCU_PSoC_63_with_BLE_Architecture_Technical_Reference_Manual-AdditionalTechnicalInformation-v11_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f946fea01ca)
  * [PSoC™ 63 with BLE Datasheet Programmable System-on-Chip datasheet](https://www.infineon.com/dgdl/Infineon-PSoC_6_MCU_PSoC_63_with_BLE_Datasheet_Programmable_System-on-Chip_(PSoC)-DataSheet-v16_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0ee4efe46c37)
  * [CAT1 PDL API Reference](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html)
  * [CAT2 PDL API Reference](https://infineon.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html)
  * [PSoC™ 4000S Family: PSoC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/dgdl/Infineon-PSoC_4000S_Family_PSoC_4_Architecture_Technical_Reference_Manual_(TRM)-AdditionalTechnicalInformation-v04_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f915c737eb7)
  * [PSoC™ 4100S and PSoC™ 4100S Plus: PSoC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/dgdl/Infineon-PSoC_4100S_and_PSoC_4100S_Plus_PSoC_4_Architecture_TRM-AdditionalTechnicalInformation-v12_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f9433460188)
  * [Infineon Technologies GitHub](https://github.com/Infineon)
  * [Infineon Technologies](http://www.infineon.com)

---
CYPRESS™ Semiconductor Corporation, 2019-2023.
