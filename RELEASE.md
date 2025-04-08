# CAPSENSE™ Middleware Library 7.0.0

### What is Included?
For a complete description of the CAPSENSE™ middleware, refer to [README.md](./README.md) and [API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html).
The revision history of the CAPSENSE™ middleware is also available at [API Reference Guide Changelog](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html#section_capsense_changelog).
The new support added in this release:
* Added support of PSOC™ 4100T Plus device family.
* Added scaling to CDAC codes in the manual mode for the CSD sensing method by using trim registers.
* Added scaling to measured sensor, electrode and shield capacitances in built-in self-test library.

### Known issues
For a complete description of the known issues and possible workarounds, refer to the [API Reference Guide - Errata section](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html#section_capsense_errata).

### Supported software and tools
This version of the CAPSENSE™ middleware was validated for compatibility with the following software and tools:

| Software and tools                                                                                | Version |
| :---                                                                                              | :----:  |
| ModusToolbox™ Software Environment                                                                | 3.5.0   |
| - ModusToolbox™ Device Configurator                                                               | 5.30    |
| - ModusToolbox™ MSC Superblock Personality for PSOC™ 4 devices in the Device Configurator         | 1.0     |
| - ModusToolbox™ MSCLP Personality for PSOC™ 4 devices in the Device Configurator                  | 3.0     |
| - ModusToolbox™ MSC Personality for PSOC™ 4 devices in the Device Configurator                    | 1.1     |
| - ModusToolbox™ CSD Personality for PSOC™ 4 devices in the Device Configurator                    | 2.0     |
| - ModusToolbox™ CSD Personality for PSOC™ 6 devices in the Device Configurator                    | 3.0     |
| - ModusToolbox™ CAPSENSE™ Configurator / Tuner                                                    | 8.0.0   |
| CAT1 Peripheral Driver Library (PDL)                                                              | 3.16.0  |
| CAT2 Peripheral Driver Library (PDL)                                                              | 2.16.0  |
| GCC Compiler                                                                                      | 11.3.1  |
| IAR Compiler                                                                                      | 9.50.2  |
| ARM Compiler 6                                                                                    | 6.22.0  |
| MBED OS                                                                                           | 5.15.8  |

### More information
For more information, refer to:
* CAPSENSE™ overview:
  * [CAPSENSE™ Middleware RELEASE.md](./RELEASE.md)
  * [CAPSENSE™ Middleware API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html)
  * [ModusToolbox™ CAPSENSE™ Configurator Tool Guide](www.infineon.com/ModusToolboxCapSenseConfig)
  * [ModusToolbox™ CAPSENSE™ Tuner Tool Guide](www.infineon.com/ModusToolboxCapSenseTuner)
  * [CAPSENSE™ Design Guide](https://www.infineon.com/dgdl/Infineon-AN85951_PSOC_4_and_PSOC_6_MCU_CapSense_Design_Guide-ApplicationNotes-v27_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0723535d4661)
  * [CSDADC Middleware API Reference Guide](https://infineon.github.io/csdadc/csdadc_api_reference_manual/html/index.html)
  * [CSDIDAC Middleware API Reference Guide](https://infineon.github.io/csdidac/csdidac_api_reference_manual/html/index.html)

* ModusToolbox™ Overview:
  * [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
  * [ModusToolbox™ Device Configurator Tool Guide](https://www.infineon.com/ModusToolboxDeviceConfig)

* Infineon Technologies AG Kits and Code Examples
  * [CAPSENSE™ Middleware Code Example for MBED OS](https://github.com/Infineon/mbed-os-example-capsense)
  * [CAPSENSE™ Middleware Code Example for FreeRTOS](https://github.com/Infineon/mtb-example-psoc6-emwin-eink-freertos)
  * [CY8CKIT-145-40XX PSOC™ 4000S CAPSENSE™ Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-145-40xx)
  * [CY8CKIT-149 PSOC™ 4100S Plus Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-149)
  * [CY8CKIT-041-40XX PSOC™ 4 S-Series Pioneer Kit](https://www.infineon.com/dgdl/Infineon-CY8CKIT-041-40XX_PSOC_4_S-Series_Pioneer_Kit_Quick_Start_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0efc44781263)
  * [CY8CKIT-041-41XX PSOC™ 4100S CAPSENSE™ Pioneer Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-041-41xx/)
  * [CY8CKIT-040T PSOC™ 4000T CAPSENSE™ Evaluation Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-040t/)

* General information:
  * [AN210781 Getting Started with PSOC™ 6 MCU with Bluetooth Low Energy (BLE) Connectivity](https://www.infineon.com/dgdl/Infineon-AN210781_Getting_Started_with_PSOC_6_MCU_with_Bluetooth_Low_Energy_(BLE)_Connectivity_on_PSOC_Creator-ApplicationNotes-v05_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d311f536528)
  * [AN215671 PSOC&trade; 6 MCU firmware design for BLE applications](https://www.infineon.com/cms/en/search.html#!term=AN215671&view=downloads)
  * [PSOC™ 6 Technical Reference Manual](https://www.infineon.com/dgdl/Infineon-PSOC_6_MCU_PSOC_63_with_BLE_Architecture_Technical_Reference_Manual-AdditionalTechnicalInformation-v11_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f946fea01ca)
  * [PSOC™ 63 with BLE Datasheet Programmable System-on-Chip datasheet](https://www.infineon.com/dgdl/Infineon-PSOC_6_MCU_PSOC_63_with_BLE_Datasheet_Programmable_System-on-Chip_(PSOC)-DataSheet-v16_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0ee4efe46c37)
  * [CAT1 PDL API Reference](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html)
  * [CAT2 PDL API Reference](https://infineon.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html)
  * [PSOC™ 4000S Family: PSOC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/dgdl/Infineon-PSOC_4000S_Family_PSOC_4_Architecture_Technical_Reference_Manual_(TRM)-AdditionalTechnicalInformation-v04_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f915c737eb7)
  * [PSOC™ 4100S and PSOC™ 4100S Plus: PSOC™ 4 Architecture Technical Reference Manual (TRM)](https://www.infineon.com/dgdl/Infineon-PSOC_4100S_and_PSOC_4100S_Plus_PSOC_4_Architecture_TRM-AdditionalTechnicalInformation-v12_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f9433460188)
  * [Infineon Technologies GitHub](https://github.com/Infineon)
  * [Infineon Technologies](http://www.infineon.com)

---
Cypress Semiconductor Corporation, 2019-2025.
