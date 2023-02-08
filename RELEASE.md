# CAPSENSE™ Middleware Library 3.0.1

### What's Included?
Please refer to the [README.md](./README.md) and the [API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html) for a complete description of the CAPSENSE™ Middleware.
The revision history of the CAPSENSE™ Middleware is also available on the [API Reference Guide Changelog](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html#section_capsense_changelog).
New important things in this release:
* Added CAPSENSE™ fifth-generation device support
* Added flash memory optimization


### Known Issues
Refer to the [API Reference Guide - Errata section](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html#section_capsense_errata) for a complete description of the known issues and possible workarounds

### Supported Software and Tools
This version of the CAPSENSE™ Middleware was validated for compatibility with the following Software and Tools:

| Software and Tools                                                                                | Version |
| :---                                                                                              | :----:  |
| ModusToolbox™ Software Environment                                                                | 3.0     |
| - ModusToolbox™ Device Configurator                                                               | 4.0     |
| - ModusToolbox™ MSC Superblock Personality for for PSoC™ 4 devices in the Device Configurator     | 1.0     |
| - ModusToolbox™ MSC Personality for PSoC™ 4 devices in Device Configurator                        | 1.1     |
| - ModusToolbox™ CSD Personality for PSoC™ 4 devices in Device Configurator                        | 2.0     |
| - ModusToolbox™ CSD Personality for PSoC™ 6 devices in Device Configurator                        | 3.0     |
| - ModusToolbox™ CAPSENSE™ Configurator / Tuner tools                                              | 5.0.0   |
| CAT1 Peripheral Driver Library (PDL)                                                              | 3.0.0   |
| CAT2 Peripheral Driver Library (PDL)                                                              | 2.0.0   |
| GCC Compiler                                                                                      | 10.3.1  |
| IAR Compiler                                                                                      | 8.42.1  |
| ARM Compiler 6                                                                                    | 6.13    |
| MBED OS                                                                                           | 5.15.8  |
| FreeRTOS                                                                                          | 10.4.5  |

### More information
The following resources contain more information:
* CAPSENSE™ Overview:
  * [CAPSENSE™ Middleware RELEASE.md](./RELEASE.md)
  * [CAPSENSE™ Middleware API Reference Guide](https://infineon.github.io/capsense/capsense_api_reference_manual/html/index.html)
  * [ModusToolbox™ CAPSENSE™ Configurator Tool Guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_CAPSENSE_Configurator_Guide_4-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d718a49017d99ab1e6531c8)
  * [ModusToolbox™ CAPSENSE™ Tuner Tool Guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_CAPSENSE_Tuner_5.0_User_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c8386267f0183a960ac735989)
  * [CAPSENSE™ Design Guide](https://www.infineon.com/dgdl/Infineon-AN85951_PSoC_4_and_PSoC_6_MCU_CapSense_Design_Guide-ApplicationNotes-v27_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0723535d4661)
  * [CSDADC Middleware API Reference Guide](https://infineon.github.io/csdadc/csdadc_api_reference_manual/html/index.html)
  * [CSDIDAC Middleware API Reference Guide](https://infineon.github.io/csdidac/csdidac_api_reference_manual/html/index.html)
* ModusToolbox™ Overview:
  * [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
  * [ModusToolbox™ Device Configurator Tool Guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Device_Configurator_4.0_User_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c8386267f0183a960bd41598f)
* Infineon Technologies Kits and Code Examples:
  * [CAPSENSE™ Middleware Code Example for MBED OS](https://github.com/Infineon/mbed-os-example-capsense)
  * [CAPSENSE™ Middleware Code Example for FreeRTOS](https://github.com/Infineon/mtb-example-psoc6-emwin-eink-freertos)
  * [CY8CKIT-145-40XX PSoC™ 4000S CAPSENSE™ Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-145-40xx)
  * [CY8CKIT-149 PSoC™ 4100S Plus Prototyping Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-149)
  * [CY8CKIT-041-40XX PSoC™ 4 S-Series Pioneer Kit](https://www.infineon.com/dgdl/Infineon-CY8CKIT-041-40XX_PSoC_4_S-Series_Pioneer_Kit_Quick_Start_Guide-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0efc44781263)
  * [CY8CKIT-041-41XX PSoC™ 4100S CAPSENSE™ Pioneer Kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-041-41xx/)
* General information:
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
