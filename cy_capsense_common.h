/***************************************************************************//**
* \file cy_capsense_common.h
* \version 7.0
*
* \brief
* This file provides the common CAPSENSE&trade; middleware definitions.
*
********************************************************************************
* \copyright
* Copyright 2018-2025, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_CAPSENSE_COMMON_H)
#define CY_CAPSENSE_COMMON_H

#include "cy_device_headers.h"
#include "cy_sysint.h"
#include "cycfg_capsense_defines.h"


#if (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP))

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Macros
*******************************************************************************/

#if (defined(CY_IP_M0S8CSDV2))

    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC4           (1u)
    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC6           (0u)

    #define CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN       (1u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN        (0u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP     (0u)

    #define CY_CAPSENSE_PSOC4_FOURTH_GEN                (1u)
    #define CY_CAPSENSE_PSOC6_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN                 (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN_LP              (0u)

#elif (defined(CY_IP_MXCSDV2))

    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC4           (0u)
    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC6           (1u)

    #define CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN       (1u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN        (0u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP     (0u)

    #define CY_CAPSENSE_PSOC4_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC6_FOURTH_GEN                (1u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN                 (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN_LP              (0u)

#elif (defined(CY_IP_M0S8MSCV3))

    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC4           (1u)
    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC6           (0u)

    #define CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN       (0u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN        (1u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP     (0u)

    #define CY_CAPSENSE_PSOC4_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC6_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN                 (1u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN_LP              (0u)

#elif (defined(CY_IP_M0S8MSCV3LP))

    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC4           (1u)
    #define CY_CAPSENSE_PLATFORM_DEVICE_PSOC6           (0u)

    #define CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN       (0u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN        (0u)
    #define CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP     (1u)

    #define CY_CAPSENSE_PSOC4_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC6_FOURTH_GEN                (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN                 (0u)
    #define CY_CAPSENSE_PSOC4_FIFTH_GEN_LP              (1u)

#endif

#define CY_CAPSENSE_PSOC4_CSDV2                         (CY_CAPSENSE_PSOC4_FOURTH_GEN)
#define CY_CAPSENSE_PSOC6_CSDV2                         (CY_CAPSENSE_PSOC6_FOURTH_GEN)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    #include "cy_csd.h"
#elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    #include "cy_msclp.h"
#else /* (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) */
    #include "cy_msc.h"
#endif

/******************************************************************************/
/** \addtogroup group_capsense_macros_general *//** \{ */
/******************************************************************************/
/** Middleware major version */
#define CY_CAPSENSE_MW_VERSION_MAJOR                    (7)
/** Middleware minor version */
#define CY_CAPSENSE_MW_VERSION_MINOR                    (0)
/** Middleware patch version */
#define CY_CAPSENSE_MW_VERSION_PATCH                    (0)
/** Middleware version */
#define CY_CAPSENSE_MW_VERSION                          (700)

#if (CY_CAPSENSE_PSOC6_FOURTH_GEN)
    /** Defined supported CSD driver version */
    #define CY_CSD_DRV_VERSION_REQUIRED                 (100)

    #if !defined (CY_CSD_DRV2_VERSION)
        /** If CSD driver version not defined by driver then define it here */
        #define CY_CSD_DRV2_VERSION                     (100)
    #else
        /** Use CSD Capture function */
        #define CY_CAPSENSE_USE_CAPTURE                 (1u)
    #endif

    /* Check used driver version */
    #if (CY_CSD_DRV_VERSION_REQUIRED > CY_CSD_DRV2_VERSION)
        #error Some of the features supported by the CAPSENSE&trade; Middleware version CY_CAPSENSE_MW_VERSION require newer version of the PDL. Update the PDL in your project.
    #endif

#endif

#if (CY_CAPSENSE_PSOC4_FOURTH_GEN)
    /** Defined supported CSD driver version */
    #define CY_CSD_DRV_VERSION_REQUIRED                 (100)

    #if !defined (CY_CSD_DRV2_VERSION)
        /** If CSD driver version not defined by driver then define it here */
        #define CY_CSD_DRV2_VERSION                     (100)
    #else
        /** Use CSD Capture function */
        #define CY_CAPSENSE_USE_CAPTURE                 (1u)
    #endif

    /* Check used driver version */
    #if (CY_CSD_DRV_VERSION_REQUIRED > CY_CSD_DRV2_VERSION)
        #error Some of the features supported by the CAPSENSE&trade; Middleware version CY_CAPSENSE_MW_VERSION require newer version of the PDL. Update the PDL in your project.
    #endif

#endif

#if (CY_CAPSENSE_PSOC4_FIFTH_GEN)
    /** Defined supported MSC driver version */
    #define CY_MSC_DRV_VERSION_REQUIRED                 (100)

    #if !defined (CY_MSC_DRV2_VERSION)
        /** If MSC driver version not defined by driver then define it here */
        #define CY_MSC_DRV2_VERSION                     (100)
    #else
        /** Use MSC Capture function */
        #define CY_CAPSENSE_USE_CAPTURE                 (1u)
    #endif

    /* Check used driver version */
    #if (CY_MSC_DRV_VERSION_REQUIRED > CY_MSC_DRV2_VERSION)
        #error Some of the features supported by the CAPSENSE&trade; Middleware version CY_CAPSENSE_MW_VERSION require newer version of the PDL. Update the PDL in your project.
    #endif

#endif

#if (CY_CAPSENSE_PSOC4_FIFTH_GEN_LP)
    #define CY_CAPSENSE_USE_CAPTURE                     (1u)
#endif

#if !defined (CY_CAPSENSE_USE_CAPTURE)
    /** Use CSD/MSC Init function */
    #define CY_CAPSENSE_USE_CAPTURE                     (0u)
#endif

/** Middleware ID */
#define CY_CAPSENSE_ID                                  (CY_PDL_DRV_ID(0x07uL))

/** CAPSENSE&trade; MW initialization is done */
#define CY_CAPSENSE_INIT_DONE                           (1u)
/** CAPSENSE&trade; MW initialization is needed */
#define CY_CAPSENSE_INIT_NEEDED                         (0u)

/** Feature enabled */
#define CY_CAPSENSE_ENABLE                              (1u)
/** Feature disabled */
#define CY_CAPSENSE_DISABLE                             (0u)

/** Regular Sensor active status mask */
#define CY_CAPSENSE_SNS_TOUCH_STATUS_MASK               (0x01u)
/** Proximity Sensor active status mask */
#define CY_CAPSENSE_SNS_PROX_STATUS_MASK                (0x01u)
/** Proximity Sensor touch active status mask */
#define CY_CAPSENSE_SNS_TOUCH_PROX_STATUS_MASK          (0x02u)
/** Sensor overflow mask */
#define CY_CAPSENSE_SNS_OVERFLOW_MASK                   (0x04u)

/** Widget active status mask */
#define CY_CAPSENSE_WD_ACTIVE_MASK                      (0x01u)
/** Widget enable status mask */
#define CY_CAPSENSE_WD_ENABLE_MASK                      (0x02u)
/** Widget working status mask */
#define CY_CAPSENSE_WD_WORKING_MASK                     (0x04u)
/** Widget maximum raw count calculation enable mask */
#define CY_CAPSENSE_WD_MAXCOUNT_CALC_MASK               (0x08u)
/** Widget row maximum raw count calculation enable mask */
#define CY_CAPSENSE_WD_MAXCOUNT_ROW_CALC_MASK           (0x10u)
/** \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros_mw_state *//** \{ */
/******************************************************************************/
/** The CAPSENSE&trade; middleware is busy */
#define CY_CAPSENSE_BUSY                                (0x80u)
/** The CAPSENSE&trade; middleware is not busy */
#define CY_CAPSENSE_NOT_BUSY                            (0x00u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    /** The CAPSENSE&trade; middleware busy mask */
    #define CY_CAPSENSE_BUSY_MASK                       (0xFFu)
    /** The MSCv3 channel busy mask */
    #define CY_CAPSENSE_BUSY_CH_MASK                    (0x01u)
    /** All MSCv3 channels busy mask */
    #define CY_CAPSENSE_BUSY_ALL_CH_MASK                (0x0Fu)
#endif

/** There was any sensor touch detected at the previous low power widget scan */
#define CY_CAPSENSE_MW_STATE_LP_ACTIVE_MASK             (0x400u)
/** The BIST is currently in progress */
#define CY_CAPSENSE_MW_STATE_BIST_MASK                  (0x800u)
/** The auto-calibration in Single CDAC mode */
#define CY_CAPSENSE_MW_STATE_CALIBRATION_SINGLE_MASK    (0x1000u)
/** The auto-calibration is currently in progress */
#define CY_CAPSENSE_MW_STATE_CALIBRATION_MASK           (0x2000u)
/** The smart sensing algorithm is currently in progress */
#define CY_CAPSENSE_MW_STATE_SMARTSENSE_MASK            (0x4000u)
/** The middleware initialization is currently in progress */
#define CY_CAPSENSE_MW_STATE_INITIALIZATION_MASK        (0x8000u)
/** The currently active or last scanned slot ID mask */
#define CY_CAPSENSE_MW_STATE_SCAN_SLOT_MASK             (0xFFFF0000u)
/** Current scan slot position in status */
#define CY_CAPSENSE_MW_STATE_SCAN_SLOT_POS              (16u)

#if (CY_CAPSENSE_PSOC4_FIFTH_GEN_LP)
    /** The last or currently scanned widget type is Active widget
     *  * \note This macro is available only for the fifth-generation low power CAPSENSE&trade;.
     */
    #define CY_CAPSENSE_MW_STATE_ACTIVE_WD_SCAN_MASK       (0x10u)
    /** The last or currently scanned widget type is Low Power widget
     *  * \note This macro is available only for the fifth-generation low power CAPSENSE&trade;.
     */
    #define CY_CAPSENSE_MW_STATE_LP_WD_SCAN_MASK           (0x20u)
    /** The mask for the last or currently scanned widget type
     *  * \note This macro is available only for the fifth-generation low power CAPSENSE&trade;.
     */
    #define CY_CAPSENSE_MW_STATE_WD_SCAN_MASK              (0x30u)
#endif
/** \} */


/******************************************************************************/
/** \addtogroup group_capsense_macros_settings *//** \{ */
/******************************************************************************/

/** Interrupt Driven scanning mode */
#define CY_CAPSENSE_SCAN_MODE_INT_DRIVEN                (0u)
/** DMA Driven scanning mode */
#define CY_CAPSENSE_SCAN_MODE_DMA_DRIVEN                (1u)

/** Sensor connection method through analog mux bus */
#define CY_CAPSENSE_AMUX_SENSOR_CONNECTION_METHOD       (0u)
/** Sensor connection method through control mux switches */
#define CY_CAPSENSE_CTRLMUX_SENSOR_CONNECTION_METHOD    (1u)

/** CIC filter mode */
#define CY_CAPSENSE_CIC_FILTER                          (0u)
/** CIC2 filter mode */
#define CY_CAPSENSE_CIC2_FILTER                         (1u)
/** Maximum CIC2 Decimation Rate */
#define CY_CAPSENSE_MAX_DECIMATION_RATE                 (255u)
/** CIC2 filter minimum valid sample number */
#define CY_CAPSENSE_CIC2_MIN_VALID_SAMPLES              (2u)
/** CIC2 filter maximum valid sample number */
#define CY_CAPSENSE_CIC2_MAX_VALID_SAMPLES              (515u)

/** Raw count counter mode saturate */
#define CY_CAPSENSE_COUNTER_MODE_SATURATE               (0u)
/** Raw count counter mode overflow */
#define CY_CAPSENSE_COUNTER_MODE_OVERFLOW               (1u)

/** Synchronization mode is disabled */
#define CY_CAPSENSE_SYNC_MODE_OFF                       (0u)
/** Synchronization is external */
#define CY_CAPSENSE_SYNC_EXTERNAL                       (1u)
/** Synchronization is internal */
#define CY_CAPSENSE_SYNC_INTERNAL                       (2u)

/** Sensing group undefined used at initialization */
#define CY_CAPSENSE_UNDEFINED_GROUP                     (0u)
/** CSD sensing group */
#define CY_CAPSENSE_CSD_GROUP                           (1u)
/** CSX sensing group */
#define CY_CAPSENSE_CSX_GROUP                           (2u)
/** BIST group */
#define CY_CAPSENSE_BIST_GROUP                          (3u)
/** BIST CSD sensor capacitance measurement group */
#define CY_CAPSENSE_BIST_CSD_GROUP                      (4u)
/** BIST CSX sensor capacitance measurement group */
#define CY_CAPSENSE_BIST_CSX_GROUP                      (5u)
/** BIST shield capacitance measurement group */
#define CY_CAPSENSE_BIST_SHIELD_GROUP                   (6u)
/** ISX sensing group */
#define CY_CAPSENSE_ISX_GROUP                           (10u)
/** MPSC-D sensing group */
#define CY_CAPSENSE_MPSC_GROUP                          (11u)

/** Total number of mode templates */
#define CY_CAPSENSE_REG_MODE_NUMBER                     (6u)

/* The values of these macros should be not changed due to CY_ID374670 */

/* Initialization Group */
/** Pin function undefined */
#define CY_CAPSENSE_SCW_FUNC_PIN_STATE_IDX_UNDEFINED    (255u)
/** Control mux switch state is ground */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_GND               (0u)
/** Control mux switch state is High-Z */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_HIGH_Z            (1u)
/** Control mux switch state defined as CSX Tx electrode */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_TX                (2u)
/** Control mux switch state defined as CSX Negative Tx electrode */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_TX_NEGATIVE       (4u)
/* Working Group */
/** Control mux switch state defined as CSX Rx electrode */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_RX                (3u)
/** Control mux switch state defined as CSD sensor */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_SNS               (5u)
/** Control mux switch state defined as connection to shield signal */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_SHIELD            (6u)
/* Corner Group */
/** Control mux switch state defined as CSDBUSC connected electrode */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_VDDA2             (7u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
/** Pin function undefined */
#define CY_CAPSENSE_PIN_STATE_IDX_UNDEFINED             (255u)
/** Pin function is ground */
#define CY_CAPSENSE_PIN_STATE_IDX_GND                   (0u)
/** Pin function is High-Z */
#define CY_CAPSENSE_PIN_STATE_IDX_HIGH_Z                (1u)
/** Pin function is CSX Rx electrode */
#define CY_CAPSENSE_PIN_STATE_IDX_CSX_RX                (2u)
/** Pin function is CSX Tx electrode */
#define CY_CAPSENSE_PIN_STATE_IDX_CSX_TX                (3u)
/** Pin function is CSX Negative Tx electrode */
#define CY_CAPSENSE_PIN_STATE_IDX_CSX_NEG_TX            (4u)
/** Pin function is CSD sensor */
#define CY_CAPSENSE_PIN_STATE_IDX_CSD_SNS               (5u)
/** Pin function is ISX Lx electrode */
#define CY_CAPSENSE_PIN_STATE_IDX_ISX_LX                (6u)
/** Pin function is ISX Rx electrode */
#define CY_CAPSENSE_PIN_STATE_IDX_ISX_RX                (7u)
/** Pin function is Active shield */
#define CY_CAPSENSE_PIN_STATE_IDX_ACT_SHIELD            (8u)
/** Pin function is Passive shield */
#define CY_CAPSENSE_PIN_STATE_IDX_PAS_SHIELD            (9u)
/** Pin function is CSX VDDA/2 connection */
#define CY_CAPSENSE_PIN_STATE_IDX_CSX_VDDA2             (10u)
/** Pin function is MPSC CSP connection */
#define CY_CAPSENSE_PIN_STATE_IDX_MPSC_CSP              (11u)
/** Pin function is MPSC CSN connection */
#define CY_CAPSENSE_PIN_STATE_IDX_MPSC_CSN              (12u)
/** Pin function is MPSC CSZ connection */
#define CY_CAPSENSE_PIN_STATE_IDX_MPSC_CSZ              (13u)

/** Number of CTRLMUX Pin States for LP */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_NUMBER            (14u)

#else /* All the rest platforms */

/** Number of CTRLMUX Pin State */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_NUMBER            (8u)
#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */

/** Number of CTRLMUX Pin State MASK registers */
#define CY_CAPSENSE_CTRLMUX_PIN_STATE_MASK_NUMBER       (3u)

/* MFS macros */
/** Number of multi-frequency scan channels */
#define CY_CAPSENSE_MAX_SUPPORTED_FREQ_NUM              (3u)
/** Total number of multi-frequency scan channels */
#if (CY_CAPSENSE_ENABLE == CY_CAPSENSE_MULTI_FREQUENCY_SCAN_EN)
    #define CY_CAPSENSE_CONFIGURED_FREQ_NUM             (3u)
#else
    #define CY_CAPSENSE_CONFIGURED_FREQ_NUM             (1u)
#endif
/** Multi-frequency channel 0 constant */
#define CY_CAPSENSE_MFS_CH0_INDEX                       (0u)
/** Multi-frequency channel 1 constant */
#define CY_CAPSENSE_MFS_CH1_INDEX                       (1u)
/** Multi-frequency channel 2 constant */
#define CY_CAPSENSE_MFS_CH2_INDEX                       (2u)
/** Multi-frequency channel number mask */
#define CY_CAPSENSE_MFS_FREQ_CHANNELS_NUM_MASK          (0xFu)
/** Multi-frequency enable mask */
#define CY_CAPSENSE_MFS_EN_MASK                         (0x10u)
/** Multi-frequency channels position */
#define CY_CAPSENSE_MFS_WIDGET_FREQ_ALL_CH_POS          (0x05u)
/** Multi-frequency channels mask */
#define CY_CAPSENSE_MFS_WIDGET_FREQ_ALL_CH_MASK         (0x60u)
/** Multi-frequency channel 1 mask */
#define CY_CAPSENSE_MFS_WIDGET_FREQ_CH_1_MASK           (0x20u)
/** Multi-frequency channel 2 mask */
#define CY_CAPSENSE_MFS_WIDGET_FREQ_CH_2_MASK           (0x40u)

/* Inactive sensor connection options */
/** Inactive sensor connection undefined */
#define CY_CAPSENSE_SNS_CONNECTION_UNDEFINED            (0u)
/** Inactive sensor connection to ground */
#define CY_CAPSENSE_SNS_CONNECTION_GROUND               (1u)
/** Inactive sensor connection to High-Z */
#define CY_CAPSENSE_SNS_CONNECTION_HIGHZ                (2u)
/** Inactive sensor connection to shield */
#define CY_CAPSENSE_SNS_CONNECTION_SHIELD               (4u)
/** Inactive CSX sensor connection to VDDA/2 voltage driven bus
 *  \note This field is available only for the fifth-generation CAPSENSE&trade;.
 */
#define CY_CAPSENSE_SNS_CONNECTION_VDDA_BY_2            (5u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
/* Shield electrode delay macros */
/** No shield signal delay */
#define CY_CAPSENSE_SH_DELAY_0NS                        (0u)
/** Shield signal delay > 5 ns */
#define CY_CAPSENSE_SH_DELAY_5NS                        (1u)
/** Shield signal delay > 10 ns */
#define CY_CAPSENSE_SH_DELAY_10NS                       (2u)
/** Shield signal delay > 20 ns */
#define CY_CAPSENSE_SH_DELAY_20NS                       (3u)

/* Idac sensing configuration */
/** Idac sourcing */
#define CY_CAPSENSE_IDAC_SOURCING                       (0u)
/** Idac sinking */
#define CY_CAPSENSE_IDAC_SINKING                        (1u)

/* Shield tank capacitor pre-charge source options */
/** Shield tank capacitor pre-charge from Vref */
#define CY_CAPSENSE_CSH_PRECHARGE_VREF                  (0u)
/** Shield tank capacitor pre-charge from IO buffer */
#define CY_CAPSENSE_CSH_PRECHARGE_IO_BUF                (1u)
#endif

/** Auto-mode of CIC2 Shift mask */
#define CY_CAPSENSE_CIC_AUTO_MASK                       (0x80u)
/** CIC2 Shift field position in register */
#define CY_CAPSENSE_CIC_FIELD_POSITION                  (28u)


/* Sense clock selection options */
/** Auto-mode of clock source selection mask */
#define CY_CAPSENSE_CLK_SOURCE_AUTO_MASK                (0x80u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    /** Clock source selection mask */
    #define CY_CAPSENSE_CLK_SOURCE_MASK                 (0x0Fu)
#else
    /** Clock source selection mask */
    #define CY_CAPSENSE_CLK_SOURCE_MASK                 (0x03u)
#endif
/** Clock source direct */
#define CY_CAPSENSE_CLK_SOURCE_DIRECT                   (0x00u)
/** Clock source SSC6 */
#define CY_CAPSENSE_CLK_SOURCE_SSC6                     (0x01u)
/** Clock source SSC7 */
#define CY_CAPSENSE_CLK_SOURCE_SSC7                     (0x02u)
/** Clock source SSC9 */
#define CY_CAPSENSE_CLK_SOURCE_SSC9                     (0x03u)
/** Clock source SSC10 */
#define CY_CAPSENSE_CLK_SOURCE_SSC10                    (0x04u)
/** Clock source PRS8 */
#define CY_CAPSENSE_CLK_SOURCE_PRS8                     (0x05u)
/** Clock source PRS12 */
#define CY_CAPSENSE_CLK_SOURCE_PRS12                    (0x06u)
/** Smart sensing algorithm Clock Source reserved bits */
#define CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_MASK          (0xF0u)
/** Smart sensing algorithm Clock Source reserved bits position */
#define CY_CAPSENSE_CLK_SOURCE_SMARTSENSE_POS           (4u)

/* Sense clock selection options for MSCv3 */
/** Clock source SSC */
#define CY_CAPSENSE_CLK_SOURCE_SSC                      (0x01u)
/** Clock source PRS */
#define CY_CAPSENSE_CLK_SOURCE_PRS                      (0x02u)
/** Clock source SSC auto-selection mask */
#define CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK            (0x04u)
/** Clock source SSC auto-selection */
#define CY_CAPSENSE_CLK_SOURCE_SSC_AUTO                 (CY_CAPSENSE_CLK_SOURCE_SSC_AUTO_MASK)
/** Clock source PRS auto-selection mask */
#define CY_CAPSENSE_CLK_SOURCE_PRS_AUTO_MASK            (0x08u)
/** Clock source PRS auto-selection */
#define CY_CAPSENSE_CLK_SOURCE_PRS_AUTO                 (CY_CAPSENSE_CLK_SOURCE_PRS_AUTO_MASK)

/* LFSR range selection options */
/** LFSR range auto-selection mask */
#define CY_CAPSENSE_LFSR_BITS_AUTO_MASK                 (0x80u)
/** LFSR range mask */
#define CY_CAPSENSE_LFSR_BITS_RANGE_MASK                (0x03u)
/** LFSR range auto-selection */
#define CY_CAPSENSE_LFSR_BITS_AUTO                      (CY_CAPSENSE_LFSR_BITS_AUTO_MASK)
/** LFSR range for LFSR_BITS=0 */
#define CY_CAPSENSE_LFSR_BITS_RANGE_0                   (0x00u)
/** LFSR range for LFSR_BITS=1 */
#define CY_CAPSENSE_LFSR_BITS_RANGE_1                   (0x01u)
/** LFSR range for LFSR_BITS=2 */
#define CY_CAPSENSE_LFSR_BITS_RANGE_2                   (0x02u)
/** LFSR range for LFSR_BITS=3 */
#define CY_CAPSENSE_LFSR_BITS_RANGE_3                   (0x03u)

/* CIC2 Decimation Rate Mode selection options */
/** The cicRate value set by users */
#define CY_CAPSENSE_CIC_RATE_MODE_MANUAL                (0u)
/** The cicRate value configured by middleware */
#define CY_CAPSENSE_CIC_RATE_MODE_AUTO                  (1u)
/** The cicRate value configured by SMARTSENSE&trade; algorithm */
#define CY_CAPSENSE_CIC_RATE_MODE_SMARTSENSE            (2u)

/* Sense clock auto-selection modes for MSCv3 */
/** This mode implements checking the following rules:
*     - An LFSR value should be selected so that the max clock dither is limited with the value,
*       specified by the LFSR Dither Limit parameter.
*     - At least one full spread spectrum polynomial should pass during the scan time.
*     - The value of the number of conversions should be an integer multiple of the repeat period
*        of the polynomial, that is specified by the Sense Clock LFSR Polynomial parameter.
*/
#define CY_CAPSENSE_SNS_CLK_SOURCE_AUTO_SEL_MODE_STRONG (0x00u)
/** This mode implements checking the following rules:
*     - An LFSR value should be selected so that the max clock dither is limited with the value,
*       specified by the LFSR Dither Limit parameter.
*     - At least one full spread spectrum polynomial should pass during the scan time.
*/
#define CY_CAPSENSE_SNS_CLK_SOURCE_AUTO_SEL_MODE_MEDIUM (0x01u)
/** This mode implements checking the following rules:
*     - An LFSR value should be selected so that the max clock dither is limited with the value,
*       specified by the LFSR Dither Limit parameter.
*/
#define CY_CAPSENSE_SNS_CLK_SOURCE_AUTO_SEL_MODE_WEAK   (0x02u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
/* Initialization switch resistance options */
/** Low switch resistance at initialization */
#define CY_CAPSENSE_INIT_SW_RES_LOW                     (0x00u)
/** Medium switch resistance at initialization */
#define CY_CAPSENSE_INIT_SW_RES_MEDIUM                  (0x01u)
/** High switch resistance at initialization */
#define CY_CAPSENSE_INIT_SW_RES_HIGH                    (0x02u)

/* Scanning switch resistance options */
/** Low switch resistance at scanning */
#define CY_CAPSENSE_SCAN_SW_RES_LOW                     (0x00u)
/** Medium switch resistance at scanning */
#define CY_CAPSENSE_SCAN_SW_RES_MEDIUM                  (0x01u)
/** High switch resistance at scanning */
#define CY_CAPSENSE_SCAN_SW_RES_HIGH                    (0x02u)

/* CSD shield switch resistance options */
/** Low shield switch resistance */
#define CY_CAPSENSE_SHIELD_SW_RES_LOW                   (0x00u)
/** Medium shield switch resistance */
#define CY_CAPSENSE_SHIELD_SW_RES_MEDIUM                (0x01u)
/** High shield switch resistance */
#define CY_CAPSENSE_SHIELD_SW_RES_HIGH                  (0x02u)
/** Low-EMI shield switch resistance */
#define CY_CAPSENSE_SHIELD_SW_RES_LOW_EMI               (0x03u)
#endif

/* CSD shield mode options */
/** Shield disabled */
#define CY_CAPSENSE_SHIELD_DISABLED                     (0u)
/** Active shield mode */
#define CY_CAPSENSE_SHIELD_ACTIVE                       (1u)
/** Passive shield mode */
#define CY_CAPSENSE_SHIELD_PASSIVE                      (2u)

/* Vref source options */
/** Vref source is taken from SRSS */
#define CY_CAPSENSE_VREF_SRSS                           (0x00u)
/** Vref source is taken from PASS */
#define CY_CAPSENSE_VREF_PASS                           (0x01u)

/* Iref source options */
/** Iref source is taken from SRSS */
#define CY_CAPSENSE_IREF_SRSS                           (0x00u)
/** Iref source is taken from PASS */
#define CY_CAPSENSE_IREF_PASS                           (0x01u)

/** Voltage limit to switch to low-voltage configuration */
#define CY_CAPSENSE_LOW_VOLTAGE_LIMIT                   (2000u)

/* Position Filter Configuration */
/** Mask of all filters enabling */
#define CY_CAPSENSE_POSITION_FILTERS_MASK               (0x000000FFu)
/** Median position filter enable mask */
#define CY_CAPSENSE_POSITION_MED_MASK                   (0x00000001u)
/** IIR position filter enable mask */
#define CY_CAPSENSE_POSITION_IIR_MASK                   (0x00000002u)
/** Adaptive IIR position filter enable mask */
#define CY_CAPSENSE_POSITION_AIIR_MASK                  (0x00000004u)
/** Average position filter enable mask */
#define CY_CAPSENSE_POSITION_AVG_MASK                   (0x00000008u)
/** Jitter position filter enable mask */
#define CY_CAPSENSE_POSITION_JIT_MASK                   (0x00000010u)
/** Mask of position filters history size */
#define CY_CAPSENSE_POSITION_FILTERS_SIZE_MASK          (0x0000FF00u)
/** Offset of position filters history size */
#define CY_CAPSENSE_POSITION_FILTERS_SIZE_OFFSET        (8u)
/** Mask of IIR coefficient of position filter */
#define CY_CAPSENSE_POSITION_IIR_COEFF_MASK             (0x00FF0000u)
/** Offset of IIR coefficient of position filter */
#define CY_CAPSENSE_POSITION_IIR_COEFF_OFFSET           (16u)

/* Raw count filter macros */
/** Offset of raw count filter history size */
#define CY_CAPSENSE_RC_FILTER_SNS_HISTORY_SIZE_OFFSET    (0u)
/** Offset of raw count median filter enable mask */
#define CY_CAPSENSE_RC_FILTER_MEDIAN_EN_OFFSET           (4u)
/** Offset of raw count median filter mode mask */
#define CY_CAPSENSE_RC_FILTER_MEDIAN_MODE_OFFSET         (5u)
/** Offset of raw count IIR filter enable mask */
#define CY_CAPSENSE_RC_FILTER_IIR_EN_OFFSET              (7u)
/** Offset of raw count IIR filter mode mask */
#define CY_CAPSENSE_RC_FILTER_IIR_MODE_OFFSET            (8u)
/** Offset of raw count average filter enable mask */
#define CY_CAPSENSE_RC_FILTER_AVERAGE_EN_OFFSET          (10u)
/** Offset of raw count average filter mode mask */
#define CY_CAPSENSE_RC_FILTER_AVERAGE_MODE_OFFSET        (11u)
/** Offset of raw count HW IIR filter enable mask */
#define CY_CAPSENSE_RC_HW_IIR_FILTER_EN_OFFSET           (13u)
/** Offset of raw count CM filter enable mask */
#define CY_CAPSENSE_RC_FILTER_CM_EN_OFFSET               (14u)

/** Mask of raw count filter history size */
#define CY_CAPSENSE_RC_FILTER_SNS_HISTORY_SIZE_MASK     ((uint16_t)((uint16_t)0x000Fu << CY_CAPSENSE_RC_FILTER_SNS_HISTORY_SIZE_OFFSET))
/** Median raw count filter enable mask */
#define CY_CAPSENSE_RC_FILTER_MEDIAN_EN_MASK            ((uint16_t)((uint16_t)0x0001u << CY_CAPSENSE_RC_FILTER_MEDIAN_EN_OFFSET))
/** Median raw count filter mode mask */
#define CY_CAPSENSE_RC_FILTER_MEDIAN_MODE_MASK          ((uint16_t)((uint16_t)0x0003u << CY_CAPSENSE_RC_FILTER_MEDIAN_MODE_OFFSET))
/** IIR raw count filter enable mask */
#define CY_CAPSENSE_RC_FILTER_IIR_EN_MASK               ((uint16_t)((uint16_t)0x0001u << CY_CAPSENSE_RC_FILTER_IIR_EN_OFFSET))
/** IIR raw count filter mode mask */
#define CY_CAPSENSE_RC_FILTER_IIR_MODE_MASK             ((uint16_t)((uint16_t)0x0003u << CY_CAPSENSE_RC_FILTER_IIR_MODE_OFFSET))
/** Average raw count filter enable mask */
#define CY_CAPSENSE_RC_FILTER_AVERAGE_EN_MASK           ((uint16_t)((uint16_t)0x0001u << CY_CAPSENSE_RC_FILTER_AVERAGE_EN_OFFSET))
/** Average raw count filter mode mask */
#define CY_CAPSENSE_RC_FILTER_AVERAGE_MODE_MASK         ((uint16_t)((uint16_t)0x0003u << CY_CAPSENSE_RC_FILTER_AVERAGE_MODE_OFFSET))
/** Raw count HW IIR filter enable mask */
#define CY_CAPSENSE_RC_HW_IIR_FILTER_EN_MASK            ((uint16_t)((uint16_t)0x0001u << CY_CAPSENSE_RC_HW_IIR_FILTER_EN_OFFSET))
/** Raw count CM filter enable mask */
#define CY_CAPSENSE_RC_FILTER_CM_EN_MASK                ((uint16_t)((uint16_t)0x0001u << CY_CAPSENSE_RC_FILTER_CM_EN_OFFSET))
/** All raw count filters enable mask */
#define CY_CAPSENSE_RC_FILTER_ALL_EN_MASK               (CY_CAPSENSE_RC_FILTER_MEDIAN_EN_MASK |\
                                                         CY_CAPSENSE_RC_FILTER_IIR_EN_MASK |\
                                                         CY_CAPSENSE_RC_FILTER_AVERAGE_EN_OFFSET)
/** Raw count IIR filter mode standard */
#define CY_CAPSENSE_IIR_FILTER_STANDARD                 ((uint16_t)((uint16_t)1u << CY_CAPSENSE_RC_FILTER_IIR_MODE_OFFSET))
/** Raw count IIR filter mode performance */
#define CY_CAPSENSE_IIR_FILTER_PERFORMANCE              ((uint16_t)((uint16_t)2u << CY_CAPSENSE_RC_FILTER_IIR_MODE_OFFSET))
/** Raw count average filter mode */
#define CY_CAPSENSE_AVERAGE_FILTER_LEN_4                ((uint16_t)((uint16_t)2u << CY_CAPSENSE_RC_FILTER_AVERAGE_MODE_OFFSET))
/** \} */

/* Slot configuration */
/** Shield only slot, can be assigned to the \ref cy_stc_capsense_scan_slot_t::wdId */
#define CY_CAPSENSE_SLOT_SHIELD_ONLY                    (0xFFFDu)
/** TX only slot, can be assigned to the \ref cy_stc_capsense_scan_slot_t::wdId */
#define CY_CAPSENSE_SLOT_TX_ONLY                        (0xFFFEu)
/** Empty slot, can be assigned to the \ref cy_stc_capsense_scan_slot_t::wdId */
#define CY_CAPSENSE_SLOT_EMPTY                          (0xFFFFu)

/* Centroid configuration */
/** Supported touch count mask */
#define CY_CAPSENSE_CENTROID_NUMBER_MASK                (0x0003u)
/** Diplex enable mask */
#define CY_CAPSENSE_DIPLEXING_MASK                      (0x0004u)
/** Ballistic multiplier enable mask */
#define CY_CAPSENSE_CENTROID_BALLISTIC_MASK             (0x0008u)
/** 3x3 centroid enable mask */
#define CY_CAPSENSE_CENTROID_3X3_MASK                   (0x0020u)
/** 5x5 centroid enable mask */
#define CY_CAPSENSE_CENTROID_5X5_MASK                   (0x0040u)
/** Edge correction enable mask */
#define CY_CAPSENSE_EDGE_CORRECTION_MASK                (0x0080u)
/**
* Centroid calculation method mask
* * Direct provides correct calculation but worse linearity on edges
*   and requires edge correction [MaxPosition / (numSns)]
* * Approximate provides better linearity on edges that leads to
*   worse accuracy in general [MaxPosition / (numSns - 1u)]
*/
#define CY_CAPSENSE_CALC_METHOD_MASK                    (0x0100u)
/** Liquid level foam sub-widget enable mask */
#define CY_CAPSENSE_LLW_FOAM_EN_MASK                    (0x0010u)
/** Liquid level tuning completed mask */
#define CY_CAPSENSE_LLW_TUNING_COMPLETED_MASK           (0x0200u)

/******************************************************************************/
/** \addtogroup group_capsense_macros_touch *//** \{ */
/******************************************************************************/
/** No touch detected */
#define CY_CAPSENSE_POSITION_NONE                       (0x00u)
/** One finger touch detected */
#define CY_CAPSENSE_POSITION_ONE                        (0x01u)
/** Two finger touches detected */
#define CY_CAPSENSE_POSITION_TWO                        (0x02u)
/** Three finger touches detected */
#define CY_CAPSENSE_POSITION_THREE                      (0x03u)
/** Multiple touches detected that exceed number of supported touches */
#define CY_CAPSENSE_POSITION_MULTIPLE                   (0xFFu)
/** Maximum possible centroid number */
#define CY_CAPSENSE_MAX_CENTROIDS                       (3u)
/** Maximum number of peaks in CSX Touchpad */
#define CY_CAPSENSE_CSX_TOUCHPAD_MAX_PEAKS              (5u)
/** Maximum age of a touch */
#define CY_CAPSENSE_CSX_TOUCHPAD_MAX_AGE                (0xFFu)
/** Undefined ID */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_UNDEFINED           (0x80u)
/** Non-assigned ID */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_ABSENT              (0x40u)
/** Touch below threshold ID */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_ON_FAIL             (0x20u)
/** Minimum ID */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_MIN                 (0u)
/** Maximum ID */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_MAX                 (7u)
/** ID mask */
#define CY_CAPSENSE_CSX_TOUCHPAD_ID_MASK                (0x00FFu)
/** Touch power mask */
#define CY_CAPSENSE_CSX_TOUCHPAD_Z_MASK                 (0x00FFu)
/** Age mask */
#define CY_CAPSENSE_CSX_TOUCHPAD_AGE_MASK               (0xFF00u)
/** Debounce mask */
#define CY_CAPSENSE_CSX_TOUCHPAD_DEBOUNCE_MASK          (0xFF00u)
/** \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros_process *//** \{ */
/******************************************************************************/
/** Applies all enabled filters in the default order to the raw counts */
#define CY_CAPSENSE_PROCESS_FILTER                      (0x01u)
/** Updates baselines using current raw count values for the widget/sensor */
#define CY_CAPSENSE_PROCESS_BASELINE                    (0x02u)
/** Calculates differences for the widget/sensor */
#define CY_CAPSENSE_PROCESS_DIFFCOUNTS                  (0x04u)
/** Runs the noise envelope filter to measure noise magnitude for the widget/sensor */
#define CY_CAPSENSE_PROCESS_CALC_NOISE                  (0x08u)
/** Updates widget thresholds based on raw counts noise magnitude */
#define CY_CAPSENSE_PROCESS_THRESHOLDS                  (0x10u)
/** Runs the widget-specific processing algorithms and updates it status */
#define CY_CAPSENSE_PROCESS_STATUS                      (0x20u)
/** Applies median filter for MFS widget raw counts (applicable only for fifth-generation CAPSENSE&trade;) */
#define CY_CAPSENSE_PROCESS_MFS_FILTER                  (0x40u)
/** Definition that combines all possible processing tasks */
#define CY_CAPSENSE_PROCESS_ALL                         (CY_CAPSENSE_PROCESS_FILTER | \
                                                         CY_CAPSENSE_PROCESS_BASELINE | \
                                                         CY_CAPSENSE_PROCESS_DIFFCOUNTS | \
                                                         CY_CAPSENSE_PROCESS_CALC_NOISE | \
                                                         CY_CAPSENSE_PROCESS_THRESHOLDS | \
                                                         CY_CAPSENSE_PROCESS_STATUS | \
                                                         CY_CAPSENSE_PROCESS_MFS_FILTER)
/** Multi-phase TX min order */
#define CY_CAPSENSE_MPTX_MIN_ORDER                      (4u)
/** Multi-phase TX max order */
#define CY_CAPSENSE_MPTX_MAX_ORDER                      (32u)

/** Multi-phase Self-Cap min order */
#define CY_CAPSENSE_MPSC_MIN_ORDER                      (5u)
/** Multi-phase Self-Cap max order */
#define CY_CAPSENSE_MPSC_MAX_ORDER                      (32u)

/** Multi-phase max order */
#define CY_CAPSENSE_MULTIPHASE_MAX_ORDER                (32u)
/** \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros_pin *//** \{ */
/******************************************************************************/
/** Connection of pin to ground */
#define CY_CAPSENSE_GROUND                              (0u)
/** Connection of pin to High-Z */
#define CY_CAPSENSE_HIGHZ                               (1u)
/** Configuring of pin as a shield */
#define CY_CAPSENSE_SHIELD                              (2u)
/** Configuring of pin as a CSD sensor */
#define CY_CAPSENSE_SENSOR                              (3u)
/** Configuring of pin as a CSX Tx electrode */
#define CY_CAPSENSE_TX_PIN                              (4u)
/** Configuring of pin as a CSX Rx electrode */
#define CY_CAPSENSE_RX_PIN                              (5u)

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
    /** Configuring of pin as a negative Tx
     * \note This macro is available only for the fifth-generation CAPSENSE&trade; and  the fifth-generation low power
     * CAPSENSE&trade;
     */
    #define CY_CAPSENSE_NEGATIVE_TX_PIN                     (6u)
    /** Configuring of pin as an ISX Lx
     * \note This macro is available only for the fifth-generation CAPSENSE&trade; and  the fifth-generation low power
     * CAPSENSE&trade;
     */
    #define CY_CAPSENSE_ISX_LX_PIN                          (7u)
    /** Configuring of pin as an ISX Rx
     * \note This macro is available only for the fifth-generation CAPSENSE&trade; and  the fifth-generation low power
     * CAPSENSE&trade;
     */
    #define CY_CAPSENSE_ISX_RX_PIN                          (8u)
    /** Configuring of pin as an VDDA/2
     * \note This macro is available only for the fifth-generation CAPSENSE&trade; and  the fifth-generation low power
     * CAPSENSE&trade;
     */
    #define CY_CAPSENSE_VDDA2                               (9u)
#endif /* ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)) */

/** Pin is not connected to scanning bus */
#define CY_CAPSENSE_SNS_DISCONNECTED                    (0u)
/** Pin is connected to scanning bus */
#define CY_CAPSENSE_SNS_CONNECTED                       (1u)

/** Defines pin state as not controlled by / disconnected from MSCLP HW block */
#define CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_DIS              (0u)
/** Defines pin state as controlled by / connected to MSCLP HW block */
#define CY_CAPSENSE_MSCLP_GPIO_MSC_ANA_EN               (1u)

/** \} */

/******************************************************************************/
/** \addtogroup group_capsense_macros_miscellaneous *//** \{ */
/******************************************************************************/
/** Defines the communication command complete bit mask */
#define CY_CAPSENSE_TU_CMD_COMPLETE_BIT                 (0x8000u)
/** Defines the status if restart was not performed in Cy_CapSense_RunTuner() */
#define CY_CAPSENSE_STATUS_RESTART_NONE                 (0x00u)
/** Defines the status if restart was done in Cy_CapSense_RunTuner() */
#define CY_CAPSENSE_STATUS_RESTART_DONE                 (0x01u)
#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    /** The size of the idac gain look-up table */
    #define CY_CAPSENSE_IDAC_GAIN_TABLE_SIZE            (7u)

    #if (CY_CAPSENSE_PLATFORM_DEVICE_PSOC4)
        #if (2u <= CY_IP_M0S8CSDV2_VERSION)
            /** Defines the number of supported idac gains for the CSDv2_ver2-based devices */
            #define CY_CAPSENSE_IDAC_GAIN_NUMBER        (7u)
        #else
            /** Defines the number of supported idac gains for the CSDv2_ver1-based devices */
            #define CY_CAPSENSE_IDAC_GAIN_NUMBER        (6u)
        #endif
    #else
        #define CY_CAPSENSE_IDAC_GAIN_NUMBER            (6u)
    #endif
#endif
/** 100% value */
#define CY_CAPSENSE_PERCENTAGE_100                      (100u)
/** An approximate duration in CPU cycles of the check loop with the software watchdog timer */
#define CY_CAPSENSE_APPROX_LOOP_DURATION                (5u)

/* Scope of scanning macros */
/** Widget scanning scope is a single sensor */
#define CY_CAPSENSE_SCAN_SCOPE_SNGL_SNS                 (0x0u)
/** Sensor scanning scope is a single widget */
#define CY_CAPSENSE_SCAN_SCOPE_ALL_SNS_MASK             (0x01u)
/** Widget scanning scope is all widgets */
#define CY_CAPSENSE_SCAN_SCOPE_ALL_WD_MASK              (0x02u)
/** Single slot scanning */
#define CY_CAPSENSE_SCAN_SNGL_SLOT                      (0x1u)
/** Multiple slot scanning */
#define CY_CAPSENSE_SCAN_MULTIPLE_SLOT                  (0x0u)


/** CAPSENSE&trade; related HW state is undefined */
#define CY_CAPSENSE_HW_CONFIG_UNDEFINED                 (0u)
/** CAPSENSE&trade; related HW is captured and configured to the default state */
#define CY_CAPSENSE_HW_CONFIG_CAPTURED_DEFAULT          (1u)
/** CAPSENSE&trade; related HW is configured to the standard regular scanning */
#define CY_CAPSENSE_HW_CONFIG_REGULAR_SCANNING          (2u)
/** CAPSENSE&trade; related HW is configured to execute the BIST functions */
#define CY_CAPSENSE_HW_CONFIG_BIST_FUNCTIONALITY        (3u)
/** CAPSENSE&trade; related HW is configured to the MPSC scanning */
#define CY_CAPSENSE_HW_CONFIG_MPSC_SCANNING             (4u)
/** CAPSENSE&trade; related HW is configured to execute a scan with the saturated channel */
#define CY_CAPSENSE_HW_CONFIG_CHANNEL_SATURATION        (5u)
/** CAPSENSE&trade; related HW is configured to the smart sense */
#define CY_CAPSENSE_HW_CONFIG_SMARTSENSE                (6u)
/** CAPSENSE&trade; related HW is configured to execute a scan to define CDAC dither parameters */
#define CY_CAPSENSE_HW_CONFIG_AUTO_DITHERING            (7u)

/** \} */


/******************************************************************************/
/** \addtogroup group_capsense_macros_bist *//** \{ */
/******************************************************************************/
/** The mask for a widget CRC test */
#define CY_CAPSENSE_BIST_CRC_WDGT_MASK                              ((uint32_t)CY_CAPSENSE_TST_WDGT_CRC_EN << 0uL)
/** The mask for a baseline integrity test */
#define CY_CAPSENSE_BIST_BSLN_INTEGRITY_MASK                        ((uint32_t)CY_CAPSENSE_TST_BSLN_INTEGRITY_EN << 1uL)
/** The mask for a rawcount integrity test */
#define CY_CAPSENSE_BIST_RAW_INTEGRITY_MASK                         ((uint32_t)CY_CAPSENSE_TST_RAW_INTEGRITY_EN << 2uL)
/** The mask for a pin integrity test */
#define CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK                         ((uint32_t)CY_CAPSENSE_TST_SNS_SHORT_EN << 3uL)
/** The mask for a sensor capacitance measurement test */
#define CY_CAPSENSE_BIST_SNS_CAP_MASK                               ((uint32_t)CY_CAPSENSE_TST_SNS_CAP_EN << 4uL)
/** The mask for a shield capacitance measurement test */
#define CY_CAPSENSE_BIST_SHIELD_CAP_MASK                            ((uint32_t)CY_CAPSENSE_TST_SH_CAP_EN << 5uL)
/** The mask for an external capacitor capacitance measurement test */
#define CY_CAPSENSE_BIST_EXTERNAL_CAP_MASK                          ((uint32_t)CY_CAPSENSE_TST_EXTERNAL_CAP_EN << 6uL)
/** The mask for a VDDA measurement test */
#define CY_CAPSENSE_BIST_VDDA_MASK                                  ((uint32_t)CY_CAPSENSE_TST_VDDA_EN << 7uL)
/** The mask for an electrode capacitance measurement test */
#define CY_CAPSENSE_BIST_ELTD_CAP_MASK                              ((uint32_t)CY_CAPSENSE_TST_ELTD_CAP_EN << 8uL)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FOURTH_GEN)
    /** The mask for all enabled self-test functions */
    #define CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK           (CY_CAPSENSE_BIST_CRC_WDGT_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_CAP_MASK | \
                                                                     CY_CAPSENSE_BIST_EXTERNAL_CAP_MASK |\
                                                                     CY_CAPSENSE_BIST_VDDA_MASK |\
                                                                     CY_CAPSENSE_BIST_SHIELD_CAP_MASK)

    /** The mask for all enabled measurement self-test functions */
    #define CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN                    (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_CAP_EN || \
                                                                      CY_CAPSENSE_TST_SH_CAP_EN ||\
                                                                      CY_CAPSENSE_TST_VDDA_EN ||\
                                                                      CY_CAPSENSE_TST_EXTERNAL_CAP_EN))

    /** The mask for all enabled self-test functions which changes HW configuration */
    #define CY_CAPSENSE_TST_HW_GROUP_EN                             (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_SHORT_EN ||\
                                                                      CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
#elif (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN)
    /** The mask for all enabled self-test functions */
    #define CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK           (CY_CAPSENSE_BIST_CRC_WDGT_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_CAP_MASK | \
                                                                     CY_CAPSENSE_BIST_ELTD_CAP_MASK | \
                                                                     CY_CAPSENSE_BIST_SHIELD_CAP_MASK)

    /** The mask for all enabled measurement self-test functions */
    #define CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN                    (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_CAP_EN || \
                                                                      CY_CAPSENSE_TST_SH_CAP_EN ||\
                                                                      CY_CAPSENSE_TST_ELTD_CAP_EN))

    /** The mask for all enabled self-test functions which changes HW configuration */
    #define CY_CAPSENSE_TST_HW_GROUP_EN                             (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_SHORT_EN ||\
                                                                      CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
#else
    /** The mask for all enabled self-test functions */
    #define CY_CAPSENSE_BIST_RUN_AVAILABLE_SELF_TEST_MASK           (CY_CAPSENSE_BIST_CRC_WDGT_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_INTEGRITY_MASK | \
                                                                     CY_CAPSENSE_BIST_SNS_CAP_MASK | \
                                                                     CY_CAPSENSE_BIST_ELTD_CAP_MASK | \
                                                                     CY_CAPSENSE_BIST_EXTERNAL_CAP_MASK |\
                                                                     CY_CAPSENSE_BIST_VDDA_MASK |\
                                                                     CY_CAPSENSE_BIST_SHIELD_CAP_MASK)

    /** The mask for all enabled measurement self-test functions */
    #define CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN                    (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_CAP_EN || \
                                                                      CY_CAPSENSE_TST_SH_CAP_EN ||\
                                                                      CY_CAPSENSE_TST_ELTD_CAP_EN ||\
                                                                      CY_CAPSENSE_TST_VDDA_EN ||\
                                                                      CY_CAPSENSE_TST_EXTERNAL_CAP_EN))

    /** The mask for all enabled self-test functions which changes HW configuration */
    #define CY_CAPSENSE_TST_HW_GROUP_EN                             (CY_CAPSENSE_BIST_EN && \
                                                                     (CY_CAPSENSE_TST_SNS_SHORT_EN ||\
                                                                      CY_CAPSENSE_TST_MEASUREMENT_GROUP_EN))
#endif

#if ((CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN) || (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP))
    /** The estimated serial resistance value of the external capacitor charging/discharging
     * circuit including GPIO, AMuxBus, PCB and all another resistances. The value is used
     * for the external capacitor charging/discharging time estimations */
    #define CY_CAPSENSE_BIST_EXT_CAP_SERIAL_RESISTANCE              (1000u)
#endif

/** \} */


/******************************************************************************/
/** \addtogroup group_capsense_macros_status *//** \{ */
/******************************************************************************/
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Successfully */
#define CY_CAPSENSE_STATUS_SUCCESS                      (0x00u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Bad input parameter */
#define CY_CAPSENSE_STATUS_BAD_PARAM                    (0x01u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Received wrong data */
#define CY_CAPSENSE_STATUS_BAD_DATA                     (0x02u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Timeout */
#define CY_CAPSENSE_STATUS_TIMEOUT                      (0x04u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Invalid state */
#define CY_CAPSENSE_STATUS_INVALID_STATE                (0x08u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Bad configuration */
#define CY_CAPSENSE_STATUS_BAD_CONFIG                   (0x10u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Overflow */
#define CY_CAPSENSE_STATUS_CONFIG_OVERFLOW              (0x20u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: HW is busy */
#define CY_CAPSENSE_STATUS_HW_BUSY                      (0x40u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: HW is locked */
#define CY_CAPSENSE_STATUS_HW_LOCKED                    (0x80u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Unable to perform calibration */
#define CY_CAPSENSE_STATUS_CALIBRATION_FAIL             (0x100u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: CapDac calibration fail - The reference/fine CapDAC calibration stage is failed as the raw count minimum across widget is out of range */
#define CY_CAPSENSE_STATUS_CALIBRATION_REF_CHECK_FAIL   (0x200u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: CapDac calibration fail - The resulting rawcounts across all sensors in widget are out of defined range */
#define CY_CAPSENSE_STATUS_CALIBRATION_CHECK_FAIL       (0x400u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Sense Clock Divider
*   is out of the valid range for the specified Clock source configuration
*/
#define CY_CAPSENSE_STATUS_BAD_CLOCK_CONFIG             (0x800u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Used scan configuration
*   is not sufficient to accumulate at least one valid CIC2 sample.
*/
#define CY_CAPSENSE_STATUS_CIC2_ACC_UNDERFLOW           (0x1000u)

/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Used scan configuration
*   leads to the overflow in the CIC2 accumulation register.
*/
#define CY_CAPSENSE_STATUS_CIC2_ACC_OVERFLOW            (0x2000u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Unable to perform a scan when MPSC sensors and
*   other sense method sensors are located in the same frame
*/
#define CY_CAPSENSE_STATUS_MIXED_SENSORS                (0x4000u)
/** Return status \ref cy_capsense_status_t of CAPSENSE&trade; operation: Bad config for LLW */
#define CY_CAPSENSE_STATUS_LLW_BAD_CONFIG               (0x8000u)

/** State of Low Power sensor processing mask */
#define CY_CAPSENSE_LP_PROCESS_ENABLED_MASK             (0x1uL)

/** \} */

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
    /* HW IIR Filter enable status for Active widgets */
    #define CY_CAPSENSE_RC_HW_IIR_FILTER_EN             (CY_CAPSENSE_REGULAR_RC_HW_IIR_FILTER_EN || \
                                                         CY_CAPSENSE_PROX_RC_HW_IIR_FILTER_EN)
#else
    #define CY_CAPSENSE_RC_HW_IIR_FILTER_EN             (0u)
#endif

/* Shift to 8 bits */
#define CY_CAPSENSE_BYTE_SHIFT                          (8u)

/* The time interval is required for settling analog part of the HW block. */
#define CY_CAPSENSE_ANALOG_SETTLING_TIME_US             (25u)

#if (CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP)
/* The time interval in us is required for settling analog part of the MSCLP HW block. */
#define CY_CAPSENSE_MSCLP_HW_SETTLING_TIME_US           (10u)

/*  The Sequencer FSM Operating Mode,
*   to be used with cy_stc_capsense_internal_context_t::operatingMode
*/
#define CY_CAPSENSE_MODE_CPU                            (0x0u) /* The regular (interrupt-driven) CPU mode */
#define CY_CAPSENSE_MODE_AS_MS                          (0x2u) /* The Autonomous-Scan Multi-Sensor Mode */
#define CY_CAPSENSE_MODE_LP_AOS                         (0x3u) /* The Low Power Always-On-Scanning Mode */

#endif /* CY_CAPSENSE_PLATFORM_BLOCK_FIFTH_GEN_LP */

#define CY_CAPSENSE_BYTES_IN_16_BITS                    (2uL)
#define CY_CAPSENSE_BYTE_IN_32_BIT                      (4uL)
#define CY_CAPSENSE_CONVERSION_MEGA                     (1000000u)
#define CY_CAPSENSE_CONVERSION_KILO                     (1000u)
#define CY_CAPSENSE_CONVERSION_HECTO                    (100u)
#define CY_CAPSENSE_CONVERSION_DEKA                     (10u)
#define CY_CAPSENSE_DIVIDER_TWO                         (2u)
#define CY_CAPSENSE_32_BIT_MASK                         (0xFFFFFFFFu)
#define CY_CAPSENSE_25_BIT_MASK                         (0x01FFFFFFu)
#define CY_CAPSENSE_16_BIT_MASK                         (0xFFFFu)
#define CY_CAPSENSE_14_BIT_MASK                         (0x3FFFu)

/*
* These defines are obsolete and kept for backward compatibility only.
* They will be removed in the future versions.
*/
#define CY_CAPSENSE_MDW_VERSION_MAJOR                   (CY_CAPSENSE_MW_VERSION_MAJOR)
#define CY_CAPSENSE_MDW_VERSION_MINOR                   (CY_CAPSENSE_MW_VERSION_MINOR)
#define CY_CAPSENSE_MDW_ID                              (CY_CAPSENSE_ID)
#define CY_CAPSENSE_SW_STS_BUSY                         (CY_CAPSENSE_BUSY)

/* CAPSENSE IMO clock values in MHz */
#define CY_CAPSENSE_IMO_CLK_25_MHZ                      (25u)
#define CY_CAPSENSE_IMO_CLK_38_MHZ                      (38u)
#define CY_CAPSENSE_IMO_CLK_46_MHZ                      (46u)

#if defined(CY_IP_M0S8MSCV3LP)
    /* CAPSENSE MRSS IMO frequency value */
    #if (CY_CAPSENSE_IMO_FREQUENCY == CY_CAPSENSE_IMO_25_MHZ)
        #define CY_CAPSENSE_MOD_CLOCK_MHZ   CY_CAPSENSE_IMO_CLK_25_MHZ
    #elif (CY_CAPSENSE_IMO_FREQUENCY == CY_CAPSENSE_IMO_38_MHZ)
        #define CY_CAPSENSE_MOD_CLOCK_MHZ   CY_CAPSENSE_IMO_CLK_38_MHZ
    #else /* (CY_CAPSENSE_IMO_FREQUENCY == CY_CAPSENSE_IMO_46_MHZ) */
        #define CY_CAPSENSE_MOD_CLOCK_MHZ   CY_CAPSENSE_IMO_CLK_46_MHZ
    #endif
#endif /* CY_IP_M0S8MSCV3LP */

#if defined(__cplusplus)
}
#endif

#endif /* (defined(CY_IP_MXCSDV2) || defined(CY_IP_M0S8CSDV2) || defined(CY_IP_M0S8MSCV3) || defined(CY_IP_M0S8MSCV3LP)) */

#endif /* CY_CAPSENSE_COMMON_H */


/* [] END OF FILE */
