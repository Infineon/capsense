/***************************************************************************//**
* \file cy_capsense_sm_base_full_wave_lp.h
* \version 5.0
*
* \brief
* This file lists a set of macros for each register bit-field for the specified
* sensing method.
*
********************************************************************************
* \copyright
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CAPSENSE_SM_BASE_FULL_WAVE_LP_H)
#define CY_CAPSENSE_SM_BASE_FULL_WAVE_LP_H

#if defined(CY_IP_M0S8MSCV3LP)

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* \note 23WW17.2
*******************************************************************************/

/*******************************************************************************
* Base HW block configuration
*******************************************************************************/

/* CTL: Configuration and Control */
#define CY_CAPSENSE_SM_REG_CTL_FLD_SENSE_EN                                 (0x1uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_MSCCMP_EN                                (0x1uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_CLK_SYNC_EN                              (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_EXT_FRAME_START_MODE                     (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_CFG_OFFSET                               (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_OPERATING_MODE                           (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_BUF_MODE                                 (0x1uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_CLK_MSC_RATIO                            (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_DEBUG_EN                                 (0x0uL)
#define CY_CAPSENSE_SM_REG_CTL_FLD_ENABLED                                  (0x0uL)

/* SCAN_CTL1: Scan Control 1 */
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_NUM_AUTO_RESAMPLE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_RESCAN_DEBUG_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_NUM_SAMPLES                        (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_RAW_COUNT_MODE                     (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_DEBUG_CONV_PH_SEL                  (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_FRAME_RES_START_ADDR               (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_RC_STORE_EN                        (0x1uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL1_FLD_RC_STORE_MODE                      (0x0uL)

/* SCAN_CTL2: Scan Control 2 */
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY_PRS             (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY                 (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_CHOP_POL                           (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_CHOP_EVEN_HOLD_EN                  (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_FRAME_CFG_START_ADDR               (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_EXT_REF_CLK_EN                     (0x0uL)
#define CY_CAPSENSE_SM_REG_SCAN_CTL2_FLD_INFINITE_SCAN_MODE                 (0x0uL)

/* INIT_CTL1: Initialization Control 1 */
#define CY_CAPSENSE_SM_REG_INIT_CTL1_FLD_NUM_INIT_CMOD_12_RAIL_CYCLES       (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL1_FLD_NUM_INIT_CMOD_12_SHORT_CYCLES      (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL1_FLD_PER_SAMPLE                         (0x0uL)

/* INIT_CTL3: Initialization Control 3 */
#define CY_CAPSENSE_SM_REG_INIT_CTL3_FLD_NUM_PRO_OFFSET_CYCLES              (0x31uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL3_FLD_NUM_PRO_OFFSET_TRIPS               (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL3_FLD_CMOD_SEL                           (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL3_FLD_INIT_MODE                          (0x0uL)

/* INIT_CTL4: Initialization Control 4 */
#define CY_CAPSENSE_SM_REG_INIT_CTL4_FLD_NUM_PRO_DUMMY_SUB_CONVS            (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL4_FLD_NUM_PRO_WAIT_KREF_DELAY_PRS        (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL4_FLD_NUM_PRO_WAIT_KREF_DELAY            (0x0uL)
#define CY_CAPSENSE_SM_REG_INIT_CTL4_FLD_PRO_BYPASS                         (0x0uL)

/* SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_SENSE_DUTY_CTL_FLD_PHASE_WIDTH                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_CYCLES            (0x1uL)
#define CY_CAPSENSE_SM_REG_SENSE_DUTY_CTL_FLD_PHASE_WIDTH_SEL               (0x0uL)

/* SENSE_PERIOD_CTL: Sense Clock Period Control */
#define CY_CAPSENSE_SM_REG_SENSE_PERIOD_CTL_FLD_LFSR_POLY                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SENSE_PERIOD_CTL_FLD_LFSR_SCALE                  (0x0uL)

/* FILTER_CTL: Filter Control */
#define CY_CAPSENSE_SM_REG_FILTER_CTL_FLD_BIT_FORMAT                        (0x0uL)
#define CY_CAPSENSE_SM_REG_FILTER_CTL_FLD_FILTER_MODE                       (0x0uL)

/* CCOMP_CDAC_CTL: Compensation CAPDAC Control */
#define CY_CAPSENSE_SM_REG_CCOMP_CDAC_CTL_FLD_SEL_CO_PRO_OFFSET             (0xFuL)
#define CY_CAPSENSE_SM_REG_CCOMP_CDAC_CTL_FLD_COMP_BLANKING_MODE            (0x0uL)
#define CY_CAPSENSE_SM_REG_CCOMP_CDAC_CTL_FLD_EPILOGUE_EN                   (0x0uL)

/* DITHER_CDAC_CTL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_DITHER_CDAC_CTL_FLD_SEL_FL                       (0x0uL)
#define CY_CAPSENSE_SM_REG_DITHER_CDAC_CTL_FLD_LFSR_POLY_FL                 (0x0uL)

/* MSCCMP_CTL: MSC Comparator Control */
#define CY_CAPSENSE_SM_REG_MSCCMP_CTL_FLD_PWR                               (0x1uL)
#define CY_CAPSENSE_SM_REG_MSCCMP_CTL_FLD_FILT                              (0x1uL)

/* AOS_CTL: Always On Scanning Control */
#define CY_CAPSENSE_SM_REG_AOS_CTL_FLD_WAKEUP_TIMER                         (0x0uL)
#define CY_CAPSENSE_SM_REG_AOS_CTL_FLD_FR_TIMEOUT_INTERVAL                  (0x0uL)
#define CY_CAPSENSE_SM_REG_AOS_CTL_FLD_STOP_ON_SD                           (0x0uL)
#define CY_CAPSENSE_SM_REG_AOS_CTL_FLD_MRSS_PWR_CYCLE_EN                    (0x0uL)

/* CE_CTL: Channel Engine Control */
#define CY_CAPSENSE_SM_REG_CE_CTL_FLD_RCF_EN                                (0x0uL)
#define CY_CAPSENSE_SM_REG_CE_CTL_FLD_BLSD_EN                               (0x0uL)
#define CY_CAPSENSE_SM_REG_CE_CTL_FLD_CE_TEST_MODE                          (0x0uL)
#define CY_CAPSENSE_SM_REG_CE_CTL_FLD_ENABLED                               (0x0uL)

/* PUMP_CTL: Local MRSS Pump Control */
#define CY_CAPSENSE_SM_REG_PUMP_CTL_FLD_PUMP_MODE                           (0x0uL)

/* IMO_CTL: Local MRSS IMO Control */
#define CY_CAPSENSE_SM_REG_IMO_CTL_FLD_FREQ                                 (0x0uL)
#define CY_CAPSENSE_SM_REG_IMO_CTL_FLD_CLOCK_SYNC_DIV                       (0x0uL)
#define CY_CAPSENSE_SM_REG_IMO_CTL_FLD_CLOCK_MSC_DIV                        (0x0uL)

/* INTR: MSCv3 Interrupt Cause Register */
#define CY_CAPSENSE_SM_REG_INTR_FLD_SUB_SAMPLE                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_SAMPLE                                  (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_SCAN                                    (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_INIT                                    (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_FRAME                                   (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_CIC2_ERROR                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_CONFIG_REQ                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_FIFO_UNDERFLOW                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_FLD_FIFO_OVERFLOW                           (0x0uL)

/* INTR_SET: MSCv3 Interrupt Set Register */
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_SUB_SAMPLE                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_SAMPLE                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_SCAN                                (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_INIT                                (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_FRAME                               (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_CIC2_ERROR                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_CONFIG_REQ                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_FIFO_UNDERFLOW                      (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_SET_FLD_FIFO_OVERFLOW                       (0x0uL)

/* INTR_LP: Low Power Interrupt Cause Register */
#define CY_CAPSENSE_SM_REG_INTR_LP_FLD_SIG_DET                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_FLD_FR_TIMEOUT                           (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_FLD_FRAME                                (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_FLD_CE_DONE                              (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_FLD_IMO_UP                               (0x0uL)

/* INTR_LP_SET: Low Power Interrupt Set Register */
#define CY_CAPSENSE_SM_REG_INTR_LP_SET_FLD_SIG_DET                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_SET_FLD_FR_TIMEOUT                       (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_SET_FLD_FRAME                            (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_SET_FLD_CE_DONE                          (0x0uL)
#define CY_CAPSENSE_SM_REG_INTR_LP_SET_FLD_IMO_UP                           (0x0uL)

/* WAKEUP_CMD: Wakeup Command Register */
#define CY_CAPSENSE_SM_REG_WAKEUP_CMD_FLD_START_FRAME_AOS                   (0x0uL)

/* MRSS_CMD: MRSS Command Register */
#define CY_CAPSENSE_SM_REG_MRSS_CMD_FLD_MRSS_START                          (0x0uL)
#define CY_CAPSENSE_SM_REG_MRSS_CMD_FLD_MRSS_STOP                           (0x0uL)
#define CY_CAPSENSE_SM_REG_MRSS_CMD_FLD_MRSS_PUMP_STOP                      (0x0uL)

/* SW_SEL_GPIO: GPIO Switch Control */
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_CSD_SENSE                     (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_CSD_SHIELD                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_CSD_MUTUAL                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_CSD_POLARITY                  (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_CSD_CHARGE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_DSI_CMOD                      (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_GPIO_FLD_SW_DSI_CSH_TANK                  (0x0uL)

/* SW_SEL_CDAC_RE: Reference CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETCC                      (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RECD                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETV                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_RETG                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_REBV                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_RE_FLD_SW_REBG                       (0x3uL)

/* SW_SEL_CDAC_CO: Compensation CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COTCA                      (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COCB                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COTV                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COTG                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COBV                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CO_FLD_SW_COBG                       (0x3uL)

/* SW_SEL_CDAC_CF: Fine CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFTCA                      (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFTCB                      (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFTV                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFTG                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFBV                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CDAC_CF_FLD_SW_CFBG                       (0x3uL)

/* SW_SEL_BGR: Bandgap Reference Switch Control */
#define CY_CAPSENSE_SM_REG_SW_SEL_BGR_FLD_SW_BGRCM                          (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_BGR_FLD_SW_IGMA                           (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_BGR_FLD_SW_BGRMA                          (0x0uL)

/* CSW_CTL_LO: CapSense Sensor Switch Control Low */
#define CY_CAPSENSE_SM_REG_CSW_CTL_LO_FLD_CSW_FUNC_MODE                     (0x0uL)

/* CSW_CTL_HI: CapSense Sensor Switch Control High */
#define CY_CAPSENSE_SM_REG_CSW_CTL_HI_FLD_CSW_FUNC_MODE                     (0x0uL)

/* DFT_CTL Register: DFT control register */
#define CY_CAPSENSE_SM_REG_DFT_CTL_FLD_DDFT_SEL                             (0x0uL)

/*******************************************************************************
* Use default trim bits from SFLASH. Do not override trim registers
*******************************************************************************/

/* TRIM_CTL: Trim Control */
#define CY_CAPSENSE_SM_REG_TRIM_CTL_FLD_TRIM_IN                             (0x0uL)
#define CY_CAPSENSE_SM_REG_TRIM_CTL_FLD_TRIM_EN                             (0x0uL)
#define CY_CAPSENSE_SM_REG_TRIM_CTL_FLD_TRIM_POLARITY                       (0x0uL)

/* CLK_IMO_TRIM1: Local IMO Trim Register 1 */
#define CY_CAPSENSE_SM_REG_CLK_IMO_TRIM1_FLD_OFFSET                         (0x0uL)

/* CLK_IMO_TRIM2: Local IMO Trim Register 2 */
#define CY_CAPSENSE_SM_REG_CLK_IMO_TRIM2_FLD_FSOFFSET                       (0x0uL)

/* CLK_IMO_TRIM3: Local IMO Trim Register 3 */
#define CY_CAPSENSE_SM_REG_CLK_IMO_TRIM3_FLD_TCTRIM                         (0x0uL)

/* PWR_BG_TRIM1: Bandgap Trim Register 1 */
#define CY_CAPSENSE_SM_REG_PWR_BG_TRIM1_FLD_REF_VTRIM                       (0x0uL)

/* PWR_BG_TRIM2: Bandgap Trim Register 2 */
#define CY_CAPSENSE_SM_REG_PWR_BG_TRIM2_FLD_REF_ITRIM                       (0x0uL)

/* PWR_BG_TRIM3: Bandgap Trim Register 3 */
#define CY_CAPSENSE_SM_REG_PWR_BG_TRIM3_FLD_REF_TCTRIM                      (0x0uL)

/*******************************************************************************
* Sensor HW configuration
*******************************************************************************/

/* SNS_SNS_LP_AOS_SNS_CTL0: LP-AoS Sensor Control 0 */
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL0_FLD_RC_COEFF             (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL0_FLD_BL_COEFF_SLOW        (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL0_FLD_BL_COEFF_FAST        (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL0_FLD_LOW_BL_RESET         (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL0_FLD_BL_UPDATE_DELAY      (0x0uL)

/* SNS_SNS_LP_AOS_SNS_CTL1: LP-AoS Sensor Control 1 */
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL1_FLD_NOISE_THR            (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL1_FLD_NOISE_THR_NEG        (0x0uL)

/* SNS_SNS_LP_AOS_SNS_CTL2: LP-AoS Sensor Control 2 */
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL2_FLD_SIGNAL_THR           (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL2_FLD_DEBOUNCE_THRESHOLD   (0x3uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL2_FLD_SIGNAL_TYPE          (0x0uL)

/* SNS_SNS_LP_AOS_SNS_CTL3: LP-AoS Sensor Control 3 */
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL3_FLD_SNS_FRC_SCALED       (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL3_FLD_BL_UPDATE_TMR        (0x10uL)

/* SNS_SNS_LP_AOS_SNS_CTL4: LP-AoS Sensor Control 4 */
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL4_FLD_SNS_BL               (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL4_FLD_BL_RESET_TMR         (0x30uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_LP_AOS_SNS_CTL4_FLD_SIG_DEBOUNCE_TMR     (0x3uL)

/* SNS_SNS_SCAN_CTL: Sensor Scan Control */
#define CY_CAPSENSE_SM_REG_SNS_SNS_SCAN_CTL_FLD_NUM_SUB_CONVS               (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_SCAN_CTL_FLD_COMP_DIV                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_SCAN_CTL_FLD_NUM_CONV                    (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_SCAN_CTL_FLD_INIT_BYPASS                 (0x0uL)

/* SNS_SNS_CDAC_CTL: Sensor CAPDAC Control */
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_SEL_RE                      (0x3euL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_SEL_CO                      (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_SEL_CF                      (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_FINE_MODE                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_CLOCK_REF_RATE              (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_FL_MODE                     (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_COMP_BLANKING_EN            (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_LFSR_SCALE_FL               (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CDAC_CTL_FLD_LFSR_SCALE_TYPE_FL          (0x0uL)

/* SNS_SNS_CTL: Sense Control and Command Register */
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_START_SCAN                       (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_VALID                            (0x1uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_LAST                             (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_MULTI_CH_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_SENSE_MODE_SEL                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_DECIM_RATE                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_SENSE_DIV                        (0x2FuL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_LFSR_MODE                        (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_SNS_CTL_FLD_LFSR_BITS                        (0x0uL)

/* SNS_FRAME_CMD: Frame Command Register */
#define CY_CAPSENSE_SM_REG_SNS_FRAME_CMD_FLD_START_FRAME                    (0x0uL)

/* SNS_CE_CMD: Channel Engine Command */
#define CY_CAPSENSE_SM_REG_SNS_CE_CMD_FLD_TEST_RAW_COUNT                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SNS_CE_CMD_FLD_CE_START                          (0x0uL)

/* SNS_FIFO_CMD: FIFO Command */
#define CY_CAPSENSE_SM_REG_SNS_FIFO_CMD_FLD_FIFO_RESET                      (0x0uL)

/* SNS_CE_INIT_CTL: Channel Engine Initialization Control */
#define CY_CAPSENSE_SM_REG_SNS_CE_INIT_CTL_FLD_SENSOR_INIT                  (0x0uL)

/*******************************************************************************
* Sensing method specific registers called MODE.
* They corresponds to MODE_STRUCTLP tab.
* Consists 7 registers per each supported mode:
* * MODE0: CSD RM
* * MODE1: CSX RM
* * MODE2: ISX RM (external VDDA/2)
* * MODE3: CSD RM with CapDAC dithering
* * MODE4: CSX RM with CapDAC dithering
* * MODE5: ISX RM with CapDAC dithering (external VDDA/2)
* * MODE6: CSD RM + ACTIVE SHIELD
* * MODE7: CSD RM with CapDAC dithering + ACTIVE SHIELD
* * MODE8: CSD RM + PASIVE SHIELD
* * MODE9: CSD RM with CapDAC dithering + PASIVE SHIELD
* * MODE10: CSD MPSC-C or  CSD MPSC-D
* * MODE11: CSD MPSC-C or CSD MPSC-D with CapDAC dithering(Dithering only wrt CsP)
* * MODE12: CMOD/VDDA BIST Balancing Phase. Balancing to VDDA/2
* * MODE13: ISX (internal VDDA/2)
* * MODE14: ISX RM with CapDAC dithering (internal VDDA/2)
*******************************************************************************/

/********************************* CSD RM *************************************/

/* CSD RM - MODE0_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM - MODE0_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM - MODE0_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM - MODE0_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM - MODE0_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM - MODE0_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM - MODE0_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE0_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************************* CSX RM *************************************/

/* CSX RM - MODE1_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x1uL)

/* CSX RM - MODE1_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSX RM - MODE1_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSX RM - MODE1_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSX RM - MODE1_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSX RM - MODE1_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSX RM - MODE1_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE1_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************************* ISX RM (External VDDA/2) *******************/

/* ISX RM - MODE2_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* ISX RM - MODE2_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* ISX RM - MODE2_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* ISX RM - MODE2_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* ISX RM - MODE2_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* ISX RM - MODE2_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* ISX RM - MODE2_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE2_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************* CSD RM with CapDAC (With Dithering) ***************************/

/* CSD RM w/dither - MODE3_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM w/dither - MODE3_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x6uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM w/dither - MODE3_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM w/dither - MODE3_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM w/dither - MODE3_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM w/dither - MODE3_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM w/dither - MODE3_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE3_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************* CSX RM with CapDAC dithering ***************************/

/* CSX RM w/dither - MODE4_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x1uL)

/* CSX RM w/dither - MODE4_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSX RM w/dither - MODE4_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSX RM w/dither - MODE4_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSX RM w/dither - MODE4_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSX RM w/dither - MODE4_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSX RM w/dither - MODE4_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE4_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************* ISX RM with CapDAC dithering ***************************/

/* ISX RM w/dither - MODE5_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* ISX RM w/dither - MODE5_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x5uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* ISX RM w/dither - MODE5_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* ISX RM w/dither - MODE5_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* ISX RM w/dither - MODE5_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* ISX RM w/dither - MODE5_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* ISX RM w/dither - MODE5_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE5_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*********************  MODE6: CSD RM + ACTIVE SHIELD ***************************/

/* CSD RM + ACTIVE SHIELD - MODE6_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_SOMB                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM + ACTIVE SHIELD - MODE6_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE6_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*********************  MODE7: CSD RM with CapDAC dithering + ACTIVE SHIELD ***************************/

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x6uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_SOMB                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM w/dither + ACTIVE SHIELD - MODE7_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE7_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*********************  MODE8: CSD RM + PASIVE SHIELD ***************************/

/* CSD RM + PASIVE SHIELD - MODE8_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM + PASIVE SHIELD - MODE8_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE8_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*********************  MODE9: CSD RM with CapDAC dithering + PASIVE SHIELD ***************************/

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x6uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD RM w/dither + PASIVE SHIELD - MODE9_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE9_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/********************************* MODE10: CSD MPSC-C or CSD MPSC-D *************************************/

/* CSD MPSC-C or CSD MPSC-D - MODE10_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* CSD MPSC-C or CSD MPSC-D - MODE10_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE10_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/***************************** MODE11: CSD MPSC-C or CSD MPSC-D with CapDAC dithering *********************************/

/* CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x2uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x6uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/*  CSD MPSC-C or CSD MPSC-D w/dither - MODE11_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE11_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*******************************************************************************
* Specific Base HW block configuration for BIST Cmod and VDDA measurement
* The HW configuration for CMOD/VDDA BIST Balancing Phase
* In Precharge Phase CMOD must be put to GROUND
*******************************************************************************/

/* SCAN_CTL2: Scan Control 2 -CMOD/VDDA BIST */
#define CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY_PRS (0x1uL)
#define CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_NUM_EPI_KREF_DELAY (0x1uL)
#define CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_REG_SCAN_CTL2_FLD_CHOP_POL           (0x1uL)

/* IMO_CTL: Local MRSS IMO Control */
#define CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_IMO_CTL_FREQ                         (0x6uL)
#define CY_CAPSENSE_BIST_CMOD_VDDA_MEAS_IMO_CTL_CLOCK_MSC_DIV                (0x3uL)

/* CMOD/VDDA BIST Balancing Phase; balance to VDDA/2 - MODE12_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_CACC                        (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_CBCD                        (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYA_CTL                     (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYA_EN                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* CMOD/VDDA BIST Balancing Phase; balance to VDDA/2 - MODE12_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPCS1                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPMA                       (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMCS2                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_CMF                        (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* CMOD/VDDA BIST Balancing Phase; balance to VDDA/2 - MODE12_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_SOMB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_CBSO                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE12_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* SW_SEL_BGR: Bandgap Reference Switch Control */
#define CY_CAPSENSE_BIST_CMOD_MEAS_SW_SEL_BGR_FLD_SW_BGRCM                   (0x1uL)

/********************************* ISX RM (Internal VDDA/2) *******************/

/* ISX RM - MODE13_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* ISX RM - MODE13_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* ISX RM - MODE13_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* ISX RM - MODE13_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* ISX RM - MODE13_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_SOMB                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* ISX RM - MODE13_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* ISX RM - MODE13_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE13_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/************************ ISX RM with CapDAC dithering (internal VDDA/2) ******/

/* ISX RM - MODE14_SENSE_DUTY_CTL: Sense Clock Duty Cycle Control */
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH2_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH3_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH0_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_FS2_PH1_EN    (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PH_GAP_2CYCLE_EN        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH0X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_GAP_PH1X_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHX_GAP_2CYCLE_EN       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_SHIFT_EN          (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SENSE_DUTY_CTL_FLD_PHASE_MODE_SEL          (0x0uL)

/* ISX RM - MODE14_SW_SEL_CDAC_FL: Flatspot/Dither CAPDAC Switch Control */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLTCA                (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLCB                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLTV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLTG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLBV                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_SW_FLBG                 (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CDAC_FL_FLD_ACTIVATION_MODE         (0x0uL)

/* ISX RM - MODE14_SW_SEL_TOP: Top Level Switch Control */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_CACB                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_CACC                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_CBCD                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_AYA_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_AYA_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_AYB_CTL                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_AYB_EN                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_BYB                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_BGRF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_RMF                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_TOP_FLD_MBF                         (0x0uL)

/* ISX RM - MODE14_SW_SEL_COMP: MSC Comparator Switch Control */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPCS1                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPCS3                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPMA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPCA                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMCB                       (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CPF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMCS2                      (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMCS4                      (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMV                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMG                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_CMF                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_COMP_FLD_HALF_WAVE_EN               (0x0uL)

/* ISX RM - MODE14_SW_SEL_SH: Shielding Switch Control */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_SOMB                         (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_CBSO                         (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_SPCS1                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_SPCS3                        (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_FSP                          (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_BUF_SEL                      (0x7uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_SH_FLD_BUF_EN                       (0x1uL)

/* ISX RM - MODE14_SW_SEL_CMOD1: CMOD Switch Control 1 */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_AMUXA                  (0x3uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_C1CA                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_C1CC                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD1_FLD_DDRV_EN                   (0x0uL)

/* ISX RM - MODE14_SW_SEL_CMOD2: CMOD Switch Control 2 */
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_AMUXA                  (0x4uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_AMUXB                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_C2CB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_C2CD                   (0x1uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_PU                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_SW_PD                     (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_REF_MODE                  (0x0uL)
#define CY_CAPSENSE_SM_REG_MODE14_SW_SEL_CMOD2_FLD_DDRV_EN                   (0x0uL)

/*******************************************************
* Pin state specific registers. One register per state:
* * CSW_FUNC0: GND
* * CSW_FUNC1: HIGH-Z
* * CSW_FUNC2: CSX RX
* * CSW_FUNC3: CSX TX
* * CSW_FUNC4: CSX NTX
* * CSW_FUNC5: CSD SNS
* * CSW_FUNC6: ISX LX (two-pin cfg)
* * CSW_FUNC7: ISX RX (two-pin cfg, external VDDA/2)
* * CSW_FUNC8: SHIELD Active
* * CSW_FUNC9: SHIELD Passive
* * CSW_FUNC10: CSX VDDA/2
* * CSW_FUNC11: MPSC CSP
* * CSW_FUNC12: MPSC CSN
* * CSW_FUNC13: MPSC CSZ
* * CSW_FUNC14: ISX RX (two-pin cfg, internal VDDA/2)
* * CSW_FUNC15: ISX RX (one-pin cfg, external VDDA/2)
* * CSW_FUNC16: ISX RX (one-pin cfg, internal VDDA/2)
*
************************************************************************/

/*********** SW_SEL_CSW_FUNC0: GND *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_SW_PU                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_SW_PD                       (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_REF_MODE                    (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC0_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC1: High-Z *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_SW_PU                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_SW_PD                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_REF_MODE                    (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC1_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC2: CSX RX *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_SW_AMUXA                    (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_SW_PU                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_SW_PD                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC2_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC3: CSX TX *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_SW_PU                       (0x6uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_SW_PD                       (0x8uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC3_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC4: CSX TX_N *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_SW_PU                       (0x7uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_SW_PD                       (0x7uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC4_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC5: CSD *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_SW_AMUXA                    (0xauL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_SW_PU                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_SW_PD                       (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC5_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC6: ISX LX (two-pin cfg) *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_SW_PU                       (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_SW_PD                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC6_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC7: ISX RX (two-pin cfg, external VDDA/2) */
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_SW_AMUXA                    (0x6uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_SW_AMUXB                    (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_SW_PU                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_SW_PD                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC7_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC8: CSD SHIELD Active *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_SW_AMUXB                    (0xauL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_SW_PU                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_SW_PD                       (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC8_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC9: CSD SHIELD Passive *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_SW_AMUXA                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_SW_PU                       (0x8uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_SW_PD                       (0x9uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC9_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC10: CSX VDDA/2 *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_SW_AMUXA                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_SW_AMUXB                   (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_SW_PU                      (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_SW_PD                      (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_REF_MODE                   (0x1uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC10_FLD_DDRV_EN                    (0x0uL)

/*********** SW_SEL_CSW_FUNC11: MPSC-D Positive *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_SW_AMUXA                   (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_SW_AMUXB                   (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_SW_PU                      (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_SW_PD                      (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_REF_MODE                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC11_FLD_DDRV_EN                    (0x0uL)

/*********** SW_SEL_CSW_FUNC12: MPSC-D Negative *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_SW_AMUXA                   (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_SW_AMUXB                   (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_SW_PU                      (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_SW_PD                      (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_REF_MODE                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC12_FLD_DDRV_EN                    (0x0uL)

/*********** SW_SEL_CSW_FUNC13: MPSC-D Zero *************/
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_SW_AMUXA                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_SW_AMUXB                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_SW_PU                      (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_SW_PD                      (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_REF_MODE                   (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC13_FLD_DDRV_EN                    (0x0uL)

/*********** SW_SEL_CSW_FUNC14: ISX RX (two-pin cfg, internal VDDA/2) */
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_SW_AMUXA                    (0xAuL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_SW_PU                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_SW_PD                       (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC14_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC15: ISX RX (one-pin cfg, external VDDA/2) */
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_SW_AMUXA                    (0x6uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_SW_AMUXB                    (0x3uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_SW_PU                       (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_SW_PD                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC15_FLD_DDRV_EN                     (0x0uL)

/*********** SW_SEL_CSW_FUNC16: ISX RX (one-pin cfg, internal VDDA/2) */
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_SW_AMUXA                    (0xAuL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_SW_AMUXB                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_SW_PU                       (0x4uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_SW_PD                       (0x2uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_REF_MODE                    (0x0uL)
#define CY_CAPSENSE_SM_REG_SW_SEL_CSW_FUNC16_FLD_DDRV_EN                     (0x0uL)


#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_M0S8MSCV3LP */

#endif /* CY_CAPSENSE_SM_BASE_FULL_WAVE_LP_H */


/* [] END OF FILE */
