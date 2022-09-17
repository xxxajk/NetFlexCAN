/*
 * File:    kinetis_flexcan.h
 * Purpose: Register and bit definitions
 */

#ifndef __KINETIS_NFLEXCAN_H
#define __KINETIS_NFLEXCAN_H

#include <stdint.h>

#define BITNUMBER(x) (1LU<<x)

#if defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#define INCLUDE_FLEXCAN_CAN1
#if defined(__IMXRT1062__)
#define INCLUDE_FLEXCAN_CAN2
#endif
#endif

/* FlexCAN module I/O Base Addresss, these seem to be in the same local on Kinetis K/KE1x */
#if defined(__IMXRT1062__)
#define                              NFLEXCAN0_BASE IMXRT_FLEXCAN1_ADDRESS
#define                              NFLEXCAN1_BASE IMXRT_FLEXCAN2_ADDRESS
#define                              NFLEXCAN2_BASE IMXRT_FLEXCAN3_ADDRESS

#else
#define                              NFLEXCAN0_BASE (0x40024000LU)
#define                              NFLEXCAN1_BASE (0x400A4000LU)
#endif

#define                             NFLEXCANb_MCR(b) (*(volatile uint32_t*)(b))
#define                           NFLEXCANb_CTRL1(b) (*(volatile uint32_t*)(b+4))
#define                           NFLEXCANb_CTRL2(b) (*(volatile uint32_t*)(b+0x34))
#define                        NFLEXCANb_RXMGMASK(b) (*(volatile uint32_t*)(b+0x10))
#define                          NFLEXCANb_IFLAG1(b) (*(volatile uint32_t*)(b+0x30))
#define                          NFLEXCANb_IMASK1(b) (*(volatile uint32_t*)(b+0x28))
#define                        NFLEXCANb_RXFGMASK(b) (*(volatile uint32_t*)(b+0x48))
#define                       NFLEXCANb_MBn_CS(b, n) (*(volatile uint32_t*)(b+0x80+n*0x10))
#define                       NFLEXCANb_MBn_ID(b, n) (*(volatile uint32_t*)(b+0x84+n*0x10))
#define                    NFLEXCANb_MBn_WORD0(b, n) (*(volatile uint32_t*)(b+0x88+n*0x10))
#define                    NFLEXCANb_MBn_WORD1(b, n) (*(volatile uint32_t*)(b+0x8C+n*0x10))
#define                    NFLEXCANb_IDFLT_TAB(b, n) (*(volatile uint32_t*)(b+0xE0+(n*4)))
#define                      NFLEXCANb_MB_MASK(b, n) (*(volatile uint32_t*)(b+0x880+(n*4)))
#define                            NFLEXCANb_ESR1(b) (*(volatile uint32_t*)(b+0x20))

/* Bit definitions and macros for FLEXCAN_MCR */
#define                       NFLEXCAN_MCR_MAXMB(x) ((x)&0x0000007FLU))
#define                        NFLEXCAN_MCR_IDAM(x) (((x)&0x00000003LU)<<8)
#define                     NFLEXCAN_MCR_MAXMB_MASK (0x0000007FLU)
#define                      NFLEXCAN_MCR_IDAM_MASK (0x00000300LU)
#define                    NFLEXCAN_MCR_IDAM_BIT_NO (8)
#define                            NFLEXCAN_MCR_AEN BITNUMBER(12)
#define                       NFLEXCAN_MCR_LPRIO_EN BITNUMBER(13)
#define                           NFLEXCAN_MCR_IRMQ BITNUMBER(16)
#define                        NFLEXCAN_MCR_SRX_DIS BITNUMBER(17)
#define                           NFLEXCAN_MCR_DOZE BITNUMBER(18)
#define                        NFLEXCAN_MCR_WAK_SRC BITNUMBER(19)
#define                        NFLEXCAN_MCR_LPM_ACK BITNUMBER(20)
#define                         NFLEXCAN_MCR_WRN_EN BITNUMBER(21)
#define                        NFLEXCAN_MCR_SLF_WAK BITNUMBER(22)
#define                           NFLEXCAN_MCR_SUPV BITNUMBER(23)
#define                        NFLEXCAN_MCR_FRZ_ACK BITNUMBER(24)
#define                       NFLEXCAN_MCR_SOFT_RST BITNUMBER(25)
#define                        NFLEXCAN_MCR_WAK_MSK BITNUMBER(26)
#define                        NFLEXCAN_MCR_NOT_RDY BITNUMBER(27)
#define                           NFLEXCAN_MCR_HALT BITNUMBER(28)
#define                            NFLEXCAN_MCR_FEN BITNUMBER(29)
#define                            NFLEXCAN_MCR_FRZ BITNUMBER(30)
#define                           NFLEXCAN_MCR_MDIS BITNUMBER(31)
// i.MX RT
//#define                           FLEXCAN_MCR_FDEN BITNUMBER(11) // CAN FD operation enable this is only on CAN3
// The Legacy Rx FIFO Enable (FEN) bit cannot be set if FDEN is asserted.

/* Bit definitions and macros for FLEXCAN_CTRL */
#define                    NFLEXCAN_CTRL_PROPSEG(x) ((x)&0x00000007LU)
#define                           NFLEXCAN_CTRL_LOM BITNUMBER(3)
#define                          NFLEXCAN_CTRL_LBUF BITNUMBER(4)
#define                         NFLEXCAN_CTRL_TSYNC BITNUMBER(5)
#define                      NFLEXCAN_CTRL_BOFF_REC BITNUMBER(6)
#define                           NFLEXCAN_CTRL_SMP BITNUMBER(7)
#define                      NFLEXCAN_CTRL_RWRN_MSK BITNUMBER(10)
#define                      NFLEXCAN_CTRL_TWRN_MSK BITNUMBER(11)
#define                           NFLEXCAN_CTRL_LPB BITNUMBER(12)
#define                       NFLEXCAN_CTRL_CLK_SRC BITNUMBER(13)
#define                       NFLEXCAN_CTRL_ERR_MSK BITNUMBER(14)
#define                      NFLEXCAN_CTRL_BOFF_MSK BITNUMBER(15)
#define                      NFLEXCAN_CTRL_PSEG2(x) (((x)&0x00000007LU)<<16)
#define                      NFLEXCAN_CTRL_PSEG1(x) (((x)&0x00000007LU)<<19)
#define                        NFLEXCAN_CTRL_RJW(x) (((x)&0x00000003LU)<<22)
#define                    NFLEXCAN_CTRL_PRESDIV(x) (((x)&0x000000FFLU)<<24)

/* Bit definitions and macros for FLEXCAN_CTRL2 */
#define                       NFLEXCAN_CTRL2_IMEUEN (BITNUMBER(31))
#define                         NFLEXCAN_CTRL2_RFFN (0x0F000000LU)
#define                  NFLEXCAN_CTRL2_RFFN_BIT_NO (24)
#define                         NFLEXCAN_CTRL2_TASD (0x00F80000LU)
#define                  NFLEXCAN_CTRL2_TASD_BIT_NO (19)
#define                          NFLEXCAN_CTRL2_MRP (BITNUMBER(18))
#define                          NFLEXCAN_CTRL2_RRS (BITNUMBER(17))
#define                        NFLEXCAN_CTRL2_EACEN (BITNUMBER(16))
#define                       NFLEXCAN_CTRL2_MUMASK (BITNUMBER(1))
#define                       NFLEXCAN_CTRL2_FUMASK (BITNUMBER(0))
#define                    NFLEXCAN_CTRL2_LOSTRLMSK (BITNUMBER(2))
#define                    NFLEXCAN_CTRL2_LOSTRMMSK (BITNUMBER(1))
#define                     NFLEXCAN_CTRL2_IMEUMASK (BITNUMBER(0))
#define               NFLEXCAN_set_rffn(ctrl2,rffn) ctrl2 = ((ctrl2) & ~NFLEXCAN_CTRL2_RFFN) | ((rffn & 0xF)<<NFLEXCAN_CTRL2_RFFN_BIT_NO)
#define                 NFLEXCAN_set_TASD(ctrl2, x) ctrl2 = ((ctrl2) & ~NFLEXCAN_CTRL2_TASD) | (x & 0x0000001FLU << NFLEXCAN_CTRL2_TASD_BIT_NO)
#if defined(__MKE16F512VLH16__) || defined(__MKE16F256VLH16__)
#define             NFLEXCAN_CTRL2_ISOCANFDENBIT_NO (BITNUMBER(12))
#define NFLEXCAN_ISOCANFD
#endif

/* Bit definitions and macros for FLEXCAN_ESR1 */
#define                        NFLEXCAN_ESR_WAK_INT BITNUMBER(0)
#define                        NFLEXCAN_ESR_ERR_INT BITNUMBER(1)
#define                       NFLEXCAN_ESR_BOFF_INT BITNUMBER(2)
#define                             NFLEXCAN_ESR_RX BITNUMBER(3)
#define                    NFLEXCAN_ESR_FLT_CONF(x) (((x)&0x00000003LU)<<4)
#define                  NFLEXCAN_ESR_FLT_CONF_MASK (0x00000030LU)
#define                             NFLEXCAN_ESR_TX BITNUMBER(6)
#define                           NFLEXCAN_ESR_IDLE BITNUMBER(7)
#define                         NFLEXCAN_ESR_RX_WRN BITNUMBER(8)
#define                         NFLEXCAN_ESR_TX_WRN BITNUMBER(9)
#define                        NFLEXCAN_ESR_STF_ERR BITNUMBER(10)
#define                        NFLEXCAN_ESR_FRM_ERR BITNUMBER(11)
#define                        NFLEXCAN_ESR_CRC_ERR BITNUMBER(12)
#define                        NFLEXCAN_ESR_ACK_ERR BITNUMBER(13)
#define                       NFLEXCAN_ESR_BIT0_ERR BITNUMBER(14)
#define                       NFLEXCAN_ESR_BIT1_ERR BITNUMBER(15)
#define                       NFLEXCAN_ESR_RWRN_INT BITNUMBER(16)
#define                       NFLEXCAN_ESR_TWRN_INT BITNUMBER(17)
#define            NFLEXCAN_ESR_get_fault_code(esr) (((esr) & NFLEXCAN_ESR_FLT_CONF_MASK)>>4)
#define                           NCAN_ERROR_ACTIVE 0
#define                          NCAN_ERROR_PASSIVE 1
#define                          NCAN_ERROR_BUS_OFF 2

/* Bit definitions and macros for FLEXCAN_MB_CS */
#define                  NFLEXCAN_MB_CS_TIMESTAMP(x) ((x)&0x0000FFFFLU)
#define                NFLEXCAN_MB_CS_TIMESTAMP_MASK (0x0000FFFFLU)
#define                     NFLEXCAN_MB_CS_LENGTH(x) (((x)&0x0000000FLU)<<16)
#define                           NFLEXCAN_MB_CS_RTR BITNUMBER(20)
#define                           NFLEXCAN_MB_CS_IDE BITNUMBER(21)
#define                           NFLEXCAN_MB_CS_SRR BITNUMBER(22)
#define                       NFLEXCAN_MB_CS_CODE(x) (((x)&0x0000000FLU)<<24)
#define                     NFLEXCAN_MB_CS_CODE_MASK (0x0F000000LU)
#define                      NFLEXCAN_MB_CS_DLC_MASK (0x000F0000LU)
#define                 NFLEXCAN_MB_CODE_RX_INACTIVE (0)
#define                    NFLEXCAN_MB_CODE_RX_EMPTY (4)
#define                     NFLEXCAN_MB_CODE_RX_FULL (2)
#define                  NFLEXCAN_MB_CODE_RX_OVERRUN (6)
#define                     NFLEXCAN_MB_CODE_RX_BUSY (1)

#define                    NFLEXCAN_MB_CS_IDE_BIT_NO (21)
#define                    NFLEXCAN_MB_CS_RTR_BIT_NO (20)
#define                    NFLEXCAN_MB_CS_DLC_BIT_NO (16)

#define                 NFLEXCAN_MB_CODE_TX_INACTIVE (8)
#define                    NFLEXCAN_MB_CODE_TX_ABORT (9)
#define                     NFLEXCAN_MB_CODE_TX_ONCE (0x0C)
#define                 NFLEXCAN_MB_CODE_TX_RESPONSE (0x0A)
#define           NFLEXCAN_MB_CODE_TX_RESPONSE_TEMPO (0x0E)
#define                        NFLEXCAN_get_code(cs) (((cs) & NFLEXCAN_MB_CS_CODE_MASK)>>24)
#define                      NFLEXCAN_get_length(cs) (((cs) & NFLEXCAN_MB_CS_DLC_MASK)>>16)
#define                   NFLEXCAN_get_timestamp(cs) ((cs) & NFLEXCAN_MB_CS_TIMESTAMP_MASK)

/* Bit definitions and macros for FLEXCAN_MB_ID */
#define                     NFLEXCAN_MB_ID_STD_MASK  (0x1FFC0000LU)
#define                     NFLEXCAN_MB_ID_EXT_MASK  (0x1FFFFFFFLU)
#define                      NFLEXCAN_MB_ID_IDEXT(x) ((x)&0x0003FFFFLU)
#define                      NFLEXCAN_MB_ID_IDSTD(x) (((x)&0x000007FFLU)<<18)
#define                       NFLEXCAN_MB_ID_PRIO(x) (((x)&0x00000007LU)<<29)
#define                   NFLEXCAN_MB_ID_PRIO_BIT_NO (29)
#define                    NFLEXCAN_MB_ID_STD_BIT_NO (18)
#define                    NFLEXCAN_MB_ID_EXT_BIT_NO (0)

/* Bit definitions and macros for FLEXCAN_MB_WORD0 */
#define                   NFLEXCAN_MB_WORD0_DATA3(x) ((x)&0x000000FFLU)
#define                   NFLEXCAN_MB_WORD0_DATA2(x) (((x)&0x000000FFLU)<<8)
#define                   NFLEXCAN_MB_WORD0_DATA1(x) (((x)&0x000000FFLU)<<16)
#define                   NFLEXCAN_MB_WORD0_DATA0(x) (((x)&0x000000FFLU)<<24)

/* Bit definitions and macros for FLEXCAN_MB_WORD1 */
#define                   NFLEXCAN_MB_WORD1_DATA7(x) ((x)&0x000000FFLU)
#define                   NFLEXCAN_MB_WORD1_DATA6(x) (((x)&0x000000FFLU)<<8)
#define                   NFLEXCAN_MB_WORD1_DATA5(x) (((x)&0x000000FFLU)<<16)
#define                   NFLEXCAN_MB_WORD1_DATA4(x) (((x)&0x000000FFLU)<<24)

// Allow different xtal usage
#if !defined(F_CAN)
#if !defined(FLEXCAN_BASE_FREQ)
#define FLEXCAN_BASE_FREQ 16000000
#define FLEXCANb_CTRL1_SETUP(x) NFLEXCANb_CTRL1(x) &= ~NFLEXCAN_CTRL_CLK_SRC
#endif
#else
// Allow alternate clock path and frequency.
#define FLEXCAN_BASE_FREQ F_CAN
#if !defined(XTAL_F)
#define FLEXCANb_CTRL1_SETUP(x) NFLEXCANb_CTRL1(x) |= NFLEXCAN_CTRL_CLK_SRC;
#endif
#endif

/********************************************************************/
#endif // __KINETIS_FLEXCAN_H
