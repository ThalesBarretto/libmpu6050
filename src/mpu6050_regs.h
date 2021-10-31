// SPDX-License-Identifier: GPL-2.0-only */
/* 
 * Copyright (C) 2021 Thales Antunes de Oliveira Barretto
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.* 
 *
 * DISCLAIMER: This is independet work and is not related in any
 * way to Invensense or any of it's related companies.
 *
 */
#ifndef __MPU6050_CORE_REGS_H_
#define __MPU6050_CORE_REGS_H_

/* MPU-6050 register definitions
 *
 * Here are defined all registers documented in:
 * MPU-6000/6050 Product Specification rev. 3.4. (08/19/2013) PS-MPU-6000A-00
 * MPU-6000/6050 Register Map and Desription rev. 4.12. (08/19/2013) RM-MPU-6000A-00
 * MPU Hardware Offset Registers Application Note rev 1.0 (02/21/2014) AN-XX-XXXX-XX
 *
 * Accel offset registers
 * 1. Initial values contain factory trim values. Read value, apply bias;
 * 2. Format is in +-8g scale (1mg = 4096 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 *
 * Gyro offset registers
 * 1. Initial values are 0;
 * 2. Format is in +-1000dps scale (1dps = 32.8 LSB)
 * 3. Bit 0 on the low byte of each axis is reserved and must be unchanged.
 */
//				  0 0x00
#define AUX_VDDIO		  1
//				  2	0x02
//				  3	0x03
//				  4	0x04
//				  5	0x05
#define XA_OFFS_USRH		  6  // 0x06
#define XA_OFFS_USRL		  7  // 0x07
#define YA_OFFS_USRH		  8  // 0x08
#define YA_OFFS_USRL		  9  // 0x09
#define ZA_OFFS_USRH		 10  // 0x0A
#define ZA_OFFS_USRL		 11  // 0x0B
#define PROD_ID			 12  // 0X0C
#define SELF_TEST_X		 13
#define SELF_TEST_Y		 14
#define SELF_TEST_Z		 15
#define SELF_TEST_A		 16
//				 17	0x11
//				 18	0x12
#define XG_OFFS_USRH		 19  // 0x13
#define XG_OFFS_USRL		 20  // 0x14
#define YG_OFFS_USRH		 21  // 0x15
#define YG_OFFS_USRL		 22  // 0x16
#define ZG_OFFS_USRH		 23  // 0x17
#define ZG_OFFS_USRL		 24  // 0x18
#define SMPLRT_DIV		 25
#define CONFIG			 26
#define GYRO_CONFIG		 27
#define ACCEL_CONFIG		 28
#define FF_THR			 29
#define FF_DUR			 30
#define MOT_THR			 31
#define MOT_DUR			 32
#define ZRMOT_THR		 33
#define ZRMOT_DUR		 34
#define FIFO_EN			 35
#define I2C_MST_CTRL		 36
#define I2C_SLV0_ADDR		 37
#define I2C_SLV0_REG		 38
#define I2C_SLV0_CTRL		 39
#define I2C_SLV1_ADDR		 40
#define I2C_SLV1_REG		 41
#define I2C_SLV1_CTRL		 42
#define I2C_SLV2_ADDR		 43
#define I2C_SLV2_REG		 44
#define I2C_SLV2_CTRL		 45
#define I2C_SLV3_ADDR		 46
#define I2C_SLV3_REG		 47
#define I2C_SLV3_CTRL		 48
#define I2C_SLV4_ADDR		 49
#define I2C_SLV4_REG		 50
#define I2C_SLV4_DO		 51
#define I2C_SLV4_CTRL		 52
#define I2C_SLV4_DI		 53
#define I2C_MAST_STATUS		 54
#define INT_PIN_CFG		 55
#define INT_ENABLE		 56
#define DMP_INT_STATUS		 57
#define INT_STATUS		 58
#define ACCEL_XOUT_H		 59
#define ACCEL_XOUT_L		 60
#define ACCEL_YOUT_H		 61
#define ACCEL_YOUT_L		 62
#define ACCEL_ZOUT_H 		 63
#define ACCEL_ZOUT_L		 64
#define TEMP_OUT_H		 65
#define TEMP_OUT_L		 66
#define GYRO_XOUT_H		 67
#define GYRO_XOUT_L		 68
#define GYRO_YOUT_H		 69
#define GYRO_YOUT_L		 70
#define GYRO_ZOUT_H		 71
#define GYRO_ZOUT_L		 72
#define EXT_SENS_DATA_00	 73
#define EXT_SENS_DATA_01	 74
#define EXT_SENS_DATA_02	 75
#define EXT_SENS_DATA_03	 76
#define EXT_SENS_DATA_04	 77
#define EXT_SENS_DATA_05	 78
#define EXT_SENS_DATA_06	 79
#define EXT_SENS_DATA_07	 80
#define EXT_SENS_DATA_08	 81
#define EXT_SENS_DATA_09	 82
#define EXT_SENS_DATA_10	 83
#define EXT_SENS_DATA_11	 84
#define EXT_SENS_DATA_12	 85
#define EXT_SENS_DATA_13	 86
#define EXT_SENS_DATA_14	 87
#define EXT_SENS_DATA_15	 88
#define EXT_SENS_DATA_16	 89
#define EXT_SENS_DATA_17	 90
#define EXT_SENS_DATA_18	 91
#define EXT_SENS_DATA_19	 92
#define EXT_SENS_DATA_20	 93
#define EXT_SENS_DATA_21	 94
#define EXT_SENS_DATA_22	 95
#define EXT_SENS_DATA_23	 96
#define MOT_DETECT_STATUS	 97
//				 98 0x62
#define I2C_SLV0_DO		 99
#define I2C_SLV1_DO		100
#define I2C_SLV2_DO		101
#define I2C_SLV3_DO		102
#define I2C_MST_DELAY_CTRL	103
#define SIGNAL_PATH_RESET	104
#define MOT_DETECT_CTRL		105
#define USER_CTRL		106
#define PWR_MGMT_1		107
#define PWR_MGMT_2		108
#define BANK_SEL 		109 // new
#define MEM_START_ADDR 		110 // new
#define MEM_R_W			111 // new
#define PRGM_START_H		112 // new
#define PRGM_START_L		113 // new
#define FIFO_COUNT_H		114
#define FIFO_COUNT_L		115
#define FIFO_R_W 		116
#define WHO_AM_I		117

/* MPU6050 bit range definitions */
#define XA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_X */
#define XG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_X */
#define YA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_Y */
#define YG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_Y */
#define ZA_TEST_42_BIT		(uint8_t)(0xE0u) /* SELF_TEST_Z */
#define ZG_TEST_40_BIT		(uint8_t)(0x1Fu) /* SELF_TEST_Z */
#define XA_TEST_10_BIT		(uint8_t)(0x30u) /* SELF_TEST_A */
#define YA_TEST_10_BIT		(uint8_t)(0x0Cu) /* SELF_TEST_A */
#define ZA_TEST_10_BIT		(uint8_t)(0x03u) /* SELF_TEST_A */

#define XG_ST_BIT		(uint8_t)(0x80u) /* GYRO_CONFIG */
#define YG_ST_BIT		(uint8_t)(0x40u) /* GYRO_CONFIG */
#define ZG_ST_BIT		(uint8_t)(0x20u) /* GYRO_CONFIG */
#define XA_ST_BIT		(uint8_t)(0x80u) /* ACCEL_CONFIG */
#define YA_ST_BIT		(uint8_t)(0x40u) /* ACCEL_CONFIG */
#define ZA_ST_BIT		(uint8_t)(0x20u) /* ACCEL_CONFIG */

#define FIFO_EN_BIT		(uint8_t)(0x40u) /* USER_CTRL */
#define I2C_MST_EN_BIT		(uint8_t)(0x20u) /* USER_CTRL */
#define I2C_IF_DIS_BIT		(uint8_t)(0x10u) /* USER_CTRL */
#define FIFO_RESET_BIT		(uint8_t)(0x04u) /* USER CTRL */
#define I2C_MST_RESET_BIT	(uint8_t)(0x02u) /* USER CTRL */
#define SIG_COND_RESET_BIT	(uint8_t)(0x01u) /* USER CTRL */

#define GYRO_RESET_BIT		(uint8_t)(0x04u) /* SIGNAL_PATH_RESET */
#define ACCEL_RESET_BIT		(uint8_t)(0x02u) /* SIGNAL_PATH_RESET */
#define TEMP_RESET_BIT		(uint8_t)(0x01u) /* SIGNAL_PATH_RESET */

#define DEVICE_RESET_BIT	(uint8_t)(0x80u) /* PWR_MGMT_1 */
#define SLEEP_BIT		(uint8_t)(0x40u) /* PWR_MGMT_1 */
#define CYCLE_BIT		(uint8_t)(0x20u) /* PWR_MGMT_1 */
#define TEMP_DIS_BIT		(uint8_t)(0x08u) /* PWR_MGMT_1 */
#define CLKSEL_BIT		(uint8_t)(0x07u) /* PWR_MGMT_1 */
#define CLKSEL_0		(uint8_t)(0x00u) /* PWR_MGMT_1 */
#define CLKSEL_1		(uint8_t)(0x01u) /* PWR_MGMT_1 */
#define CLKSEL_2		(uint8_t)(0x02u) /* PWR_MGMT_1 */
#define CLKSEL_3		(uint8_t)(0x03u) /* PWR_MGMT_1 */
#define CLKSEL_4		(uint8_t)(0x04u) /* PWR_MGMT_1 */
#define CLKSEL_5		(uint8_t)(0x05u) /* PWR_MGMT_1 */
#define CLKSEL_6		(uint8_t)(0x06u) /* PWR_MGMT_1 */
#define CLKSEL_7		(uint8_t)(0x07u) /* PWR_MGMT_1 */

#define LP_WAKE_CTL_BIT		(uint8_t)(0xC0u) /* PWR_MGMT_2 */
#define STDBY_XA_BIT		(uint8_t)(0x20u) /* PWR_MGMT_2 */
#define STDBY_YA_BIT		(uint8_t)(0x10u) /* PWR_MGMT_2 */
#define STDBY_ZA_BIT		(uint8_t)(0x08u) /* PWR_MGMT_2 */
#define STDBY_XG_BIT		(uint8_t)(0x04u) /* PWR_MGMT_2 */
#define STDBY_YG_BIT		(uint8_t)(0x02u) /* PWR_MGMT_2 */
#define STDBY_ZG_BIT		(uint8_t)(0x01u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_0		(uint8_t)(0x00u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_1		(uint8_t)(0x40u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_2		(uint8_t)(0x80u) /* PWR_MGMT_2 */
#define LP_WAKE_CTL_3		(uint8_t)(0xC0u) /* PWR_MGMT_2 */

#define DLPF_CFG_BIT		(uint8_t)(0x07u) /* CONFIG */
#define DLPF_CFG_0		(uint8_t)(0x00u) /* CONFIG */
#define DLPF_CFG_1		(uint8_t)(0x01u) /* CONFIG */
#define DLPF_CFG_2		(uint8_t)(0x02u) /* CONFIG */
#define DLPF_CFG_3		(uint8_t)(0x03u) /* CONFIG */
#define DLPF_CFG_4		(uint8_t)(0x04u) /* CONFIG */
#define DLPF_CFG_5		(uint8_t)(0x05u) /* CONFIG */
#define DLPF_CFG_6		(uint8_t)(0x06u) /* CONFIG */
#define DLPF_CFG_7		(uint8_t)(0x07u) /* CONFIG */

#define FS_SEL_BIT		(uint8_t)(0x18u) /* GYRO_CONFIG */
#define FS_SEL_0		(uint8_t)(0x00u) /* GYRO_CONFIG */
#define FS_SEL_1		(uint8_t)(0x08u) /* GYRO_CONFIG */
#define FS_SEL_2		(uint8_t)(0x10u) /* GYRO_CONFIG */
#define FS_SEL_3		(uint8_t)(0x18u) /* GYRO_CONFIG */

#define AFS_SEL_BIT		(uint8_t)(0x18u) /* ACCEL_CONFIG */
#define AFS_SEL_0		(uint8_t)(0x00u) /* ACCEL_CONFIG */
#define AFS_SEL_1		(uint8_t)(0x08u) /* ACCEL_CONFIG */
#define AFS_SEL_2		(uint8_t)(0x10u) /* ACCEL_CONFIG */
#define AFS_SEL_3		(uint8_t)(0x18u) /* ACCEL_CONFIG */

#define TEMP_FIFO_EN_BIT	(uint8_t)(0x80u) /* FIFO_EN */
#define XG_FIFO_EN_BIT		(uint8_t)(0x40u) /* FIFO_EN */
#define YG_FIFO_EN_BIT		(uint8_t)(0x20u) /* FIFO_EN */
#define ZG_FIFO_EN_BIT		(uint8_t)(0x10u) /* FIFO_EN */
#define ACCEL_FIFO_EN_BIT	(uint8_t)(0x08u) /* FIFO_EN */
#define SLV2_FIFO_EN_BIT	(uint8_t)(0x04u) /* FIFO_EN */
#define SLV1_FIFO_EN_BIT	(uint8_t)(0x02u) /* FIFO_EN */
#define SLV0_FIFO_EN_BIT	(uint8_t)(0x01u) /* FIFO_EN */

#define INT_LEVEL_BIT		(uint8_t)(0x80u) /* INT_PIN_CFG */
#define INT_OPEN_BIT		(uint8_t)(0x40u) /* INT_PIN_CFG */
#define LATCH_INT_BIT		(uint8_t)(0x20u) /* INT_PIN_CFG */
#define INT_RD_CLEAR_BIT	(uint8_t)(0x10u) /* INT_PIN_CFG */
#define FSYNC_INT_LEVEL_BIT	(uint8_t)(0x08u) /* INT_PIN_CFG */
#define FSYNC_INT_EN_BIT	(uint8_t)(0x04u) /* INT_PIN_CFG */
#define I2C_BYPASS_EN_BIT	(uint8_t)(0x02u) /* INT_PIN_CFG */

#define FIFO_OFLOW_EN_BIT	(uint8_t)(0x10u) /* INT_ENABLE */
#define I2C_MST_INT_EN_BIT	(uint8_t)(0x08u) /* INT_ENABLE */
#define DATA_RDY_EN_BIT		(uint8_t)(0x01u) /* INT_ENABLE */

#define FIFO_OFLOW_INT_BIT	(uint8_t)(0x10u) /* INT_STATUS */
#define I2C_MST_INT_BIT		(uint8_t)(0x08u) /* INT_STATUS */
#define DATA_RDY_INT_BIT	(uint8_t)(0x01u) /* INT_STATUS */

#define MULT_MST_EN_BIT		(uint8_t)(0x80u) /* I2C_MST_CTRL */
#define WAIT_FOR_ES_BIT		(uint8_t)(0x40u) /* I2C_MST_CTRL */
#define SLV3_FIFO_EN_BIT	(uint8_t)(0x20u) /* I2C_MST_CTRL */
#define I2C_MST_P_NSR_BIT	(uint8_t)(0x10u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_BIT		(uint8_t)(0x0Fu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_0		(uint8_t)(0x00u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_1		(uint8_t)(0x01u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_2		(uint8_t)(0x02u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_3		(uint8_t)(0x03u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_4		(uint8_t)(0x04u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_5		(uint8_t)(0x05u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_6		(uint8_t)(0x06u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_7		(uint8_t)(0x07u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_8		(uint8_t)(0x08u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_9		(uint8_t)(0x09u) /* I2C_MST_CTRL */
#define I2C_MST_CLK_10		(uint8_t)(0x0Au) /* I2C_MST_CTRL */
#define I2C_MST_CLK_11		(uint8_t)(0x0Bu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_12		(uint8_t)(0x0Cu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_13		(uint8_t)(0x0Du) /* I2C_MST_CTRL */
#define I2C_MST_CLK_14		(uint8_t)(0x0Eu) /* I2C_MST_CTRL */
#define I2C_MST_CLK_15		(uint8_t)(0x0Fu) /* I2C_MST_CTRL */

#endif /* __MPU6050_CORE_REGS_H_ */

