// SPDX-License-Identifier: MIT 
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
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
//				0x00u
#define AUX_VDDIO		0x01u
//				0x02u
//				0x03u
//				0x04u
//				0x05u
#define XA_OFFS_USRH		0x06u
#define XA_OFFS_USRL		0x07u
#define YA_OFFS_USRH		0x08u
#define YA_OFFS_USRL		0x09u
#define ZA_OFFS_USRH		0x0au
#define ZA_OFFS_USRL		0x0bu
#define PROD_ID			0x0cu
#define SELF_TEST_X		0x0du
#define SELF_TEST_Y		0x0eu
#define SELF_TEST_Z		0x0fu
#define SELF_TEST_A		0x10u
//				0x11u
//				0x12u
#define XG_OFFS_USRH		0x13u
#define XG_OFFS_USRL		0x14u
#define YG_OFFS_USRH		0x15u
#define YG_OFFS_USRL		0x16u
#define ZG_OFFS_USRH		0x17u
#define ZG_OFFS_USRL		0x18u
#define SMPLRT_DIV		0x19u
#define CONFIG			0x1au
#define GYRO_CONFIG		0x1bu
#define ACCEL_CONFIG		0x1cu
#define FF_THR			0x1du
#define FF_DUR			0x1eu
#define MOT_THR			0x1fu
#define MOT_DUR			0x20u
#define ZRMOT_THR		0x21u
#define ZRMOT_DUR		0x22u
#define FIFO_EN			0x23u
#define I2C_MST_CTRL		0x24u
#define I2C_SLV0_ADDR		0x25u
#define I2C_SLV0_REG		0x26u
#define I2C_SLV0_CTRL		0x27u
#define I2C_SLV1_ADDR		0x28u
#define I2C_SLV1_REG		0x29u
#define I2C_SLV1_CTRL		0x2au
#define I2C_SLV2_ADDR		0x2bu
#define I2C_SLV2_REG		0x2cu
#define I2C_SLV2_CTRL		0x2du
#define I2C_SLV3_ADDR		0x2eu
#define I2C_SLV3_REG		0x2fu
#define I2C_SLV3_CTRL		0x30u
#define I2C_SLV4_ADDR		0x31u
#define I2C_SLV4_REG		0x32u
#define I2C_SLV4_DO		0x33u
#define I2C_SLV4_CTRL		0x34u
#define I2C_SLV4_DI		0x35u
#define I2C_MAST_STATUS		0x36u
#define INT_PIN_CFG		0x37u
#define INT_ENABLE		0x38u
#define DMP_INT_STATUS		0x39u
#define INT_STATUS		0x3au
#define ACCEL_XOUT_H		0x3bu
#define ACCEL_XOUT_L		0x3cu
#define ACCEL_YOUT_H		0x3du
#define ACCEL_YOUT_L		0x3eu
#define ACCEL_ZOUT_H 		0x3fu
#define ACCEL_ZOUT_L		0x40u
#define TEMP_OUT_H		0x41u
#define TEMP_OUT_L		0x42u
#define GYRO_XOUT_H		0x43u
#define GYRO_XOUT_L		0x44u
#define GYRO_YOUT_H		0x45u
#define GYRO_YOUT_L		0x46u
#define GYRO_ZOUT_H		0x47u
#define GYRO_ZOUT_L		0x48u
#define EXT_SENS_DATA_00	0x49u
#define EXT_SENS_DATA_01	0x4au
#define EXT_SENS_DATA_02	0x4bu
#define EXT_SENS_DATA_03	0x4cu
#define EXT_SENS_DATA_04	0x4du
#define EXT_SENS_DATA_05	0x4eu
#define EXT_SENS_DATA_06	0x4fu
#define EXT_SENS_DATA_07	0x50u
#define EXT_SENS_DATA_08	0x51u
#define EXT_SENS_DATA_09	0x52u
#define EXT_SENS_DATA_10	0x53u
#define EXT_SENS_DATA_11	0x54u
#define EXT_SENS_DATA_12	0x55u
#define EXT_SENS_DATA_13	0x56u
#define EXT_SENS_DATA_14	0x57u
#define EXT_SENS_DATA_15	0x58u
#define EXT_SENS_DATA_16	0x59u
#define EXT_SENS_DATA_17	0x5au
#define EXT_SENS_DATA_18	0x5bu
#define EXT_SENS_DATA_19	0x5cu
#define EXT_SENS_DATA_20	0x5du
#define EXT_SENS_DATA_21	0x5eu
#define EXT_SENS_DATA_22	0x5fu
#define EXT_SENS_DATA_23	0x60u
#define MOT_DETECT_STATUS	0x61u
//				0x62u
#define I2C_SLV0_DO		0x63u
#define I2C_SLV1_DO		0x64u
#define I2C_SLV2_DO		0x65u
#define I2C_SLV3_DO		0x66u
#define I2C_MST_DELAY_CTRL	0x67u
#define SIGNAL_PATH_RESET	0x68u
#define MOT_DETECT_CTRL		0x69u
#define USER_CTRL		0x6au
#define PWR_MGMT_1		0x6bu
#define PWR_MGMT_2		0x6cu
#define BANK_SEL 		0x6du
#define MEM_START_ADDR 		0x6eu
#define MEM_R_W			0x6fu
#define PRGM_START_H		0x70u
#define PRGM_START_L		0x71u
#define FIFO_COUNT_H		0x72u
#define FIFO_COUNT_L		0x73u
#define FIFO_R_W 		0x74u
#define WHO_AM_I		0x75u

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
#define EXT_SYNC_SET_BIT	(uint8_t)(0x38u) /* CONFIG */
#define EXT_SYNC_SET_0		(uint8_t)(0x00u) /* CONFIG */
#define EXT_SYNC_SET_1		(uint8_t)(0x08u) /* CONFIG */
#define EXT_SYNC_SET_2		(uint8_t)(0x10u) /* CONFIG */
#define EXT_SYNC_SET_3		(uint8_t)(0x18u) /* CONFIG */
#define EXT_SYNC_SET_4		(uint8_t)(0x20u) /* CONFIG */
#define EXT_SYNC_SET_5		(uint8_t)(0x28u) /* CONFIG */
#define EXT_SYNC_SET_6		(uint8_t)(0x30u) /* CONFIG */
#define EXT_SYNC_SET_7		(uint8_t)(0x38u) /* CONFIG */

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

