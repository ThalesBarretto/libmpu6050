#include "mpu6050_core.h"
#include "mpu6050_regs.h"

uint8_t mpu_regvalues[128];

char mpu_regnames[ 128 ][ 32 ] = {
[ AUX_VDDIO ]		= "AUX_VDDIO",
[ XA_OFFS_USRH ] 	= "XA_OFFS_USRH",
[ XA_OFFS_USRL ] 	= "XA_OFFS_USRL",
[ YA_OFFS_USRH ] 	= "YA_OFFS_USRH",
[ YA_OFFS_USRL ] 	= "YA_OFFS_USRL",
[ ZA_OFFS_USRH ] 	= "ZA_OFFS_USRH",
[ ZA_OFFS_USRL ] 	= "ZA_OFFS_USRL",
[ XG_OFFS_USRH ] 	= "XG_OFFS_USRH",
[ XG_OFFS_USRL ] 	= "XG_OFFS_USRL",
[ YG_OFFS_USRH ] 	= "YG_OFFS_USRH",
[ YG_OFFS_USRL ] 	= "YG_OFFS_USRL",
[ ZG_OFFS_USRH ] 	= "ZG_OFFS_USRH",
[ ZG_OFFS_USRL ]	= "ZG_OFFS_USRL",
[ PROD_ID ] 		= "PROD_ID",
[ SELF_TEST_X ] 	= "SELF_TEST_X",
[ SELF_TEST_Y ]		= "SELF_TEST_Y",
[ SELF_TEST_Z ]		= "SELF_TEST_Z",
[ SELF_TEST_A ]		= "SELF_TEST_A",
[ SMPLRT_DIV ]		= "SMPLRT_DIV",
[ CONFIG ]		= "CONFIG",
[ GYRO_CONFIG ]		= "GYRO_CONFIF",
[ ACCEL_CONFIG ]	= "ACCEL_CONFIG",
[ FF_THR ]		= "FF_THR",
[ FF_DUR ]		= "FF_DUR",
[ MOT_THR ]		= "MOT_THR",
[ MOT_DUR ]		= "MOT_DUR",
[ ZRMOT_THR ]		= "ZRMOT_THR",
[ ZRMOT_DUR ]		= "ZRMOT_DUR",
[ FIFO_EN ]		= "FIFO_EN",
[ I2C_MST_CTRL ]	= "I2C_MST_CTRL",
[ I2C_SLV0_ADDR ] 	= "I2C_SLV0_ADDR",
[ I2C_SLV0_REG ]	= "I2C_SLV0_REG",
[ I2C_SLV0_CTRL ] 	= "I2C_SLV0_CTRL",
[ I2C_SLV1_ADDR ]	= "I2C_SLV1_ADDR",
[ I2C_SLV1_REG ]	= "I2C_SLV1_REG",
[ I2C_SLV1_CTRL ]	= "I2C_SLV1_CTRL",
[ I2C_SLV2_ADDR ]	= "I2C_SLV2_ADDR",
[ I2C_SLV2_REG ]	= "I2C_SLV2_REG",
[ I2C_SLV2_CTRL ]	= "I2C_SLV2_CTRL",
[ I2C_SLV3_ADDR ]	= "I2C_SLV3_ADDR",
[ I2C_SLV3_REG ]	= "I2C_SLV3_REG",
[ I2C_SLV3_CTRL ]	= "I2C_SLV3_CTRL",
[ I2C_SLV4_ADDR ]	= "I2C_SLV4_ADDR",
[ I2C_SLV4_REG ]	= "I2C_SLV4_REG",
[ I2C_SLV4_DO ]		= "I2C_SLV4_DO",
[ I2C_SLV4_CTRL ]	= "I2C_SLV4_CTRL",
[ I2C_SLV4_DI ]		= "I2C_SLV4_DI",
[ I2C_MAST_STATUS ]	= "I2C_MST_STATUS",
[ INT_PIN_CFG ]		= "INT_PIN_CFG",
[ INT_ENABLE ]		= "INT_ENABLE",
[ DMP_INT_STATUS ] 	= "DMP_INT_STATUS",
[ INT_STATUS ]		= "INT_STATUS",
[ ACCEL_XOUT_H ] 	= "ACCEL_XOUT_H",
[ ACCEL_XOUT_L ] 	= "ACCEL_XOUT_L",
[ ACCEL_YOUT_H ] 	= "ACCEL_YOUT_H",
[ ACCEL_YOUT_L ] 	= "ACCEL_YOUT_L",
[ ACCEL_ZOUT_H ] 	= "ACCEL_ZOUT_H ",
[ ACCEL_ZOUT_L ] 	= "ACCEL_ZOUT_L",
[ TEMP_OUT_H ] 		= "TEMP_OUT_H",
[ TEMP_OUT_L ] 		= "TEMP_OUT_L",
[ GYRO_XOUT_H ] 	= "GYRO_XOUT_H",
[ GYRO_XOUT_L ] 	= "GYRO_XOUT_L",
[ GYRO_YOUT_H ] 	= "GYRO_YOUT_H",
[ GYRO_YOUT_L ] 	= "GYRO_YOUT_L",
[ GYRO_ZOUT_H ] 	= "GYRO_ZOUT_H",
[ GYRO_ZOUT_L ] 	= "GYRO_ZOUT_L",
[ EXT_SENS_DATA_00 ]	= "EXT_SENS_DATA_00",
[ EXT_SENS_DATA_01 ]	= "EXT_SENS_DATA_01",
[ EXT_SENS_DATA_02 ]	= "EXT_SENS_DATA_02",
[ EXT_SENS_DATA_03 ]	= "EXT_SENS_DATA_03",
[ EXT_SENS_DATA_04 ]	= "EXT_SENS_DATA_04",
[ EXT_SENS_DATA_05 ]	= "EXT_SENS_DATA_05",
[ EXT_SENS_DATA_06 ]	= "EXT_SENS_DATA_06",
[ EXT_SENS_DATA_07 ]	= "EXT_SENS_DATA_07",
[ EXT_SENS_DATA_08 ]	= "EXT_SENS_DATA_08",
[ EXT_SENS_DATA_09 ]	= "EXT_SENS_DATA_09",
[ EXT_SENS_DATA_10 ]	= "EXT_SENS_DATA_10",
[ EXT_SENS_DATA_11 ]	= "EXT_SENS_DATA_11",
[ EXT_SENS_DATA_12 ]	= "EXT_SENS_DATA_12",
[ EXT_SENS_DATA_13 ]	= "EXT_SENS_DATA_13",
[ EXT_SENS_DATA_14 ]	= "EXT_SENS_DATA_14",
[ EXT_SENS_DATA_15 ]	= "EXT_SENS_DATA_15",
[ EXT_SENS_DATA_16 ]	= "EXT_SENS_DATA_16",
[ EXT_SENS_DATA_17 ]	= "EXT_SENS_DATA_17",
[ EXT_SENS_DATA_18 ]	= "EXT_SENS_DATA_18",
[ EXT_SENS_DATA_19 ]	= "EXT_SENS_DATA_19",
[ EXT_SENS_DATA_20 ]	= "EXT_SENS_DATA_20",
[ EXT_SENS_DATA_21 ]	= "EXT_SENS_DATA_21",
[ EXT_SENS_DATA_22 ]	= "EXT_SENS_DATA_22",
[ EXT_SENS_DATA_23 ]	= "EXT_SENS_DATA_23",
[ MOT_DETECT_STATUS ]	= "MOT_DETECT_STATUS",
[ I2C_SLV0_DO ]		= "I2C_SLV0_DO",
[ I2C_SLV1_DO ]		= "I2C_SLV1_DO",
[ I2C_SLV2_DO ]		= "I2C_SLV2_DO",
[ I2C_SLV3_DO ]		= "I2C_SLV0_DO",
[ I2C_MST_DELAY_CTRL ]	= "I2C_MST_DELAY_CTRL",
[ SIGNAL_PATH_RESET ]	= "SIGNAL_PATH_RESET",
[ MOT_DETECT_CTRL ]	= "MOT_DETECT_CTRL",
[ USER_CTRL ]		= "USER_CTRL",
[ PWR_MGMT_1 ]		= "PWR_MGMT_1",
[ PWR_MGMT_2 ]		= "PWR_MGMT_2",
[ MEM_R_W ] 		= "MEM_R_W",
[ BANK_SEL ] 		= "BANK_SEL",
[ MEM_START_ADDR ] 	= "MEM_START_ADDR",
[ PRGM_START_H ] 	= "PRGM_START_H",
[ PRGM_START_L ] 	= "PRGM_START_L",
[ FIFO_COUNT_H ]	= "FIFO_COUNT_H",
[ FIFO_COUNT_L ]	= "FIFO_COUNT_L",
[ FIFO_R_W ]		= "FIFO_R_W",
[ WHO_AM_I ]		= "WHO_AM_I",
};

int mpu_ctl_dump(struct mpu_dev *dev, char *fn)
{

	FILE *fp;
	if ( (fp = fopen(fn, "w+")) == NULL) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fn);
		return -1;
	}

	fprintf(fp, "MPU REGISTER DUMP\n");
	fprintf(fp,"%8s %8s %20s %4s", "reg(hex)", "reg(dec)", "name", "val");

	uint8_t regs[128][3];
	for (int i = 0; i < 128; i++) {
		regs[i][0] = i;
		regs[i][1] = mpu_read_byte(dev, i, &regs[i][0]);
		fprintf(fp, " %2x     %3d    %-20s %2x\n",
			regs[i][0], 
			regs[i][0], 
			mpu_regnames[i], 
			regs[i][1]);
	}
	fflush(fp);
	fclose(fp);
	return 0;
}
