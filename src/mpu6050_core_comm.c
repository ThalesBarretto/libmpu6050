#include "mpu6050_core.h"
#include <i2c/smbus.h> 		/* for i2c_smbus_x */
#include <linux/i2c.h> 		/* for i2c_smbus_x */
#include <linux/i2c-dev.h>	/* for i2c_smbus_x */
#include <sys/types.h>

int mpu_read_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		mpu_reg_t *val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_read_byte_data(*(dev->bus), reg);

	if (res < 0) /* read failed - bus error */
		return -1;

	*val = (mpu_reg_t) res;
	return 0;
	
}

int mpu_write_byte(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		const mpu_reg_t val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_write_byte_data(*(dev->bus), reg, val);

	if (res < 0) /* read failed - bus error */
		return -1;

	return 0;
	
}

int mpu_read_word(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		mpu_word_t *val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_read_word_data(*(dev->bus), reg);

	if (res < 0) /* write byte failed - bus error */
		return -1;

	*val = (mpu_word_t) res;
	return 0;
	
}

int mpu_write_word(struct mpu_dev * const dev,
		const mpu_reg_t reg,
		const mpu_word_t val)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	__s32 res = i2c_smbus_write_word_data(*(dev->bus), reg, val);

	if (res < 0) /* write word failed - bus error */
		return -1;

	return 0;
	
}
