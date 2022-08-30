#include "mpu6050_private.h"

int mpu_read_byte(struct mpu_dev * const dev, const mpu_reg_t reg, mpu_reg_t *val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	__s32 res = i2c_smbus_read_byte_data(*(dev->bus), reg);

	if (res < 0) /* read failed - bus error */
		return -1;

	*val = (mpu_reg_t) res;
	return 0;

}

int mpu_write_byte(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_reg_t val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	__s32 res = i2c_smbus_write_byte_data(*(dev->bus), reg, val);

	if (res < 0) /* read failed - bus error */
		return -1;

	return 0;

}

int mpu_read_word(struct mpu_dev * const dev, const mpu_reg_t reg, mpu_word_t *val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	__s32 res = i2c_smbus_read_word_data(*(dev->bus), reg);

	if (res < 0) /* write byte failed - bus error */
		return -1;

	*val = (mpu_word_t) res;
	return 0;

}

int __attribute__((unused)) mpu_write_word(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_word_t val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	__s32 res = i2c_smbus_write_word_data(*(dev->bus), reg, val);

	if (res < 0) /* write word failed - bus error */
		return -1;

	return 0;

}

int mpu_read_data(struct mpu_dev * const dev, const mpu_reg_t reg, int16_t *val)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint16_t dh;  /* unsigned for bit fiddling data high */
	uint16_t dl;  /* unsigned for bit fiddling data low  */
	uint8_t buf;

	if (mpu_read_byte(dev, reg, &buf) < 0)
		return -1;

	dh = (uint16_t)(buf & 0x00FF) << 8;

	if (mpu_read_byte(dev, reg+1, &buf) < 0)
		return -1;

	dl = (uint16_t)(buf & 0x00FF);
	*val = (uint16_t)(dh | dl);

	return 0;
}
