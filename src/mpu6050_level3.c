#include "mpu6050_private.h"

int mpu_diagnose(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	printf("%-20s %6d\n"	  ,"File Descriptor"	, *(dev->bus));
	printf("%-20s %#6x\n"	  ,"ADDRESS"		, dev->addr);
	printf("%-20s %#6x\n"	  ,"PRODUCT ID"		, dev->prod_id);
	printf("%-20s %6.0lf %s\n","Wake frequency"	, dev->wake_freq	,"(Hz)");
	printf("%-20s %6.0lf %s\n","CLOCK frequency"	, dev->clock_freq	,"(Hz)");
	printf("%-20s %6d %s\n"	  ,"Gyro Output Rate"	, dev->gor		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Sampling rate"	, dev->sr		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Sampling time"	, dev->st		,"(s)");
	printf("%-20s %6.0lf %s\n","Accel Full Range"	, dev->afr		,"(g)");
	printf("%-20s %6.0lf %s\n","Accel LBS"		, dev->albs		,"(LSB/g)");
	printf("%-20s %6.0lf %s\n","Accel bandwidth"	, dev->abdw		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Accel delay"	, dev->adly		,"(ms)");
	printf("%-20s %6.0lf %s\n","Gyro Full Range"	, dev->gfr		,"(degrees/s)");
	printf("%-20s %6.0lf %s\n","Gyro LBS"		, dev->glbs		,"(LSB/degress/s)");
	printf("%-20s %6.0lf %s\n","Gyro bandwidth"	, dev->gbdw		,"(Hz)");
	printf("%-20s %6.0lf %s\n","Gyro delay"		, dev->gdly		,"(ms)");
	printf("----------------------------------------\n");
	printf("%-20s %d\n","SLEEP BIT"		, dev->cfg->sleep);
	printf("%-20s %d\n","CYCLE BIT"		, dev->cfg->cycle);
	printf("%-20s %d\n","TEMP_DIS BIT"	, dev->cfg->temp_dis);
	printf("%-20s %d\n","STDBY_XA BIT"	, dev->cfg->stdby_xa);
	printf("%-20s %d\n","STDBY_YA BIT"	, dev->cfg->stdby_ya);
	printf("%-20s %d\n","STDBY_ZA BIT"	, dev->cfg->stdby_za);
	printf("%-20s %d\n","STDBY_XG BIT"	, dev->cfg->stdby_xg);
	printf("%-20s %d\n","STDBY_YG BIT"	, dev->cfg->stdby_yg);
	printf("%-20s %d\n","STDBY_ZG BIT"	, dev->cfg->stdby_zg);
	printf("%-20s %d\n","I2C_MST_EN BIT"	, dev->cfg->i2c_mst_en);
	printf("%-20s %d\n","I2C_IF_DIS BIT"	, dev->cfg->i2c_if_dis);
	printf("%-20s %d\n","FIFO_EN BIT"	, dev->cfg->fifo_en);
	printf("%-20s %d\n","TEMP_FIFO_EN BIT"	, dev->cfg->temp_fifo_en);
	printf("%-20s %d\n","FIFO Accel BIT"	, dev->cfg->accel_fifo_en);
	printf("%-20s %d\n","FIFO Gyro X BIT"	, dev->cfg->xg_fifo_en);
	printf("%-20s %d\n","FIFO Gyro Y BIT"	, dev->cfg->yg_fifo_en);
	printf("%-20s %d\n","FIFO Gyro Z BIT"	, dev->cfg->zg_fifo_en);
	printf("%-20s %d\n","FIFO SLV0"		, dev->cfg->slv0_fifo_en);
	printf("%-20s %d\n","FIFO SLV1"		, dev->cfg->slv1_fifo_en);
	printf("%-20s %d\n","FIFO SLV2"		, dev->cfg->slv2_fifo_en);
	printf("%-20s %d\n","FIFO SLV3"		, dev->cfg->slv3_fifo_en);
	printf("%-20s %d\n","FIFO SLV4"		, dev->cfg->slv4_fifo_en);
	printf("%-20s %d\n","FSYNC_INT_EN BIT"	, dev->cfg->fsync_int_en);
	printf("%-20s %d\n","FIFO_OFLOW_EN BIT"	, dev->cfg->fifo_oflow_en);
	printf("%-20s %d\n","I2C_MST_INT_EN BIT", dev->cfg->i2c_mst_int_en);
	printf("%-20s %d\n","DATA_RDY_EN BIT"	, dev->cfg->data_rdy_en);
	printf("%-20s %d\n","raw[0]"		, dev->dat->raw[0]);
	printf("----------------------------------------\n");
	printf("ADDRESSES:\n");
	printf("dat[1](%p) Ax=%p\n", (void *)dev->dat->dat[1], (void *)dev->Ax);
	printf("dat[2](%p) Ay=%p\n", (void *)dev->dat->dat[2], (void *)dev->Ay);
	printf("dat[3](%p) Az=%p\n", (void *)dev->dat->dat[3], (void *)dev->Az);
	printf("dat[4](%p) t =%p\n", (void *)dev->dat->dat[4], (void *)dev->t);
	printf("dat[5](%p) Gx=%p\n", (void *)dev->dat->dat[5], (void *)dev->Gx);
	printf("dat[6](%p) Gy=%p\n", (void *)dev->dat->dat[6], (void *)dev->Gy);
	printf("dat[7](%p) Gz=%p\n", (void *)dev->dat->dat[7], (void *)dev->Gz);
	printf("----------------------------------------\n");
	printf("ADDRESSES:\n");
	printf("squ[4](%p) t =%p\n", (void *)&dev->dat->squ[4], (void *)dev->t);
	printf("----------------------------------------\n");
	printf("SCALING:\n");
	printf("scl[1](%lf) %lf\n", dev->dat->scl[1], 1/dev->albs);
	printf("scl[2](%lf) %lf\n", dev->dat->scl[2], 1/dev->albs);
	printf("scl[3](%lf) %lf\n", dev->dat->scl[3], 1/dev->albs);
	printf("scl[4](%lf) %lf\n", dev->dat->scl[4], 1/340.0);
	printf("scl[5](%lf) %lf\n", dev->dat->scl[5], 1/dev->glbs);
	printf("scl[6](%lf) %lf\n", dev->dat->scl[6], 1/dev->glbs);
	printf("scl[7](%lf) %lf\n", dev->dat->scl[7], 1/dev->glbs);
	printf("----------------------------------------\n");
	return 0;
}

int __attribute__((unused)) mpu_ctl_selftest_fifo_disable(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val & (~FIFO_EN_BIT)) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	sleep(1);

	val = ~(ACCEL_FIFO_EN_BIT | XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT);
	if (mpu_cfg_set_val(dev, FIFO_EN, val)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_enable_accel(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
			return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, ACCEL_CONFIG, val | XA_ST_BIT | YA_ST_BIT | ZA_ST_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_enable_gyro(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, GYRO_CONFIG, val | XG_ST_BIT | YG_ST_BIT | ZG_ST_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, ACCEL_CONFIG, val & (~(XA_ST_BIT | YA_ST_BIT | ZA_ST_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest_disable_gyro(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, GYRO_CONFIG, val & (~(XG_ST_BIT | YG_ST_BIT | ZG_ST_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_wake(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, PWR_MGMT_1, &val) < 0) { /* unconfigured */
		if (mpu_write_byte(dev, PWR_MGMT_1, 0x00) < 0) { /* write failed */
			return -1;
		} else {
			return 0;
		}
	}

	/* clear DEVICE_RESET, SLEEP and CYCLE bits */
	val &= ~(DEVICE_RESET_BIT | SLEEP_BIT | CYCLE_BIT);
	if (mpu_cfg_set_val(dev, PWR_MGMT_1, val) < 0)
		return -1;

	/* cleaer all STDBY bits */
	if (mpu_cfg_set_val(dev, PWR_MGMT_2, 0x00) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_count(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint16_t tem = 0;
	if (mpu_read_word(dev, FIFO_COUNT_H, &tem) < 0)
		return -1;

	uint16_t buf = (tem << 8) | (tem >> 8);
	dev->fifocnt = buf;

	return 0;
}

int mpu_ctl_fifo_flush(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint8_t dat = 0;
       	mpu_ctl_fifo_count(dev);
	for (int i = 0; i < dev->fifocnt; i++) {
		if (mpu_read_byte(dev, FIFO_R_W, &dat) < 0)
			return -1;
	}
	dev->samples = 0;

	return 0;
}

int mpu_ctl_fifo_enable(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val | FIFO_EN_BIT) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_enable_temp(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | TEMP_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}
int mpu_ctl_fifo_enable_accel(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | ACCEL_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_enable_gyro(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val | XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT)  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_disable(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val = 0;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, USER_CTRL, val & (~FIFO_EN_BIT)) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	sleep(1);

	return 0;
}

int mpu_ctl_fifo_disable_temp(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~TEMP_FIFO_EN_BIT))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_disable_accel(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~ACCEL_FIFO_EN_BIT))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_fifo_disable_gyro(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;
	if (mpu_cfg_set_val(dev, FIFO_EN, val & (~(XG_FIFO_EN_BIT | YG_FIFO_EN_BIT | ZG_FIFO_EN_BIT)))  < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int __attribute__((unused)) mpu_ctl_fifo_reset(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | FIFO_RESET_BIT) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_fifo_read_data(struct mpu_dev *dev, int16_t *data)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	/* HIC SUNT DRACONES */
	uint16_t dh;  /* unsigned for bit fiddling data high */
	uint16_t dl;  /* unsigned for bit fiddling data low  */
	uint8_t buf;
	if (mpu_read_byte(dev, FIFO_R_W, &buf) < 0)
		return -1;
	dh = (uint16_t)(buf & 0x00FF) << 8;
	if (mpu_read_byte(dev, FIFO_R_W, &buf) < 0)
		return -1;
	dl = (uint16_t)(buf & 0x00FF);
	*data = (uint16_t)(dh | dl);

	return 0;
}

int mpu_ctl_fifo_data(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	int len = 1 + dev->dat->raw[0];
	if (1 == len) {
		return 0;
	}

	if (mpu_ctl_fifo_count(dev) < 0)
		return -1;

	if (dev->fifocnt > dev->fifomax) { /* buffer overflow */
		mpu_ctl_fifo_flush(dev);
	} else {
		while (dev->fifocnt < len) { /* buffer underflow */
			nanosleep(&(dev->dly), NULL);
			mpu_ctl_fifo_count(dev);
		}
	}

	for (int i = 1; i < len; i++) {
		if (mpu_fifo_read_data(dev, &dev->dat->raw[i]) < 0)
			return -1;
		dev->dat->dat[i][0] = dev->dat->raw[i] * dev->dat->scl[i];
		dev->dat->dat[i][1] = dev->dat->dat[i][0];
	}
	if (dev->cfg->temp_fifo_en) {
		*(dev->t) += 36.53;
	}
#define square(x) ((x) * (x))
	if (dev->cfg->accel_fifo_en) {
		*(dev->Ax) -= (mpu_data_t)dev->cal->xa_bias;
		*(dev->Ay) -= (mpu_data_t)dev->cal->ya_bias;
		*(dev->Az) -= (mpu_data_t)dev->cal->za_bias;
		*(dev->AM) = (mpu_data_t)sqrt(square(*(dev->Ax)) + square(*(dev->Ay)) + square(*(dev->Az)));
	}
	if (dev->cfg->xg_fifo_en && dev->cfg->yg_fifo_en && dev->cfg->zg_fifo_en) {
		*(dev->Gx) -= (mpu_data_t)dev->cal->xg_bias;
		*(dev->Gy) -= (mpu_data_t)dev->cal->yg_bias;
		*(dev->Gz) -= (mpu_data_t)dev->cal->zg_bias;
	}
#undef square
	dev->samples++;

	return 0;
}
int mpu_ctl_i2c_mst_reset(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | I2C_MST_RESET_BIT) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_sig_cond_reset(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	uint8_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (mpu_write_byte(dev, USER_CTRL, val | SIG_COND_RESET_BIT) < 0)
		return -1;

	sleep(1);
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

inline void mpu_ctl_fix_axis(struct mpu_dev *dev)
{
	if (dev->cfg->accel_fifo_en) {
		*(dev->Ax) *= -1;
		*(dev->Ay) *= -1;
		*(dev->Az) *= -1;
	}
}

