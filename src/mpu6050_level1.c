#include "mpu6050_private.h"

int mpu_cfg_get_val(struct mpu_dev *dev, const mpu_reg_t reg, mpu_reg_t *val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (reg == 0) /* register 0 invalid */
		return -1;

	for (size_t i = 0; i < ARRAY_LEN(dev->cfg->regs); i++) {
		if (0 == dev->cfg->regs[i][0]) 	/* register 0 means unconfigured */
			continue;
		if (reg == dev->cfg->regs[i][0]) {/* seek register */
			*val = dev->cfg->regs[i][1];
			return 0;
		}
	}


	return -1; /* reg not found */
}

int mpu_cfg_set_val(struct mpu_dev *dev, const mpu_reg_t reg, const mpu_reg_t val)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (reg == 0) /* register 0 invalid */
		return -1;

	for (size_t i = 0; i < ARRAY_LEN(dev->cfg->regs); i++) {
		if (0 == dev->cfg->regs[i][0]) 	/* register 0 means unconfigured */
			continue;

		if(reg == dev->cfg->regs[i][0]) { /* seek register */
			dev->cfg->regs[i][1] = val;
			return 0;
		}
	}

	return -1; /* reg not found */
}

int mpu_cfg_write(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t reg; /* device register address */
	mpu_reg_t val; /* device register value */

	for (size_t i = 0; i < ARRAY_LEN(dev->cfg->regs); i++) {
		reg = dev->cfg->regs[i][0];
		val = dev->cfg->regs[i][1];

		if (mpu_write_byte(dev, reg, val) < 0) /* write error */
			return -1;
	}

	return 0;
}

int mpu_cfg_validate(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t dev_val; /* device register value */

	for (size_t i = 0; i < ARRAY_LEN(dev->cfg->regs); i++) {
		mpu_reg_t reg 	  = dev->cfg->regs[i][0];
		mpu_reg_t cfg_val = dev->cfg->regs[i][1];

		if (mpu_read_byte(dev, reg, &dev_val) < 0) /* read error */
			return -1;

		if (cfg_val != dev_val) /* value mismatch */
			return -1;
	}

	return 0;
}

int mpu_cfg_parse(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (mpu_cfg_parse_PWR_MGMT(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_FIFO_EN(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_ACCEL_CONFIG(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_GYRO_CONFIG(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_CONFIG(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_SMPLRT_DIV(dev) < 0)
	       return -1;
	if (mpu_cfg_parse_USER_CTRL(dev) < 0)
	       return -1;
	if (mpu_cfg_parse_INT_PIN_CFG(dev) < 0)
	        return -1;
	if (mpu_cfg_parse_INT_ENABLE(dev) < 0)
	         return -1;

	return 0;
}

int mpu_cfg_parse_PWR_MGMT(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val1;
	mpu_reg_t val2;

	if (mpu_cfg_get_val(dev, PWR_MGMT_1, &val1) < 0)
		return -1;

	if (mpu_cfg_get_val(dev, PWR_MGMT_2, &val2) < 0)
		return -1;

	if (val1 & DEVICE_RESET_BIT) /* must not be set */
		return -1;

	dev->cfg->sleep    = (val1 & SLEEP_BIT);
	dev->cfg->cycle    = (val1 & CYCLE_BIT);
	dev->cfg->temp_dis = (val1 & TEMP_DIS_BIT);
	dev->cfg->stdby_xa = (val2 & STDBY_XA_BIT);
	dev->cfg->stdby_ya = (val2 & STDBY_YA_BIT);
	dev->cfg->stdby_za = (val2 & STDBY_ZA_BIT);
	dev->cfg->stdby_xg = (val2 & STDBY_XG_BIT);
	dev->cfg->stdby_yg = (val2 & STDBY_YG_BIT);
	dev->cfg->stdby_zg = (val2 & STDBY_ZG_BIT);

	double	wake_freq = 0;
	int lp_wake_ctrl = (val2 & LP_WAKE_CTL_BIT) >> 6;

	if (!(dev->cfg->cycle)) {
		wake_freq = 0;
	} else {
		switch(lp_wake_ctrl) {
		case 0 : wake_freq = 1.2; break;
		case 1 : wake_freq = 5;	  break;
		case 2 : wake_freq = 20;  break;
		case 3 : wake_freq = 40;  break;
		}
	}
	dev->wake_freq	= wake_freq;

	return 0;
}

int mpu_cfg_parse_CONFIG(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, CONFIG, &val) < 0)
		return -1;

	/* ignore EXT_SYNC_SET to avoid complicating senors readings */
	if (val & EXT_SYNC_SET_BIT) { /* EXT_SYNC_SET not supported */
	       return -1;
	}

	int dlpf_cfg = (uint8_t)(val & DLPF_CFG_BIT);
	switch (dlpf_cfg) {
		case 0 : dev->abdw = 260; dev->adly =  0;
		         dev->gbdw = 256; dev->gdly =  0.98;
		         dev->gor  = 8000;
		         break;
		case 1 : dev->abdw = 184; dev->adly =  2.0;
		         dev->gbdw = 188; dev->gdly =  1.90;
		         dev->gor  = 1000;
		         break;
		case 2 : dev->abdw =  94; dev->adly =  3.0;
		         dev->gbdw =  98; dev->gdly =  2.80;
		         dev->gor  =  1000;
		         break;
		case 3 : dev->abdw =  44; dev->adly =  4.9;
		         dev->gbdw =  42; dev->gdly =  4.8;
		         dev->gor  =  1000;
		         break;
		case 4 : dev->abdw =  21; dev->adly =  8.5;
		         dev->gbdw =  20; dev->gdly =  8.3;
		         dev->gor  =  1000;
		         break;
		case 5 : dev->abdw =  10; dev->adly = 13.8;
		         dev->gbdw =  10; dev->gdly = 13.4;
		         dev->gor  =  1000;
		         break;
		case 6 : dev->abdw =   5; dev->adly = 19.0;
		         dev->gbdw =   5; dev->gdly = 18.6;
		         dev->gor  = 1000;
		         break;
		case 7 :
			return -1; /* error: reserved value */
		default :
			return -1; /* error: invalid value */
	}

	return 0;
}

int mpu_cfg_parse_SMPLRT_DIV(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (dev->gor == 0) /* must have gyro output rate */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, SMPLRT_DIV, &val) < 0)
		return -1;

	 /* all guaranteed to be greater than zero */
	double gyro_output_rate	  = (double)dev->gor;
	double samplerate_divisor = (double)(val + 1);
	double sampling_rate	  = gyro_output_rate / samplerate_divisor;
	double sampling_time	  = 1 / sampling_rate;

	dev->sr   = sampling_rate;
	dev->st	  = sampling_time;
	dev->dly.tv_sec = lrint(sampling_time);
	dev->dly.tv_nsec = 1000000000 * lrint(sampling_time - dev->dly.tv_sec);

	return 0;
}

int mpu_cfg_parse_ACCEL_CONFIG(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;

	dev->cfg->xa_st = (val & XA_ST_BIT);
	dev->cfg->ya_st = (val & YA_ST_BIT);
	dev->cfg->za_st = (val & ZA_ST_BIT);

	int afs_sel = (val & AFS_SEL_BIT) >> 3;
	switch (afs_sel) {
		case 0 : dev->afr =  2; dev->albs = 16384; break;
		case 1 : dev->afr =  4; dev->albs =  8192; break;
		case 2 : dev->afr =  8; dev->albs =  4096; break;
		case 3 : dev->afr = 16; dev->albs =  2048; break;
		default	:
			return -1; /* invalid value */
	}

	return 0;
}

int mpu_cfg_parse_GYRO_CONFIG(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;

	dev->cfg->xg_st = (val & XG_ST_BIT);
	dev->cfg->yg_st = (val & YG_ST_BIT);
	dev->cfg->zg_st = (val & ZG_ST_BIT);

	int fs_sel = (val & FS_SEL_BIT) >> 3;
	switch (fs_sel) {
		case 0 : dev->gfr =  250; dev->glbs = 131.0; break;
		case 1 : dev->gfr =  500; dev->glbs =  65.5; break;
		case 2 : dev->gfr = 1000; dev->glbs =  32.8; break;
		case 3 : dev->gfr = 2000; dev->glbs =  16.4; break;
		default	:
			return -1; /* invalid value */
	}

	return 0;
}

int mpu_cfg_parse_USER_CTRL(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;

	if (val & (FIFO_RESET_BIT | I2C_MST_RESET_BIT | SIG_COND_RESET_BIT | I2C_IF_DIS_BIT))
		return -1;

	dev->cfg->fifo_en    = (val & FIFO_EN_BIT);
	dev->cfg->i2c_mst_en = (val & I2C_MST_EN_BIT);
	dev->cfg->i2c_if_dis = (val & I2C_IF_DIS_BIT);

	return 0;
}

int mpu_cfg_parse_FIFO_EN(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;

	dev->cfg->temp_fifo_en  = (val & TEMP_FIFO_EN_BIT);
	dev->cfg->xg_fifo_en    = (val & XG_FIFO_EN_BIT);
	dev->cfg->yg_fifo_en    = (val & YG_FIFO_EN_BIT);
	dev->cfg->zg_fifo_en    = (val & ZG_FIFO_EN_BIT);
	dev->cfg->accel_fifo_en = (val & ACCEL_FIFO_EN_BIT);
	dev->cfg->slv2_fifo_en  = (val & SLV2_FIFO_EN_BIT);
	dev->cfg->slv1_fifo_en  = (val & SLV1_FIFO_EN_BIT);
	dev->cfg->slv0_fifo_en  = (val & SLV0_FIFO_EN_BIT);

	mpu_word_t words = 0; /* sensors written to fifo at each sampling time */
	if(dev->cfg->temp_fifo_en)	words += 1;
	if(dev->cfg->xg_fifo_en)	words += 1;
	if(dev->cfg->yg_fifo_en)	words += 1;
	if(dev->cfg->zg_fifo_en)	words += 1;
	if(dev->cfg->accel_fifo_en)	words += 3;
	if(dev->cfg->slv2_fifo_en)	words += 1;
	if(dev->cfg->slv1_fifo_en)	words += 1;
	if(dev->cfg->slv0_fifo_en)	words += 1;

	/* CRUCIAL - raw[0] = buffered sensors  */
	dev->dat->raw[0] = words;
	dev->fifosensors = words;
	dev->fifomax = 1023;

	return 0;
}

int mpu_cfg_parse_INT_ENABLE(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, INT_ENABLE, &val) < 0)
		return -1;

	dev->cfg->fifo_oflow_en = (val & FIFO_OFLOW_EN_BIT);
	dev->cfg->i2c_mst_en	= (val & I2C_MST_INT_EN_BIT);
	dev->cfg->data_rdy_en   = (val & DATA_RDY_EN_BIT);

	if (dev->cfg->i2c_mst_en) /* I2C_MST_EN not supported */
		return -1;

	return 0;
}

int mpu_cfg_parse_INT_PIN_CFG(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, INT_PIN_CFG, &val) < 0)
		return -1;

	if (val & FSYNC_INT_EN_BIT) /* FSYNC_INT_EN not supported */
		return -1;

	if (val & FSYNC_INT_EN_BIT) { dev->cfg->fsync_int_en = true; }

	return 0;
}
