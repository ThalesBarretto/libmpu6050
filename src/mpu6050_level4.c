// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include "mpu6050_private.h"
/*
 * API design
 * level 4 - User API see mpu6050_core.h
 * level 3 - basic controls
 * level 2 - internal structure management
 * level 1 - configuration registers parsing
 * level 0 - I2C bus communication
 */

int mpu_init(const char * const restrict path, struct mpu_dev ** mpudev, const int mode)
{
	if (NULL != *mpudev ) /* device not empty */
		return -1;

	size_t pathlen = strlen(path);
	if (NULL == (path ) || (pathlen < 6)) /* invalid path */
		return -1;

	struct mpu_dev *dev = NULL;
	if (mpu_dev_allocate(&dev) < 0) /* no memory allocated */
		return -1;

	if (mpu_dev_bind(path, MPU6050_ADDR, dev) < 0) /* could't bind */
		goto mpu_init_error;

	if (mpu_ctl_wake(dev) < 0) /* wake up failed */
		goto mpu_init_error;

	if (mpu_cfg_set_CLKSEL(dev, CLKSEL_3) < 0)
		goto mpu_init_error;

	if (mpu_dat_reset(dev) < 0) /* clean data pointers */
		goto mpu_init_error;

	switch (mode) {
		case MPU6050_RESET:
			if (mpu_cfg_reset(dev) < 0) /* assign default config */
				goto mpu_init_error;
			if (mpu_cal_reset(dev) < 0) /* assign unity gain, zero errors */
				goto mpu_init_error;
			if (mpu_cfg_set(dev) < 0) /* assign default config */
				goto mpu_init_error;
			break;
		case MPU6050_RESTORE:
			mpu_dev_parameters_restore(MPU6050_CFGFILE, dev); /* read config from device */
			/* fill device structure */
			if (mpu_cfg_parse(dev) < 0)
				return -1;
			break;
		default:
			fprintf(stderr, "mode unrecognized\n");
			goto mpu_init_error;
	}

	if (mpu_dat_set(dev) < 0) /* assign data pointers */
		goto mpu_init_error;

	if (mpu_read_byte(dev, PROD_ID, &(dev->prod_id)) < 0) /* get product id */
		goto mpu_init_error;

	if (mpu_ctl_fifo_flush(dev) < 0) /* DONT FORGET TO FLUSH FIFO */
	       return -1;

	*mpudev = dev; /* success */
	return 0;

mpu_init_error:
	if (NULL != dev->bus ) {
		if (close(*(dev->bus)) < 0) /* close failed */
			exit(EXIT_FAILURE);
	}

	if (mpu_destroy(dev) < 0) /* cleanup failed, check for bugs */
		exit(EXIT_FAILURE);

	return -1;
}

int mpu_destroy(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_dat_reset(dev);
	close(*(dev->bus));

	free(dev->cal); dev->cal = NULL;
	free(dev->dat); dev->dat = NULL;
	free(dev->cfg); dev->cfg = NULL;
	free(dev->bus); dev->bus = NULL;
	free(dev); dev = NULL;

	return 0;
}

int mpu_get_data(struct mpu_dev *dev)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	mpu_ctl_fifo_data(dev);
	mpu_ctl_fix_axis(dev);

	return 0;
}

int mpu_ctl_dlpf(struct mpu_dev *dev, unsigned int dlpf)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (dlpf > 6) /* invalid dlpf_cfg value */
		return -1;

	/* get device current DLPF_CFG value */
	mpu_reg_t val;
	if (mpu_read_byte(dev, CONFIG, &val) < 0)
		return -1;

	/* break circular dependencies */
	if((val & DLPF_CFG_BIT) == dlpf)
		return 0;

	unsigned int old_rate_hz = (unsigned int)dev->sr;

	if (mpu_cfg_get_val(dev, CONFIG, &val) < 0)
		return -1;

	val &= ~DLPF_CFG_BIT;	/* mask bits */
	val |= dlpf;		/* set bits */

	if (mpu_cfg_set_val(dev, CONFIG, val) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	dev->dlpf = dlpf;

	unsigned int new_rate_hz = (unsigned int)dev->sr;
	if (old_rate_hz != new_rate_hz) {
		if (mpu_ctl_samplerate(dev, old_rate_hz) < 0)
			return -1;
	}

	return 0;
}

int mpu_ctl_samplerate(struct mpu_dev *dev, unsigned int rate_hz)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	/* break circular dependencies */
	unsigned int old_rate_hz = (unsigned int)dev->sr;
	if( rate_hz == old_rate_hz) {
		return 0;
	}

	switch (rate_hz) { 		/* base 8kHz  / 1kHz	*/
		case 500: break;	/*	  40 	5	*/
		case 250: break;	/*	  40 	5	*/
		case 200: break;	/*	  40 	5	*/
		case 100: break;	/*	  80 	10	*/
		case  50: break;	/*	 160 	20	*/
		default: return -1;	/* rate not supported	*/
	}

	/* get DLPF_CFG value */
	mpu_reg_t val;
	if (mpu_read_byte(dev, CONFIG, &val) < 0)
		return -1;

	unsigned int fs_base = (val & DLPF_CFG_BIT) ? 1000 : 8000;
	unsigned int divider = (fs_base / rate_hz);
	val = (mpu_reg_t) (divider - 1); /* rate_hz = fs_base / (1 + SMPLRT_DIV) */

	if (mpu_cfg_set_val(dev, SMPLRT_DIV, val) < 0)
		return -1;
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_accel_range(struct mpu_dev *dev, unsigned int range)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t afs_sel = 0;
	switch (range) {
		case  2: afs_sel = AFS_SEL_0; break;
		case  4: afs_sel = AFS_SEL_1; break;
		case  8: afs_sel = AFS_SEL_2; break;
		case 16: afs_sel = AFS_SEL_3; break;
		default: return -1;	/* invalid range */
	}

	mpu_reg_t reg = ACCEL_CONFIG;
	mpu_reg_t val;

	if (mpu_read_byte(dev, reg, &val) < 0)
		return -1;

	val &= ~AFS_SEL_BIT;	/* mask bits */
	val |= afs_sel;		/* set bits */

	if (mpu_cfg_set_val(dev, reg, val) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	if (mpu_ctl_fifo_flush(dev) < 0)
	       return -1;

	return 0;
}

int mpu_ctl_gyro_range(struct mpu_dev *dev, unsigned int range)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	mpu_reg_t fs_sel = 0;
	switch (range) {
		case  250: fs_sel = FS_SEL_0; break;
		case  500: fs_sel = FS_SEL_1; break;
		case 1000: fs_sel = FS_SEL_2; break;
		case 2000: fs_sel = FS_SEL_3; break;
		default:
			return -1;	/* invalid range */
	}

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;

	val &= ~FS_SEL_BIT;	/* mask bits */
	val |= fs_sel;		/* set bits */

	if (mpu_cfg_set_val(dev, GYRO_CONFIG, val) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;
	if (mpu_ctl_fifo_flush(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_clocksource(struct mpu_dev *dev, unsigned int clksel)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	switch(clksel) {
		case CLKSEL_0: break; /* Internal 8 Mhz oscillator */
		case CLKSEL_1: break; /* PLL with X axis gyroscope */
		case CLKSEL_2: break; /* PLL with Y axis gyroscope */
		case CLKSEL_3: break; /* PLL with Z axis gyroscope */
		default: /* not supported */
			return -1;
	}

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, PWR_MGMT_1, &val) < 0)
		return -1;

	val  &= ~CLKSEL_BIT;	/* mask bits */
	val  |= clksel;		/* set  bits */

	if (mpu_cfg_set_val(dev, PWR_MGMT_1, val) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_calibrate(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	/* prepare the device for calibration */
	struct mpu_cfg *cfg_old = calloc(1, sizeof(struct mpu_cfg));
	memcpy((void *)cfg_old,  (void *)dev->cfg, sizeof(struct mpu_cfg));

	mpu_ctl_calibration_reset(dev); /* clear OFFS_USRH registers and calibration data */
	mpu_ctl_dlpf(dev, 5);
	mpu_ctl_accel_range(dev, 16);
	mpu_ctl_gyro_range(dev, 1000);
	mpu_ctl_fifo_flush(dev);
	dev->cal->samples = 1000;

	long double xa_bias = 0;
	long double ya_bias = 0;
	long double za_bias = 0;
	long double xg_bias = 0;
	long double yg_bias = 0;
	long double zg_bias = 0;
	long double AM_bias = 0;
	for (int i = 0; i < dev->cal->samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_bias += *(dev->Ax);
		ya_bias += *(dev->Ay);
		za_bias += *(dev->Az);
		xg_bias += *(dev->Gx);
		yg_bias += *(dev->Gy);
		zg_bias += *(dev->Gz);
		AM_bias += *(dev->AM);
	}
	/* take the average difference */
	xa_bias /= dev->cal->samples;
	ya_bias /= dev->cal->samples;
	za_bias /= dev->cal->samples;
	//AM_bias /= dev->cal->samples;
	xg_bias /= dev->cal->samples;
	yg_bias /= dev->cal->samples;
	zg_bias /= dev->cal->samples;

	/* in LSB's, scale things to 1'g acceleration */
	long double a_factor = (dev->albs *  dev->cal->AM_bias);
	dev->cal->xa_cust = (dev->cal->xa_orig - (int16_t)((xa_bias) * a_factor));
	dev->cal->ya_cust = (dev->cal->ya_orig - (int16_t)((ya_bias) * a_factor));
	dev->cal->za_cust = (dev->cal->za_orig - (int16_t)((za_bias) * a_factor));
	mpu_write_byte(dev, XA_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->xa_cust)>>8)&0xFF));
	mpu_write_byte(dev, XA_OFFS_USRL,(uint8_t)((((uint16_t)dev->cal->xa_cust)    &0xFE) | (dev->cal->xa_orig & 0x1)));
	mpu_write_byte(dev, YA_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->ya_cust)>>8)&0xFF));
	mpu_write_byte(dev, YA_OFFS_USRL,(uint8_t)((((uint16_t)dev->cal->ya_cust)    &0xFE) | (dev->cal->ya_orig & 0x1)));
	mpu_write_byte(dev, ZA_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->za_cust)>>8)&0xFF));
	mpu_write_byte(dev, ZA_OFFS_USRL,(uint8_t)((((uint16_t)dev->cal->za_cust)    &0xFE) | (dev->cal->za_orig & 0x1)));

	dev->cal->xg_cust = (dev->cal->xg_orig - (int16_t)(xg_bias * dev->glbs));
	dev->cal->yg_cust = (dev->cal->yg_orig - (int16_t)(yg_bias * dev->glbs));
	dev->cal->zg_cust = (dev->cal->zg_orig - (int16_t)(zg_bias * dev->glbs));
	mpu_write_byte(dev, XG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->xg_cust)>>8)&0xFF));
	mpu_write_byte(dev, XG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->xg_cust)&0xFF);
	mpu_write_byte(dev, YG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->yg_cust)>>8)&0xFF));
	mpu_write_byte(dev, YG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->yg_cust)&0xFF);
	mpu_write_byte(dev, ZG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->zg_cust)>>8)&0xFF));
	mpu_write_byte(dev, ZG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->zg_cust)&0xFF);

	/* second pass - fine */
	xa_bias = 0;
	ya_bias = 0;
	za_bias = 0;
	AM_bias = 0;
	xg_bias = 0;
	yg_bias = 0;
	zg_bias = 0;
	/* store register values */
	mpu_ctl_fifo_flush(dev);
	for (int i = 0; i <  dev->cal->samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_bias += *(dev->Ax);
		ya_bias += *(dev->Ay);
		za_bias += *(dev->Az);
		AM_bias += *(dev->AM);
		xg_bias += *(dev->Gx);
		yg_bias += *(dev->Gy);
		zg_bias += *(dev->Gz);
	}
	xg_bias /= dev->cal->samples;
	yg_bias /= dev->cal->samples;
	zg_bias /= dev->cal->samples;
	mpu_read_data(dev, XG_OFFS_USRH, &dev->cal->xg_orig);
	mpu_read_data(dev, YG_OFFS_USRH, &dev->cal->yg_orig);
	mpu_read_data(dev, ZG_OFFS_USRH, &dev->cal->zg_orig);
	dev->cal->xg_cust = (dev->cal->xg_orig - (int16_t)(xg_bias * dev->glbs));
	dev->cal->yg_cust = (dev->cal->yg_orig - (int16_t)(yg_bias * dev->glbs));
	dev->cal->zg_cust = (dev->cal->zg_orig - (int16_t)(zg_bias * dev->glbs));
	mpu_write_byte(dev, XG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->xg_cust)>>8)&0xFF));
	mpu_write_byte(dev, XG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->xg_cust)&0xFF);
	mpu_write_byte(dev, YG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->yg_cust)>>8)&0xFF));
	mpu_write_byte(dev, YG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->yg_cust)&0xFF);
	mpu_write_byte(dev, ZG_OFFS_USRH,(uint8_t)((((uint16_t)dev->cal->zg_cust)>>8)&0xFF));
	mpu_write_byte(dev, ZG_OFFS_USRL,(uint8_t)((uint16_t)dev->cal->zg_cust)&0xFF);

	xa_bias = 0;
	ya_bias = 0;
	za_bias = 0;
	AM_bias = 0;
	xg_bias = 0;
	yg_bias = 0;
	zg_bias = 0;
	for (int i = 0; i < dev->cal->samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_bias += *(dev->Ax);
		ya_bias += *(dev->Ay);
		za_bias += *(dev->Az);
		AM_bias += *(dev->AM);
		xg_bias += *(dev->Gx);
		yg_bias += *(dev->Gy);
		zg_bias += *(dev->Gz);
	}
	/* take the average difference */
	xa_bias /= dev->cal->samples;
	ya_bias /= dev->cal->samples;
	za_bias /= dev->cal->samples;
	AM_bias /= dev->cal->samples;
	xg_bias /= dev->cal->samples;
	yg_bias /= dev->cal->samples;
	zg_bias /= dev->cal->samples;

	/* remove gravity */
	AM_bias -= 1;
	za_bias -= 1;
	dev->cal->xa_bias = xa_bias;
	dev->cal->ya_bias = ya_bias;
	dev->cal->za_bias = za_bias;
	dev->cal->xg_bias = xg_bias;
	dev->cal->yg_bias = yg_bias;
	dev->cal->zg_bias = zg_bias;
	dev->cal->AM_bias = AM_bias;

	/* restore old config */
	memcpy((void *)dev->cfg, (void *)cfg_old, sizeof(struct mpu_cfg));
	free(cfg_old);
	cfg_old = NULL;

	mpu_cfg_set(dev);
	mpu_dat_set(dev);
	mpu_dev_parameters_save(MPU6050_CFGFILE, dev);
	mpu_ctl_fifo_flush(dev);

	return 0;
}

int mpu_ctl_reset(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (mpu_write_byte(dev, PWR_MGMT_1,DEVICE_RESET_BIT) < 0)
		return -1;

	sleep(1);
	if(mpu_ctl_wake(dev) < 0)
		return -1;

	sleep(1);
	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_reset(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

int mpu_ctl_selftest(struct mpu_dev *dev, char *fname)
{
	if (MPUDEV_IS_NULL(dev))
		return -1;

	/* prepare the device for self-test */
	struct mpu_cfg *cfg_old = calloc(1, sizeof(struct mpu_cfg));
	memcpy((void *)cfg_old,  (void *)dev->cfg, sizeof(struct mpu_cfg));

	mpu_ctl_dlpf(dev,0);
	mpu_ctl_samplerate(dev, 100);
	mpu_ctl_clocksource(dev, 3);
	mpu_ctl_accel_range(dev, 8);
	mpu_ctl_gyro_range(dev, 250);
	mpu_ctl_fifo_disable_accel(dev);
	mpu_ctl_fifo_disable_temp(dev);
	mpu_ctl_fifo_disable_gyro(dev);

	long double xa_st_on  = 0;
	long double ya_st_on  = 0;
	long double za_st_on  = 0;
	long double xg_st_on  = 0;
	long double yg_st_on  = 0;
	long double zg_st_on  = 0;
	long double xa_st_off = 0;
	long double ya_st_off = 0;
	long double za_st_off = 0;
	long double xg_st_off = 0;
	long double yg_st_off = 0;
	long double zg_st_off = 0;

	int samples = 100;
	/* Accel self-test response */
	mpu_ctl_fifo_enable_accel(dev);
	mpu_ctl_selftest_enable_accel(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_st_on += *(dev->Ax);
		ya_st_on += *(dev->Ay);
		za_st_on += *(dev->Az);
	}
	mpu_ctl_selftest_disable_accel(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_st_off += *(dev->Ax);
		ya_st_off += *(dev->Ay);
		za_st_off += *(dev->Az);
	}
	mpu_ctl_fifo_disable_accel(dev);
	/* Gyro self-test response */
	mpu_ctl_fifo_enable_gyro(dev);
	mpu_ctl_selftest_enable_gyro(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xg_st_on += *(dev->Gx);
		yg_st_on += *(dev->Gy);
		zg_st_on += *(dev->Gz);
	}
	mpu_ctl_selftest_disable_gyro(dev);
	for (int i = 0; i < samples; i++) {
		mpu_ctl_fifo_data(dev);
		xg_st_off += *(dev->Gx);
		yg_st_off += *(dev->Gy);
		zg_st_off += *(dev->Gz);
	}
	mpu_ctl_fifo_disable_gyro(dev);

	long double xa_str = fabsl(xa_st_on - xa_st_off)/(long double)samples;
	long double ya_str = fabsl(ya_st_on - ya_st_off)/(long double)samples;
	long double za_str = fabsl(za_st_on - za_st_off)/(long double)samples;
	long double xg_str = fabsl(xg_st_on - xg_st_off)/(long double)samples;
	long double yg_str = fabsl(yg_st_on - yg_st_off)/(long double)samples;
	long double zg_str = fabsl(zg_st_on - zg_st_off)/(long double)samples;

	/* get self-test registers */
	uint8_t stx = 0;
	uint8_t sty = 0;
	uint8_t stz = 0;
	uint8_t sta = 0;
	mpu_read_byte(dev, SELF_TEST_X, &stx);
	mpu_read_byte(dev, SELF_TEST_Y, &sty);
	mpu_read_byte(dev, SELF_TEST_Z, &stz);
	mpu_read_byte(dev, SELF_TEST_A, &sta);
	uint8_t xa_st = ((stx & XA_TEST_42_BIT) >> 3) | ((sta & XA_TEST_10_BIT) >> 4);
	uint8_t ya_st = ((sty & YA_TEST_42_BIT) >> 3) | ((sta & YA_TEST_10_BIT) >> 2);
	uint8_t za_st = ((stz & ZA_TEST_42_BIT) >> 3) |  (sta & ZA_TEST_10_BIT);
	uint8_t xg_st =  (stx & XG_TEST_40_BIT);
	uint8_t yg_st =  (sty & YG_TEST_40_BIT);
	uint8_t zg_st =  (stz & ZG_TEST_40_BIT);

	/* ACCEL get factory trim values - datasheet formula for +-8g */
	long double ft_xa = 4096.0L * 0.34L * pow(0.92L/0.34L, ((long double)(xa_st -1)/14.0L));
	long double ft_ya = 4096.0L * 0.34L * pow(0.92L/0.34L, ((long double)(ya_st -1)/14.0L));
	long double ft_za = 4096.0L * 0.34L * pow(0.92L/0.34L, ((long double)(za_st -1)/14.0L));
	/* GYRO get factory trim values - datasheet fomula for +-250 dps */
	long double ft_xg =  25.0L * 131.0L * pow(1046L, (long double)(xg_st -1));
	long double ft_yg = -25.0L * 131.0L * pow(1046L, (long double)(yg_st -1));
	long double ft_zg =  25.0L * 131.0L * pow(1046L, (long double)(zg_st -1));

	long double shift_xa = (xa_str - ft_xa)/ft_xa;
	long double shift_ya = (ya_str - ft_ya)/ft_ya;
	long double shift_za = (za_str - ft_za)/ft_za;
	long double shift_xg = (xg_str - ft_xg)/ft_xg;
	long double shift_yg = (yg_str - ft_yg)/ft_yg;
	long double shift_zg = (zg_str - ft_zg)/ft_zg;

	FILE *fp;
	if (NULL == (fp = fopen(fname, "w+"))) {
		fprintf(stderr, "Couldn't open \"%s\" for self-test results! - logging to stderr.\n", fname);
	} else {
		fprintf(fp, "Self-test results: Xa = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_xa), fabsl(shift_xa) < 14.0L ? "PASS" : "FAIL");
		fprintf(fp, "Self-test resutls: Ya = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_ya), fabsl(shift_ya) < 14.0L ? "PASS" : "FAIL");
		fprintf(fp, "Self-test resutls: Za = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_za), fabsl(shift_za) < 14.0L ? "PASS" : "FAIL");
		fprintf(fp, "Self-test resutls: Xg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_xg), fabsl(shift_xg) < 14.0L ? "PASS" : "FAIL");
		fprintf(fp, "Self-test resutls: Yg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_yg), fabsl(shift_yg) < 14.0L ? "PASS" : "FAIL");
		fprintf(fp, "Self-test resutls: Zg = %Lf%% shift from factory trim (%4s)\n", fabsl(shift_zg), fabsl(shift_zg) < 14.0L ? "PASS" : "FAIL");
	};

	/* restore old config */
	memcpy((void *)dev->cfg, (void *)cfg_old, sizeof(struct mpu_cfg));
	free(cfg_old);
	cfg_old = NULL;

	mpu_cfg_set(dev);
	mpu_dat_set(dev);
	mpu_ctl_fifo_flush(dev);

	return 0;
}

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
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if(NULL == fn) {
		fprintf(stderr, "%s failed: NULL filename\n", __func__);
		return -1;
	}

	FILE *fp;
	if (NULL == (fp = fopen(fn, "w+"))) {
		fprintf(stderr, "%s failed: Unable to open file \"%s\"\n", __func__, fn);
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

