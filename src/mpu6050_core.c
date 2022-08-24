// SPDX-License-Identifier: MIT
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#include "mpu6050_core.h"
#include "mpu6050_regs.h"

#include <stdlib.h>		/* for malloc(), free(), exit() */
#include <stdint.h>		/* for uint8_t, uint16_t, etc */
#include <string.h>		/* for memcpy(), strlen() */
#include <stdio.h>
#include <tgmath.h>		/* for sin, cos, tan, atan2 etc */
#include <fcntl.h>		/* for open() */
#include <unistd.h>		/* for close(), write(), getopt(), size_t */
#include <sys/types.h>		/* for ssize_t */
#include <sys/ioctl.h>		/* for ioctl() */
#include <linux/i2c-dev.h>	/* for i2c_smbus_x */
#include <linux/i2c-dev.h>	/* for i2c_smbus_x */
#include <i2c/smbus.h> 		/* for i2c_smbus_x */
#include <linux/i2c.h> 		/* for i2c_smbus_x */

/* stores calibration related values for reference */
struct mpu_cal {
	mpu_data_t gra;		/* mean(sqrt(ax2,ay2,az2)[])		*/
	mpu_data_t off[32];	/* ax - mean(ax[n]),.	..		*/
	mpu_data_t gai[32];	/* ax[expec_1g]/ax[measured_1g],...	*/
	mpu_data_t dri[32];	/* delta(mean(ax[n])/delta(time),...	*/
	int16_t xa_orig;	/* original XA calibration register value */
	int16_t ya_orig;	/* original YA calibration register value */
	int16_t za_orig;	/* original ZA calibration register value */
	int16_t xg_orig;	/* original XG calibration register value */
	int16_t yg_orig;	/* original YG calibration register value */
	int16_t zg_orig;	/* original ZG calibration register value */
	int16_t xa_cust;	/* custom XA calibration register value */
	int16_t ya_cust;	/* custom YA calibration register value */
	int16_t za_cust;	/* custom ZA calibration register value */
	int16_t xg_cust;	/* custom XG calibration register value */
	int16_t yg_cust;	/* custom YG calibration register value */
	int16_t zg_cust;	/* custom ZG calibration register value */
	int samples;		/* samples used in calibration */
	long double xa_bias;	/* found XA value bias */
	long double ya_bias;	/* found YA value bias */
	long double za_bias;	/* found ZA value bias */
	long double xg_bias;	/* found XG value bias */
	long double yg_bias;	/* found YG value bias */
	long double zg_bias;	/* found ZG value bias */
	long double AM_bias;	/* found AM value bias */
	long double GM_bias;	/* found GM value bias */
};

/* stores sensor data collection related values */
struct mpu_dat {
	int16_t raw[32];	/* raw sensor data	*/
	mpu_data_t scl[32];	/* scaling factors	*/
	mpu_data_t dat[32][2];	/* scaled data		*/
	mpu_data_t squ[32];	/* squared data		*/
	mpu_data_t mea[32];	/* data mean		*/
	mpu_data_t var[32];	/* data variance	*/
	mpu_data_t AM;		/* accel magnitude	*/
	mpu_data_t GM;		/* gyro rate magnitude	*/
};

/* Mirrors configuration register values and their meaning */
struct mpu_cfg {
	mpu_reg_t cfg[16][2];	/* configuration register values */
	/* configuration bits   -> REGISTER location */
	bool sleep; 		/* PWR_MGMGT_1 */
	bool cycle; 		/* PWR_MGMGT_1 */
	bool temp_dis;		/* PWR_MGMGT_1 */
	bool xg_st;		/* GYRO_CONFIG */
	bool yg_st;		/* GYRO_CONFIG */
	bool zg_st;		/* GYRO_CONFIG */
	bool xa_st;		/* ACCEL_CONFIG */
	bool ya_st;		/* ACCEL_CONFIG */
	bool za_st;		/* ACCEL_CONFIG */
	bool stdby_xa; 		/* PWR_MGMGT_2 */
	bool stdby_ya; 		/* PWR_MGMGT_2 */
	bool stdby_za; 		/* PWR_MGMGT_2 */
	bool stdby_xg; 		/* PWR_MGMGT_2 */
	bool stdby_yg; 		/* PWR_MGMGT_2 */
	bool stdby_zg; 		/* PWR_MGMGT_2 */
	bool fifo_en;		/* USER_CTRL */
	bool i2c_mst_en; 	/* USER_CTRL */
	bool i2c_if_dis;	/* USER_CTRL */
	bool temp_fifo_en;  	/* FIFO_EN */
	bool accel_fifo_en; 	/* FIFO_EN */
	bool xg_fifo_en;  	/* FIFO_EN */
	bool yg_fifo_en;  	/* FIFO_EN */
	bool zg_fifo_en;  	/* FIFO_EN */
	bool slv0_fifo_en;  	/* FIFO_EN */
	bool slv1_fifo_en;  	/* FIFO_EN */
	bool slv2_fifo_en;  	/* FIFO_EN */
	bool slv3_fifo_en;  	/* I2C_MST_CTRL */
	bool slv4_fifo_en;  	/* I2C_MST_CTRL */
	bool fsync_int_en;  	/* INT_PIN_CFG	__not_supported__ */
	bool fifo_oflow_en;  	/* INT_ENABLE */
	bool i2c_mst_int_en;  	/* INT_ENABLE */
	bool data_rdy_en;  	/* INT_ENABLE */
};

#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

/* The default values for configuration registers */
const struct mpu_cfg mpu6050_defcfg = {
	.cfg =	{
		{ PWR_MGMT_1,   0x03},	/* power on, temp enabled, clock gyro_z */
		{ PWR_MGMT_2,   0x00},	/* no standby, full on			*/
		{ CONFIG,       0x00},	/* dlpf off				*/
		{ SMPLRT_DIV,   0x4F},	/* divisor = 80(1+79), rate = 100	*/
		{ ACCEL_CONFIG, 0x00},	/* +-2g 				*/
		{ GYRO_CONFIG,  0x00},	/* +-250 deg/s 				*/
		{ USER_CTRL,    0x60},	/* fifo enabled, aux i2c master mode	*/
		{ FIFO_EN,  	0xF8},	/* temp, accel, gyro buffered		*/
		{ INT_PIN_CFG,  0x00},	/* interrupts disabled			*/
		{ INT_ENABLE,   0x00},	/* interrupts disabled			*/
	}
};

/* this should be defined in time.h, but the linter complains */
extern int nanosleep(const struct timespec *rqtp, struct timespec *rmtp);

/* helpers - internal use only */
#define MPUDEV_IS_NULL(dev)	((NULL == (dev)) ||      \
				 (NULL == (dev)->dat) || \
				 (NULL == (dev)->cfg) || \
				 (NULL == (dev)->bus) || \
				 (NULL == (dev)->cal))

#define MPUDEV_NOT_NULL(dev)	((NULL != (dev)) &&      \
				 (NULL != (dev)->dat) && \
				 (NULL != (dev)->cfg) && \
				 (NULL != (dev)->bus) && \
				 (NULL != (dev)->cal))

/*
 * API design
 * level 4 - User API see mpu6050_core.h
 * level 3 - basic controls
 * level 2 - internal structure management
 * level 1 - configuration registers parsing
 * level 0 - I2C bus communication
 */

/* level 3 - basic controls */
static int mpu_diagnose(struct mpu_dev *dev);
static int mpu_ctl_selftest_enable_accel (struct mpu_dev *dev);
static int mpu_ctl_selftest_enable_gyro  (struct mpu_dev *dev);
static int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev);
static int mpu_ctl_selftest_disable_gyro (struct mpu_dev *dev);
static int mpu_ctl_wake(struct mpu_dev *dev);
static int mpu_ctl_fifo_count(struct mpu_dev *dev);
static int mpu_ctl_fifo_flush(struct mpu_dev *dev);
static int mpu_ctl_fifo_enable_temp  (struct mpu_dev *dev);
static int mpu_ctl_fifo_enable_accel (struct mpu_dev *dev);
static int mpu_ctl_fifo_enable_gyro  (struct mpu_dev *dev);
static int mpu_ctl_fifo_disable_temp (struct mpu_dev *dev);
static int mpu_ctl_fifo_disable_accel(struct mpu_dev *dev);
static int mpu_ctl_fifo_disable_gyro (struct mpu_dev *dev);
static int mpu_fifo_data(struct mpu_dev *dev, int16_t *data);
static inline void mpu_ctl_fix_axis(struct mpu_dev *dev);
static int mpu_ctl_fifo_data(struct mpu_dev *dev);
static int mpu_ctl_fifo_reset(struct mpu_dev *dev);
static int mpu_ctl_i2c_mst_reset(struct mpu_dev *dev);

/* level 2 - internal structure management */
static int mpu_dev_bind(const char *path,
			const mpu_reg_t address,
			struct mpu_dev *dev);

static int mpu_dev_allocate(struct mpu_dev **dev);
static int mpu_cfg_set(struct mpu_dev *dev);
static int mpu_dat_set(struct mpu_dev *dev);
static int mpu_cfg_reset(struct mpu_dev *dev);
static int mpu_dat_reset(struct mpu_dev *dev);
static int mpu_cal_reset(struct mpu_dev *dev);

static int mpu_dev_parameters_save(char *fn, struct mpu_dev *dev);
static int mpu_dev_parameters_restore(char *fn, struct mpu_dev *dev);

static int mpu_ctl_calibration_reset(struct mpu_dev *dev);
static int mpu_ctl_calibration_restore(struct mpu_dev *dev, struct mpu_cal *bkp);

static int mpu_cfg_set_CLKSEL(struct mpu_dev *dev,
			      mpu_reg_t clksel);

/* level 1 - configuration registers parsing */
static int mpu_cfg_get_val(struct mpu_dev *dev,
			   const mpu_reg_t reg,
			   mpu_reg_t *val);

static int mpu_cfg_set_val(struct mpu_dev *dev,
			   const mpu_reg_t reg,
			   const mpu_reg_t val);
static int mpu_cfg_write(	struct mpu_dev *dev);
static int mpu_cfg_validate(	struct mpu_dev *dev);
static int mpu_cfg_parse(	struct mpu_dev *dev);
static int mpu_cfg_parse_PWR_MGMT(	struct mpu_dev *dev);
static int mpu_cfg_parse_CONFIG(	struct mpu_dev *dev);
static int mpu_cfg_parse_SMPLRT_DIV(	struct mpu_dev *dev);
static int mpu_cfg_parse_ACCEL_CONFIG(	struct mpu_dev *dev);
static int mpu_cfg_parse_GYRO_CONFIG(	struct mpu_dev *dev);
static int mpu_cfg_parse_USER_CTRL(	struct mpu_dev *dev);
static int mpu_cfg_parse_FIFO_EN(	struct mpu_dev *dev);
static int mpu_cfg_parse_INT_ENABLE(	struct mpu_dev *dev);
static int mpu_cfg_parse_INT_PIN_CFG(	struct mpu_dev *dev);

/* level 0  i2c bus communication */
static int mpu_read_byte( struct mpu_dev * const dev, const mpu_reg_t reg, mpu_reg_t *val);
static int mpu_read_word( struct mpu_dev * const dev, const mpu_reg_t reg, mpu_word_t *val);
static int mpu_read_data( struct mpu_dev * const dev, const mpu_reg_t reg, int16_t *val);
static int mpu_write_byte(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_reg_t val);
static int mpu_write_word(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_word_t val);

int mpu_init(	const char * const restrict path,
		struct mpu_dev ** mpudev,
		const int mode)
{
	if (*mpudev != NULL) /* device not empty */
		return -1;

	size_t pathlen = strlen(path);
	if ((path == NULL) || (pathlen < 6)) /* invalid path */
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
	if (dev->bus != NULL) {
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

static int mpu_ctl_wake(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus))
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
	val &= (mpu_reg_t) ~(DEVICE_RESET_BIT | SLEEP_BIT | CYCLE_BIT);
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

static int mpu_cfg_set_CLKSEL(struct mpu_dev *dev, mpu_reg_t clksel)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	if (clksel > 7) /* out of range  */
		return -1;

	switch(clksel) {
		case CLKSEL_0 : dev->clksel = 0; break; /* Internal 8 Mhz oscillator		*/
		case CLKSEL_1 : dev->clksel = 1; break; /* PLL with X axis gyroscope		*/
		case CLKSEL_2 : dev->clksel = 2; break; /* PLL with Y axis gyroscope		*/
		case CLKSEL_3 : dev->clksel = 3; break; /* PLL with Z axis gyroscope		*/
		case CLKSEL_4 : dev->clksel = 4; break; /* PLL with external 32.768 kHz reference	*/
		case CLKSEL_5 : dev->clksel = 5; break; /* PLL with external 19.2 MHz reference	*/
		case CLKSEL_6 : dev->clksel = 6; break; /* RESERVED */
		case CLKSEL_7 : dev->clksel = 7; break; /* Stops the clock, keeps timing in reset */
		default: /* not supported */
			return -1;
	}

	mpu_reg_t val;
	if (mpu_read_byte(dev, PWR_MGMT_1, &val) < 0)
		return -1;

	val  &= ~CLKSEL_BIT; /* mask CLK_SEL bits */
	val  |= clksel;	 /* set  CLK_SEL bits */

	if (mpu_write_byte(dev, PWR_MGMT_1, val) < 0)
		return -1;

	return 0;
}

int mpu_ctl_dlpf(struct mpu_dev *dev, unsigned int dlpf)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;
	if (dlpf > 7) { /* invalid dlpf_cfg value */
		return -1;
	}

	/* get DLPF_CFG value */
	mpu_reg_t val;
	if (mpu_read_byte(dev, CONFIG, &val) < 0) {
		return -1;
	}
	/* break circular dependencies */
	if((val & DLPF_CFG_BIT) == (dlpf & DLPF_CFG_BIT)) {
		return 9;
	}

	unsigned int old_rate_hz = (unsigned int)dev->sr;

	if (mpu_cfg_get_val(dev, CONFIG, &val) < 0)
		return -1;
	val &= ~DLPF_CFG_BIT;
	val |= (dlpf & DLPF_CFG_BIT);

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
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	/* break circular dependencies */
	unsigned int old_rate_hz = (unsigned int)dev->sr;
	if( rate_hz == old_rate_hz) {
		return 9;
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
	if ((NULL == dev) || (NULL == dev->bus))
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

	val &= ~AFS_SEL_BIT;  /* mask bits */
	val |= afs_sel; 	/* set bits */

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
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	mpu_reg_t fs_sel = 0;
	switch (range) {
		case  250: fs_sel = FS_SEL_0; break;
		case  500: fs_sel = FS_SEL_1; break;
		case 1000: fs_sel = FS_SEL_2; break;
		case 2000: fs_sel = FS_SEL_3; break;
		default: return -1;	/* invalid range */
	}

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;

	val &= ~FS_SEL_BIT;  /* mask bits */
	val |= fs_sel; 		/* set bits */

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

int mpu_ctl_clocksource(struct mpu_dev *dev, mpu_reg_t clksel)
{
	if ((NULL == dev) || (NULL == dev->bus))
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

	val  &= ~CLKSEL_BIT; /* mask CLK_SEL bits */
	val  |= clksel;	 /* set  CLK_SEL bits */

	if (mpu_cfg_set_val(dev, PWR_MGMT_1, val) < 0)
		return -1;

	if (mpu_cfg_set(dev) < 0)
		return -1;
	if (mpu_dat_set(dev) < 0)
		return -1;

	return 0;
}

static int mpu_cfg_reset(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus) || (NULL == dev->cfg))
		return -1;

	memcpy((void *)dev->cfg, (void *)&mpu6050_defcfg, sizeof(struct mpu_cfg));

	if (mpu_cfg_set(dev) < 0) /* couldn't set config */
		return -1;

	return 0;
}

static int mpu_cfg_set(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus) || (NULL == dev->cfg))
		return -1;

	/* write config to device registers */
	if (mpu_cfg_write(dev) < 0)
		return -1;

	/* ensure device is configured */
	if (mpu_cfg_validate(dev) < 0)
		return -1;

	/* fill device structure */
	if (mpu_cfg_parse(dev) < 0)
		return -1;


	mpu_dev_parameters_save(MPU6050_CFGFILE, dev);

	return 0;
}

static int mpu_cfg_write(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t reg; /* device register address */
	mpu_reg_t val; /* device register value */

	unsigned int i = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);

	for (i = 0; i < len; i++) {
		reg = dev->cfg->cfg[i][0];
		val = dev->cfg->cfg[i][1];

		if (mpu_write_byte(dev, reg, val) < 0) /* write error */
			return -1;
	}


	return 0;
}

static int mpu_cfg_validate(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t dev_val; /* device register value */

	unsigned int i = 1;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);

	for (i = 0; i < len; i++) {
		mpu_reg_t reg 	  = dev->cfg->cfg[i][0];
		mpu_reg_t cfg_val = dev->cfg->cfg[i][1];

		if (mpu_read_byte(dev, reg, &dev_val) < 0) /* read error */
			return -1;


		if (cfg_val != dev_val) /* value mismatch */
			return -1;
	}


	return 0;
}

static int mpu_cfg_get_val(struct mpu_dev *dev, const mpu_reg_t reg, mpu_reg_t *val)
{
	if ((NULL == dev) || (NULL == dev->cfg) || (val == NULL))
		return -1;

	if (reg == 0) /* register 0 invalid */
		return -1;

	unsigned int i   = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);

	for (i = 0; i < len; i++) {
		if (0 == dev->cfg->cfg[i][0]) 	/* register 0 invalid */
			continue;
		if (reg == dev->cfg->cfg[i][0]) {/* seek register */
			*val = dev->cfg->cfg[i][1];
			return 0;
		}

	}


	return -1; /* reg not found */
}

static int mpu_cfg_set_val(struct mpu_dev *dev, const mpu_reg_t reg, const mpu_reg_t val)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	if (reg == 0) /* register 0 invalid */
		return -1;

	unsigned int i = 0;
	unsigned int len = sizeof(dev->cfg->cfg)/sizeof(dev->cfg->cfg[0]);

	for (i = 0; i < len; i++) {
		if (0 == dev->cfg->cfg[i][0]) 	/* register 0 invalid */
			continue;

		if(reg == dev->cfg->cfg[i][0]) { /* seek register */
			dev->cfg->cfg[i][1] = val;
			return 0;
		}
	}

	return -1; /* reg not found */
}

static int mpu_cfg_parse_PWR_MGMT(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	mpu_reg_t val1;
	mpu_reg_t val2;

	if (mpu_cfg_get_val(dev, PWR_MGMT_1, &val1) < 0)
		return -1;

	if (mpu_cfg_get_val(dev, PWR_MGMT_2, &val2) < 0)
		return -1;

	if (val1 & DEVICE_RESET_BIT) /* must not be set */
		return -1;

	if (val1 & SLEEP_BIT)	 { dev->cfg->sleep = true;    }
	if (val1 & CYCLE_BIT)	 { dev->cfg->cycle = true;    }
	if (val1 & TEMP_DIS_BIT) { dev->cfg->temp_dis = true; }
	if (val2 & STDBY_XA_BIT) { dev->cfg->stdby_xa = true; }
	if (val2 & STDBY_YA_BIT) { dev->cfg->stdby_ya = true; }
	if (val2 & STDBY_ZA_BIT) { dev->cfg->stdby_za = true; }
	if (val2 & STDBY_XG_BIT) { dev->cfg->stdby_xg = true; }
	if (val2 & STDBY_YG_BIT) { dev->cfg->stdby_yg = true; }
	if (val2 & STDBY_ZG_BIT) { dev->cfg->stdby_zg = true; }

	double	wake_freq = 0;
	int lp_wake_ctrl = (val2 & 0xC0u) >> 6;

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

static int mpu_cfg_parse_USER_CTRL(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, USER_CTRL, &val) < 0)
		return -1;
	if (val & FIFO_RESET_BIT) {
		//printf("FIFO_RESET_BIT set \n");
		return -1;
	}
	if (val & I2C_MST_RESET_BIT) {
		//printf("I2C_MST_RESET_BIT set\n");
		return -1;

	}
	if (val & SIG_COND_RESET_BIT) {
		//printf("SIG_COND_RESET_BIT set\n");
		return -1;
	}

	if(val & I2C_IF_DIS_BIT) /* this is valid but will disable i2c communication */
		return -1;

	dev->cfg->fifo_en	= (val & FIFO_EN_BIT)	 ? true : false;
	dev->cfg->i2c_mst_en	= (val & I2C_MST_EN_BIT) ? true : false;
	dev->cfg->i2c_if_dis	= (val & I2C_IF_DIS_BIT) ? true : false;

	return 0;
}

static int mpu_cfg_parse_FIFO_EN(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, FIFO_EN, &val) < 0)
		return -1;

	bool temp_fifo_en  = false;
	bool xg_fifo_en    = false;
	bool yg_fifo_en    = false;
	bool zg_fifo_en    = false;
	bool accel_fifo_en = false;
	bool slv2_fifo_en  = false;
	bool slv1_fifo_en  = false;
	bool slv0_fifo_en  = false;
	mpu_word_t words = 0; /* sensors written to fifo at each sampling time */

	if (val & TEMP_FIFO_EN_BIT)	{ temp_fifo_en  = true; words += 1; }
	if (val & XG_FIFO_EN_BIT)	{ xg_fifo_en    = true; words += 1; }
	if (val & YG_FIFO_EN_BIT)	{ yg_fifo_en    = true; words += 1; }
	if (val & ZG_FIFO_EN_BIT)	{ zg_fifo_en    = true; words += 1; }
	if (val & ACCEL_FIFO_EN_BIT)	{ accel_fifo_en = true; words += 3; }
	if (val & SLV2_FIFO_EN_BIT)	{ slv2_fifo_en  = true; words += 1; }
	if (val & SLV1_FIFO_EN_BIT)	{ slv1_fifo_en  = true; words += 1; }
	if (val & SLV0_FIFO_EN_BIT)	{ slv0_fifo_en  = true; words += 1; }

	dev->cfg->temp_fifo_en  = temp_fifo_en;
	dev->cfg->xg_fifo_en    = xg_fifo_en;
	dev->cfg->yg_fifo_en    = yg_fifo_en;
	dev->cfg->zg_fifo_en    = zg_fifo_en;
	dev->cfg->accel_fifo_en = accel_fifo_en;
	dev->cfg->slv2_fifo_en  = slv2_fifo_en;
	dev->cfg->slv1_fifo_en  = slv1_fifo_en;
	dev->cfg->slv0_fifo_en  = slv0_fifo_en;

	/* CRUCIAL - raw[0] = buffered sensors  */
	dev->dat->raw[0] = words;
	dev->fifosensors = words;
	dev->fifomax = 1023;

	return 0;
}

static int mpu_cfg_parse_ACCEL_CONFIG(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, ACCEL_CONFIG, &val) < 0)
		return -1;

	dev->cfg->xa_st = (val & XA_ST_BIT) ? true : false;
	dev->cfg->ya_st = (val & YA_ST_BIT) ? true : false;
	dev->cfg->za_st = (val & ZA_ST_BIT) ? true : false;

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

static int mpu_cfg_parse_GYRO_CONFIG(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, GYRO_CONFIG, &val) < 0)
		return -1;

	dev->cfg->xg_st = (val & XG_ST_BIT) ? true : false;
	dev->cfg->yg_st = (val & YG_ST_BIT) ? true : false;
	dev->cfg->zg_st = (val & ZG_ST_BIT) ? true : false;

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

static int mpu_cfg_parse_CONFIG(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, CONFIG, &val) < 0)
		return -1;

	/* ignore EXT_SYNC_SET to avoid complicating senors readings */
	if (val & 0x38) { /* EXT_SYNC_SET not supported */
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

static int mpu_cfg_parse_SMPLRT_DIV(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
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

static int mpu_cfg_parse_INT_PIN_CFG(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, INT_PIN_CFG, &val) < 0)
		return -1;

	if (val & FSYNC_INT_EN_BIT) /* FSYNC_INT_EN not supported */
		return -1;

	if (val & FSYNC_INT_EN_BIT) { dev->cfg->fsync_int_en = true; }

	return 0;
}

static int mpu_cfg_parse_INT_ENABLE(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
		return -1;

	mpu_reg_t val;
	if (mpu_cfg_get_val(dev, INT_ENABLE, &val) < 0)
		return -1;

	bool i2c_mst_en	   = false;
	bool fifo_oflow_en = false;
	bool data_rdy_en   = false;
	if (val & FIFO_OFLOW_EN_BIT)	{ fifo_oflow_en = true; }
	if (val & I2C_MST_INT_EN_BIT)	{ i2c_mst_en = true;	}
	if (val & DATA_RDY_EN_BIT)	{ data_rdy_en = true;	}

	if (i2c_mst_en)	/* I2C_MST_EN not supported */
		return -1;

	dev->cfg->fifo_oflow_en = fifo_oflow_en ;
	dev->cfg->i2c_mst_en	= i2c_mst_en;
	dev->cfg->data_rdy_en   = data_rdy_en;

	return 0;
}

static int mpu_dat_set(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->dat) || (NULL == dev->cal))
		return -1;

	/* Associate data with meaningful names */
	int count = 1;
	if (dev->cfg->accel_fifo_en) {
		dev->dat->AM = 0;
		dev->AM  = &dev->dat->AM;
		dev->dat->scl[count] = 1.0/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Ax  = &dev->dat->dat[count][0];
		dev->Ax2 = &dev->dat->squ[count];
		dev->Axo = &dev->cal->off[count];
		dev->Axg = &dev->cal->gai[count];
		dev->Axd = &dev->cal->dri[count];
		dev->Axm = &dev->dat->mea[count];
		dev->Axv = &dev->dat->var[count];
		count++;
		dev->dat->scl[count] = 1.0/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Ay  = &dev->dat->dat[count][0];
		dev->Ay2 = &dev->dat->squ[count];
		dev->Ayo = &dev->cal->off[count];
		dev->Ayg = &dev->cal->gai[count];
		dev->Ayd = &dev->cal->dri[count];
		dev->Aym = &dev->dat->mea[count];
		dev->Ayv = &dev->dat->var[count];
		count++;
		dev->dat->scl[count] = 1.0/(double)dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->Az  = &dev->dat->dat[count][0];
		dev->Az2 = &dev->dat->squ[count];
		dev->Azo = &dev->cal->off[count];
		dev->Azg = &dev->cal->gai[count];
		dev->Azd = &dev->cal->dri[count];
		dev->Azm = &dev->dat->mea[count];
		dev->Azv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->temp_fifo_en)	{
		dev->dat->scl[count] = 1/340.0;
		dev->dat->dat[count][0] = 0;
		dev->t  = &dev->dat->dat[count][0];
		dev->to = &dev->cal->off[count];
		dev->tg = &dev->cal->gai[count];
		dev->td = &dev->cal->dri[count];
		dev->tm = &dev->dat->mea[count];
		dev->tv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->xg_fifo_en)	{
		dev->dat->GM = 0;
		dev->GM  = &dev->dat->GM;
		dev->dat->scl[count] = 1.0/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gx  = &dev->dat->dat[count][0];
		dev->Gx2 = &dev->dat->squ[count];
		dev->Gxo = &dev->cal->off[count];
		dev->Gxg = &dev->cal->gai[count];
		dev->Gxd = &dev->cal->dri[count];
		dev->Gxm = &dev->dat->mea[count];
		dev->Gxv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->yg_fifo_en)	{
		dev->dat->GM = 0;
		dev->GM  = &dev->dat->GM;
		dev->dat->scl[count] = 1.0/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gy  = &dev->dat->dat[count][0];
		dev->Gy2 = &dev->dat->squ[count];
		dev->Gyo = &dev->cal->off[count];
		dev->Gyg = &dev->cal->gai[count];
		dev->Gyd = &dev->cal->dri[count];
		dev->Gym = &dev->dat->mea[count];
		dev->Gyv = &dev->dat->var[count];
		count++;
	}
	if (dev->cfg->zg_fifo_en)	{
		dev->dat->GM = 0;
		dev->GM  = &dev->dat->GM;
		dev->dat->scl[count] = 1.0/(double)dev->glbs;
		dev->dat->dat[count][0] = 0;
		dev->Gz  = &dev->dat->dat[count][0];
		dev->Gz2 = &dev->dat->squ[count];
		dev->Gzo = &dev->cal->off[count];
		dev->Gzg = &dev->cal->gai[count];
		dev->Gzd = &dev->cal->dri[count];
		dev->Gzm = &dev->dat->mea[count];
		dev->Gzv = &dev->dat->var[count];
		count++;
	}
	/* don't know if the ordering is correct */
	if (dev->cfg->slv0_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv0_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv1_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv1_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv2_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv2_dat = &dev->dat->dat[count][0];
		count++; }
	if (dev->cfg->slv3_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->slv3_dat = &dev->dat->dat[count][0];
		count++;
	}
	if (dev->cfg->slv4_fifo_en) {
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv4_dat = &dev->dat->dat[count][0];
		count++;
	}

	return 0;
}


static int mpu_dat_reset(struct mpu_dev *dev)
{
	if (NULL == dev)
		return -1;

	int i;
	ssize_t len = (sizeof(dev->dat->scl)/sizeof(mpu_data_t));

	for(i = 1; i < len; i++) {
		dev->dat->scl[i] = 1;
		dev->dat->dat[i][0] = 0;
	}

	/* ensure everything points to null */
	dev->AM  = NULL;
	dev->GM  = NULL;
	dev->Ax  = NULL;
	dev->Ax2 = NULL;
	dev->Axo = NULL;
	dev->Axg = NULL;
	dev->Axd = NULL;
	dev->Axm = NULL;
	dev->Axv = NULL;
	dev->Ay  = NULL;
	dev->Ay2 = NULL;
	dev->Ayo = NULL;
	dev->Ayg = NULL;
	dev->Ayd = NULL;
	dev->Aym = NULL;
	dev->Ayv = NULL;
	dev->Az  = NULL;
	dev->Az2 = NULL;
	dev->Azo = NULL;
	dev->Azg = NULL;
	dev->Azd = NULL;
	dev->Azm = NULL;
	dev->Azv = NULL;
	dev->t	 = NULL;
	dev->to	 = NULL;
	dev->tg	 = NULL;
	dev->td	 = NULL;
	dev->tm	 = NULL;
	dev->tv	 = NULL;
	dev->Gx  = NULL;
	dev->Gx2 = NULL;
	dev->Gxo = NULL;
	dev->Gxg = NULL;
	dev->Gxd = NULL;
	dev->Gxm = NULL;
	dev->Gxv = NULL;
	dev->Gy  = NULL;
	dev->Gy2 = NULL;
	dev->Gyo = NULL;
	dev->Gyg = NULL;
	dev->Gyd = NULL;
	dev->Gym = NULL;
	dev->Gyv = NULL;
	dev->Gz  = NULL;
	dev->Gz2 = NULL;
	dev->Gzo = NULL;
	dev->Gzg = NULL;
	dev->Gzd = NULL;
	dev->Gzm = NULL;
	dev->Gzv = NULL;
	dev->slv0_dat = NULL;
	dev->slv1_dat = NULL;
	dev->slv2_dat = NULL;
	dev->slv3_dat = NULL;

	return 0;
}

static int mpu_cfg_parse(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cfg))
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

static int mpu_cal_reset(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->cal))
		return -1;

	int i = 0;
	ssize_t len = (sizeof(dev->cal->off)/sizeof(mpu_data_t));
	for (i = 0; i < len; i++) {
		dev->cal->gai[i] = 1;
		dev->cal->off[i] = 0;
		dev->cal->dri[i] = 0;
	}
	//dev->cal->gra = 0.94;		/* local gravity acceleration in m/s2 */
	dev->cal->gra = 1;		/* local gravity acceleration in m/s2 */
	dev->cal->xa_orig = 0;
	dev->cal->ya_orig = 0;
	dev->cal->za_orig = 0;
	dev->cal->xg_orig = 0;
	dev->cal->yg_orig = 0;
	dev->cal->zg_orig = 0;
	dev->cal->xa_cust = 0;
	dev->cal->ya_cust = 0;
	dev->cal->za_cust = 0;
	dev->cal->xg_cust = 0;
	dev->cal->yg_cust = 0;
	dev->cal->zg_cust = 0;
	dev->cal->xa_bias = 0.0L;
	dev->cal->ya_bias = 0.0L;
	dev->cal->za_bias = 0.0L;
	dev->cal->xg_bias = 0.0L;
	dev->cal->yg_bias = 0.0L;
	dev->cal->zg_bias = 0.0L;
	dev->cal->samples = 1000;

	return 0;
}

static int mpu_dev_allocate(struct mpu_dev **dev)
{
	if (*dev != NULL) /* dev not empty */
		return -1;

	if ((*dev = (struct mpu_dev *)calloc(1, sizeof(struct mpu_dev))) == NULL)
		goto exit_dev;

	if (((*dev)->bus = (int *)calloc(1, sizeof(int))) == NULL)
		goto exit_dev_bus;

	if (((*dev)->cfg = (struct mpu_cfg *)calloc(1, sizeof(struct mpu_cfg))) == NULL)
		goto exit_dev_cfg;

	if (((*dev)->cal = (struct mpu_cal *)calloc(1, sizeof(struct mpu_cal))) == NULL)
		goto exit_dev_cal;

	if (((*dev)->dat = (struct mpu_dat *)calloc(1, sizeof(struct mpu_dat))) == NULL)
		goto exit_dev_dat;

	return 0;

	/* ensure pointers are NULL after free() */
exit_dev_dat:	free((*dev)->dat); (*dev)->dat = NULL;
exit_dev_cal:	free((*dev)->cal); (*dev)->cal = NULL;
exit_dev_cfg:	free((*dev)->cfg); (*dev)->cfg = NULL;
exit_dev_bus:	free((*dev)->bus); (*dev)->bus = NULL;
exit_dev:	free(*dev); dev = NULL;

	return -1;
}

static int mpu_dev_bind(const char *path, const mpu_reg_t address, struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus)) /* invalid pointers */
		return -1;

	if (address == (mpu_reg_t)0x00) /* invalid address */
		return -1;

	size_t pathlen = strlen(path);
	if ((path == NULL) || (pathlen < 6)) /* invalid path */
		return -1;


	int fd;
	if ((fd = open(path, O_RDWR)) < 0) /* open failed */
		return -1;

	/* From now on, fails must close the file descritor */
	if (ioctl(fd, I2C_SLAVE, address) < 0) /* bus error */
		goto dev_bind_exit;

	/* success */
	*(dev->bus) = fd;
	dev->addr = address;

	return 0;

dev_bind_exit:
	if ((close(fd)) < 0) /* cleanup or exit */
		exit(EXIT_FAILURE);

	return -1;
}


int mpu_get_data(struct mpu_dev *dev)
{
	mpu_ctl_fifo_data(dev);
	mpu_ctl_fix_axis(dev);

	return 0;
}


static int mpu_ctl_fifo_data(struct mpu_dev *dev)
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
		if (mpu_fifo_data(dev, &dev->dat->raw[i]) < 0)
			return -1;
		dev->dat->dat[i][0] = dev->dat->raw[i] * dev->dat->scl[i];
		dev->dat->dat[i][1] = dev->dat->dat[i][0];
	}
	if (dev->cfg->temp_fifo_en) {
		*(dev->t) += 36.53;
	}
	if (dev->cfg->accel_fifo_en) {
		*(dev->Ax) -= (mpu_data_t)dev->cal->xa_bias;
		*(dev->Ay) -= (mpu_data_t)dev->cal->ya_bias;
		*(dev->Az) -= (mpu_data_t)dev->cal->za_bias;
		*(dev->Ax2) = (mpu_data_t)*(dev->Ax) * *(dev->Ax);
		*(dev->Ay2) = (mpu_data_t)*(dev->Ay) * *(dev->Ay);
		*(dev->Az2) = (mpu_data_t)*(dev->Az) * *(dev->Az);
		*(dev->AM) = (mpu_data_t)sqrt(*(dev->Ax2) + *(dev->Ay2) + *(dev->Az2));
	}
	if (dev->cfg->xg_fifo_en && dev->cfg->yg_fifo_en && dev->cfg->zg_fifo_en) {
		*(dev->Gx) -= (mpu_data_t)dev->cal->xg_bias;
		*(dev->Gy) -= (mpu_data_t)dev->cal->yg_bias;
		*(dev->Gz) -= (mpu_data_t)dev->cal->zg_bias;
		*(dev->Gx2) = *(dev->Gx) * *(dev->Gx);
		*(dev->Gy2) = *(dev->Gy) * *(dev->Gy);
		*(dev->Gz2) = *(dev->Gz) * *(dev->Gz);
		*(dev->GM) = (mpu_data_t)sqrt(*(dev->Gx2) + *(dev->Gy2) + *(dev->Gz2));
	}
	dev->samples++;

	return 0;
}

static int mpu_ctl_fifo_count(struct mpu_dev *dev)
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

static int mpu_ctl_fifo_flush(struct mpu_dev *dev)
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

static int mpu_fifo_data(struct mpu_dev *dev, int16_t *data)
{
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

static int mpu_read_data(struct mpu_dev * const dev, const mpu_reg_t reg, int16_t *val)
{
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

int mpu_ctl_selftest(struct mpu_dev *dev, char *fname)
{
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
	if ((fp = fopen(fname, "w+")) == NULL) {
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


static int __attribute__((unused)) mpu_ctl_fifo_reset(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_i2c_mst_reset(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_sig_cond_reset(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_fifo_enable(struct mpu_dev *dev)
{
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

static int mpu_ctl_fifo_enable_accel(struct mpu_dev *dev)
{
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

static int mpu_ctl_fifo_enable_gyro(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_fifo_enable_temp(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_fifo_disable(struct mpu_dev *dev)
{
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

static int mpu_ctl_fifo_disable_accel(struct mpu_dev *dev)
{
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

static int mpu_ctl_fifo_disable_gyro(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_fifo_disable_temp(struct mpu_dev *dev)
{
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

static int __attribute__((unused)) mpu_ctl_selftest_fifo_disable(struct mpu_dev *dev)
{
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

static int mpu_ctl_selftest_enable_accel(struct mpu_dev *dev)
{
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

static int mpu_ctl_selftest_enable_gyro(struct mpu_dev *dev)
{
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

static int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev)
{
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

static int mpu_ctl_selftest_disable_gyro(struct mpu_dev *dev)
{
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

int mpu_ctl_reset(struct mpu_dev *dev)
{
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

static int mpu_ctl_calibration_reset(struct mpu_dev *dev)
{
	mpu_ctl_reset(dev); /* clear the OFFS_USRH registers */

	mpu_read_data(dev, XA_OFFS_USRH, &dev->cal->xa_orig);
	mpu_read_data(dev, YA_OFFS_USRH, &dev->cal->ya_orig);
	mpu_read_data(dev, ZA_OFFS_USRH, &dev->cal->za_orig);
	mpu_read_data(dev, XG_OFFS_USRH, &dev->cal->xg_orig);
	mpu_read_data(dev, YG_OFFS_USRH, &dev->cal->yg_orig);
	mpu_read_data(dev, ZG_OFFS_USRH, &dev->cal->zg_orig);
	dev->cal->xa_cust = 0;
	dev->cal->ya_cust = 0;
	dev->cal->za_cust = 0;
	dev->cal->xg_cust = 0;
	dev->cal->yg_cust = 0;
	dev->cal->zg_cust = 0;
	dev->cal->xa_bias = 0.0L;
	dev->cal->ya_bias = 0.0L;
	dev->cal->za_bias = 0.0L;
	dev->cal->xg_bias = 0.0L;
	dev->cal->yg_bias = 0.0L;
	dev->cal->zg_bias = 0.0L;
	dev->cal->samples = 1000;

	return 0;
}

static int __attribute__((unused)) mpu_ctl_calibration_restore(struct mpu_dev *dev, struct mpu_cal *bkp)
{
	/*
	 * restoring calibration registers didn't prove useful
	 * best bet is to use just the biases, or recalibrate
	 */
	mpu_ctl_calibration_reset(dev); /* clear OFFS_USRH registers and calibration data */
	dev->cal->xa_cust = bkp->xa_cust;
	dev->cal->ya_cust = bkp->ya_cust;
	dev->cal->za_cust = bkp->za_cust;

	dev->cal->xg_cust = bkp->xg_cust;
	dev->cal->yg_cust = bkp->yg_cust;
	dev->cal->zg_cust = bkp->zg_cust;

	dev->cal->xa_bias = bkp->xa_bias;
	dev->cal->ya_bias = bkp->ya_bias;
	dev->cal->za_bias = bkp->za_bias;
	dev->cal->xg_bias = bkp->xg_bias;
	dev->cal->yg_bias = bkp->yg_bias;
	dev->cal->zg_bias = bkp->zg_bias;
	dev->cal->AM_bias = bkp->AM_bias;
	dev->cal->GM_bias = bkp->GM_bias;

	mpu_ctl_fifo_flush(dev);

	return 0;
}

int mpu_ctl_calibrate(struct mpu_dev *dev)
{
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
	long double GM_bias = 0;
	for (int i = 0; i < dev->cal->samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_bias += *(dev->Ax);
		ya_bias += *(dev->Ay);
		za_bias += *(dev->Az);
		xg_bias += *(dev->Gx);
		yg_bias += *(dev->Gy);
		zg_bias += *(dev->Gz);
		AM_bias += *(dev->AM);
		GM_bias += *(dev->GM);
	}
	/* take the average difference */
	xa_bias /= dev->cal->samples;
	ya_bias /= dev->cal->samples;
	za_bias /= dev->cal->samples;
	//AM_bias /= dev->cal->samples;
	xg_bias /= dev->cal->samples;
	yg_bias /= dev->cal->samples;
	zg_bias /= dev->cal->samples;
	//GM_bias /= dev->cal->samples;

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
	GM_bias = 0;
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
		GM_bias += *(dev->GM);
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
	GM_bias = 0;
	for (int i = 0; i < dev->cal->samples; i++) {
		mpu_ctl_fifo_data(dev);
		xa_bias += *(dev->Ax);
		ya_bias += *(dev->Ay);
		za_bias += *(dev->Az);
		AM_bias += *(dev->AM);
		xg_bias += *(dev->Gx);
		yg_bias += *(dev->Gy);
		zg_bias += *(dev->Gz);
		GM_bias += *(dev->GM);
	}
	/* take the average difference */
	xa_bias /= dev->cal->samples;
	ya_bias /= dev->cal->samples;
	za_bias /= dev->cal->samples;
	AM_bias /= dev->cal->samples;
	xg_bias /= dev->cal->samples;
	yg_bias /= dev->cal->samples;
	zg_bias /= dev->cal->samples;
	GM_bias /= dev->cal->samples;

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
	dev->cal->GM_bias = GM_bias;

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

static inline void mpu_ctl_fix_axis(struct mpu_dev *dev)
{
	if (dev->cfg->accel_fifo_en) {
		*(dev->Ax) *= -1;
		*(dev->Ay) *= -1;
		*(dev->Az) *= -1;
	}
}

static int mpu_dev_parameters_save(char *fn, struct mpu_dev *dev)
{
	FILE *dmp;
	dmp = fopen(fn, "w+");
	fwrite(dev->cfg, sizeof(*(dev->cfg)), 1, dmp);
	fwrite(dev->cal, sizeof(*(dev->cal)), 1, dmp);
	fclose(dmp);
	return 0;
}

static int mpu_dev_parameters_restore(char *fn, struct mpu_dev *dev)
{
	FILE * fp;
	if ( (fp = fopen(fn, "r")) == NULL) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fn);
		exit(EXIT_FAILURE);
	}
	fread(dev->cfg, sizeof(*(dev->cfg)), 1, fp);
	fread(dev->cal, sizeof(*(dev->cal)), 1, fp);
	fclose(fp);

	mpu_ctl_fifo_flush(dev);
	return 0;
}

static int __attribute__((unused)) mpu_diagnose(struct mpu_dev *dev)
{
	if ((NULL == dev) || (NULL == dev->bus))
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
	printf("squ[1](%p) Ax=%p\n", (void *)&dev->dat->squ[1], (void *)dev->Ax2);
	printf("squ[2](%p) Ay=%p\n", (void *)&dev->dat->squ[2], (void *)dev->Ay2);
	printf("squ[3](%p) Az=%p\n", (void *)&dev->dat->squ[3], (void *)dev->Az2);
	printf("squ[4](%p) t =%p\n", (void *)&dev->dat->squ[4], (void *)dev->t);
	printf("squ[5](%p) Gx=%p\n", (void *)&dev->dat->squ[5], (void *)dev->Gx2);
	printf("squ[6](%p) Gy=%p\n", (void *)&dev->dat->squ[6], (void *)dev->Gy2);
	printf("squ[7](%p) Gz=%p\n", (void *)&dev->dat->squ[7], (void *)dev->Gz2);
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
	if(NULL == fn) {
		fprintf(stderr, "%s failed: NULL filename\n", __func__);
		return -1;
	}

	FILE *fp;
	if ( (fp = fopen(fn, "w+")) == NULL) {
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

static int mpu_read_byte(struct mpu_dev * const dev,
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

static int mpu_write_byte(struct mpu_dev * const dev,
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

static int mpu_read_word(struct mpu_dev * const dev,
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

static int __attribute__((unused)) mpu_write_word(struct mpu_dev * const dev,
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
