#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _MPU6050_PRIVATE_H_ /* MPU6050_PRIVATE_H */
#define _MPU6050_PRIVATE_H_

#include "mpu6050_public.h"
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
#include <i2c/smbus.h> 		/* for i2c_smbus_x */
#include <linux/i2c.h> 		/* for i2c_smbus_x */

#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

#define ARRAY_LEN(x) sizeof((x))/sizeof((x[0]))

/* stores calibration related values for reference */
struct mpu_cal {
	mpu_data_t gra;		/* mean(sqrt(ax2,ay2,az2)[])		*/
	mpu_data_t off[32];	/* ax - mean(ax[n]),.		*/
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
};

/* stores sensor data collection related values */
#define FIFOMAXWORDS 12 // temp, 3 accels, 3 gyros, 5 slaves[0-4]
struct mpu_dat {
	int16_t raw[FIFOMAXWORDS];	/* raw sensor data	*/
	mpu_data_t scl[FIFOMAXWORDS];	/* scaling factors	*/
	mpu_data_t dat[FIFOMAXWORDS][2];/* scaled data		*/
	mpu_data_t squ[FIFOMAXWORDS];	/* squared data		*/
	mpu_data_t mea[FIFOMAXWORDS];	/* data mean		*/
	mpu_data_t var[FIFOMAXWORDS];	/* data variance	*/
	mpu_data_t AM;			/* accel magnitude	*/
	unsigned int fifo_words;
};
#undef FIFOMAXWORDS

/* Mirrors configuration register values and their meaning */
struct mpu_cfg {
	mpu_reg_t regs[16][2];	/* configuration register values */
	/* 32 configurations	 9 REGISTER locations */
	bool sleep;		/* PWR_MGMGT_1 */
	bool cycle;		/* PWR_MGMGT_1 */
	bool temp_dis;		/* PWR_MGMGT_1 */
	bool xg_st;		/* GYRO_CONFIG */
	bool yg_st;		/* GYRO_CONFIG */
	bool zg_st;		/* GYRO_CONFIG */
	bool xa_st;		/* ACCEL_CONFIG */
	bool ya_st;		/* ACCEL_CONFIG */
	bool za_st;		/* ACCEL_CONFIG */
	bool stdby_xa;		/* PWR_MGMGT_2 */
	bool stdby_ya;		/* PWR_MGMGT_2 */
	bool stdby_za;		/* PWR_MGMGT_2 */
	bool stdby_xg;		/* PWR_MGMGT_2 */
	bool stdby_yg;		/* PWR_MGMGT_2 */
	bool stdby_zg;		/* PWR_MGMGT_2 */
	bool fifo_en;		/* USER_CTRL */
	bool i2c_mst_en;	/* USER_CTRL */
	bool i2c_if_dis;	/* USER_CTRL */
	bool temp_fifo_en;	/* FIFO_EN */
	bool accel_fifo_en;	/* FIFO_EN */
	bool xg_fifo_en;	/* FIFO_EN */
	bool yg_fifo_en;	/* FIFO_EN */
	bool zg_fifo_en;	/* FIFO_EN */
	bool slv0_fifo_en;	/* FIFO_EN */
	bool slv1_fifo_en;	/* FIFO_EN */
	bool slv2_fifo_en;	/* FIFO_EN */
	bool slv3_fifo_en;	/* I2C_MST_CTRL */
	bool slv4_fifo_en;	/* I2C_MST_CTRL */
	bool fsync_int_en;	/* INT_PIN_CFG	__not_supported__ */
	bool fifo_oflow_en;	/* INT_ENABLE */
	bool i2c_mst_int_en;	/* INT_ENABLE */
	bool data_rdy_en;	/* INT_ENABLE */
};

/* The default values for configuration registers */
static const struct mpu_cfg mpu6050_defcfg = {
	.regs =	{
		{ PWR_MGMT_1,   0x03},	/* power on, temp enabled, clock gyro_z */
		{ PWR_MGMT_2,   0x00},	/* no standby, full on			*/
		{ CONFIG,       0x00},	/* dlpf off				*/
		{ SMPLRT_DIV,   0x4F},	/* divisor = 80(1+79), rate = 100	*/
		{ ACCEL_CONFIG, 0x00},	/* +-2g					*/
		{ GYRO_CONFIG,  0x00},	/* +-250 deg/s				*/
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

/* level 4 - public api - see public header */

/* level 3 - basic controls */
int mpu_diagnose(		  struct mpu_dev *dev);
int mpu_ctl_selftest_fifo_disable(struct mpu_dev *dev);
int mpu_ctl_selftest_enable_accel(struct mpu_dev *dev);
int mpu_ctl_selftest_enable_gyro( struct mpu_dev *dev);
int mpu_ctl_selftest_disable_accel(struct mpu_dev *dev);
int mpu_ctl_selftest_disable_gyro(struct mpu_dev *dev);
int mpu_ctl_wake(		  struct mpu_dev *dev);
int mpu_fifo_data(		  struct mpu_dev *dev, int16_t *data); //FIXME remove
int mpu_ctl_fifo_enable(	  struct mpu_dev *dev);
int mpu_ctl_fifo_enable_temp(	  struct mpu_dev *dev);
int mpu_ctl_fifo_enable_accel(	  struct mpu_dev *dev);
int mpu_ctl_fifo_enable_gyro(	  struct mpu_dev *dev);
int mpu_ctl_fifo_disable(	  struct mpu_dev *dev);
int mpu_ctl_fifo_disable_temp(	  struct mpu_dev *dev);
int mpu_ctl_fifo_disable_accel(	  struct mpu_dev *dev);
int mpu_ctl_fifo_disable_gyro(	  struct mpu_dev *dev);
int mpu_ctl_fifo_reset(		  struct mpu_dev *dev);
int mpu_ctl_fifo_count(		  struct mpu_dev *dev);
int mpu_ctl_fifo_flush(		  struct mpu_dev *dev);
int mpu_ctl_fifo_data(		  struct mpu_dev *dev); //TODO where to i put this
int mpu_ctl_i2c_mst_reset(	  struct mpu_dev *dev);
int mpu_ctl_sig_cond_reset(	  struct mpu_dev *dev);
void mpu_ctl_fix_axis(		  struct mpu_dev *dev);

/* level 2 - internal structure management */
int mpu_dev_bind(const char *path, const mpu_reg_t address, struct mpu_dev *dev);
int mpu_dev_allocate(		  struct mpu_dev **dev);
int mpu_cfg_set(		  struct mpu_dev *dev);
int mpu_dat_set(		  struct mpu_dev *dev);
int mpu_cfg_reset(		  struct mpu_dev *dev);
int mpu_dat_reset(		  struct mpu_dev *dev);
int mpu_cal_reset(		  struct mpu_dev *dev);

int mpu_dev_parameters_save(	  char *fn, struct mpu_dev *dev);
int mpu_dev_parameters_restore(	  char *fn, struct mpu_dev *dev);

int mpu_ctl_calibration_reset(	  struct mpu_dev *dev);
int mpu_ctl_calibration_restore(  struct mpu_dev *dev, struct mpu_cal *bkp);

int mpu_cfg_set_CLKSEL(struct mpu_dev *dev, mpu_reg_t clksel); //FIXME remove

/* level 1 - configuration registers parsing */
int mpu_cfg_get_val(struct mpu_dev *dev, const mpu_reg_t reg, mpu_reg_t *val);
int mpu_cfg_set_val(struct mpu_dev *dev, const mpu_reg_t reg, const mpu_reg_t val);
int mpu_cfg_write(		  struct mpu_dev *dev);
int mpu_cfg_validate(		  struct mpu_dev *dev);
int mpu_cfg_parse(		  struct mpu_dev *dev);
int mpu_cfg_parse_PWR_MGMT(	  struct mpu_dev *dev);
int mpu_cfg_parse_CONFIG(	  struct mpu_dev *dev);
int mpu_cfg_parse_SMPLRT_DIV(	  struct mpu_dev *dev);
int mpu_cfg_parse_ACCEL_CONFIG(	  struct mpu_dev *dev);
int mpu_cfg_parse_GYRO_CONFIG(	  struct mpu_dev *dev);
int mpu_cfg_parse_USER_CTRL(	  struct mpu_dev *dev);
int mpu_cfg_parse_FIFO_EN(	  struct mpu_dev *dev);
int mpu_cfg_parse_INT_ENABLE(	  struct mpu_dev *dev);
int mpu_cfg_parse_INT_PIN_CFG(	  struct mpu_dev *dev);

/* level 0  i2c bus communication */
int mpu_read_byte( struct mpu_dev * const dev, const mpu_reg_t reg, mpu_reg_t *val);
int mpu_read_word( struct mpu_dev * const dev, const mpu_reg_t reg, mpu_word_t *val);
int mpu_read_data( struct mpu_dev * const dev, const mpu_reg_t reg, int16_t *val);
int mpu_write_byte(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_reg_t val);
int mpu_write_word(struct mpu_dev * const dev, const mpu_reg_t reg, const mpu_word_t val);

#endif /* MPU6050_PRIVATE_H */
#ifdef __cplusplus
	}
#endif
