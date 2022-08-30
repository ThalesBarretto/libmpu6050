// SPDX-License-Identifier: MIT 
/* Copyright (C) 2021 Thales Antunes de Oliveira Barretto */
#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _MPU6050_H_
#define _MPU6050_H_
#include <stdint.h>		/* for uint8_t, uint16_t, etc */
#include <stdbool.h>		/* for bool */
#include <time.h>		/* for timespec_t */

/* See Product specification rev. 3.4., p. 40, 46. */
#define MSG_AXIS "" \
"---------------------------------------------------\n" \
"              MPU-6050 SENSOR AXIS SETUP           \n" \
"---------------------------------------------------\n" \
"                          Z                        \n" \
"                          ^                        \n" \
"                          |    Y                   \n" \
"        sensor body       |   ^                    \n" \
"        (top view)        |  /                     \n" \
"       ____________       | /                      \n" \
"      /*          /'      |/                       \n" \
"     / MPU-60050 /.       0---------> X            \n" \
"    /           /.                                 \n" \
"   /           /.                                  \n" \
"  /___________/.                                   \n" \
"  '...........'                                    \n" \
"                                                   \n" \
"___________________________________________________\n"

typedef uint8_t  mpu_reg_t;
typedef uint16_t mpu_word_t;
typedef double   mpu_data_t;
struct mpu_cfg;
struct mpu_cal;
struct mpu_dat;
struct mpu_dev;

/*
 * MUST Enable device tree for i2c-1 inside /boot/config.txt
 * 	[all]
 * 	dtparam=i2c_arm=on
 * 	dtparam=i2c_arm_baudrate=400000
 *
 * Supported features
 * 	Buffered readings	- all reading come from fifo buffer
 * 	Accelerometer		- enable/disable, range setting
 * 	Gyroscope 	  	- enable/disable, range setting
 * 	Temperature sensor	- enable/disable
 * 	Sampling rate control 	- 50,100,200, 250, 500 Hz
 * 	Digital Low Pass filter	- refer to datasheet
 * 	Self-tests		- refer to datasheet, write report to file
 * 	Register dump		- write register values to file
 * 	Calibration		- device must stay leveled and static
 *
 * Unsupported features
 * 	eDMP (embedded Digital Motion Proccessor) - proprietary blob
 * 	Low-power modes		- not our use case
 * 	External interrupts	- not our use case
 * 	External clock sources	- not our use case
 * 	Secondary i2c bus	- not our use case
 *
 * Return value for all function calls:
 * 	On success, return 0.
 * 	On failure, return -1;
 */
#define MPU6050_RESET 0
#define MPU6050_RESTORE 1

#ifndef MPU6050_CFGFILE
#define MPU6050_CFGFILE "mpu6050_cfg.bin"
#endif

int mpu_init(const char * const path, struct mpu_dev **mpudev, const int mode);
int mpu_destroy		(struct mpu_dev *dev);
int mpu_get_data	(struct mpu_dev *dev);
int mpu_ctl_calibrate	(struct mpu_dev *dev);
int mpu_ctl_reset	(struct mpu_dev *dev);
int mpu_ctl_dlpf	(struct mpu_dev *dev, unsigned int dlpf);
int mpu_ctl_samplerate	(struct mpu_dev *dev, unsigned int hertz);
int mpu_ctl_accel_range	(struct mpu_dev *dev, unsigned int range);
int mpu_ctl_gyro_range	(struct mpu_dev *dev, unsigned int range);
int mpu_ctl_clocksource	(struct mpu_dev *dev, unsigned int clksel);
int mpu_ctl_selftest	(struct mpu_dev *dev, char *filename);
int mpu_ctl_dump	(struct mpu_dev *dev, char *filename);

struct mpu_dev {
	/* basic interface setting */
	int	*bus;		/* bus file decriptor */
	uint8_t	addr;		/* device i2c bus address */
	uint8_t prod_id;	/* product id */
	/* internal data - managed through special handlers */
	struct	mpu_cfg	*cfg;	/* config register state */
	struct	mpu_dat	*dat;	/* sensor readings */
	struct	mpu_cal	*cal;	/* calibration data */
	struct  timespec dly;	/* fifo data delay */
	/* readable config - result of special handlers */
	bool	aolpm;		/* accelerometer-only low power mode */
	double	wake_freq;	/* low-power cycling freq */
	double	clock_freq;	/* determined by CLKSEL */
	int	clksel;		/* clock source selection (CLKSEL) */
	int	dlpf;		/* Digital Lowpass filter setting */
	int	fifosensors;	/* number of sensors buffered */
	int	fifomax;	/* fifo buffer capacity in bytes */
	int	fifocnt;	/* bytes available in fifo */
	unsigned int gor;	/* gyro output rate (Hz) */
	double sr;		/* sampling rate */
	double st;		/* sampling time */
	double afr;		/* accelerometer full range in g-force */
	double albs;		/* accelerometer least bit sensitivity */
	double abdw;		/* accelerometer bandwidth in Hertz (Hz)*/
	double adly;		/* accelerometer delay in miliseconds (ms) */
	double gfr;		/* gyroscope full range in degrees per second */
	double glbs;		/* gyroscope least bit sensitivity */
	double gbdw;		/* gyroscope bandwidth in Hertz (Hz) */
	double gdly;		/* gyroscope delay in miliseconds (ms) */
	/* readable data */
	unsigned long long samples; /* sample counter			*/
	mpu_data_t	*AM;
	mpu_data_t	*Ax;
	mpu_data_t	*Ay;
	mpu_data_t	*Az;
	mpu_data_t	*Gx;
	mpu_data_t	*Gy;
	mpu_data_t	*Gz;
	mpu_data_t	*t;
	mpu_data_t	*slv0_dat, *slv1_dat, *slv2_dat, *slv3_dat, *slv4_dat;
};

#endif /* _MPU6050_H_ */

#ifdef __cplusplus
	}
#endif
