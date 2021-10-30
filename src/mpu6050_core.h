#ifdef __cplusplus
	extern "C" {
#endif
#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdlib.h>		/* for malloc(), free(), exit() */
#include <stdio.h>		/* for printf(), scanf(), fopen(), perror() */
#include <stdint.h>		/* for uint8_t, uint16_t, etc */
#include <stdbool.h>		/* for bool */
#include <time.h>		/* for timespec_t */

/*
 * Orientation of Sensor Axis
 *
 * Axis should be individually checked for: sequence, polarity, matching.
 *  I has been fount that:s
 * 	Accelerometer readings are inverse-polarity with respect to sensor body
 *	Gyroscope is correct with respect to the sensor body
 *
 * The correct orientation and polarity is as shown:
 *  (compare with: Product specification rev. 3.4., p. 40, 46.)
 */
#define MSG_AXIS "" \
"---------------------------------------------------\n" \
"              MPU-6050 SENSOR AXIS SETUP           \n" \
"---------------------------------------------------\n" \
"                      (-Az,+Gz CCW)                \n" \
"                          ^                        \n" \
"                          | (-Ay,+Gy CCW)          \n" \
"       sensor body        |   ^		    \n" \
"        (top view)        |  /                     \n" \
"       ____________       | /                      \n" \
"      /0          /'      |/                       \n" \
"     / MPU-60050 /.       0---------->(-Ax,+Gx CCW)\n" \
"    /           /.        Standard Axis            \n" \
"   /           /. 	   Configuration            \n" \
"  /___________/.   				    \n" \
"  '...........'    				    \n" \
"                                                   \n" \
"___________________________________________________\n"



typedef uint8_t  mpu_reg_t;
typedef uint16_t mpu_word_t;
typedef double   mpu_data_t;

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
	double str_xa;		/* self-test response */
	double str_ya;		/* self-test response */
	double str_za;		/* self-test response */
	double str_xg;		/* self-test response */
	double str_yg;		/* self-test response */
	double str_zg;		/* self-test response */
	double ft_xa;		/* factory_trim value */
	double ft_ya;		/* factory_trim value */
	double ft_za;		/* factory_trim value */
	double ft_xg;		/* factory_trim value */
	double ft_yg;		/* factory_trim value */
	double ft_zg;		/* factory_trim value */
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
	unsigned long long samples;	/* sample counter			*/
	mpu_data_t	*AM;
	mpu_data_t	*GM;
	mpu_data_t	*Ax, *Ax2, *Axo, *Axg, *Axm, *Axv, *Axd;
	mpu_data_t	*Ay, *Ay2, *Ayo, *Ayg, *Aym, *Ayv, *Ayd;
	mpu_data_t	*Az, *Az2, *Azo, *Azg, *Azm, *Azv, *Azd;
	mpu_data_t	*Gx, *Gx2, *Gxo, *Gxg, *Gxm, *Gxv, *Gxd;
	mpu_data_t	*Gy, *Gy2, *Gyo, *Gyg, *Gym, *Gyv, *Gyd;
	mpu_data_t	*Gz, *Gz2, *Gzo, *Gzg, *Gzm, *Gzv, *Gzd;
	mpu_data_t	*t, 	    *to,  *tg,  *tm,  *tv,  *td;
	mpu_data_t	*slv0_dat, *slv1_dat, *slv2_dat, *slv3_dat, *slv4_dat;
};

struct mpu_cal {
	mpu_data_t gra;		/* mean(sqrt(ax2,ay2,az2)[])		*/
	mpu_data_t off[32];	/* ax - mean(ax[n]), ...		*/
	mpu_data_t gai[32];	/* ax[expec_1g]/ax[measured_1g], ...	*/
	mpu_data_t dri[32];	/* delta(mean(ax[n])/delta(time), ...	*/
	int16_t xa_orig;
	int16_t ya_orig;
	int16_t za_orig;
	int16_t xg_orig;
	int16_t yg_orig;
	int16_t zg_orig;
	int16_t xa_cust;
	int16_t ya_cust;
	int16_t za_cust;
	int16_t xg_cust;
	int16_t yg_cust;
	int16_t zg_cust;
	int samples;
	long double xa_bias;
	long double ya_bias;
	long double za_bias;
	long double xg_bias;
	long double yg_bias;
	long double zg_bias;
	long double AM_bias;
	long double GM_bias;
};

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

struct mpu_cfg {
	mpu_reg_t cfg[16][2];	/* configuration state	*/
	/* configuration bits   -> REGISTER	 */
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

/*
 * Enable device tree for i2c-1 inside /boot/config.txt
 * 	[all]
 * 	dtparam=i2c_arm=on
 * 	dtparam=i2c_arm_baudrate=400000
 *
 * Supported features
 * 	Accelerometer		- enable/disable, range setting
 * 	Gyroscope 	  	- enable/disable, range setting
 * 	Temperature sensor	- enable/disable
 * 	Sampling rate control 	- 10,20,25,50,100,200 Hz
 * 	Buffered readings	- ensures regular sampling interval
 * 	DLPF (Digital Low Pass filter) - refer to datasheet
 * 	Self-tests
 * 	Register dump
 * 	Calibration
 *
 * Unsupported features
 * 	eDMP (embedded Digital Motion Proccessor) - blob, undocumented
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

int mpu_init(	const char * const path,
		struct mpu_dev **mpudev,
		const int mode);

int mpu_destroy(struct mpu_dev * mpudev);
int mpu_get_data(struct mpu_dev * mpudev);

int mpu_ctl_calibrate		(struct mpu_dev *dev);
int mpu_ctl_reset		(struct mpu_dev *dev);
int mpu_ctl_dump		(struct mpu_dev *dev, char *filename);
int mpu_ctl_selftest		(struct mpu_dev *dev, char *filename);

int mpu_ctl_fifo_enable		(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_accel	(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_gyro	(struct mpu_dev *dev);
int mpu_ctl_fifo_enable_temp	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_accel	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_gyro	(struct mpu_dev *dev);
int mpu_ctl_fifo_disable_temp	(struct mpu_dev *dev);
int mpu_ctl_fifo_count		(struct mpu_dev *dev);
int mpu_ctl_fifo_flush		(struct mpu_dev *dev);

int mpu_ctl_sig_cond_reset	(struct mpu_dev *dev);

int mpu_ctl_wake		(struct mpu_dev *dev);

int mpu_ctl_samplerate(		struct mpu_dev *dev, unsigned int hertz);
int mpu_ctl_dlpf(		struct mpu_dev *dev, unsigned int dlpf);
int mpu_ctl_accel_range(	struct mpu_dev *dev, unsigned int range);
int mpu_ctl_gyro_range(		struct mpu_dev *dev, unsigned int range);
int mpu_ctl_clocksource(	struct mpu_dev *dev, mpu_reg_t clksel);

int mpu_read_byte(		struct mpu_dev * const dev,
				const mpu_reg_t reg,	/* device register */
				mpu_reg_t * val);	/* value destination */

int mpu_read_word(		struct mpu_dev * const dev,
				const mpu_reg_t reg,	/* device register */
				mpu_word_t * val);	/* value destination */

int mpu_read_data(		struct mpu_dev * const dev,
				const mpu_reg_t reg,	/* device register */
				int16_t * val);		/* value destination */

int mpu_write_byte(		struct mpu_dev * const dev,
				const mpu_reg_t reg,	/* device register */
				const mpu_reg_t val);	/* value to write */

int mpu_write_word		(struct mpu_dev * const dev,
				const mpu_reg_t reg,	/* device register */
				const mpu_word_t val);	/* value to write */

#endif /* _MPU6050_H_ */

#ifdef __cplusplus
	}
#endif
