---
title: libmpu6050
section: 3
header: User Manual
footer: libmpu6050 1.0
date: August 19, 2022
---

NAME
====

libmpu6050 - userspace driver for MPU6050 Inertial Measuring Unit

PURPOSE
=======

This library handles an *MPU6050* IMU device over a *Raspberry Pi 3B* *I2C* bus. Other devices are out of scope and unsupported. The *i2c* interface will be exclusively held for the *MPU6050*, you *will NOT* be able to use other devices on that bus during operation.

The library main purpose is to abstract the low level handling of the *MPU6050* device over the *i2c* bus. It allows the user to handle the device in a more natural way, without compromising correctecnes of data and safe condifgurations.

SYNOPSIS
========

*HEADER*

`#include <`*libmpu6050/mpu6050_core.h*`>`

*FUNCTIONS*

`int` *mpu_init*`(const char * const ` *path*`,`
`		struct mpu_dev **`*mpudev*`,`
` 		const int` *mode*`);`

`int` *mpu_destroy*`(struct mpu_dev *`*dev*`);`

`int` *mpu_get_data*`(struct mpu_dev *`*dev*`);`

`int` *mpu_ctl_calibrate*`(struct mpu_dev *`*dev*`);`

`int` *mpu_ctl_reset*`(struct mpu_dev *`*dev*`);`

`int` *mpu_ctl_dump*`(struct mpu_dev *`*dev*`, char *`*filename*`);`

`int` *mpu_ctl_selftest*`(struct mpu_dev *`*dev*`, char *`*filename*`);`

`int` *mpu_ctl_samplerate*`(struct mpu_dev *`*dev*`, unsigned int` *hertz*`);`

`int` *mpu_ctl_dlpf*`(struct mpu_dev *`*dev*`, unsigned int` *dlpf*`);`

`int` *mpu_ctl_accel_range*`(struct mpu_dev *`*dev*`, unsigned int` *range*`);`

`int` *mpu_ctl_gyro_range*`(struct mpu_dev *`*dev*`, unsigned int` *range*`);`

`int` *mpu_ctl_clocksource*`(struct mpu_dev *`*dev*`, mpu_reg_t` *clksel*`);`
`

*MACROS*

`#define` *MPU6050_RESET 0*

`#define` *MPU6050_RESTORE 1*

`#define` *MPU6050_CFGFILE "mpu6050_cfg.bin"*

*TYPES*

`typedef uint8_t` *mpu_reg_t* `;`

`typedef uint16_t` *mpu_word_t* `;`

`typedef double` *mpu_data_t* `;`

` `*struct mpu_dev* `{`
```
	int	*bus;		/* bus file decriptor */
	uint8_t addr ;		/* device i2c bus address */
	uint8_t prod_id;	/* product id */
	struct	mpu_cfg	*cfg;	/* config register state */
	struct	mpu_dat	*dat;	/* sensor readings */
	struct	mpu_cal	*cal;	/* calibration data */
	struct  timespec dly;	/* fifo data delay */
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
	unsigned long long samples; /* sample counter	*/
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
```
`};`

DESCRIPTION
===========

*ENABLING I2C INTERFACE*
	
The i2c-1 device on the Raspberry Pi 3B must be enable inside **/boot/config.txt**:
	
```
[all]
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```
	
*CONNECTING THE BREAKOUT BOARD*

This library provides communication over the i2c-1 bus through appropriate header pins.
The device must be properly connected the Raspberry Pi board.
Please follow the wiring diagram and  mounting instructions.

*PRINCIPLES*

One `struct mpu_dev` tracks sensor state and data, and must be handled exclusively by the provided interfaces.
First you must call `mpu_init()` to setup the device then the other functions to configure the device.
Call `mpu_get_data()` to update and then read the sensor values on the *struct mpu_dev*.
When you're done, you should `mpu_destroy()` to free the bus.
See mpu6050-demo for an in-depth example and use case.


1. *FUCNTIONS*

`int` *mpu_init*`(const char * const ` *path*`,`
`	    	struct mpu_dev **`*mpudev*`,`
` 		const int` *mode*`)`

handles device initialization.

- *path* is the filesystem path for the i2c bus, ideally *"/dev/i2c-1"*

- *\*\*mpudev* is a pointer to a pointer to an unallocated *struct mpu_dev* assigned with  *NULL*

- *mode* is either *MPU6050_RESET* or *MPU6050_RESTORE* where you can choose to performe a device reset on initialization or try to recover the last saved configuration for the device.

Upon *SUCCESS(0)* device is ready and \*dev holds the device data

Upon *FAILURE(1)* the \*\*dev contents are not modified.

*EXAMPLE*
```
	struct mpu_dev *dev = NULL;
	mpu_init("/dev/i2c-1", &dev, MPU6050_RESET);
	if (NULL == dev)
		abort();
```

`int` *mpu_destroy*`(struct mpu_dev *`*dev*`)`

frees the memory and releases the bus.

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* bus is freed and *dev* memory deallocated.

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.

*EXAMPLE*
```
	mpu_destroy(dev);
```


`int` *mpu_get_data*`(struct mpu_dev *`*dev*`)`

Updates the device data structure with the next reading from the sensors. The data can be retrived at *\*(mpudev->Ax)*, *\*(mpudev->Ay)*, etc. You should continously call `mpu_get_data()` to avoid the device buffer overflow, which would cause the reading count to start from zero.

Reading data is a synchronous operation, that is, it will wait for data to get into the device buffer, and will only return when the data is effectively retrieved and available.

The embedded buffer on the MPU6050 will collect samples at exact sample rate, so that you can rely on that to get accurate sampling intervals between the samples. The library buffered data collection implementations allows you to collect samples at irregular intervals, as long as you dont let the buffer overflow. This solves the problem of running an operating system without real-time guarantees on the i2c bus.

*EXAMPLE*
```
	mpu_get_data(dev);
```

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* new data is available

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.


`int` *mpu_ctl_calibrate*`(struct mpu_dev *`*dev*`)`

Performs a simple calibration routing that lasts for about ten seconds. During the procedure the device must rest still and leveled. After the calibration the device registers will be updated and the config file will be written with the adequate values and offsets.  It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* device calibration registers and file are updated

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.

*EXAMPLE*
```
	mpu_ctl_calibrate(dev);
```


`int` *mpu_ctl_reset*`(stuct mpu_dev *`*dev*`)`

Performs a devices reset and puts the device into standard configuration. It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* device performed reset and has standard configuration

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.

*EXAMPLE*
```
	mpu_ctl_reset(dev);
```


`int` *mpu_ctl_dump*`(strct mpu_dev *`*dev*`, char *`*filename*`)`

Dumps the device register values to a file. Please refer to the device datasheet and documentation for more info on the register value meaning. It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

- *filename* is a filsystem path to the filename.

Upon *SUCCESS(0)* dump file has been written

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.

*EXAMPLE*
```
	mpu_ctl_dump(dev, "mydumpfile.txt");
```



`int` *mpu_ctl_selfest*`(struct mpu_dev *`*dev*`, char *`*filename*`)`

Performs a device self test and dumps the results to a file. Please refer to device documentation for more info. It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

- *filename* is a filsystem path to the filename.

Upon *SUCCESS(0)* device finished self-test and report file has been written

Upon *FAILURE(1)* wrong argument values or bus error, you should abort.


*EXAMPLE*
```
	mpu_ctl_selftest(dev, "mytestreport.txt");
```


`int` *mpu_ctl_samplerate*`(struct mpu_dev *`*dev*`, unsigned int` *hertz*`);`

Selects the desired data sampling rate in samples per second, or Hertz. Supported rates are 50, 100, 200, 250 and 500 that allow for rounded time division with the device internal clocks. Although supported, you can hardly keep up with the data rates above 200Hz so the device will constatly overflow the buffers. The most practical values are 50, 100 and 200.  It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*

- *hertz* is the desired samplerate [50,100,200,250,500]

Upon *SUCCESS(0)* device samplig rate setting is updated.

Upon *FAILURE(1)* invalid setting or  bus error.

*EXAMPLE*
```
	mpu_ctl_samplerate(dev, 100);
```

`int` *mpu_ctl_dlpf*`(strct mpu_dev *`*dev*`, unsigned int` *dlpf*`)`

Changes the DLPF setting value that changes the embedded Digital Low Pass Filter. A value of 0 means the filter is disabled, while values 1-6 progressively increase the filtering effort, while reducing bandwidth.  It is a synchronoous operations, which means that the function returns only after the requested operation completed. Please refer to the device datasheet.

- *dev* is a pointer to an initialized *struct mpu_dev*.

- *dkpf* is the desired DLPF value [0-3]

Upon *SUCCESS(0)* device DLPF setings are updated

Upon *FAILURE(1)* invalid setting or  bus error.


*EXAMPLE*
```
	mpu_ctl_dlpf(dev, 0);
```


`int` *mpu_ctl_accel_range*`(struct mpu_dev *`*dev*`, unsigned int` *range*`)`

Sets the Accelerometer range and sensitivity, with values 2, 4, 8 and 16 meaning the range of the reported reading will be between +-2G, +-4G, +-8G and +-16G. The device has an internal resolution of 16-bit, which can impair the effective resolution, but the library converts all data to appropriate *mpu_data_t*.  It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* device acceleromter range seetings are updated

Upon *FAILURE(1)* invalid setting or  bus error.

*EXAMPLE*
```
	mpu_ctl_accel_range(dev, 2);
```

`int` *mpu_ctl_gyrorange*`(struct mpu_dev *`*dev*`, unsigned int` *range*`)`

Sets the Gyroscope range and sensitivity, with values 250, 500, 1000 and 2000 meaning the range of the reported reading will be between +-250dps, +-500dps, +-1000dps and +-2000dps. The device has an internal resolution of 16-bit, which can impair the effective resolution, but the library converts all data to appropriate *mpu_data_t*.  It is a synchronoous operations, which means that the function returns only after the requested operation completed.

- *dev* is a pointer to an initialized *struct mpu_dev*.

Upon *SUCCESS(0)* device gyroscope range settings are updated

Upon *FAILURE(1)* invalid setting or  bus error.

*EXAMPLE*
```
	mpu_ctl_gyro_range(dev, 250);
```

`int` *mpu_ctl_clocksource*`(struct mpu_dev *`*dev*`, mpu_reg_t` *clksel*`)`

Sets the internal clock source register values [0-3] to use the appropriate device clock reference. The most sensible setting is [3]. To use one of the gyroscopes clock: [1] for X-axis gyroscope, [2] for Y-axis gyroscope and [3] for Z-axis gyroscope. Using the value [0] will select the internal 8Mhz oscillator which can negatively impact the sampling rate capabilities and the device responsiveness.  It is a synchronoous operations, which means that the function returns only after the requested operation completed. Please refer to the product datasheet and documentation for more details.

- *dev* is a pointer to an initialized *struct mpu_dev*.

- *clksel* is the desired CLKSEL register value [0-3]

Upon *SUCCESS(0)* device clocksource setting has desired configuration

Upon *FAILURE(1)* invalid setting or  bus error.

*EXAMPLE*
```
	mpu_ctl_gyro_clocksource(dev, 3);
```

2. *DATA*

The readings (*X*,*Y*,*Z*) are reported as follows.

* *ACCELEROMETERS (Ax,Ay,Az)* follow along the figure positive axis direction.

* *GYROSCOPES (Gx,Gy,Gz)* follow the right-hand rule, counter clockwise (CCW) on axis.

```
 -------------------------------------------
|                          Z                |
|                          ^                |
|                          |    Y           |
|        sensor body       |   ^            |
|        (top view)        |  /             |
|       ____________       | /              |
|      /*          /'      |/               |
|     / MPU-60050 /.       0---------> X    |
|    /           /.                         |
|   /           /.                          |
|  /___________/.                           |
|  '...........'                            |
 -------------------------------------------.

```

The sensor data from `struct mpu_dev *dev` available at:

* Accelerometers:
	* X-axis (G's): *\*(dev->Ax)* squared: *\*(dev->Ax2)*
	* Y-axis (G's): *\*(dev->Ay)* squared: *\*(dev->Ay2)*
	* Z-axis (G's): *\*(dev->Az)* squared: *\*(dev->Az2)*
	* TOTAL acceleration (G's): *\*(dev->AM)*

* Gyroscopes:
	* X-axis (dps): *\*(dev->Gx)* squared: *\*(dev->Gx2)*
	* Y-axis (dps): *\*(dev->Gy)* squared: *\*(dev->Gy2)*
	* Z-axis (dps): *\*(dev->Gz)* squared: *\*(dev->Gz2)*

* Temperature (Celsius): *\*(dev->t*);

* Sampling rate (Hz): *dev->sr*

* Sampling time (s): *dev->st*

* Sample count : *dev->samples*



SUMMARY
========

## SUPPORTED FEATURES

*Buffered readings*
: Takes reading at regular intervals from device internal fifo buffer

*Accelerometer control*
: enable/disable, range setting (+-2G, +-4G, +-8G, +-16G)

*Gyroscope control*
: enable/disable, range setting (+-250dps, +-500dps +-1000dps +-2000dps)

*Temperature sensor control*
: enable/disable

*Sampling rate control*
: Set sampling rate to 10,50,100,200 Hz

*Digital Low Pass filter control*
: enable/disable and configure the embedded DLPF (refer to datasheet for details)

*Calibration*
: sets calibration registers, fine tune the offsets, device must stay leveled and static

*Register dump*
: writes current register values to file

*Self-tests*
: triggers the device self-test, write report to file. Refer to the datasheet.


## UNSUPPORTED features

*eDMP (embedded Digital Motion Proccessor)*

*Low-power modes*

*External interrupts*

*External clock sources*

*Secondary i2c bus*


SEE ALSO
========

mpu6050-demo(1)

BUGS
====

Please report bugs at https://github.com/ThalesBarretto/libmpu6050/issues

AUTHOR
======

Thales Antunes de Oliveira Barretto <thales.barretto.git@gmail.com>

LICENSE
=======

MIT License.

COPYRIGHT
========

Copyright (c) Thales Antunes de Oliveira Barretto

