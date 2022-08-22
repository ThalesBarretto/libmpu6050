# libmpu6050: unofficial MPU6050 userspace driver

**What it is**: a library facilitating interaction with the Invensense MPU-6050 device on a Raspberry Pi 3B.

## Requirements

1. Raspberri Pi 3B running the official and up-to-date distro.
2. Invensense MPU6050 attached to the Raspbery Pi 3B 
3. Dependencies libi2c libi2c-dev pandoc gzip

## Building, installing, removing

0. Install the depedendencies:

* `libi2c` - runtime dependecy
* `libi2c-dev` - make dependecy
* `pandoc` - make dependency for documentation
* `gzip` - make dependency for documentation

```bash
pi@pi ~ $> sudo apt update
pi@pi ~ $> sudo apt install libi2c libi2c-dev pandoc gzip
```

1. Clone
2. Make 
3. Make install
4. Read the manual page

```bash
pi@pi ~ $> mkdir repo && cd repo
pi@pi ~/repo $> git clone https://github.com/ThalesBarretto/libmpu6050.git
pi@pi ~/repo $> cd libmpu6050
pi@pi ~/repo/libmpu6050 $> make
pi@pi ~/repo/libmpu6050 $> make install
pi@pi ~/repo/libmpu6050 $> man libmpu6050
```

## Clean and Removal

* make clean
* make uninstall

```bash
pi@pi ~/repo/libmpu6050 $> make clean
pi@pi ~/repo/libmpu6050 $> make uninstall
```

## Usage, examples

See [mpu6050-demo](<https://github.com/ThalesBarretto/mpu6050>) for examples.

## Design principles

The API exposes the `struct mpu_dev` object which, when initialized, represents the MPU6050 device state. It contains all the relevant device info according to specs, and can be read by your application for many purposes.

The functions serve the purpose to set the device configuration and get sensor data. Every reading must call the function to get data, so that it can update the referred `struct mpu_dev` with the next sensor readings for applicatoion use.

After initalization up untill destroy, the I2C buss will be held exclusively for the MPU6050 device.

Sharing of the I2C bus is unsupported, and there is no coordination mechanism in place.

In fact, the device can easily saturate the bus when reading over 200Hz, making it unfeasible to share it.

All functions are synchronous, returning only after completion or error. On success, they return 0.

There is no resource locking mechanism, although only one running instance is supported.

All functions are thread-unsafe. One should not multithread with libmpu6050 functions.

## Contributing

Contributions are welcome on github, on three channels:

* Open an [issue](<https://github.com/ThalesBarretto/libmpu6050/issues>).
* Start or enter a [discussion](<https://github.com/ThalesBarretto/libmpu6050/discussions>).
* Send a [pull request](<https://github.com/ThalesBarretto/libmpu6050/pulls>).
