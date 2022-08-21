# libmpu6050: unofficial MPU6050 userspace driver

**What it is**: a library facilitating interaction with the Invensense MPU-6050 device on a Raspberry Pi 3B.

## Requirements

1. Raspberri Pi 3B running the official and up-to-date distro.
2. Invensense MPU6050 attached to the Raspbery Pi 3B 
3. Dependencies libi2c libi2c-dev pandoc

## Building, installing, removing

0. Install the depedendencies:

`libi2c` - runtime dependecy
`libi2c-dev` - make dependecy
`pandoc` - make dependency for documentation

```bash
pi@pi ~ $> sudo apt update
pi@pi ~ $> sudo apt install libi2c libi2c-dev pandoc
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

## Contributing

* Please send pull requests on github.
