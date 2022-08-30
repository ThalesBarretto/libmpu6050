#include "mpu6050_private.h"

int mpu_dev_bind(const char *path, const mpu_reg_t address, struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (address == (mpu_reg_t)0x00) /* invalid address */
		return -1;

	/* path must contain at least "/dev/i2c" */
	size_t pathlen = strlen(path);
	if (NULL == (path ) || (pathlen < 6)) /* invalid path */
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

int mpu_dev_allocate(struct mpu_dev **dev)
{
	if(MPUDEV_NOT_NULL(*dev)) /* already initialized object */
		return -1;

	if (NULL == (*dev = (struct mpu_dev *)calloc(1, sizeof(struct mpu_dev))))
		goto exit_dev;

	if (NULL == ((*dev)->bus = (int *)calloc(1, sizeof(int))))
		goto exit_dev_bus;

	if (NULL == ((*dev)->cfg = (struct mpu_cfg *)calloc(1, sizeof(struct mpu_cfg))))
		goto exit_dev_cfg;

	if (NULL == ((*dev)->cal = (struct mpu_cal *)calloc(1, sizeof(struct mpu_cal))))
		goto exit_dev_cal;

	if (NULL == ((*dev)->dat = (struct mpu_dat *)calloc(1, sizeof(struct mpu_dat))))
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

int mpu_cfg_set(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
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

int mpu_dat_set(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	/* Associate data with meaningful names */
	int count = 0;
	if (dev->cfg->accel_fifo_en) {
		count++;
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
	}
	if (dev->cfg->temp_fifo_en)	{
		count++;
		dev->dat->scl[count] = 1/340.0;
		dev->dat->dat[count][0] = 0;
		dev->t  = &dev->dat->dat[count][0];
		dev->to = &dev->cal->off[count];
		dev->tg = &dev->cal->gai[count];
		dev->td = &dev->cal->dri[count];
		dev->tm = &dev->dat->mea[count];
		dev->tv = &dev->dat->var[count];
	}
	if (dev->cfg->xg_fifo_en)	{
		count++;
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
	}
	if (dev->cfg->yg_fifo_en)	{
		count++;
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
	}
	if (dev->cfg->zg_fifo_en)	{
		count++;
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
	}
	/* don't know if the ordering is correct */
	if (dev->cfg->slv0_fifo_en) {
		count++;
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv0_dat = &dev->dat->dat[count][0];
	}
	if (dev->cfg->slv1_fifo_en) {
		count++;
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv1_dat = &dev->dat->dat[count][0];
	}
	if (dev->cfg->slv2_fifo_en) {
		count++;
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv2_dat = &dev->dat->dat[count][0];
	}
	if (dev->cfg->slv3_fifo_en) {
		count++;
		dev->dat->scl[count] = 1/dev->albs;
		dev->slv3_dat = &dev->dat->dat[count][0];
	}
	if (dev->cfg->slv4_fifo_en) {
		count++;
		dev->dat->scl[count] = 1/dev->albs;
		dev->dat->dat[count][0] = 0;
		dev->slv4_dat = &dev->dat->dat[count][0];
	}

	return 0;
}

int mpu_cfg_reset(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	memcpy((void *)dev->cfg, (void *)&mpu6050_defcfg, sizeof(struct mpu_cfg));

	if (mpu_cfg_set(dev) < 0) /* couldn't set config */
		return -1;

	return 0;
}

int mpu_dat_reset(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	/* start at [1], index [0] should not be changed */
	for(size_t i = 1; i < ARRAY_LEN(dev->dat->scl); i++) {
		dev->dat->scl[i] = 1; /* scaling factor of 1 */
		dev->dat->dat[i][0] = 0; /* all data zeroed */
	}
	/* enforce forbidden access after reset */
	dev->AM = NULL;
	dev->GM = NULL;
	dev->Ax = dev->Ax2 = dev->Axo = dev->Axg = dev->Axm = dev->Axv = dev->Axd = NULL;
	dev->Ay = dev->Ay2 = dev->Ayo = dev->Ayg = dev->Aym = dev->Ayv = dev->Ayd = NULL;
	dev->Az = dev->Az2 = dev->Azo = dev->Azg = dev->Azm = dev->Azv = dev->Azd = NULL;
	dev->Gx = dev->Gx2 = dev->Gxo = dev->Gxg = dev->Gxm = dev->Gxv = dev->Gxd = NULL;
	dev->Gy = dev->Gy2 = dev->Gyo = dev->Gyg = dev->Gym = dev->Gyv = dev->Gyd = NULL;
	dev->Gz = dev->Gz2 = dev->Gzo = dev->Gzg = dev->Gzm = dev->Gzv = dev->Gzd = NULL;
	dev->t = 	  dev->to =  dev->tg =  dev->tm =  dev->tv =  dev->td = NULL;
	dev->slv0_dat = dev->slv1_dat = dev->slv2_dat = dev->slv3_dat = dev->slv4_dat = NULL;

	return 0;
}

int mpu_cal_reset(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	for (size_t i = 0; i < ARRAY_LEN(dev->cal->off); i++) {
		dev->cal->gai[i] = 1;
		dev->cal->off[i] = 0;
		dev->cal->dri[i] = 0;
	}
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

int mpu_dev_parameters_save(char *fn, struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if(NULL == fn) {
		fprintf(stderr, "%s failed: NULL filename\n", __func__);
		return -1;
	}

	FILE *dmp;
	if (NULL ==  (dmp = fopen(fn, "w+"))) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fn);
		exit(EXIT_FAILURE);
	}
	fwrite(dev->cfg, sizeof(*(dev->cfg)), 1, dmp);
	fwrite(dev->cal, sizeof(*(dev->cal)), 1, dmp);
	fclose(dmp);
	return 0;
}

int mpu_dev_parameters_restore(char *fn, struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if(NULL == fn) {
		fprintf(stderr, "%s failed: NULL filename\n", __func__);
		return -1;
	}

	FILE * fp;
	if (NULL ==  (fp = fopen(fn, "r"))) {
		fprintf(stderr, "Unable to open file \"%s\"\n", fn);
		exit(EXIT_FAILURE);
	}
	fread(dev->cfg, sizeof(*(dev->cfg)), 1, fp);
	fread(dev->cal, sizeof(*(dev->cal)), 1, fp);
	fclose(fp);

	mpu_ctl_fifo_flush(dev);
	return 0;
}

int mpu_ctl_calibration_reset(struct mpu_dev *dev)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

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

int mpu_ctl_calibration_restore(struct mpu_dev *dev, struct mpu_cal *bkp)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

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

int mpu_cfg_set_CLKSEL(struct mpu_dev *dev, mpu_reg_t clksel)
{
	if(MPUDEV_IS_NULL(dev)) /* incomplete or uninitialized object */
		return -1;

	if (clksel > 7) /* out of range  */
		return -1;

	switch(clksel) {
		case CLKSEL_0 : dev->clksel = 0; break; /* Internal 8 Mhz oscillator		*/
		case CLKSEL_1 : dev->clksel = 1; break; /* PLL with X axis gyroscope		*/
		case CLKSEL_2 : dev->clksel = 2; break; /* PLL with Y axis gyroscope		*/
		case CLKSEL_3 : dev->clksel = 3; break; /* PLL with Z axis gyroscope		*/
		case CLKSEL_4 : dev->clksel = 4; break; /* PLL with external 32.768 kHz reference*/
		case CLKSEL_5 : dev->clksel = 5; break; /* PLL with external 19.2 MHz reference	*/
		case CLKSEL_6 : dev->clksel = 6; break; /* RESERVED */
		case CLKSEL_7 : dev->clksel = 7; break; /* Stops the clock, keeps timing in reset */
		default: /* not supported */
			return -1;
	}

	mpu_reg_t val;
	if (mpu_read_byte(dev, PWR_MGMT_1, &val) < 0)
		return -1;

	val  &= ~CLKSEL_BIT;	/* mask bits */
	val  |= clksel;		/* set  bits */

	if (mpu_write_byte(dev, PWR_MGMT_1, val) < 0)
		return -1;

	return 0;
}

