#include "mpu6050_core.h"

int mpu_diagnose(struct mpu_dev * dev)
{
	if(MPUDEV_IS_NULL(dev))
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
	printf("%-20s %d\n","SLEEP BIT"		, dev->cfg->sleep );
	printf("%-20s %d\n","CYCLE BIT"		, dev->cfg->cycle );
	printf("%-20s %d\n","TEMP_DIS BIT"	, dev->cfg->temp_dis );
	printf("%-20s %d\n","STDBY_XA BIT"	, dev->cfg->stdby_xa );
	printf("%-20s %d\n","STDBY_YA BIT"	, dev->cfg->stdby_ya );
	printf("%-20s %d\n","STDBY_ZA BIT"	, dev->cfg->stdby_za );
	printf("%-20s %d\n","STDBY_XG BIT"	, dev->cfg->stdby_xg );
	printf("%-20s %d\n","STDBY_YG BIT"	, dev->cfg->stdby_yg );
	printf("%-20s %d\n","STDBY_ZG BIT"	, dev->cfg->stdby_zg );
	printf("%-20s %d\n","I2C_MST_EN BIT"	, dev->cfg->i2c_mst_en );
	printf("%-20s %d\n","I2C_IF_DIS BIT"	, dev->cfg->i2c_if_dis );
	printf("%-20s %d\n","FIFO_EN BIT"	, dev->cfg->fifo_en );
	printf("%-20s %d\n","TEMP_FIFO_EN BIT"	, dev->cfg->temp_fifo_en );
	printf("%-20s %d\n","FIFO Accel BIT"	, dev->cfg->accel_fifo_en );
	printf("%-20s %d\n","FIFO Gyro X BIT"	, dev->cfg->xg_fifo_en );
	printf("%-20s %d\n","FIFO Gyro Y BIT"	, dev->cfg->yg_fifo_en );
	printf("%-20s %d\n","FIFO Gyro Z BIT"	, dev->cfg->zg_fifo_en );
	printf("%-20s %d\n","FIFO SLV0"		, dev->cfg->slv0_fifo_en );
	printf("%-20s %d\n","FIFO SLV1"		, dev->cfg->slv1_fifo_en );
	printf("%-20s %d\n","FIFO SLV2"		, dev->cfg->slv2_fifo_en );
	printf("%-20s %d\n","FIFO SLV3"		, dev->cfg->slv3_fifo_en );
	printf("%-20s %d\n","FIFO SLV4"		, dev->cfg->slv4_fifo_en );
	printf("%-20s %d\n","FSYNC_INT_EN BIT"	, dev->cfg->fsync_int_en );
	printf("%-20s %d\n","FIFO_OFLOW_EN BIT"	, dev->cfg->fifo_oflow_en );
	printf("%-20s %d\n","I2C_MST_INT_EN BIT", dev->cfg->i2c_mst_int_en );
	printf("%-20s %d\n","DATA_RDY_EN BIT"	, dev->cfg->data_rdy_en );
	printf("%-20s %d\n","raw[0]"		, dev->dat->raw[0] );
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
