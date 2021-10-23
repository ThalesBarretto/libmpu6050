#ifdef __cplusplus
	extern "C" {	
#endif
#ifndef __MPU6050_MODEL_H_
#define __MPU6050_MODEL_H_

struct __attribute__((__packed__)) mpu6050 {
	unsigned int					: 8;	// reg0
	unsigned int					: 8;	// reg1
	unsigned int					: 8;	// reg2
	unsigned int aux_vddio				: 8;	// reg3
	unsigned int					: 8;	// reg4
	unsigned int					: 8;	// reg5
	union __attribute__((__packed__)) {		 	// reg6
		unsigned int xa_offs_usr 		: 16;
		struct __attribute__((__packed__)) {
			unsigned int xa_offs_usrh	: 8;
			unsigned int xa_offs_usrl	: 8;
		};
	};
	union __attribute__((__packed__)) {			// reg7
		unsigned int ya_offs_usr 		: 16;
		struct __attribute__((__packed__)) {
			unsigned int ya_offs_usrh	: 8;
			unsigned int ya_offs_usrl	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int za_offs_usr 		: 16;
		struct __attribute__((__packed__)) {
			unsigned int za_offs_usrh	: 8;
			unsigned int za_offs_usrl	: 8;
		};
	};
	unsigned int prod_id 				: 8;
	union __attribute__((__packed__)) {
		unsigned int self_test_x 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int xg_test_42 	: 5;
			unsigned int xa_test_42 	: 3;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int self_test_y 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int ya_test_42 	: 3;
			unsigned int yg_test_42 	: 5;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int self_test_z 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int za_test_42 	: 3;
			unsigned int zg_test_42 	: 5;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int self_test_a 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int za_test_10		: 2;
			unsigned int ya_test_10		: 2;
			unsigned int xa_test_10		: 2;
			unsigned int			: 2;
		};
	};
	unsigned int 					: 8; // reg 17
	unsigned int 					: 8; // reg 18
	union __attribute__((__packed__)) {
		unsigned int xg_offs_usr		: 16;
		struct __attribute__((__packed__)) {
			unsigned int xg_offs_usrh	: 8;
			unsigned int xg_offs_usrl	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int yg_offs_usr		: 16;
		struct __attribute__((__packed__)) {
			unsigned int yg_offs_usrh	: 8;
			unsigned int yg_offs_usrl	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int zg_offs_usr 		: 16;
		struct __attribute__((__packed__)) {
			unsigned int zg_offs_usrh	: 8;
			unsigned int zg_offs_usrl	: 8;
		};
	};
	unsigned int smplrt_div 			: 8;
	union __attribute__((__packed__)) {
		unsigned int config 			: 8;
		struct __attribute__((__packed__)) {
			unsigned int dlpf_cfg		: 3;
			unsigned int ext_sync_set	: 3;
			unsigned int			: 2;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int gyro_config 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int			: 3;
			unsigned int fs_sel		: 2;
			unsigned int zg_st		: 1;
			unsigned int yg_st		: 1;
			unsigned int xg_st		: 1;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int accel_config		 : 8;
		struct __attribute__((__packed__)) {
			unsigned int			: 3;
			unsigned int afs_sel		: 2;
			unsigned int za_st		: 1;
			unsigned int ya_st		: 1;
			unsigned int xa_st		: 1;
		};
	};
	unsigned int ff_thr				: 8;
	unsigned int ff_dur				: 8;
	unsigned int mot_thr				: 8;
	unsigned int mot_dur				: 8;
	unsigned int zrmot_thr				: 8;
	unsigned int zrmot_dur				: 8;
	union __attribute__((__packed__)) {
		unsigned int fifo_enable 		: 8; // used FIFO_EN register
		struct __attribute__((__packed__)) {
			unsigned int slv0_fifo_en	: 1;
			unsigned int slv1_fifo_en	: 1;
			unsigned int slv2_fifo_en	: 1;
			unsigned int accel_fifo_en	: 1;
			unsigned int zg_fifo_en		: 1;
			unsigned int yg_fifo_en		: 1;
			unsigned int xg_fifo_en		: 1;
			unsigned int temp_fifo_en	: 1;
		};
	};

	union __attribute__((__packed__)) {
		unsigned int i2c_mst_ctrl 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int i2c_mst_clk	: 4;
			unsigned int i2c_mst_p_nsr	: 1;
			unsigned int slv3_fifo_en	: 1;
			unsigned int wait_for_es	: 1;
			unsigned int mult_mst_en	: 1;
		};
	};
	unsigned int i2c_slv0_addr			: 8;
	unsigned int i2c_slv0_reg			: 8;
	unsigned int i2c_slv0_ctrl			: 8;
	unsigned int i2c_slv1_addr			: 8;
	unsigned int i2c_slv1_reg			: 8;
	unsigned int i2c_slv1_ctrl			: 8;
	unsigned int i2c_slv2_addr			: 8;
	unsigned int i2c_slv2_reg			: 8;
	unsigned int i2c_slv2_ctrl			: 8;
	unsigned int i2c_slv3_addr			: 8;
	unsigned int i2c_slv3_reg			: 8;
	unsigned int i2c_slv3_ctrl			: 8;
	unsigned int i2c_slv4_addr			: 8;
	unsigned int i2c_slv4_reg			: 8;
	unsigned int i2c_slv4_do			: 8;
	unsigned int i2c_slv4_ctrl			: 8;
	unsigned int i2c_slv4_di			: 8;
	unsigned int i2c_mast_status 			: 8;
	union __attribute__((__packed__)) {
		unsigned int int_pin_cfg 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int int_level		: 1;
			unsigned int int_open		: 1;
			unsigned int latch_int		: 1;
			unsigned int int_rd_clear	: 1;
			unsigned int fsync_int_level	: 1;
			unsigned int fsync_int_en	: 1;
			unsigned int i2c_bypass_en	: 1;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int int_enable 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int data_rdy_en	: 1;
			unsigned int 			: 2;
			unsigned int i2c_mst_int_en	: 1;
			unsigned int fifo_oflow_en	: 1;
			unsigned int zmot_en		: 1;
			unsigned int mot_en		: 1;
			unsigned int ff_en		: 1;
		};

	};
	unsigned int dmp_int_status 			: 8;
	union __attribute__((__packed__)) {
		unsigned int int_status 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int data_rdy_int	: 1;
			unsigned int 			: 2;
			unsigned int i2c_mst_int_int	: 1;
			unsigned int fifo_oflow_int	: 1;
			unsigned int 			: 3;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int accel_xout			: 16;
		struct __attribute__((__packed__)) {
			unsigned int accel_xout_l	: 8;
			unsigned int accel_xout_h	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int accel_yout			: 16;
		struct __attribute__((__packed__)) {
			unsigned int accel_yout_h	: 8;
			unsigned int accel_yout_l	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int accel_zout			: 16;
		struct __attribute__((__packed__)) {
			unsigned int accel_zout_h	: 8;
			unsigned int accel_zout_l	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int temp_out			: 16;
		struct __attribute__((__packed__)) {
			unsigned int temp_out_h		: 8;
			unsigned int temp_out_l		: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int gyro_xout			: 16;
		struct __attribute__((__packed__)) {
		unsigned int gyro_xout_h		: 8;
		unsigned int gyro_xout_l		: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int gyro_yout			: 16;
		struct __attribute__((__packed__)) {
			unsigned int gyro_yout_h	: 8;
			unsigned int gyro_yout_l	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int gyro_zout			: 16;
		struct __attribute__((__packed__)) {
			unsigned int gyro_zout_h	: 8;
			unsigned int gyro_zout_l	: 8;
		};
	};
	unsigned int ext_sens_data_00			: 8;
	unsigned int ext_sens_data_01			: 8;
	unsigned int ext_sens_data_02			: 8;
	unsigned int ext_sens_data_03			: 8;
	unsigned int ext_sens_data_04			: 8;
	unsigned int ext_sens_data_05			: 8;
	unsigned int ext_sens_data_06			: 8;
	unsigned int ext_sens_data_07			: 8;
	unsigned int ext_sens_data_08			: 8;
	unsigned int ext_sens_data_09			: 8;
	unsigned int ext_sens_data_10			: 8;
	unsigned int ext_sens_data_11			: 8;
	unsigned int ext_sens_data_12			: 8;
	unsigned int ext_sens_data_13			: 8;
	unsigned int ext_sens_data_14			: 8;
	unsigned int ext_sens_data_15			: 8;
	unsigned int ext_sens_data_16			: 8;
	unsigned int ext_sens_data_17			: 8;
	unsigned int ext_sens_data_18			: 8;
	unsigned int ext_sens_data_19			: 8;
	unsigned int ext_sens_data_20			: 8;
	unsigned int ext_sens_data_21			: 8;
	unsigned int ext_sens_data_22			: 8;
	unsigned int ext_sens_data_23			: 8;
	union __attribute__((__packed__)) {
		unsigned int mot_detect_status		: 8;
		struct __attribute__((__packed__)) {
			unsigned int mot_zpos		: 1;
			unsigned int mot_zneg		: 1;
			unsigned int mot_ypos		: 1;
			unsigned int mot_yneg		: 1;
			unsigned int mot_xpos		: 1;
			unsigned int mot_xneg		: 1;
		};
	};
	unsigned int 					: 8; // reg98
	unsigned int i2c_slv0_do			: 8;
	unsigned int i2c_slv1_do			: 8;
	unsigned int i2c_slv2_do			: 8;
	unsigned int i2c_slv3_do			: 8;
	union __attribute__((__packed__)) {
		unsigned int i2c_mst_delay_ctrl		: 8;
		struct __attribute__((__packed__)) {
			unsigned int i2c_slv1_dly_en	: 1;
			unsigned int i2c_slv2_dly_en	: 1;
			unsigned int i2c_slv3_dly_en	: 1;
			unsigned int i2c_slv4_dly_en	: 1;
			unsigned int 			: 2;
			unsigned int delay_es_shadow	: 1;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int signal_path_reset		: 8;
		struct __attribute__((__packed__)) {
			unsigned int temp_reset		: 1;
			unsigned int accel_reset	: 1;
			unsigned int gyro_reset		: 1;
			unsigned int 			: 5;
		};
	};
	unsigned int mot_detect_ctrl			: 8;
	union __attribute__((__packed__)) {
		unsigned int user_ctrl			: 8;
		struct __attribute__((__packed__)) {
			unsigned int sig_cond_reset	: 1;
			unsigned int i2c_mst_reset	: 1;
			unsigned int fifo_reset		: 1;
			unsigned int 			: 1;
			unsigned int i2c_if_dis		: 1;
			unsigned int i2c_mst_en		: 1;
			unsigned int fifo_en		: 1;
			unsigned int 			: 1;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int pwr_mgmt_1			: 8;
		struct __attribute__((__packed__)) {
			unsigned int clksel		: 3;
			unsigned int temp_dis		: 1;
			unsigned int 			: 1;
			unsigned int cycle		: 1;
			unsigned int sleep		: 1;
			unsigned int device_reset	: 1;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int pwr_mgmt_2 		: 8;
		struct __attribute__((__packed__)) {
			unsigned int stdby_zg		: 1;
			unsigned int stdby_yg		: 1;
			unsigned int stdby_xg		: 1;
			unsigned int stdby_za		: 1;
			unsigned int stdby_ya		: 1;
			unsigned int stdby_xa		: 1;
			unsigned int lp_wake_ctl	: 2;
		};
	};
	unsigned int bank_sel				: 8;
	unsigned int mem_start_addr 			: 8;
	union __attribute__((__packed__)) {
		unsigned int prgrm_start 		: 16;
		struct __attribute__((__packed__)) {
			unsigned int prgm_start_h 	: 8;
			unsigned int prgm_start_l	: 8;
		};
	};
	union __attribute__((__packed__)) {
		unsigned int fifo_count			: 16;
		struct __attribute__((__packed__)) {
			unsigned int fifo_count_h	: 8;
			unsigned int fifo_count_l	: 8;
		};
	};
	unsigned int fifo_r_w : 8;
	unsigned int who_am_i : 8;
};

#endif /* __MPU6050_MODEL_H_ */

#ifdef __cplusplus
	}
#endif
