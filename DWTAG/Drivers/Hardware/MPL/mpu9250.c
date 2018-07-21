
#include "mpu9250.h"


static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
const unsigned char ismpu_calib=1;
const long gyro_bias_cst[3]={0x0005A4CD,0x079DA480,0x0011069E};
const long accel_bias_cst[3]={0x00F00000,0xFF264000,0x3D804000};//不完全，实际需要实现的功能是开辟一块flash区域来保存相关数据。
int MPU9250_Init(void)
{
	struct int_param_s int_param;
	long gyro_bias[3];
	long accel_bias[3];
	short gyro_bias2[3]={-40,-60,55};
	unsigned char regdata;
	unsigned char *p;
	int i;
	__set_PRIMASK(1);
	MPU_enable();
	Delay_ms(5);
	if(mpu_init(&int_param))
	{
		__set_PRIMASK(0);
		return -1;
	}
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	if(!ismpu_calib)
	{
		run_self_test();
	}
	else
	{
		memcpy(gyro_bias,gyro_bias_cst,sizeof(long)*3);
		memcpy(accel_bias,accel_bias_cst,sizeof(long)*3);
		dmp_set_gyro_bias(gyro_bias);
		dmp_set_accel_bias(accel_bias);
	}
	p=(unsigned char *)&gyro_bias2[0];//calib the gyro
	I2C_WriteRegister_9250(0x68,20,1,p);
	I2C_WriteRegister_9250(0x68,19,1,p+1);
	p=(unsigned char *)&gyro_bias2[1];
	I2C_WriteRegister_9250(0x68,22,1,p);
	I2C_WriteRegister_9250(0x68,21,1,p+1);
	p=(unsigned char *)&gyro_bias2[2];
	I2C_WriteRegister_9250(0x68,24,1,p);
	I2C_WriteRegister_9250(0x68,23,1,p+1);
	mpu_set_gyro_fsr(1000);//2000 16.4 1000 32.8 500 65.5 250 131
	mpu_set_accel_fsr(2);//2 16384 4 8192 8 4096 16 2048
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	I2C_ReadRegister_9250(0x68,35,1,&regdata);//I want to put the slv0(compass data) into fifo
	regdata|=0x01;
	I2C_WriteRegister_9250(0x68,35,1,&regdata);
	I2C_ReadRegister_9250(0x68,36,1,&regdata);
	regdata|=0x40;
	I2C_WriteRegister_9250(0x68,36,1,&regdata);//delay the dataready interrupt untill external senser's data is ready
	mpu_set_sample_rate(100);//in this fun compass sample rate is setted  parameter/2
	mpu_set_compass_sample_rate(100);
	
	
//		mpu_get_sample_rate(&gyro_rate);
//		mpu_get_gyro_fsr(&gyro_fsr);
//		mpu_get_accel_fsr(&accel_fsr);
	
//	dmp_load_motion_driver_firmware();
//	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
//	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP  | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
//	
//	dmp_set_fifo_rate(10);
	
//	mpu_set_dmp_state(1); 
	__set_PRIMASK(0);
	return 0;
}
void MPU_enable(void)
{
	MPU_CE_Port->BSRR = MPU_CE_Pin;
}
void MPU_disable(void)
{
	MPU_CE_Port->BRR = MPU_CE_Pin;
}

void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);

    }
	else
	{
		
	}
}						
/** Converts an orientation matrix made up of 0,+1,and -1 to a scalar representation.
* @param[in] mtx Orientation matrix to convert to a scalar.
* @return Description of orientation matrix. The lowest 2 bits (0 and 1) represent the column the one is on for the
* first row, with the bit number 2 being the sign. The next 2 bits (3 and 4) represent
* the column the one is on for the second row with bit number 5 being the sign.
* The next 2 bits (6 and 7) represent the column the one is on for the third row with
* bit number 8 being the sign. In binary the identity matrix would therefor be:
* 010_001_000 or 0x88 in hex.
*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}
