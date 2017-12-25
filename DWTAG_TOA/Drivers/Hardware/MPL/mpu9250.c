
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

void MPU9250_Init(void)
{
			struct int_param_s int_param;
		__set_PRIMASK(1);

		mpu_init(&int_param);
		mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
					//run_self_test();
		mpu_set_gyro_fsr(1000);//2000 16.4 1000 32.8 500 65.5 250 131
		mpu_set_accel_fsr(2);//2 16384 4 8192 8 4096 16 2048
		mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
		mpu_set_sample_rate(100);
//		mpu_get_sample_rate(&gyro_rate);
//		mpu_get_gyro_fsr(&gyro_fsr);
//		mpu_get_accel_fsr(&accel_fsr);
		dmp_load_motion_driver_firmware();
//			//
		dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));


//			//
		dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP  | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		    DMP_FEATURE_GYRO_CAL);
		dmp_set_fifo_rate(10);

	__set_PRIMASK(0);
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
