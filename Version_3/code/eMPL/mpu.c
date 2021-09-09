#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>

#define FSR 2000
//#define GYRO_SENS       ( 131.0f * 250.f / (float)FSR )
#define GYRO_SENS       16.375f 
#define QUAT_SENS       1073741824.f //2^30

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f

struct s_mympu mympu;

struct s_quat { float w, x, y, z; }; 

union u_quat {
	struct s_quat _f;
	long _l[4];
} q;

static int ret;
static short gyro[3];

// SAAR CODE:
// MPU
short accel[3];

static short sensors;
static unsigned char fifoCount;

int mympu_open(unsigned int rate) {
 	mpu_select_device(0);
 	mpu_init_structures();
	
	ret = mpu_init(NULL);
	
#ifdef MPU_DEBUG
	if (ret) return 10+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_init", ret);
#endif	
	
	ret = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL); 
#ifdef MPU_DEBUG
	if (ret) return 20+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_set_sensors", ret);
#endif	

  ret = mpu_set_gyro_fsr(FSR);
#ifdef MPU_DEBUG
	if (ret) return 30+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_set_gyro_fsr", ret);
#endif	

  ret = mpu_set_accel_fsr(2);
#ifdef MPU_DEBUG
	if (ret) return 40+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_set_accel_fsr", ret);
#endif	

  mpu_get_power_state((unsigned char *)&ret);
#ifdef MPU_DEBUG
	if (!ret) return 50+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_get_power_state", ret);
#endif	

  ret = mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);
#ifdef MPU_DEBUG
	if (ret) return 60+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("mpu_configure_fifo", ret);
#endif	

	dmp_select_device(0);
	dmp_init_structures();

	ret = dmp_load_motion_driver_firmware();
#ifdef MPU_DEBUG
	if (ret) return 80+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("dmp_load_motion_driver_firmware", ret);
#endif	

	ret = dmp_set_fifo_rate(rate);
#ifdef MPU_DEBUG
	if (ret) return 90+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("dmp_set_fifo_rate", ret);
#endif	

	ret = mpu_set_dmp_state(1);
#ifdef MPU_DEBUG
	if (ret) return 100+ret;
#endif	
#ifdef SAAR_DEBUG
	LOG("mpu_set_dmp_state", ret);
#endif	

// SAAR CODE:
// MPU
	ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |DMP_FEATURE_ANDROID_ORIENT); // Set by SAAR
//	ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL); // Set by Rpicopter
//	ret = dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL); // Commented by Rpicopter

#ifdef MPU_DEBUG
	if (ret) return 110+ret;
#endif
#ifdef SAAR_DEBUG
	LOG("dmp_enable_feature", ret);
#endif	

	return 0;
}

static inline float rad2deg( float rad )
{
        //return (180.f/PI) * rad;
	return 57.2957795131f * rad;
}

static float test, sqy, sqz, sqw, sqx;
static void quaternionToEuler( const struct s_quat *q, float* x, float* y, float* z )
{
        sqw = q->w * q->w;
				sqx = q->x * q->x;
        sqy = q->y * q->y;
        sqz = q->z * q->z;

        test = q->x * q->z - q->w * q->y;

        if( test > 0.5f - EPSILON )
        {
                *x = 2.f * atan2( q->y, q->w );
                *y = PI_2;
                *z = 0;
        }
        else if( test < -0.5f + EPSILON )
        {
                *x = -2.f * atan2( q->y, q->w );
                *y = -PI_2;
                *z = 0;
        }
        else
        {
					// SAAR CODE:
					// MPU
					
								// Rpicopter formulas:
                *x = atan2( 2.f * ( q->x * q->w + q->y * q->z ), 1.f - 2.f * ( sqz + sqw ) );
                *y = asin( 2.f * test );
                *z = atan2( 2.f * ( q->x * q->y - q->z * q->w ), 1.f - 2.f * ( sqy + sqz ) );
					/*					
								// SAAR formulas:
                *x = atan2( 2.f * ( q->w * q->x + q->y * q->z ), 1.f - 2.f * ( sqx + sqy ) );
                *y = asin( 2.f * (q->w * q->y - q->x * q->z) );
                *z = atan2( 2.f * ( q->w * q->z + q->x * q->y ), 1.f - 2.f * ( sqy + sqz ) );
					*/
        }
}

/*
static inline float wrap_180(float x) {
	return (x<-180.f?x+360.f:(x>180.f?x-180.f:x));
}
*/

int mympu_update(void) {

	do {
				// SAAR CODE:
				// MPU
				//ret = dmp_read_fifo(gyro, NULL, q._l, NULL, &sensors, &fifoCount);  // Rpicopter Code
				ret = dmp_read_fifo(gyro, accel, q._l, NULL, &sensors, &fifoCount);		// SAAR CODE
						
				/* will return:
					0  - if ok
					1  - no packet available
					2  - if BIT_FIFO_OVERFLOWN is set
					3  - if frame corrupted
					<0 - if error
				*/
				if (ret!=0) return ret;

	} while (fifoCount>1);
	
	q._f.w = (float)q._l[0] / (float)QUAT_SENS;
	q._f.x = (float)q._l[1] / (float)QUAT_SENS;
	q._f.y = (float)q._l[2] / (float)QUAT_SENS;
	q._f.z = (float)q._l[3] / (float)QUAT_SENS;

	// SAAR CODE:
	// MPU
	//LOG_QUAT(q._f.w, q._f.x, q._f.y, q._f.z);
	
	quaternionToEuler( &q._f, &mympu.ypr[2], &mympu.ypr[1], &mympu.ypr[0] );
	
	/* need to adjust signs and do the wraps depending on the MPU mount orientation */ 
	/* if axis is no centered around 0 but around i.e 90 degree due to mount orientation */
	/* then do:  mympu.ypr[x] = wrap_180(90.f+rad2deg(mympu.ypr[x])); */
	
	// SAAR CODE:
	// MPU
	//mympu.ypr[0] = wrap_180(90.f+rad2deg(mympu.ypr[0]));
	//mympu.ypr[1] = wrap_180(90.f+rad2deg(mympu.ypr[1]));
	//mympu.ypr[2] = wrap_180(90.f+rad2deg(mympu.ypr[2]));
	
	mympu.ypr[0] = rad2deg(mympu.ypr[0]);
	mympu.ypr[1] = rad2deg(mympu.ypr[1]);
	mympu.ypr[2] = rad2deg(mympu.ypr[2]);

	/* need to adjust signs depending on the MPU mount orientation */ 
	mympu.gyro[0] = -(float)gyro[2] / GYRO_SENS;
	mympu.gyro[1] = (float)gyro[1] / GYRO_SENS;
	mympu.gyro[2] = (float)gyro[0] / GYRO_SENS;

	return 0;
}

