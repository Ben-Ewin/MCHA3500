#include "IMU.h"
#include "tm_stm32_mpu6050.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* Variable declarations */
TM_MPU6050_t IMU_datastruct;

TM_MPU6050_Result_t status;

/* Function defintions */
void IMU_init(void)
{
    /* Initialise IMU with AD0 LOW, accelleration sensitivity +-4g, gyroscope +-250 deg/s */
    TM_MPU6050_Init(&IMU_datastruct, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s);

}

void IMU_read(void)
{
    /* Read all IMU values */
    TM_MPU6050_ReadAll(&IMU_datastruct);
}

float get_gyroX(void)
{
    /* Convert accelleration reading to ms^-2 */
    int16_t gyroX_raw = IMU_datastruct.Gyroscope_X;

    // scale = size of data type/(g * acceleration range)
    float gyroX = (float)gyroX_raw / 7.5096e+03f;
    /* return the Y acceleration */
    return gyroX;
}

float get_accY(void)
{
    /* Convert accelleration reading to ms^-2 */
    int16_t accY_raw = IMU_datastruct.Accelerometer_Y;

    // scale = size of data type/(g * acceleration range)
    float accY = (float)accY_raw / 835.0408f;
    /* return the Y acceleration */
    return accY;
}

float get_accZ(void)
{
    /* Convert accelleration reading to ms^-2 */
    int16_t accZ_raw = IMU_datastruct.Accelerometer_Z;

    // scale = size of data type/(g * acceleration range)
    float accZ = (float)accZ_raw / 835.0408f;
    /* return the Y acceleration */
    return accZ;
}

double get_acc_angle(void)
{
    /* compute IMU angle using accY and accZ using atan2 */
    double Theta_X = atan2(get_accZ(), get_accY());
    /* return the IMU angle */
    return Theta_X;
}