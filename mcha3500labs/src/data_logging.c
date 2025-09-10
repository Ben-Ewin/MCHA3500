#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "data_logging.h"
#include "pendulum.h"
#include "IMU.h"

/* Variable declarations */
uint16_t logCount;

static float pot_value = 0.0;

osTimerId_t dataLogging_id;

uint32_t start_time;
uint32_t current_time;
uint32_t elapsed_time_ms;

static void (*log_function)(void);

/* Function declarations */
static void log_pointer(void *argument);

/* Function declarations */
static void log_data(void);

/* Function defintions */
static void log_data(void)
{
    current_time = osKernelGetTickCount();
    /*  Read the potentiometer voltage */
    pot_value = pendulum_read_voltage();
    
    /* Calculate elapsed time in milliseconds */
    elapsed_time_ms = (current_time - start_time) * osKernelGetTickFreq() / 1000;
    
    /* Print the sample time and potentiometer voltage to the serial terminal in the format [time],[voltage] */
    printf("%.3f, %.3f\n", elapsed_time_ms / 1000.0f, pot_value);
    
    /*  Increment log count */
    logCount = logCount + 1;
    
    /* Stop logging once 2 seconds is reached */
    if (logCount >= 400)
    {
        logging_stop();
    }
}

void logging_init(void)
{
    /* TODO: Initialise timer for use with pendulum data logging */
    dataLogging_id = osTimerNew(log_pointer, osTimerPeriodic, (void *)5, NULL);
}

static void log_pointer(void *argument)
{
    UNUSED(argument);
    /* Call function pointed to by log_function */
    (*log_function)();
}

void logging_start(void)
{
    log_function = &log_data;

    /*  Reset the log counter */
    logCount = 0;
    /*  Start data logging timer at 200Hz */
    osStatus_t status = osTimerStart(dataLogging_id, 5);
    UNUSED(status);
    start_time = osKernelGetTickCount();

    log_data();
}

void logging_stop(void)
{
    /*  Stop data logging timer */
    osStatus_t status = osTimerStop(dataLogging_id);
    UNUSED(status);
}

void imu_logging_start(void)
{
    /* Change function pointer to the imu logging function (log_imu) */
    log_function = &log_imu;

    /* Reset the log counter */
    logCount = 0;

    /* Start data logging at 200Hz */
    osStatus_t status = osTimerStart(dataLogging_id, 5);
    UNUSED(status);
    start_time = osKernelGetTickCount();

    log_imu();
}

static void log_imu(void)
{
    current_time = osKernelGetTickCount();
    /* Read IMU */
    IMU_read();
    /* Get the imu angle from accelerometer readings */
    double Theta_X = get_acc_angle();
    /* Get the imu X gyro reading */
    float GyroX = get_gyroX();
    /* Read the potentiometer voltage */
    pot_value = pendulum_read_voltage();
    /* Print the time, accelerometer angle, gyro angular velocity and pot voltage values to the Serial terminal in the format %f,%f,%f,%f\n */
    elapsed_time_ms = (current_time - start_time) * osKernelGetTickFreq() / 1000;
    printf("%f, %f, %f, %f\n", elapsed_time_ms / 1000.0f, Theta_X, GyroX, pot_value);
    /* Increment log count */
    logCount = logCount + 1;
    /* Stop logging once 5 seconds is reached */
    if (logCount >= 1000)
    {
        logging_stop();
    }
}