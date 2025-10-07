#include "dummy_task.h"
#include "pendulum.h"
#include "motor.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "IMU.h"

static void dummy_task_update(void *arg);

static osThreadId_t _dummyTaskThreadID;
static osThreadAttr_t _dummyTaskThreadAttr = 
{
    .name = "heartbeat",
    .priority = osPriorityIdle,
    .stack_size = 128
};

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

static float pot_value = 0.0;
static int32_t enc_value = 0;

void dummy_task_init(void)
{
    if (!_is_init)
    {
        // CMSIS-RTOS API v2 Timer Documentation: https://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__TimerMgmt.html
        _dummyTaskThreadID = osThreadNew(dummy_task_update, NULL, &_dummyTaskThreadAttr);   // Create the thread in the OS scheduler. 
        // Note: The thread starts automatically when osThreadNew is called
        _is_running = 1;
        _is_init = 1;
    }
}

void dummy_task_start(void)
{
    if (!_is_running)
    {
        osThreadResume(_dummyTaskThreadID);
        _is_running = 1;
    }
}

void dummy_task_stop(void)
{
    if (_is_running)
    {
        osThreadSuspend(_dummyTaskThreadID);
        _is_running = 0;
    }
}

uint8_t dummy_task_is_running(void)
{
    return _is_running;
}

void dummy_task_update(void *arg)
{
    UNUSED(arg);
    while(1)
    {
        // pot_value = pendulum_read_voltage();
        // // printf("Potentiometer value: %f\n", pot_value);

        // enc_value = motor_encoder_getValue();
        // // printf("Encoder value: %ld\n\n", enc_value);
        // // Non-blocking delay to wait


        IMU_read();
        float GyroX = get_gyroX();
        float GyroY = get_gyroY();
        float GyroZ = get_gyroZ();
        float AccX = get_accX();
        float AccY = get_accY();
        float AccZ = get_accZ();
        double Theta_X = get_acc_angleX();
        double Theta_Y = get_acc_angleY();
        double Theta_Z = get_acc_angleZ();
        // printf("GyroX: %lf\n", GyroX);
        // printf("GyroY: %lf\n", GyroY);
        // printf("GyroZ: %lf\n", GyroZ);
        // printf("AccX: %lf\n", AccX);
        // printf("AccY: %lf\n", AccY);
        // printf("AccZ: %lf\n", AccZ);
        printf("Theta X: %lf\n", Theta_X);
        printf("Theta Y: %lf\n", Theta_Y);
        printf("Theta Z: %lf\n\n", Theta_Z);

        
        osDelay(1000);
    }
}

void dummy_task_deinit(void)
{
    _is_init = 0;
    _is_running = 0;
}
