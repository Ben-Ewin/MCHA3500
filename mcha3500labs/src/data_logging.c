#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "data_logging.h"
#include "pendulum.h"

/* Variable declarations */
uint16_t logCount;

static float pot_value = 0.0;

osTimerId_t dataLogging_id;

uint32_t start_time;
uint32_t current_time;
uint32_t elapsed_time_ms;

/* Function declarations */
static void log_pendulum(void *argument);

/* Function defintions */
static void log_pendulum(void *argument)
{
    /*  Supress compiler warnings for unused arguments */
    UNUSED(argument);

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
        pend_logging_stop();
    }
}

void logging_init(void)
{
    /* TODO: Initialise timer for use with pendulum data logging */
    dataLogging_id = osTimerNew(log_pendulum, osTimerPeriodic, (void *)5, NULL);
}

void pend_logging_start(void)
{
    /*  Reset the log counter */
    logCount = 0;
    /*  Start data logging timer at 200Hz */
    osStatus_t status = osTimerStart(dataLogging_id, 5);
    start_time = osKernelGetTickCount();

    log_pendulum(NULL); // where tf is this function supposed to go to run continuously???????????????????
}

void pend_logging_stop(void)
{
    /*  Stop data logging timer */
    osStatus_t status = osTimerStop(dataLogging_id);
}