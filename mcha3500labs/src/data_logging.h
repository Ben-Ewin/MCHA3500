#ifndef DATALOGGING_H
#define DATALOGGING_H
/* Add function prototypes here */
void logging_init(void);
void logging_start(void);
void logging_stop(void);
void imu_logging_start(void);
static void log_imu(void);

#endif