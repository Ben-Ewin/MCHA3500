#ifndef IMU_H
#define IMU_H
/* Add function prototypes here */
void IMU_init(void);
void IMU_read(void);
float get_gyroX(void);
float get_gyroY(void);
float get_gyroZ(void);
float get_accX(void);
float get_accY(void);
float get_accZ(void);
double get_acc_angleX(void);
double get_acc_angleY(void);
double get_acc_angleZ(void);

#endif