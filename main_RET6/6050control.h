#ifndef __6050CONTROL_H
#define __6050CONTROL_H

#define GRAVITY 16384
#define LOW_PASS_FILTR 100
#define ALPHA 0.8  // Filter factor, with values between 0 and 1, where a larger value indicates a weaker filtering effect.
void init_6050(void);
void mpu_6050_corretion(void);
void TIM7_Configuration(void);
void get_6050_data(void);
void init_pid_angle(void);

typedef struct {
    float q; // process noise covariance
    float r; // measurement noise covariance
    float x; // estimated value
    float p; // estimation error covariance
    float k; // adaptive Kalman gain
} KalmanState;
	
#endif
