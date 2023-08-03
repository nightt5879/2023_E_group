#include "stm32f10x.h"                  // Device header
#include "6050control.h"
#include <math.h>
#include <stdlib.h>
#include "MPU6050.h"
#include "motor.h"
#include "GPIO.h"

#define MAX_ACC 0.3
#define ANGLE_MIN 0.1
#define MAX_OUTPUT_ANGLE 5


//pid
float kp_angle = 0.20, ki_angle = 0.00001, kd_angle = 2.1;
//delta V will control the car to correct the angle
float delta_v = 0;

extern uint8_t ID;  // the mpu-6050 ID
int16_t AX, AY, AZ, GX, GY, GZ; //MPU-6050 raw data
int16_t AX_CORR, AY_CORR, AZ_CORR, GX_CORR, GY_CORR, GZ_CORR; // MPU6050 zero point data correction.
int16_t ax_corr_done, ay_corr_done, az_corr_done, gx_corr_done, gy_corr_done, gz_corr_done; // MPU-6050 correction data
float ax, ay, az, gx, gy, gz; // the calculated six-aixs data. The unit of A is g/s and the unit of G is °/s.
float ax_prev = 0, ay_prev = 0, az_prev = 0, gx_prev = 0, gy_prev = 0, gz_prev = 0;// use for the low pass filter
// test the interrupt time of the timer 7
uint16_t interrupt_a = 0;
uint8_t interrupt_b = 0;
//below are the x y and z angle result from the 6050
float angle_z = 0;
float distance_x = 0;
float distance_y = 0;
//6050 test the speed
float speed_x = 0;
float speed_y = 0;
uint8_t ax_zero = 0;
uint8_t ay_zero = 0;

//extern from the motor.c
extern float distance_x_encoder, distance_y_encoder, angle_z_encoder;
float angle_z_filter;  // the angle_z after the filter
//function define
void kalman_init(KalmanState* state, float q, float r, float p, float initial_value); // kalman filter
float kalman_update(KalmanState* state, float measurement); // kalman update


KalmanState state_ax, state_ay, state_gz, state_angle_z; // the 3 kalman filter state
/**
  * @brief  low pass filter
  * @retval None
  */
float low_pass_filter(float input, float prev) {
    return ALPHA * input + (1.0 - ALPHA) * prev;
}

/**
  * @brief  get the rightnow data of the 6050
  * @retval None
  */
void mpu_6050_corretion(void)
{
	uint8_t i = 0;
	//let the 6050 stable
	while(i < 10)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		i++;
	}
	i = 0;
	// get 2 times data and get the average
	while(i < 2)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		AX_CORR += AX;
		AY_CORR += AY;
		AZ_CORR += AZ - GRAVITY; // the g of the gravity
		GX_CORR += GX;
		GY_CORR += GY;
		GZ_CORR += GZ;
		i++;
	}
	AX_CORR /= 2;
	AY_CORR /= 2;
	AZ_CORR /= 2;
	GX_CORR /= 2;
	GY_CORR /= 2;
	GZ_CORR /= 2;
	angle_z = 0;  //reset the angle_z
}

/**
  * @brief  init the 6050
  * @retval None
  */
void init_6050(void)
{
	MPU6050_Init();
	ID = MPU6050_GetID();
	mpu_6050_corretion();  // correct the data of the 6050
	// Initialize the Kalman filter state for each sensor reading

    kalman_init(&state_ax, 0.1, 0.1, 1, 0); // You may need to adjust the noise parameters and initial value
    kalman_init(&state_ay, 0.1, 0.1, 1, 0); // based on your specific sensor characteristics
    kalman_init(&state_gz, 0.1, 0.1, 1, 0); 
	kalman_init(&state_angle_z, 0.1, 0.1, 1, 0);
}


/**
  * @brief  get the data of the 6050 
  * no return value, but the data is stored in the global variables, and use in the timer interrupt
  * @retval None
  */
void get_6050_data(void)
{
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	// Check the absolute values of the corrected data and set small values to 0.
	// ax_corr_done = abs(AX - AX_CORR) < LOW_PASS_FILTR ? 0 : AX - AX_CORR;
	// ay_corr_done = abs(AY - AY_CORR) < LOW_PASS_FILTR ? 0 : AY - AY_CORR;
	// az_corr_done = abs(AZ - AZ_CORR - GRAVITY) < LOW_PASS_FILTR ? 0 : AZ - AZ_CORR - GRAVITY;
	// gx_corr_done = abs(GX - GX_CORR) < LOW_PASS_FILTR ? 0 : GX - GX_CORR;
	// gy_corr_done = abs(GY - GY_CORR) < LOW_PASS_FILTR ? 0 : GY - GY_CORR;
	gz_corr_done = abs(GZ - GZ_CORR) < LOW_PASS_FILTR ? 0 : GZ - GZ_CORR;
	//low pass filter
	// Use the low pass filter function on the corrected data.
	// ax = low_pass_filter(ax_corr_done / 32768.0 * 2.0, ax_prev);
	// ay = low_pass_filter(ay_corr_done / 32768.0 * 2.0, ay_prev);
	// az = low_pass_filter(az_corr_done / 32768.0 * 2.0, az_prev);
	// gx = low_pass_filter(gx_corr_done / 32768.0 * 1000.0, gx_prev);
	// gy = low_pass_filter(gy_corr_done / 32768.0 * 1000.0, gy_prev);
	// gz = low_pass_filter(gz_corr_done / 32768.0 * 1000.0, gz_prev);
	// ax = ax_corr_done / 32768.0 * 2.0;
	// ay = ay_corr_done / 32768.0 * 2.0;
	// az = az_corr_done / 32768.0 * 2.0;
	// gx = gx_corr_done / 32768.0 * 2000.0;
	// gy = gy_corr_done / 32768.0 * 2000.0;
	gz =gz_corr_done / 32768.0 * 2000.0;
	//
	// ax = kalman_update(&state_ax, ax);
	// ay = kalman_update(&state_ay, ay);
	gz = kalman_update(&state_gz, gz);

	// Update the previous values for the next loop.
	// ax_prev = ax;
	// ay_prev = ay;
	// az_prev = az;
	// gx_prev = gx;
	// gy_prev = gy;
	// gz_prev = gz;
}

/**
  * @brief  TIM7 configuration
  * @retval None
  */
void TIM7_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 999;  // 1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // the second priority, the first is the UART
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

/**
  * @brief  initialize the pid of angle
  * @param  None
  * @retval None
  */
typedef struct 
{
    float setpoint;         // Desired value
    float kp, ki, kd;       // PID coefficients
    float error;       // now error
    float prev_error;       // last time error
	float acc_error; 		// accumulate error
    float output;           // the add output
} PID_Position;
PID_Position pid_angle;  //the 4 motors pid controller

/**
  * @brief  pid init
  * @param  PID_Controller *pid: the structure of the pid controller
  * @param  float kp: the kp of the pid controller
  * @param  float ki: the ki of the pid controller
  * @param  float kd: the kd of the pid controller
  * @param  float setpoint: the setpoint of the pid controller
  * @retval None
  */
void pid_angle_init(PID_Position *pid, float kp, float ki, float kd, float setpoint)
{
    pid->setpoint = setpoint;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0;
    pid->acc_error = 0.0;
}

void init_pid_angle(void)
{
	pid_angle_init(&pid_angle, kp_angle, ki_angle, kd_angle, 0.0);
}

/**
  * @brief  pid compute
  * here is the poisition pid controller
  * @param  PID_Position *pid: the structure of the pid controller
  * @param  float measurement: the measurement value
  * @retval None
  */
void pid_compute_angle(PID_Position *pid, float measurement)
{
    // Calculate error
    float error = pid->setpoint - measurement;
	// two small ignore
	if (abs(error) < ANGLE_MIN) error = 0.0;
    
    // Proportional term
    float P = pid->kp * error; //P = Kp * e[n]
    
    // Integral term
	pid->acc_error += error;
	float I = pid->ki * pid->acc_error; //I = Ki * sum(e[n])
	//limit the I
	if (I > MAX_ACC) I = MAX_ACC;
	else if (I < -MAX_ACC) I = -MAX_ACC;


    // Derivative term
    float D = pid->kd * (error - pid->prev_error); //D = Kd * (e[n] - e[n-1])

    // Update previous error
	pid->prev_error = error;

    // Calculate control variable
    float calculated_value = P + I + D; // get the output
	//control the min and max of the output
	if (calculated_value > MAX_OUTPUT_ANGLE) calculated_value = MAX_OUTPUT_ANGLE;
	else if (calculated_value < -MAX_OUTPUT_ANGLE) calculated_value = -MAX_OUTPUT_ANGLE;
    pid->output = calculated_value;

}
/**
  * @brief  TIM7 interrupt handler
  * @retval None
  */
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
		// interrupt_a++;
		// if(interrupt_a == 1000)
		// {
		// 	interrupt_a = 0;
		// 	interrupt_b++;
		// }
		// get_6050_data();
		//get the 1ms data, make ax ay to the move distance of the x y, and the gz to the angle
		angle_z -= gz * 0.001;
		// angle_z_encoder = kalman_update(&state_angle_z, angle_z_encoder);
		// angle_z_filter = complementary_filter(angle_z_encoder, angle_z, ALPHA_Z);
		pid_compute_angle(&pid_angle, angle_z); //get the pid output
		delta_v = pid_angle.output;
		//ax is the acceleration, 
		//so it needs to be multiplied by t squared to obtain the displacement 
		//during this period of time (discrete approximation).
		// speed_x += ax * 9.8 * (0.001) * 100; //
		// speed_y += ay * 9.8 * (0.001) * 100;
		// If multiple consecutive readings of acceleration values are detected as 0, the speed will be adjusted to
		// if (ax == 0) ax_zero ++;
		// if (ay == 0) ay_zero ++;
		// if (ay_zero > 10) 
		// {
		// 	ay_zero = 0;
		// 	speed_x = 0;
		// }
		// if (ax_zero > 10) 
		// {
		// 	ax_zero = 0;
		// 	speed_y = 0;
		// }
		//values that are too small will not be accumulated.
		// if (abs(speed_x) > 1)    distance_x += speed_x * 0.001; //cm
		// if (abs(speed_y) > 1)    distance_y += speed_y * 0.001; //cm
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}



/**
  * @brief  initialize the kalman filter
  * @param  None
  * @retval None
  */
void kalman_init(KalmanState* state, float q, float r, float p, float initial_value) {
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
}

/**
  * @brief  kalman update
  * @param  KalmanState* state: the structure of the kalman filter
  * @param  float measurement: the measurement value
  * @retval None
  */
float kalman_update(KalmanState* state, float measurement) {
    // prediction update
    state->p = state->p + state->q;

    // measurement update
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;

    return state->x;
}
