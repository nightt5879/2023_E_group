#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
void motor_init(void);
void control_motor(uint8_t motor_select,int16_t duty_cycle); 
void control_motor_speed(float speeds[], uint8_t control_flags[]);
void init_pid(void);
void TIM1_ETR_Config(void);
void TIM8_ETR_Config(void);
void TIM3_ETR_Config(void);
void TIM2_ETR_Config(void);
void dir_gpio_input_Config(void);
uint8_t Read_FL_Direction(void);
uint8_t Read_FR_Direction(void);
uint8_t Read_BL_Direction(void);
uint8_t Read_BR_Direction(void);
void TIM6_Configuration(void);
void TIM6_IRQHandler(void);
void toggle_delta_v(int enable);
void control_move(float target_disatance_1, float target_distance_2);
void close_to_target(void);
void stop_the_car(void);
float complementary_filter(float input1, float input2, float alpha);


extern float fl_target_speed, fr_target_speed, bl_target_speed, br_target_speed; // send the speed to the uart
extern int16_t distance_x_uart, distance_y_uart, correction_speed; // get the disatnce target from the uart
extern uint8_t one_move_flag;
extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];
#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0

#define RADIUS 3.15f  //the radius of the wheel cm
#define PI 3.14f  //the pi
#define COS45 0.707f //cos45 for the speed calculate
#define MAX_OUTPUT 200 //max pwm
#define MIN_OUTPUT 0  // min pwm
#define POSITION_THRESHOLD_X 1.0f 
#define POSITION_THRESHOLD_Y 3.0f
#define SPEED_THRESHOLD 0.3f   
#define X_FACTOR 11.6f
#define Y_FACTOR 20.5f
#define Z_FACTOR 64.17f
#define ALPHA_X 1.0f
#define ALPHA_Y 1.0f
#define ALPHA_Z 1.0f
#define CORNER_X 3.0f
#define CORNER_Y 4.0f
#define CORR_TEST 1.5f
#define DISTANCE_THRESHOLD_X 0.0f
#define DISTANCE_THRESHOLD_Y 0.0f
// when close to the corner you need a small speed, so we should use a biger KI
#define X_KI 0.00000f
#define Y_KI 0.00000f
#define X_KI_CORNER 0.000000f
#define Y_KI_CORNER 0.00000f
#define CORNER_X_TH 40.0f
#define CORNER_Y_TH 5.0f
/**
  * @brief  initialize the pid control structure
  * @param  None
  * @retval None
  */
typedef struct 
{
    float setpoint;         // Desired value
    float kp, ki, kd;       // PID coefficients
    float prev0_error;       // last time error
    float prev1_error;       // last last time error
    float output;           // the add output
} PID_Controller;
extern PID_Controller pid_fl, pid_fr, pid_bl, pid_br;

	
#endif
