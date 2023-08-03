#include "stm32f10x.h"
#include "delay.h"
#include "motor.h"
#include "UART.h"
#include "6050control.h"
#include "MPU6050.h"
#include <stdlib.h>
#include "GPIO.h"
//below are the test define
#define TEST_PWM_DUTY 100
#define TARGET_DISTANCE 40
#define TEST_SPEED 3
#define STBLE_TIME 30
#define MOVE_X 40
#define MOVE_Y 45.0f
//below are the function define
void stop_car(void);
void init(void);
void send_to_win(void);
void move_control_main(float target_x, float target_y);

float speeds[] = {TEST_SPEED, TEST_SPEED, TEST_SPEED, TEST_SPEED}; // use for control the 4 motor
uint8_t control_flags[] = {1,1,1,1};  // 1 mean control the motor, 0 mean not control the motor
extern float fl_speed, fr_speed, bl_speed,br_speed;  // the four speed of the motor test from encoder
extern float distance_x_encoder, distance_y_encoder, angle_z_encoder; //the distance move of the car (encoder)
extern float angle_z, distance_x, distance_y, speed_x, speed_y, delta_v; //the 6050 data from the car (angle and speed)
//move and the control of the car
extern float distance_x_filter,distance_y_filter,move_target_distance_x,move_target_distance_y;
extern float fr_target_speed, fl_target_speed, br_target_speed, bl_target_speed;
// the flag and the modle input of the gray
extern uint8_t distance_flag, corner_flag, right_modle[], left_modle[], front_modle[], back_modle[];
extern int16_t distance_x_uart, distance_y_uart; // get the disatnce target from the uart
extern uint8_t one_move_flag, pid_distance_flag;  // when the uart send a message, this flag will turning to 1, you can find it in the UART.X
float data[CH_COUNT]; // the data send to the computer
uint8_t send_flag = 0; // when into the motor interrupt, this flag will turning to 1, send the data to the computer
uint16_t test_flag = 0, break_flag = 0, test_time_flag = 0;   // using for test
uint8_t test_id;  //sometime need to use send mpu-6050 id, cheaking if the mpu-6050 is working
int main(void)
{
	init();
	toggle_delta_v(1);
	// distance_y_uart = MOVE_Y;
	// distance_x_uart = -MOVE_X;
	fl_target_speed = -TEST_SPEED;
	fr_target_speed = TEST_SPEED;
	bl_target_speed = TEST_SPEED;
	br_target_speed = -TEST_SPEED;
	distance_x_uart = -MOVE_X;
	one_move_flag = 1;
	corner_flag = 1;
	// pid_distance_flag = 1;
	// corner_flag = 0;
	// stop_the_car();
	// distance_x_uart = -CORNER_X;
	while (1)
	{	
		get_6050_data(); //I2C communication is too low, we get the data in main and use data in interrupt
		// test_id = MPU6050_GetID();
		send_to_win();
		if(distance_flag == 1)
		{
			distance_flag = 0;
			Serial_SendByte(0x01);
		}
		// if (test_time_flag >= 100)
		// {
		// 	stop_car();
		// }
		// Delay_ms(50);
	}
}

void stop_car(void)
{
	fl_target_speed = 0;
	fr_target_speed = 0;
	bl_target_speed = 0;
	br_target_speed = 0;
}

void init(void)
{
	SystemInit();
	Delay_ms(100);
	//6050
	init_6050();
	//control of the motor, include 4PWM and 4DIR
	motor_init();
	//init the gray input
	init_gray_scale_module_gpio();
	//the four encoders
	TIM1_ETR_Config();
	TIM8_ETR_Config();
	TIM3_ETR_Config();
	TIM2_ETR_Config();
	dir_gpio_input_Config();
	//init the pid and the interrupt (10ms)
	init_pid();
	TIM6_Configuration();
	//UART init
	UART4_Init();
	//init the angle pid and interrupt init (1ms)
	init_pid_angle();
	TIM7_Configuration();
	//serial to the raspberry
	Serial_Init();
	//get the system clock
}

void send_to_win(void)
{
	if (send_flag == 1)
	{
		send_flag = 0;
		data[0] = fl_speed;
		data[1] = fr_speed;
		data[2] = bl_speed;
		data[3] = br_speed;
		data[4] = distance_x_encoder;
		data[5] = distance_y_encoder;
		data[6] = angle_z_encoder;
		data[7] = distance_x;
		data[8] = distance_y;
		data[9] = angle_z;
		data[10] = speed_x;
		data[11] = speed_y;
		data[12] = delta_v;
		data[13] = pid_fl.setpoint;
		data[14] = pid_fr.setpoint;
		data[15] = pid_bl.setpoint;
		data[16] = pid_br.setpoint;
		data[17] = distance_x_filter;
		data[18] = distance_y_filter;
		data[19] = move_target_distance_x;
		data[20] = move_target_distance_y;
		data[21] = test_id;
		send_data(data, CH_COUNT);
	}
}
