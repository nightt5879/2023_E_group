#include "motor.h"
#include "GPIO.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <math.h>
#include <stdlib.h>
#include "UART.h"
#include "6050control.h"

extern uint8_t send_flag; // send to the computer flag
extern uint16_t test_time_flag; // the test time flag, give the main accuarte time
extern float angle_z, distance_x, distance_y, angle_z_filter; //the 6050 data distance and angele_z
extern float delta_v; //angle pid return delta_v

// float kp = 0.5, ki = 0.6, kd = 0.5;  // 
float kp = 10, ki = 3, kd = 10;  // the motor speed pid
float move_kp_y = 0.10, move_ki_y = Y_KI, move_kd_y = 0;  // distance y pid
float move_kp_x = 0.2, move_ki_x = 0.001, move_kd_x = 1;  // distance x pid

uint16_t fl_counter = 0,fr_counter = 0, bl_counter = 0, br_counter = 0; //counter of the encoder
float fl_speed = 0, fr_speed = 0, bl_speed = 0, br_speed = 0;;  // all speeds are the vertical component velocity of the wheel, measured in r/s.
float fl_num = 0, fr_num = 0, bl_num = 0, br_num = 0; //Number of rotations made by the wheel in 10ms
int8_t fl_speed_dir = 0, fr_speed_dir = 0, bl_speed_dir = 0, br_speed_dir = 0; //the speed postive and negetive 
float fl_target_speed = 0, fr_target_speed = 0, bl_target_speed = 0, br_target_speed = 0; // the target speed you want to achieve
uint8_t fl_direction = 0, fr_direction = 0, bl_direction = 0, br_direction = 0; // the motor direction
uint8_t fl_dir = 0, fr_dir = 0, bl_dir = 0, br_dir = 0; // the motro direction input of the gpio
uint16_t test_a = 0, test_b = 0; // use for test
float distance_x_encoder = 0, distance_y_encoder = 0, angle_z_encoder = 0; // encoder cauculate the x y distance and angele_z
float distance_x_filter, distance_y_filter; //the filtter data
int delta_v_enable = 0; // Define the global variable. Initially set it to 0 (delta_v disabled)
//move pid value
float move_target_distance_x = 0;
float move_target_distance_y = 0;
uint8_t distance_flag = 0; //distance flag
uint8_t left_modle[5], right_modle[5], front_modle[5], back_modle[5]; //gray input
uint8_t corner_flag = 0;  //the flag of the conner
uint8_t pid_distance_flag = 0,slow_flag = 1;

PID_Controller pid_fl, pid_fr, pid_bl, pid_br;  //the 4 motors pid controller
PID_Controller pid_move_x, pid_move_y;  //the distance pid controller

// function define
float complementary_filter(float input1, float input2, float alpha);
void close_to_target(void);
void cheak_corner(void);
void stop_the_car(void);
void stop_the_car_no_clear_speed(void);

/**
  * @brief  initialize the motor include the pwm and the direction gpio
  * @param  None
  * @retval None
  */
void motor_init(void)
{
	pwm_init();
	motor_dir_gpio_init();
}

/**
  * @brief  control the motor
  * @param  motor_select: the motor you want to control the define can find in motor.h
  * @param  direction: the direction of the motor  the define can find in motor.h
  * @param  duty_cycle: the duty cycle of the pwm. you can chose negetive or positive
  * @retval None
  */
void control_motor(uint8_t motor_select,int16_t duty_cycle) 
{
    if (duty_cycle < 0)  //it mean backward
    {
        pwm_set_duty_cycle(motor_select, -duty_cycle);
        motor_dir_select(motor_select, MOTOR_BACKWARD);
    }
    else if (duty_cycle >= 0)
    {
        pwm_set_duty_cycle(motor_select, duty_cycle);
        motor_dir_select(motor_select, MOTOR_FORWARD);
    }
}

/**
  * @brief  give the target speed and dir for the motor
  * @param  speeds: [FL, FR, BL, BR] float speed of motor in cm/s - mean backward
  * @param  control_flags: [FL, FR, BL, BR] 1 for control, 0 for not control
  * @retval None
  */
void control_motor_speed( float speeds[], uint8_t control_flags[])
{
    if (control_flags[0])
    {
        fl_target_speed = speeds[0];
    }
    if (control_flags[1])
    {
        fr_target_speed = speeds[1];
    }
    if (control_flags[2])
    {
        bl_target_speed = speeds[2];
    }
    if (control_flags[3])
    {
        br_target_speed = speeds[3];
    }
}
/**
  * @brief  initialize the ETR for BR motor
  * @param  None
  * @retval None
  */
void TIM1_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * @brief  initialize the ETR for BL motor
  * @param  None
  * @retval None
  */
void TIM8_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM8, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM8, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM8, ENABLE);
}

/**
  * @brief  initialize the ETR for FL motor
  * @param  None
  * @retval None
  */
void TIM3_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  initialize the ETR for FR motor
  * @param  None
  * @retval None
  */
void TIM2_ETR_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  initialize the dir gpio input
  * @param  None
  * @retval None
  */
void dir_gpio_input_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure PC.09 (FL), PC.08 (FR), PA.04 (BL), PB.12 (BR) as input pull-up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// the motor in the right side 0 means forward, 1 means backward, and the motor in the left side is the opposite
/**
  * @brief  read the FL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_FL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
}

/**
  * @brief  read the FR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_FR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
}

/**
  * @brief  read the BL motor direction
  * @param  None
  * @retval 1: forward, 0: backward
  */
uint8_t Read_BL_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
}

/**
  * @brief  read the BR motor direction
  * @param  None
  * @retval 0: forward, 1: backward
  */
uint8_t Read_BR_Direction(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
}

/**
  * @brief  initialize the TIM5 for encoder
  * @param  None
  * @retval None
  */
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 9999;  //
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // the thrid priority, first is UART, sencond is the angle
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}



/**
  * @brief  initialize the pid control
  * @param  None
  * @retval None
  */
void pid_init(PID_Controller *pid, float kp, float ki, float kd, float setpoint)
{
    pid->setpoint = setpoint;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev0_error = 0.0;
    pid->prev1_error = 0.0;
}

/**
  * @brief  initialize the 4 pid control
  * the kp ki kd in the header file
  * @param  None
  * @retval None
  */
void init_pid(void)
{
    pid_init(&pid_fl, kp, ki, kd, fl_target_speed);
    pid_init(&pid_fr, kp, ki, kd, fr_target_speed);
    pid_init(&pid_bl, kp, ki, kd, bl_target_speed);
    pid_init(&pid_br, kp, ki, kd, br_target_speed);
    pid_init(&pid_move_x, move_kp_x, move_ki_x, move_kd_x, 0.0);
    pid_init(&pid_move_y, move_kp_y, move_ki_y, move_kd_y, 0.0);
}

/**
  * @brief  pid compute
  * the increment output += to the .output structure, you can use it to control the motor
  * max and min output in the head of the file
  * @param  PID_Controller *pid: the structure of the pid controller
  * @param  float measurement: the measurement value
  * @retval None
  */
void pid_compute(PID_Controller *pid, float measurement)
{
    // test_b += 1;
    // Calculate error
    float error = pid->setpoint - measurement;
    
    // Proportional term
    float P = pid->kp * (error - pid->prev0_error);  // P = Kp * (e[n] - e[n-1]) incremental PID
    
    // Integral term
    float I = pid->ki * error;  // I = Ki * e[n]

    // Derivative term
    float D = pid->kd * (error - 2 * pid->prev0_error + pid->prev1_error);  // D = Kd * (e[n] - 2e[n-1] + e[n-2])

    // Update previous error
    pid->prev1_error = pid->prev0_error;
    pid->prev0_error = error;

    // Calculate control variable
    float calculated_value = P + I + D;
    //add to the output
    pid->output += calculated_value;
    //control the min and max of the output
    if (pid->output > MAX_OUTPUT)
        pid->output = MAX_OUTPUT;
    else if (pid->output < -MAX_OUTPUT)
        pid->output = -MAX_OUTPUT;
}

void control_move(float target_disatance_1, float target_distance_2)
{
  move_target_distance_x = target_disatance_1;
  move_target_distance_y = target_distance_2;
}
/**
  * @brief  the interrupt handler for TIM5 using for PID control of the motor
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        send_flag = 1;
        fl_dir = Read_FL_Direction();
        fr_dir = Read_FR_Direction();
        bl_dir = Read_BL_Direction();
        br_dir = Read_BR_Direction();
        // PID control code goes here.
        br_counter = TIM_GetCounter(TIM1);
        bl_counter = TIM_GetCounter(TIM8);
        fl_counter = TIM_GetCounter(TIM3);
        fr_counter = TIM_GetCounter(TIM2);
        // reset the counter
        TIM_SetCounter(TIM1, 0);
        TIM_SetCounter(TIM8, 0);
        TIM_SetCounter(TIM3, 0);
        TIM_SetCounter(TIM2, 0);
        //the spped need the postive and negative
        if (fl_dir == 1) fl_speed_dir = 1;
        else fl_speed_dir = -1;
        if (fr_dir == 0) fr_speed_dir = 1;
        else fr_speed_dir = -1;
        if (bl_dir == 1) bl_speed_dir = 1;
        else bl_speed_dir = -1;
        if (br_dir == 0) br_speed_dir = 1;
        else br_speed_dir = -1;
        //The gear ratio is 3:7, and the encoder has 1024 lines. into the ISR is 10ms
        //Therefore, the speed of wheel (vertical) is (counter/7) * 3 / 1024 * 100 * R * COS45  r/s
        //the cycle in 10ms postive mean forward, negative mean backward
        fl_num = fl_speed_dir *((float)fl_counter / 7) * 3 / 1024;
        fr_num = fr_speed_dir *((float)fr_counter / 7) * 3 / 1024;
        bl_num = bl_speed_dir *((float)bl_counter / 7) * 3 / 1024;
        br_num = br_speed_dir * ((float)br_counter / 7) * 3 / 1024;
        // get the speed of the wheel R/s
        fl_speed = fl_num * 100;
        fr_speed = fr_num * 100;
        bl_speed = bl_num * 100;
        br_speed = br_num * 100;
        //get the resived speed
        // Apply PID control
        if(delta_v_enable)   // angle control
        {
            pid_fl.setpoint = fl_target_speed + delta_v;  // Update the setpoint if it has changed
            pid_fr.setpoint = fr_target_speed - delta_v;
            pid_bl.setpoint = bl_target_speed + delta_v;  // Update the setpoint if it has changed
            pid_br.setpoint = br_target_speed - delta_v;
        } else
        {
            pid_fl.setpoint = fl_target_speed;  // Update the setpoint if it has changed
            pid_fr.setpoint = fr_target_speed;
            pid_bl.setpoint = bl_target_speed;  // Update the setpoint if it has changed
            pid_br.setpoint = br_target_speed;
        }
        pid_compute(&pid_fl, fl_speed);
        control_motor(MOTOR_FL,(int)pid_fl.output);
        pid_compute(&pid_fr, fr_speed);
        control_motor(MOTOR_FR,(int)pid_fr.output);
        pid_compute(&pid_bl, bl_speed);
        control_motor(MOTOR_BL, (int)pid_bl.output);
        pid_compute(&pid_br, br_speed);
        control_motor(MOTOR_BR, (int)pid_br.output);
        // get the x y disatnce and the Z angle
        distance_x_encoder += ((fl_num - bl_num + br_num - fr_num) / 2) * X_FACTOR; // the X distance
        distance_y_encoder += ((fl_num + bl_num + fr_num + br_num) / 4) * Y_FACTOR; // the Y distance
        angle_z_encoder += ((fl_num + bl_num - fr_num - br_num) / 4) * Z_FACTOR; // the Z angle
        // complementary filter
        distance_x_filter = complementary_filter(distance_x_encoder, distance_x, ALPHA_X);
        distance_y_filter = complementary_filter(distance_y_encoder, distance_y, ALPHA_Y);
        if (abs(distance_x_filter) < DISTANCE_THRESHOLD_X)
        {
            distance_x_filter = 0;
        }
        if (abs(distance_y_filter) < DISTANCE_THRESHOLD_Y)
        {
            distance_y_filter = 0;
        }
        // below are the distance PID control we dont need it right now
        move_target_distance_x = distance_x_uart;
        move_target_distance_y = distance_y_uart;
        pid_move_x.setpoint = move_target_distance_x;
        pid_move_y.setpoint = move_target_distance_y;
        if (pid_distance_flag)
        {
            pid_compute(&pid_move_x, distance_x_filter);
            pid_compute(&pid_move_y, distance_y_filter);
            // if the distance is less than the threshold, then stop the motor
            // here in the aixs Y the motor all the positive and in the X aixs bl and fr are negative. need to be stacked together
            fl_target_speed = pid_move_y.output + pid_move_x.output;
            fr_target_speed = pid_move_y.output - pid_move_x.output;
            bl_target_speed = pid_move_y.output - pid_move_x.output;
            br_target_speed = pid_move_y.output + pid_move_x.output;
        }
        close_to_target(); // it is close to the target
        if (corner_flag)
        {
            cheak_corner();
        }
        test_time_flag ++;
        
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update); // Clear the interrupt flag
    }
		
}

/**
  * @brief  enanble or disable the delta_v
  * @param  None
  * @retval None
  */
void toggle_delta_v(int enable)
{
    delta_v_enable = enable;
}

/**
  * @brief  Complementary filtering
  * @param  input1, input2, alpha
  * here apha is the weight of the input1
  * @retval the result of the filtering
  */
float complementary_filter(float input1, float input2, float alpha) {
    return alpha * input1 + (1 - alpha) * input2;
}

/**
  * @brief  close to the target
  * it mean when the distance is less than the threshold, and the motor speed is less than the threshold
  * the most important thing is the one_move_flag, if without it, the car will stop when it start
  * @param  None
  * @retval None
  */
//  &&
//           abs(fl_target_speed) < SPEED_THRESHOLD &&
//           abs(fr_target_speed) < SPEED_THRESHOLD &&
//           abs(bl_target_speed) < SPEED_THRESHOLD &&
//           abs(br_target_speed) < SPEED_THRESHOLD
void close_to_target(void)
{
    if (abs(move_target_distance_x - distance_x_filter) < 10 &&
          abs(move_target_distance_y - distance_y_filter) < 10 &&
          slow_flag == 1)
        {
          slow_flag = 0;
          fl_target_speed *= 0.5;
          fr_target_speed *= 0.5;
          bl_target_speed *= 0.5;
          br_target_speed *= 0.5;
        }
    if (abs(move_target_distance_x - distance_x_filter) < POSITION_THRESHOLD_X &&
          abs(move_target_distance_y - distance_y_filter) < POSITION_THRESHOLD_Y )
        {
          move_ki_x = X_KI;
          move_ki_y = Y_KI;
          if (one_move_flag == 1&& (distance_x_uart != 0 || distance_y_uart != 0)) //it mean control the move
          {
              one_move_flag = 0;
              distance_flag = 1;
              distance_x_uart = 0;
              distance_y_uart = 0;
          }
          pid_distance_flag = 0; // stop the pid control
          stop_the_car();
          // Serial_SendByte(0x01);
          // Serial_SendPacket(); // send to the raspberry
        }
}

/**
  * @brief  cheak the corner
  * i don't not whether it is useful, and whether use it in the future -- 2023/7/22
  * @param  None
  * @retval None
  */
void cheak_corner(void)
{
   read_gray_scale_module(right_modle, left_modle,front_modle,back_modle);
  //  if (left_modle[0] || left_modle[4] || right_modle[0] || right_modle[4])  // if find the corner stop the car
  //  {
  //       stop_the_car();
  //  }
   // close_to_target();
  //  if (abs(move_target_distance_x - distance_x_filter) < CORNER_X_TH &&
  //         abs(move_target_distance_y - distance_y_filter) < CORNER_Y_TH)  // enough close to the target
  //   {
  //       if ((left_modle[0] || left_modle[4]) && distance_x_uart < 0) // move to the left
  //       {
  //             // pid_distance_flag = 1;
  //             corner_flag = 0;
  //             // move_ki_x = X_KI_CORNER;
  //             stop_the_car();
  //             // distance_x_uart = -CORNER_X;
  //       }
  //       else if((right_modle[0] || right_modle[4]) && distance_x_uart > 0) // move to the right
  //       {
  //             pid_distance_flag = 1;
  //             corner_flag = 0;
  //             // move_ki_x = X_KI_CORNER;
  //             stop_the_car_no_clear_speed();
  //             distance_x_uart = CORNER_X;
  //       }
  //       else if((left_modle[2] || right_modle[2]) && distance_y_uart > 0 ) // move to the forward
  //       {
  //             pid_distance_flag = 1;
  //             corner_flag = 0;
  //             // move_ki_y = Y_KI_CORNER;
  //             stop_the_car_no_clear_speed();
  //             distance_y_uart = CORNER_Y;
  //       }
  //       else if((left_modle[2] || right_modle[2]) && distance_y_uart < 0) // move to the backward
  //       {
  //             pid_distance_flag = 1;
  //             corner_flag = 0;
  //             // move_ki_y = Y_KI_CORNER;
  //             stop_the_car_no_clear_speed();
  //             distance_y_uart = -CORNER_Y;
  //       }
  //   }
}

/**
  * @brief  stop the car
  * clear all the flag so the car will stop
  * you should konw that the speed pid of the 4 motor, so when it come to ZERO.It not just clear the PWM control of the motor
  * it will lock the motor, better than just clear the PWM(because of inertia)
  * @param  None
  * @retval None
  */
void stop_the_car(void)
{
    fl_target_speed = 0;
    fr_target_speed = 0;
    bl_target_speed = 0;
    br_target_speed = 0;
    distance_x_encoder = 0;
    distance_y_encoder = 0;
    distance_x_uart = 0;
    distance_y_uart = 0;
    // angle_z_encoder = 0;
    distance_x = 0;
    distance_y = 0;
    distance_x_filter = 0;
    distance_y_filter = 0;
    // angle_z_filter = 0;
    move_target_distance_x = 0;
    move_target_distance_y = 0;
}

/**
  * @brief  stop the car but not clear the speed 
  * the reason why we use it is: at the beagining. i want to use the gray input to know the current position of the car
  * when it come to the current position, we can flash the pid control
  * not clear the speed the car will get a more smooth deceleration
  * @param  None
  * @retval None
  */

void stop_the_car_no_clear_speed(void)
{
      distance_x_encoder = 0;
      distance_y_encoder = 0;
      distance_x = 0;
      distance_y = 0;
      distance_x_filter = 0;
      distance_y_filter = 0;
      distance_x_uart = 0;
      distance_y_uart = 0;
}
