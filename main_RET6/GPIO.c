#include "GPIO.h"
#include "stm32f10x.h"                  // Device header
#include "motor.h"


#define MIN_DUTY_CYCLE  6000 // 0.5ms at 50Hz
#define MAX_DUTY_CYCLE  30000 // 2.5ms at 50Hz
#define MIDDLE_DUTY_CYCLE (12000) // 1.5ms at 200Hz
float motor1_angle = MID_Y, motor2_angle = MID_X;
/**
  * @brief  Initializes TIM4 pwm output
  * @retval None
  */
void pwm_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // enable the internal clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // Enable GPIOB Clock
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
	// Init All The PWM IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // TIM4_CH1,2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM4);  
	
	// All Timer is the same 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 40000 - 1;      // ARR pwm frequency = (72MHz / 4) / 20000 = 50Hz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 6 - 1;      // PSC, pre-divide by 4 to achieve 18MHz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // do not use the repetition counter
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);  // init the timer
	
	// All The Servo is the Same Mod
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = MIDDLE_DUTY_CYCLE;		// CCR start from 1.5ms duty cycle
	
	// The 2 Servoes Init (since you are using only PB6 and PB7)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4, ENABLE);
}


/**
  * @brief  change the pwm duty cycle
  * @param CHx: the channel of the pwm
  * @param Compare: the duty cycle
  * @retval None
  */
void pwm_set_duty_cycle(uint8_t CHx, uint16_t Compare)  // change the duty cycle 1 to 4 CHx
{
  // if (Compare > MAX_OUTPUT) Compare = MAX_OUTPUT;  // limit the duty cycle
  // else if (Compare <= MIN_OUTPUT) Compare = MIN_OUTPUT;
    switch (CHx){
        case 1:TIM_SetCompare1(TIM4, Compare);break;
        case 2:TIM_SetCompare2(TIM4, Compare);break;
        case 3:TIM_SetCompare3(TIM4, Compare);break;
        case 4:TIM_SetCompare4(TIM4, Compare);break;
        default:break;
    }
}

void set_angle(uint8_t servo, float angle)
{
  // Limit the angle to the allowable range
  if (angle > MAX_ANGLE) angle = MAX_ANGLE;
  else if (angle < MIN_ANGLE) angle = MIN_ANGLE;
  
  // Calculate the corresponding duty cycle
  uint16_t duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * angle / MAX_ANGLE;
  
  // Choose the appropriate servo channel (PB6 or PB7)
  uint8_t channel = (servo == 0) ? 1 : 2;
  
  // Set the PWM duty cycle
  pwm_set_duty_cycle(channel, duty_cycle);
}

  // 设置伺服电机的速度和方向
  void set_speed(int16_t motor1_speed, int16_t motor2_speed)
  {
    // 计算新的角度
    if ((motor1_speed != 0) || (motor2_speed != 0))
    {
        motor1_angle += motor1_speed * ANGLE_INCREMENT; // ANGLE_INCREMENT是每步的角度变化
        motor2_angle += motor2_speed * ANGLE_INCREMENT;
    }

    // 限制角度范围
    if (motor1_angle > MAX_ANGLE_Y) motor1_angle = MAX_ANGLE_Y;
    if (motor1_angle < MIN_ANGLE_Y) motor1_angle = MIN_ANGLE_Y;
    if (motor2_angle > MAX_ANGLE_X) motor2_angle = MAX_ANGLE_X;
    if (motor2_angle < MIN_ANGLE_X) motor2_angle = MIN_ANGLE_X;

    // 设置电机角度
    set_angle(0, motor1_angle);
    set_angle(1, motor2_angle);
  }


/**
  * @brief  Initializes motor direction gpio
  * @retval None
  */
void motor_dir_gpio_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//Change the mapping of specified pins GPIO_Remap_SWJ_Disable completely disables SWJ (JTAG+SW-DP)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// Change the mapping of the specified pin GPIO_Remap_SWJ_JTAGDisable, disable JTAG-DP + enable SW-DP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	//init all the dir gpio
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4); 	
    //make all the IO output low level
    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}

/**
  * @brief  change the motor direction
  * @param motor_select: the motor to change direction, here 1,2,3,4 mean the FL FB BL BR 
  *                     (forward_left, forward_back, back_left, back_right)
  * @param diration: the direction to change, here 1 means forward, 0 means back
  * @retval None
  */
void motor_dir_select(uint8_t motor_select, uint8_t diration)
{
    if (motor_select == 1) 
    {
        if (diration == 1) //forward
        {
			GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5); 	
        }
        else //backward
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5); 	
        }
    }
    else if (motor_select == 2)
    {
        if (diration == 1)  //forward
        {
			GPIO_ResetBits(GPIOC, GPIO_Pin_12); 	
			GPIO_ResetBits(GPIOB, GPIO_Pin_3); 	
        }
        else //backward
        {
			GPIO_SetBits(GPIOC, GPIO_Pin_12); 	
			GPIO_SetBits(GPIOB, GPIO_Pin_3); 
        }
    }
    else if (motor_select == 3)
    {
        if (diration == 1)  //forward
        {
			GPIO_SetBits(GPIOA, GPIO_Pin_10 |GPIO_Pin_11); 	 	
        }
        else //backward
        {
			GPIO_ResetBits(GPIOA, GPIO_Pin_10 |GPIO_Pin_11); 
        }
    }
    else if (motor_select == 4)
    {
        if (diration == 1)  //forward
        {
			GPIO_SetBits(GPIOA, GPIO_Pin_8 |GPIO_Pin_9); 	 	
        }
        else //backward
        {
			GPIO_ResetBits(GPIOA, GPIO_Pin_8 |GPIO_Pin_9); 
        }
    }
    else
    {
		//it mean no the right select for motor do nothing
    }
}

/**
  * @brief  init the gray scale module gpio
  * @param None
  * @retval None
  */
void init_gray_scale_module_gpio(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    // Configure PC7, PC6, PC0, PC1, PC2, PC3, PC15, PC14, PC13, PC4, PC5 as input pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure PB15, PB14, PB13, PB0, PB1 as input pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure PA1, PA5, PA6, PA7 as input pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  read the gray scale module value
  * @param rightModuleValues: the value of the right module
  * @param leftModuleValues: the value of the left module
  * @param frontModuleValues: the value of the front module
  * @param backModuleValues: the value of the back module
  * @retval None
  */
void read_gray_scale_module(uint8_t* rightModuleValues, uint8_t* leftModuleValues, uint8_t* frontModuleValues, uint8_t* backModuleValues) 
{
    // Read the value of the right module's GPIO pins
    rightModuleValues[0] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
    rightModuleValues[1] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
    rightModuleValues[2] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
    rightModuleValues[3] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    rightModuleValues[4] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);

    // Read the value of the left module's GPIO pins
    leftModuleValues[0] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
    leftModuleValues[1] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
    leftModuleValues[2] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
    leftModuleValues[3] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
    leftModuleValues[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);

    // Read the value of the front module's GPIO pins
    frontModuleValues[0] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
    frontModuleValues[1] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
    frontModuleValues[2] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    frontModuleValues[3] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
    frontModuleValues[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);

    // Read the value of the back module's GPIO pins
    backModuleValues[0] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
    backModuleValues[1] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);
    backModuleValues[2] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
    backModuleValues[3] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
    backModuleValues[4] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
}
