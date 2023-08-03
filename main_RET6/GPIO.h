#ifndef __GPIO_H
#define __GPIO_H
#include <stdint.h>

void pwm_init(void);
void pwm_set_duty_cycle(uint8_t CHx, uint16_t Compare);
void motor_dir_gpio_init(void);
void motor_dir_select(uint8_t motor_select, uint8_t diration);
void init_gray_scale_module_gpio(void);
void read_gray_scale_module(uint8_t* rightModuleValues, uint8_t* leftModuleValues, uint8_t* frontModuleValues, uint8_t* backModuleValues);
void set_angle(uint8_t servo, float angle);
void set_speed(int16_t motor1_speed, int16_t motor2_speed);



#define MID_X 90
#define MID_Y 90  // 75
#define ANGLE_INCREMENT 0.00001f
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define MAX_ANGLE_X 150
#define MIN_ANGLE_X 30
#define MIN_ANGLE_Y 50  // 40
#define MAX_ANGLE_Y 150 // 110


#endif
