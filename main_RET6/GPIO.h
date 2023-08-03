#ifndef __GPIO_H
#define __GPIO_H
#include <stdint.h>

void pwm_init(void);
void pwm_set_duty_cycle(uint8_t CHx, uint16_t Compare);
void motor_dir_gpio_init(void);
void motor_dir_select(uint8_t motor_select, uint8_t diration);
void init_gray_scale_module_gpio(void);
void read_gray_scale_module(uint8_t* rightModuleValues, uint8_t* leftModuleValues, uint8_t* frontModuleValues, uint8_t* backModuleValues);


#endif
