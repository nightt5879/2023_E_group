#include "move.h"
#include "motor.h"

#define ALPHA_DISTANCE 1.0f //the disatnce filter factor
/**
  * @brief  the complementary filter
  * @param  None
  * @retval None
  */
float complementary_filtering(float input1, float input2)
{
	return ALPHA_DISTANCE * input1 + (1 - ALPHA_DISTANCE) * input2;
}




