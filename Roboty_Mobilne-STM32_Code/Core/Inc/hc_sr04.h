#ifndef HC_SR04_H
#define HC_SR04_H
#include <main.h>

typedef uint32_t TIM_Channel;

struct us_sensor_str
{
	TIM_HandleTypeDef *htim_echo;
	TIM_HandleTypeDef *htim_trig;
	TIM_Channel trig_channel;

	volatile uint32_t distance_cm;
};

void hc_sr04_init(struct us_sensor_str *us_sensor, TIM_HandleTypeDef *htim_echo, TIM_HandleTypeDef *htim_trig, TIM_Channel channel);
uint32_t hc_sr04_convert_us_to_cm(uint32_t distance_us);

#endif
