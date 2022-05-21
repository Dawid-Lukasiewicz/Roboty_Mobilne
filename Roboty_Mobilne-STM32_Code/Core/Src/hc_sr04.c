#include <hc_sr04.h>

#define HC_SR04_US_TO_CM_CONVERTER	58

void HC_SR04_Init(HC_SR04_Typedef* hc_sr04, TIM_HandleTypeDef *htim_echo, TIM_HandleTypeDef *htim_trig, TIM_Channel trig_channel)
{
	hc_sr04->htim_echo = htim_echo;
	hc_sr04->htim_trig = htim_trig;
	hc_sr04->trig_channel = trig_channel;

	HAL_TIM_IC_Start_IT(hc_sr04->htim_echo, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(hc_sr04->htim_trig, hc_sr04->trig_channel);
}

uint32_t HC_SR04_Convert_us_to_cm(uint32_t distance_us)
{
	return (distance_us / HC_SR04_US_TO_CM_CONVERTER);
}
