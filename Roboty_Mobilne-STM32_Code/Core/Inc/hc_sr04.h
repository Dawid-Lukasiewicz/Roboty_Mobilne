#ifndef HC_SR04_H
#define HC_SR04_H

#include "stm32l4xx_hal.h"

typedef uint32_t TIM_Channel;

typedef struct _HC_SR04
{
	TIM_HandleTypeDef *htim_echo;
	TIM_HandleTypeDef *htim_trig;
	TIM_Channel trig_channel;

	volatile uint32_t distance_cm;
} HC_SR04_Typedef;

void HC_SR04_Init(HC_SR04_Typedef *hc_sr04, TIM_HandleTypeDef *htim_echo, TIM_HandleTypeDef *htim_trig, TIM_Channel channel);
uint32_t HC_SR04_Convert_us_to_cm(uint32_t distance_us);

#endif
