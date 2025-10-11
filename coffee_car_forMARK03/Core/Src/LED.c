/*
 * LED.c
 *
 *  Created on: Aug 15, 2022
 *      Author: UDI
 */

#include "stdio.h"
#include "gpio.h"
void LED(){
//	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); //red
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); //blue

}

