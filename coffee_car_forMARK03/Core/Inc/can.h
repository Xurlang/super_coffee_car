/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
extern int target_linear_velocity,target_angular_velocity;

extern CAN_TxHeaderTypeDef Can_Tx;//can发�?�结构体
extern CAN_RxHeaderTypeDef CAN_Rx;//can接收结构�???
extern uint8_t CAN_Tx_Data[];//can发�?�数据数�???
extern uint8_t CAN_Rx_Data[];//can接收数据数组
extern uint32_t pTxMailbox;//can邮箱变量
extern uint8_t error;//错误进程地址
extern uint8_t jerk_process;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN1_Filter(void);//CAN1滤波器初始化
HAL_StatusTypeDef CAN1_Tx(uint32_t CAN_ID, uint8_t CAN_Tx_Data[]);//CAN1发�?�函�???
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);//CAN1接受回调函数

union velo
{
	int dble;
	int16_t I16[2];
};

void motor_velo_mode_init(uint32_t ID);
void motor_enable(uint32_t ID);
void motor_disable(uint32_t ID);
void motor_quick_stop(uint32_t ID);
void motor_clear_error(uint32_t ID);
void motor_transmit_velo(uint32_t ID,int velocity_left,int velocity_right);
void controler_control_motor(void);
void coffee_car_test(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

