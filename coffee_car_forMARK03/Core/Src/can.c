/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

double ipc_target_angular_velocity, ipc_target_linear_velocity;
//-------------------------------auxiliary_funtion-----------------------------------------------//
void abs_limit(double *num, double Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

void smooth(double *act, double target)
{
    if (fabs(target - (*act)) < fabs(target * 0.01))
    {
        *act = target;
    }
    else
    {
        *act = (*act) + (target - (*act)) * 0.01;
    }
}
//------------------------------------------------------------------------------------------------//
float target_linear_velocity, target_angular_velocity;
float target_velocity_left, target_velocity_right;
float r = 0.0655;        // 轮子半径，单位m 0.131/2
float WHEEL_BASE = 0.39; // 轮距，单位m
int target_rpm_left, target_rpm_right;
GPIO_PinState temp = GPIO_PIN_SET;
CAN_FilterTypeDef CAN_Filter_Structure;                                    // can滤波器结构体
CAN_TxHeaderTypeDef Can_Tx;                                                // can发�?�结构体
CAN_RxHeaderTypeDef CAN_Rx;                                                // can接收结构�???
uint8_t CAN_Tx_Data[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77}; // can发�?�数据数�???
uint8_t CAN_Rx_Data[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77}; // can接收数据数组
uint32_t pTxMailbox = 1;                                                   // can邮箱变量
bool flag_init = false;
bool flag_enabled = false;
bool flag_quick_stop = false;
uint8_t jerk_process = 0;    // 急停测试进程地址
uint8_t p_limit_process = 0; // 正限位测试进程地�??
uint8_t n_limit_process = 0; // 负限位测试进程地�??
uint8_t origin_process = 0;  // 原点测试进程地址
uint8_t error = 0;           // 错误进程地址
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspInit 0 */

        /* USER CODE END CAN1_MspInit 0 */
        /* CAN1 clock enable */
        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* CAN1 interrupt Init */
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
        /* USER CODE BEGIN CAN1_MspInit 1 */

        /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */

        /* USER CODE END CAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

        /* CAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
        /* USER CODE BEGIN CAN1_MspDeInit 1 */

        /* USER CODE END CAN1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

void CAN1_Filter(void) // CAN滤波器初始化
{
    CAN_Filter_Structure.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter_Structure.FilterBank = 1;
    CAN_Filter_Structure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter_Structure.FilterIdHigh = 0x0000;
    CAN_Filter_Structure.FilterIdLow = 0x0000;
    CAN_Filter_Structure.FilterMaskIdHigh = 0x0000;
    CAN_Filter_Structure.FilterMaskIdLow = 0x0000;
    CAN_Filter_Structure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_Structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //	CAN1_Filter_Structure.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Structure);
}

// CAN1���ͺ���
HAL_StatusTypeDef CAN1_Tx(uint32_t CAN_ID, uint8_t CAN_Tx_Data[])
{
    Can_Tx.StdId = CAN_ID;
    Can_Tx.ExtId = CAN_ID;
    Can_Tx.IDE = 0;
    Can_Tx.RTR = 0;
    Can_Tx.DLC = 8;
    if (hcan1.Instance == CAN1)
    {
        HAL_CAN_AddTxMessage(&hcan1, &Can_Tx, CAN_Tx_Data, &pTxMailbox);
        return HAL_OK;
    }
    return HAL_ERROR;
}
// coffee_car velo_mode_init
void motor_velo_mode_init(uint32_t ID)
{
    int process_num = 0;
    bool flag_initing = false;
    flag_initing = true;
    process_num = 1;
    while (flag_initing)
    {
        switch (process_num)
        {
        case 1: // synchronous mode
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x0F;
            CAN_Tx_Data[2] = 0x20;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 2: // velo mode
        {
            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x03;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 3: // left acc_time :ms
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x01;
            CAN_Tx_Data[4] = 0x64;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 4: // right acc_time :ms
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x02;
            CAN_Tx_Data[4] = 0x64;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 5: // left decelerate_time :ms
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x01;
            CAN_Tx_Data[4] = 0x64;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 6: // right decelerate_time :ms
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x02;
            CAN_Tx_Data[4] = 0x64;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 7:
        {
            // check state later
            flag_initing = false;
            flag_init = true;
            break;
        }
        default:
            flag_initing = false;
            break;
        }
    }
}

void motor_enable(uint32_t ID)
{
    int process_num = 0;
    bool flag_enabling = false;
    flag_enabling = true;
    process_num = 1;
    while (flag_enabling)
    {
        switch (process_num)
        {
        case 1:
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x06;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 2:
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x07;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 3:
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x0F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(5);
            process_num++;
            break;
        }
        case 4:
        {
            // check state later
            flag_enabling = false;
            flag_enabled = true;
            flag_quick_stop = false;
            break;
        }
        default:
            flag_enabling = false;
            break;
        }
    }
}

void motor_disable(uint32_t ID)
{
    CAN_Tx_Data[0] = 0x2B;
    CAN_Tx_Data[1] = 0x40;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x00;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(ID, CAN_Tx_Data);
    flag_quick_stop = true;
    HAL_Delay(5);
}

void motor_quick_stop(uint32_t ID)
{ // enabled
    CAN_Tx_Data[0] = 0x2B;
    CAN_Tx_Data[1] = 0x40;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x02;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(ID, CAN_Tx_Data);
    flag_quick_stop = true;
    HAL_Delay(5);
}

union velo velo_left, velo_right;
void motor_transmit_velo(uint32_t ID, int velocity_left, int velocity_right)
{
    velo_left.dble = velocity_left;
    velo_right.dble = velocity_right;
    CAN_Tx_Data[0] = 0x23;
    CAN_Tx_Data[1] = 0xFF;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x03;
    CAN_Tx_Data[4] = velo_left.I16[0];
    CAN_Tx_Data[5] = velo_left.I16[1];
    CAN_Tx_Data[6] = velo_right.I16[0];
    CAN_Tx_Data[7] = velo_right.I16[1];
    CAN1_Tx(ID, CAN_Tx_Data);
    HAL_Delay(5);
}

void motor_clear_error(uint32_t ID)
{
    CAN_Tx_Data[0] = 0x2B;
    CAN_Tx_Data[1] = 0x40;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x80;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(ID, CAN_Tx_Data);
    HAL_Delay(5);
}
extern double send_linear_vel;  // 线速度值
extern double send_angular_vel; // 角速度值

union velo act_velo_left, act_velo_right;
void read_motor_state(uint32_t ID)
{
    // velo 606C
    CAN_Tx_Data[0] = 0x23;
    CAN_Tx_Data[1] = 0x6C;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x03;
    CAN_Tx_Data[4] = 0x00;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(ID, CAN_Tx_Data);
    HAL_Delay(5);

    // act_velo_left.I16[0] = CAN_Rx_Data[4];
    // act_velo_left.I16[1] = CAN_Rx_Data[5];
    // act_velo_right.I16[0] = CAN_Rx_Data[6];
    // act_velo_right.I16[1] = CAN_Rx_Data[7];
    // 6. 解析32位组合速度数据（小端模式：D4=最高位，D7=最低位）
    int16_t speed_combined = (CAN_Rx_Data[4] << 24) | (CAN_Rx_Data[5] << 16) | (CAN_Rx_Data[6] << 8) | CAN_Rx_Data[7];
    double left_speed_raw = (int16_t)(speed_combined & 0xFFFF);          // 提取低16位（左电机）
    double right_speed_raw = (int16_t)((speed_combined >> 16) & 0xFFFF); // 提取高16位（右电机）

    // 7. 转换为实际转速（0.1r/min → r/min）
    double left_rpm = left_speed_raw / 10.0f;
    double right_rpm = right_speed_raw / 10.0f;

    // 4. 步骤1：电机实际转速 → 左/右车轮实际线速度（m/s）
    double actual_wheel_v_left = (left_rpm * 2 * 3.1415926 * r) / 60.0f;
    double actual_wheel_v_right = (right_rpm * 2 * 3.1415926 * r) / 60.0f;

    // 5. 步骤2：车轮线速度 → 车辆实际线速度、角速度
    send_linear_vel = (actual_wheel_v_left + actual_wheel_v_right) / 2.0f;        // 线速度（平均）
    send_angular_vel = (actual_wheel_v_right - actual_wheel_v_left) / WHEEL_BASE; // 角速度（速度差/轮距）
}

int startup_step = 1;

// 全局滤波器变量
float filtered_linear_velocity = 0, filtered_angular_velocity = 0;

void controler_control_motor()
{
    switch (startup_step)
    {
    case 1:
        motor_enable(0x601);
        startup_step = 2;
        HAL_Delay(5);
        break;
    case 2:
        // 滤波
        //  filtered_linear_velocity = 0.955f * filtered_linear_velocity + 0.045f * target_linear_velocity;
        //  filtered_angular_velocity = 0.955f * filtered_angular_velocity + 0.045f * target_angular_velocity;

        // target_velocity_left = - target_linear_velocity + target_angular_velocity;
        // target_velocity_right =  (target_linear_velocity + target_angular_velocity);

        //
        target_velocity_left = target_linear_velocity - (target_angular_velocity * WHEEL_BASE) / 2.0f;
        target_velocity_right = target_linear_velocity + (target_angular_velocity * WHEEL_BASE) / 2.0f;
        //    target_velocity_left = target_angular_velocity - (target_linear_velocity * WHEEL_BASE) / 2.0f;
        //    target_velocity_right =target_angular_velocity  + (target_linear_velocity * WHEEL_BASE) / 2.0f;

        // 实际速度转为电机转速
        target_rpm_left = -(int16_t)(target_velocity_left * 60 / (2 * 3.1416 * r));
        target_rpm_right = (int16_t)(target_velocity_right * 60 / (2 * 3.1416 * r));
        // if(abs(target_velocity_left )< 5)
        // 	{target_velocity_left = 0;}

        // if(abs(target_velocity_right) < 5)
        // 	{target_velocity_right = 0;}

        // if(target_velocity_left > 1000) //防止超过最大转速
        // 	{target_velocity_left = 1000;}
        // if(target_velocity_left < -1000)
        // 	{target_velocity_left = -1000;}
        // if(target_velocity_right > 1000)
        // 	{target_velocity_right = 1000;}
        // if(target_velocity_right < -1000)
        // 	{target_velocity_right = -1000;}
        motor_transmit_velo(0x601, target_rpm_left, target_rpm_right);
        //		motor_transmit_velo(0x601,-200,200);
        break;
    case 5:
        motor_disable(0x601);
        break;
    case 6:
        motor_clear_error(0x601);
        break;
    default:
        break;
    }
}

void coffee_car_test()
{
    if ((!flag_init))
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        motor_velo_mode_init(0x601); // 配置电机进入速度模式（Velocity mode）
    }
    else if (flag_init)
    {
        if (!flag_enabled)
        {
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            motor_enable(0x601); // 电机使能控制函数//紧接在速度模式初始化之后的，必须要
        }
        else
        {
            // motor_transmit_velo(0x601,0.0,0.0);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            controler_control_motor();
        }
    }
}
// 配置 TPDO0：映射 0x606C 左/右电机速度（通过 SDO 命令）
HAL_StatusTypeDef Config_TPDO0(void)
{
    CAN_TxHeaderTypeDef tx_hdr;
    uint32_t tx_mailbox;
    uint8_t sdo_data[8];

    // SDO 发送配置：COB-ID=0x601（节点1），固定8字节
    tx_hdr.StdId = 0x601;
    tx_hdr.RTR = CAN_RTR_DATA;
    tx_hdr.DLC = 8;
    tx_hdr.TransmitGlobalTime = DISABLE;

    // 1. 清空 TPDO0 映射（索引0x1A00，子索引0设0）
    sdo_data[0] = 0x2F;
    sdo_data[1] = 0x00;
    sdo_data[2] = 0x1A;
    sdo_data[3] = 0x00;
    sdo_data[4] = 0x00;
    sdo_data[5] = 0x00;
    sdo_data[6] = 0x00;
    sdo_data[7] = 0x00;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    // 2. 映射左电机速度（0x606C 01 到 TPDO0 映射1）
    sdo_data[0] = 0x23;
    sdo_data[1] = 0x00;
    sdo_data[2] = 0x1A;
    sdo_data[3] = 0x01;
    sdo_data[4] = 0x20;
    sdo_data[5] = 0x01;
    sdo_data[6] = 0x6C;
    sdo_data[7] = 0x60;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    // 3. 映射右电机速度（0x606C 02 到 TPDO0 映射2）
    sdo_data[0] = 0x23;
    sdo_data[1] = 0x00;
    sdo_data[2] = 0x1A;
    sdo_data[3] = 0x02;
    sdo_data[4] = 0x20;
    sdo_data[5] = 0x02;
    sdo_data[6] = 0x6C;
    sdo_data[7] = 0x60;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    // 4. 启用2个映射（索引0x1A00，子索引0设2）
    sdo_data[0] = 0x2F;
    sdo_data[1] = 0x00;
    sdo_data[2] = 0x1A;
    sdo_data[3] = 0x00;
    sdo_data[4] = 0x02;
    sdo_data[5] = 0x00;
    sdo_data[6] = 0x00;
    sdo_data[7] = 0x00;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    // 5. 发送NMT启动命令（激活PDO，COB-ID=0x000）
    CAN_TxHeaderTypeDef nmt_hdr;
    uint8_t nmt_data[2] = {0x01, 0x01}; // 命令0x01=启动，节点1
    nmt_hdr.StdId = 0x000;
    nmt_hdr.RTR = CAN_RTR_DATA;
    nmt_hdr.DLC = 2;
    nmt_hdr.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &nmt_hdr, nmt_data, &tx_mailbox) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}
// 配置 TPDO0：映射 0x606C（实际速度）+ 设传输类型（Node-ID=1）
HAL_StatusTypeDef TPDO0_Setup(void)
{
    CAN_TxHeaderTypeDef sdo_tx_hdr;
    uint32_t tx_mailbox;
    uint8_t sdo_data[8]; // SDO 命令固定8字节

    // SDO 客户端 COB-ID=0x601（对应 Node-ID=1）
    sdo_tx_hdr.StdId = 0x601;
    sdo_tx_hdr.RTR = CAN_RTR_DATA;
    sdo_tx_hdr.DLC = 8;
    sdo_tx_hdr.TransmitGlobalTime = DISABLE;

   // 1. 清空 TPDO0 原有映射（索引0x1A00，子索引0设0）
       sdo_data[0] = 0x2F;
       sdo_data[1] = 0x00;
       sdo_data[2] = 0x1A;
       sdo_data[3] = 0x00;
       sdo_data[4] = 0x00;
       sdo_data[5] = 0x00;
       sdo_data[6] = 0x00;
       sdo_data[7] = 0x00;
       if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
       {
           return HAL_ERROR;
       }
       HAL_Delay(100); // 等待驱动器应答
    // 2. 映射电机速度（0x606C 03 → TPDO0 映射1）
        sdo_data[0] = 0x23;
        sdo_data[1] = 0x00;
        sdo_data[2] = 0x1A;
        sdo_data[3] = 0x01;
        sdo_data[4] = 0x20;
        sdo_data[5] = 0x03;
        sdo_data[6] = 0x6C;
        sdo_data[7] = 0x60;
        if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        {
            return HAL_ERROR;
        }
        HAL_Delay(100);

    // 触发方式 定时器
        sdo_data[0] = 0x2F;
        sdo_data[1] = 0x00;
        sdo_data[2] = 0x18;
        sdo_data[3] = 0x02;
        sdo_data[4] = 0xFF;
        sdo_data[5] = 0x00;
        sdo_data[6] = 0x00;
        sdo_data[7] = 0x00;
        if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        {
            return HAL_ERROR;
        }
        HAL_Delay(100);

    // 触发方式 定时器 时间10ms
        sdo_data[0] = 0x2B; // 文档是2B  AI给的是2F  实际调试注意这里看看哪个对
        sdo_data[1] = 0x00;
        sdo_data[2] = 0x18;
        sdo_data[3] = 0x05;
        sdo_data[4] = 0x14; //
        sdo_data[5] = 0x00;
        sdo_data[6] = 0x00;
        sdo_data[7] = 0x00;
        if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        {
            return HAL_ERROR;
        }
        HAL_Delay(100);

    // 4. 启用映射（索引0x1A00，子索引0设1）
        sdo_data[0] = 0x2F;
        sdo_data[1] = 0x00;
        sdo_data[2] = 0x1A;
        sdo_data[3] = 0x00;
        sdo_data[4] = 0x01;
        sdo_data[5] = 0x00;
        sdo_data[6] = 0x00;
        sdo_data[7] = 0x00;
        if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
        {
            return HAL_ERROR;
        }
        HAL_Delay(100);

    //保存eeprom
         sdo_data[0] = 0x2B;
         sdo_data[1] = 0x10;
         sdo_data[2] = 0x20;
         sdo_data[3] = 0x00;
         sdo_data[4] = 0x01;
         sdo_data[5] = 0x00;
         sdo_data[6] = 0x00;
         sdo_data[7] = 0x00;
         if (HAL_CAN_AddTxMessage(&hcan1, &sdo_tx_hdr, sdo_data, &tx_mailbox) != HAL_OK)
         {
             return HAL_ERROR;
         }
         HAL_Delay(100);
    // 6. 发送 NMT 启动命令（激活 PDO，Node-ID=1）
    CAN_TxHeaderTypeDef nmt_hdr;
    uint8_t nmt_data[2] = {0x01, 0x01}; // 0x01=启动操作，0x01=Node-ID
    nmt_hdr.StdId = 0x000;              // NMT 命令 COB-ID 固定0x000
    nmt_hdr.RTR = CAN_RTR_DATA;
    nmt_hdr.DLC = 2;
    nmt_hdr.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &nmt_hdr, nmt_data, &tx_mailbox) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(100);
    return HAL_OK;
}
// 定义全局变量存储双电机速度（单位：0.1r/min，后续需转换为r/min）
int16_t left_motor_speed = 0;
int16_t right_motor_speed = 0;
float left_speed_rpm = 0;
float right_speed_rpm = 0;

// CAN1接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // if ((HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &CAN_Rx, CAN_Rx_Data)) != HAL_OK)
    // {
    // 	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    // 		Error_Handler();
    //   }
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 读取接收FIFO0中的报文

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        // 确认是TPDO0报文（COB-ID=0x181）
        if (rx_header.StdId == 0x181)
        {
            // 解析左电机速度
            left_motor_speed = -(int16_t)(rx_data[0] | (rx_data[1] << 8));
            // 解析右电机速度
            right_motor_speed = (int16_t)(rx_data[2] | (rx_data[3] << 8));

            left_speed_rpm = left_motor_speed / 10;
            right_speed_rpm = right_motor_speed / 10;
        }
    }
}

/* USER CODE END 1 */
