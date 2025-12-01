/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_subipc.h"
#include "controler.h"
#include "motor_can.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int count = 0;
int stop_count = 0;
uint8_t RX[10];
uint8_t speed_data[10];
uint32_t ID = 0;
uint32_t ID1 = 0x601;
uint32_t ID2 = 0x602;
uint32_t ID3 = 0x603;
uint8_t ID1_Rx_Data_4;
uint8_t ID2_Rx_Data_4;
uint8_t ID3_Rx_Data_4;
uint8_t ID1_Rx_Data_5;
uint8_t ID2_Rx_Data_5;
uint8_t ID3_Rx_Data_5;
uint8_t rising_falling_flag = 0;
uint16_t test_state;
uint16_t test_flag = 0;
uint8_t jerk_process4 = 0;
uint8_t jerk_door = 0;
uint8_t jerk_triangle = 0;
uint8_t jerk_bia_bin = 0;
uint8_t flag_door = 0;
uint8_t flag_triangle = 0;
uint8_t flag_big_bin = 0;
uint8_t flag_door_goback = 0;
uint8_t flag_big_bin_and_triangle = 0;
uint8_t star = 0;
int len = 0;
int countmain = 0;
extern int target_rpm_left, target_rpm_right;
volatile bool device_responded = true;
uint32_t last_check_time = 0;
uint8_t data_buf[1 + 4 + 4 + 1] = {0};

float linear_x = 0.0;
float angular_z = 0.0;
uint8_t rx_flag = 0;
uint8_t send_speed_counter = 0;
extern union velo act_velo_left, act_velo_right;
float send_linear_vel = 0.0;  // 线速度值
float send_angular_vel = 0.0; // 角速度值
extern float left_speed_rpm;
extern float right_speed_rpm;
extern float WHEEL_BASE;
extern float r; // 轮子半径，单位m 0.131/2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *f)
{
    ITM_SendChar(ch);
    return ch;
    // while(!(USART1->SR & (1<<7)));
    // USART1->DR = ch;
    // return ch;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rec[1024];
//---------------------------------AT9S_Control---------------------------------------------------//

//------------------------------------------------------------------------------------------------//
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */

    CAN1_Filter();
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
    printf("working !\r\n");
    HAL_UART_Receive_IT(&huart3, RX, 1);
    HAL_UART_Receive_IT(&huart6, speed_data, 10);
 
    HAL_TIM_Base_Start_IT(&htim1);
    // 检测can是否连接成功
    if (HAL_CAN_Start(&hcan1) == HAL_OK)
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    }
    HAL_Delay(100);

    //	printf("initialization success\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    motor_enable(0x601); // 电机使能控制函数//紧接在速度模式初始化之后的，必须要
    HAL_Delay(100);
    // TPDO0_Setup() ;
    // 发送 NMT 启动命令（激活 PDO，Node-ID=1）
    CAN_TxHeaderTypeDef nmt_hdr;
    uint32_t tx_mailbox;
    uint8_t nmt_data[2] = {0x01, 0x01}; // 0x01=启动操作，0x01=Node-ID
    nmt_hdr.StdId = 0x000;              // NMT 命令 COB-ID 固定0x000
    nmt_hdr.RTR = CAN_RTR_DATA;
    nmt_hdr.DLC = 2;
    nmt_hdr.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &nmt_hdr, nmt_data, &tx_mailbox) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(1000);

    while (1)
    {
        // HAL_UART_Transmit(&huart6,&a,1,100);
        // if(rx_flag==1)
        // {
        //     rx_flag = 0;

        // }

        countmain++;

        // 发送实际速度20ms一次
        if (send_speed_counter >= 5)
        {
            send_speed_counter = 0;
            // 读取电机速度
            // read_motor_state(0x601);

            // 4. 步骤1：电机实际转速 → 左/右车轮实际线速度（m/s）
            float actual_wheel_v_left = (left_speed_rpm * 2 * 3.1415926 * r) / 60.0f;
            float actual_wheel_v_right = (right_speed_rpm * 2 * 3.1415926 * r) / 60.0f;

            // 5. 步骤2：车轮线速度 → 车辆实际线速度、角速度
            send_linear_vel = (actual_wheel_v_left + actual_wheel_v_right) / 2.0f;        // 线速度（平均）
            send_angular_vel = (actual_wheel_v_right - actual_wheel_v_left) / WHEEL_BASE; // 角速度（速度差/轮距）

            int index = 0;

            data_buf[index++] = 0xAA;

            // 存储线速度（float类型，4字节）
            memcpy(&data_buf[index], &send_linear_vel, 4);
            index += 4;

            // 存储角速度（float类型，4字节）
            memcpy(&data_buf[index], &send_angular_vel, 4);
            index += 4;

            // 添加帧尾
            data_buf[index++] = 0x55;

            if (huart6.gState == HAL_UART_STATE_READY)
            {
                HAL_UART_Transmit_IT(&huart6, data_buf, index);
            }
        }

        coffee_car_test();

        // HAL_Delay(1);

        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

        // 步进电机

        coffee_motor_control();

        if (test_flag == 1)
        {
            jerk_process = 3;
            test_location_2(0x603, 1);
        }
        if (test_flag == 2)
        {
            jerk_process = 2;
            test_location_2(0x603, 2);
        }
        if (test_flag == 7) // 消除报警
        {
            test_remove_alarm();
        }
        if (test_flag == 9) // 急停 打印电机状态和限位信号状态
        {
            soft_scram();
            read_motor_state_4();
            read_motor_state_5();
            jerk_process = 0;
            test_flag = 0;
        }
        if (test_flag == 111)
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x02;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID3, CAN_Tx_Data);
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (1 == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
    {
        //		HAL_Delay(10);
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
        rising_falling_flag++;
        if (rising_falling_flag > 2)
        {
            rising_falling_flag = 0;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        printf("uart tx end\r\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ReBuff;
    //    if (huart->Instance == USART3)
    //    {
    //        ReBuff = RX[0];
    //        if (ReBuff == 0x0f)
    //        {
    //            len = 0;
    //            rec[0] = 0x0f;
    //        }
    //        if (rec[0] == 0x0f)
    //        {
    //            rec[len] = ReBuff;
    //        }
    //        len++;
    //        if (len >= 24)
    //        {
    //            if (len > 1000)
    //            {
    //                len = 0;
    //            }
    //            else
    //            {
    //                receive_data_handle();
    //            }
    //        }
    //        //		USART_SendData (UART1,ReBuff);
    //        //		printf(" %x \r\n",ReBuff);
    //        HAL_UART_Receive_IT(&huart3, RX, 1);
    //    }
    if (huart->Instance == USART6)
    {
        if (speed_data[0] == 0xAA && speed_data[9] == 0x55)
        {
            memcpy(&target_linear_velocity, &speed_data[1], 4);
            memcpy(&target_angular_velocity, &speed_data[5], 4);

            stop_count = 0;
        }
        HAL_UART_Receive_IT(&huart6, speed_data, 10);
        // printf("REV : %x\r\n", RX[0]);
        // if (RX[0] == 0x31)
        // {
        //     test_flag = 1;
        //     printf("1\r\n");
        // }
        // if (RX[0] == 0x32)
        // {
        //     test_flag = 2;
        //     printf("2\r\n");
        // }
        // if (RX[0] == 0x33)
        // {
        //     test_flag = 3;
        //     printf("3\r\n");
        // }
        // if (RX[0] == 0x34)
        // {
        //     test_flag = 4;
        //     printf("4\r\n");
        // }
        // if (RX[0] == 0x35)
        // {
        //     test_flag = 5;
        //     printf("5\r\n");
        // }
        // if (RX[0] == 0x36)
        // {
        //     test_flag = 6;
        //     printf("6\r\n");
        // }
        // if (RX[0] == 0x37)
        // {
        //     test_flag = 7;
        //     printf("7\r\n");
        // }
        // if (RX[0] == 0x38)
        // {
        //     test_flag = 8;
        //     printf("8\r\n");
        // }
        // if (RX[0] == 0x39)
        // {
        //     test_flag = 9;
        //     printf("9\r\n");
        // }
        // if (RX[0] == 0x40)
        // {
        //     test_flag = 10;
        //     printf("10\r\n");
        // }
        // HAL_UART_Receive_IT(&huart1, RX, 1);
        // printf("receive accomplish\r\n");
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1) // 10ms 1次
    {
        stop_count++;
        send_speed_counter++;

        if (stop_count >= 50) // 0.5s
        {
            stop_count = 0;
            target_linear_velocity = 0;
            target_angular_velocity = 0;
        }
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
