#include "chassis_subipc.h"
#include "can.h"
uint8_t ipc_rec[30];
int ipc_len;
extern uint8_t door_flag;
extern uint8_t brake_big_bin_order;
extern uint8_t brake_triangle_order;
union velo1
{
    double d;
    uint8_t c[8];
};

struct chassis  //real
{
    double time; 
    double linear_x;
    double angular_z;
    double distance;
    double imu_pitch;
    double imu_roll;
    double imu_yaw;
};
union velo1 plan_linear_x,plan_angular_y;
extern struct  chassis apllo_pi;
extern double ipc_target_linear_velocity;
extern double ipc_target_angular_velocity;
void ipc_receive_data_handle(){
    double wheel_d; 
    double wheel_C;
    double chassis_d;
    double PI;
		PI = 3.1415926;
		wheel_d = 0.131;
		chassis_d = 0.33;
    wheel_C = PI * wheel_d;
    uint8_t sum;
		sum = 0x00;
//		for(int i = 0;i < 19;i++){
//        printf("%x    ",ipc_rec[i]);
//    }
    for(int i = 0;i < 19;i++){
        sum+=ipc_rec[i];
    }
//    sum = sum & 0xff;
		printf("%x\r\n",sum);
    if(ipc_rec[19] == sum){
			for(int i=0;i < 8;i++){
					plan_linear_x.c[i] = ipc_rec[i+1];
					plan_angular_y.c[i]= ipc_rec[i+9];
			}
			printf("y=%x ,z=%x\r\n",ipc_rec[17],ipc_rec[18]);
			ipc_target_linear_velocity = plan_linear_x.d / wheel_C * 60.0;
			ipc_target_angular_velocity = plan_angular_y.d/(2*PI) * (2*PI*chassis_d) /wheel_C*60.0;
//			printf("x=%f ,y=%f\r\n",ipc_target_linear_velocity,ipc_target_angular_velocity);
			
    }
}

// uint8_t ipc_ReBuff;
// void UART5_IRQHandler(void)
// {
// 	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
// 	{		
// 		ipc_ReBuff = USART_ReceiveData(UART5);
// 		if(ipc_ReBuff == 0x0f && ipc_len != 19){
// 				ipc_len = 0;
// 				ipc_rec[0] = 0x0f;
// 		}
// 		if(ipc_rec[0] == 0x0f){
// 			ipc_rec[ipc_len] = ipc_ReBuff;
// 		}
// 		ipc_len++;
// 		if(ipc_len >= 22){
// 			if(ipc_len > 1000){
// 				ipc_len = 0;
// 			}
// 			else{
// 				ipc_receive_data_handle();
// 			}
// 		}
// 		if(ipc_rec[17] == 0x31)
// 		{
// 			door_flag = 1;
			
// 			printf("1\r\n");
// 		}
// 		if(ipc_rec[18] == 0x32)
// 		{
// 			door_flag = 2;
			
// 			printf("2\r\n");
// 		}
// 		if(ipc_rec[19] == 0x33)
// 		{
// 			door_flag = 3;
			
// 			printf("3\r\n");
// 		}
// 		if(ipc_rec[20] == 0x34)
// 		{
// 			brake_big_bin_order = 4;
			
// 			printf("4\r\n");
// 		}
// 		if(ipc_rec[21] == 0x35)
// 		{
// 			brake_triangle_order = 5;
			
// 			printf("5\r\n");
// 		}
	
// //		USART_SendData (USART1,ipc_ReBuff);
// //		printf("%x\r\n",ipc_ReBuff);
// 	}
// } 
