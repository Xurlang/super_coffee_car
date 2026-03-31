/*
 * step_motor.c
 *
 *  Created on: 2022年9月29日
 *      Author: UDI
 */
#include "step_motor.h"
#include "stdio.h"
//void motor_config_u8(uint32_t ID,uint16_t add,){
//	CAN_Tx_Data[0] = 0x2F;
//
//
//	CAN1_Tx(ID, CAN_Tx_Data);
//
//}
//
//void motor_config_u16(uint32_t ID,uint16_t add,){
//	CAN_Tx_Data[0] = 0x2B;
//
//	CAN1_Tx(ID, CAN_Tx_Data);
//}
//
//void motor_config_u32(uint32_t ID,uint16_t add,){
//	CAN_Tx_Data[0] = 0x23;
//	CAN1_Tx(ID, CAN_Tx_Data);
//}
void motor_config(uint32_t ID){
	int step_0 = 0;
	switch(step_0){
		case 0:{
			//x0 forward_lim
			CAN_Tx_Data[0] = 0x2F;
			CAN_Tx_Data[1] = 0x30;
			CAN_Tx_Data[2] = 0x20;
			CAN_Tx_Data[4] = 0x02;
			CAN_Tx_Data[7] = 0x02;
			CAN1_Tx(ID, CAN_Tx_Data);
			step_0++;
		}
		case 1:{
			//x1 reverse_lim
			CAN_Tx_Data[0] = 0x2F;
			CAN_Tx_Data[1] = 0x30;
			CAN_Tx_Data[2] = 0x20;
			CAN_Tx_Data[4] = 0x03;
			CAN_Tx_Data[7] = 0x03;
			CAN1_Tx(ID, CAN_Tx_Data);
			step_0++;
		}
		case 2:{
			//y0、y1 --vcc
			CAN_Tx_Data[0] = 0x2F;
			CAN_Tx_Data[1] = 0x30;
			CAN_Tx_Data[2] = 0x20;
			CAN_Tx_Data[4] = 0x0C;
			CAN_Tx_Data[7] = 0x03;
			CAN1_Tx(ID, CAN_Tx_Data);
			step_0++;
		}
	}
}

void tr_velocity(int velocity,uint8_t *b0,uint8_t *b1,uint8_t *b2,uint8_t *b3){
	if(velocity>3000){
		velocity = 3000;
	}
	else if(velocity < -3000){
		velocity = -3000;
	}

	if(velocity < 0){
		velocity = velocity + 65536;
		*b2 = 0xFF;
		*b3 = 0xFF;
	}
	*b0 = velocity;
	*b1 = velocity%256;
}

//void motor_velo_start(uint32_t ID){
//	switch(step){
//		case 0:{
//
//		}
//	}
//
//}
