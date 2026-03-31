#include "controler.h"
#include "stdbool.h"
#include "can.h"
#include "math.h"
#include "stdlib.h"
extern int len;
extern uint8_t rec[1024]; 
extern int startup_step;
int channels[12];
int last_channels[12];
int i;

void parse_controler_data(){
  if(rec[0] == 0x0f && rec[24]==0x00){
    for(i=0;i<12;i++){
        last_channels[i] = channels[i];
      }
      channels[0] = -(((rec[1] | rec[2] << 8) & 0x07FF) - 1000)/694.0*100;
      channels[1] = -(((rec[2] >> 3 | rec[3] << 5) & 0x07FF) - 1000)/694.0*100;
      channels[2] = -(((rec[3] >> 6 | rec[4] << 2 | rec[5] << 10) & 0x07FF) - 306)/1388.0*100   + 100.0;
      channels[3] = -(((rec[5] >> 1 | rec[6] << 7) & 0x07FF)- 1000)/694.0*100-9.0;
      channels[4] = (((rec[6] >> 4 | rec[7] << 4) & 0x07FF)-1000)/694;
      channels[5] = -(((rec[7] >> 7 | rec[8] << 1 | rec[9] << 9) & 0x07FF)-1000)/694.0*100;
      channels[6] = -(((rec[i] >> 2 | rec[10] << 6) & 0x07FF)-1000)/694;
      channels[7] = -(((rec[10] >> 5 | rec[11] << 3) & 0x07FF)-1000)/694.0*100;
      channels[8] = -(((rec[12] | rec[13] << 8) & 0x07FF)-1000)/694;
      channels[9] = -(((rec[13] >> 3 | rec[14] << 5) & 0x07FF)-1000)/694;
      channels[10] = -(((rec[14] >> 6 | rec[15] << 2 | rec[16] << 10) & 0x07FF)-1000)/694;
      channels[11] = -(((rec[16] >> 1 | rec[17] << 7) & 0x07FF)-1000)/694;
//			for(i=0;i<12;i++){
//				printf("%d    ",channels[i]);
//			if(i == 11){
//				printf("\r\n");
//			 }
//			}
//      return true;
    }
    else{
//      return false;
    }
}
extern uint8_t test_flag;
void coffee_motor_control(){
  /*控制？？？*/

	if(channels[4] == -1){
      test_flag = 1;
    }

	if(channels[4] == 1){
      test_flag = 2;
    }

  if(channels[4] == 0){
      test_flag = 111;
    }
	
}
void velo_mode_controler(){
  /*10通道控制是否输出速度*/
  if (channels[9] == 1){
      if ( abs(channels[1]) >= 10){
        //最大转速1000
         target_linear_velocity = 1.5* channels[1];
				      
      }
			else if ( abs(channels[1]) <10){
        // target_linear_velocity *= 0.99f;
        // if(target_linear_velocity < 1){
        //   target_linear_velocity = 0.0;
        // }
        target_linear_velocity = 0.0;
      }
		
      if ( abs(channels[3]) >= 10){
         target_angular_velocity = 0.2*channels[3];
				
      } 
			else if ( abs(channels[3]) <= 10){
         target_angular_velocity = 0.0;
      }   			
    }
    else if(channels[9] == -1){
//      target_linear_velocity = ipc_target_linear_velocity;
//      target_angular_velocity = ipc_target_angular_velocity;
    }
//    else if(channels[9] == 0){
//      target_linear_velocity = 0.0;
//      target_angular_velocity = 0.0;
//    }
}

void motor_mode_change(){
  /*通道12是使能失能*/
    if(channels[11] == 0){
      startup_step = 5;
    }
    else if(channels[11] == 1 && last_channels[11] != 1){
      startup_step = 1;
    }
    else if(channels[11] == -1){
      startup_step = 6;
    }
}
extern bool flag_quick_stop;
void receive_data_handle(){
	bool flag;
	flag = true;
	parse_controler_data();
	
	if (flag)
	{
//		printf("%d \r\n",startup_step);
		velo_mode_controler();
		motor_mode_change();
		if(flag_quick_stop){
			coffee_motor_control();
		}
		flag = false;
	}
	else{
		flag = false;
	}
}
