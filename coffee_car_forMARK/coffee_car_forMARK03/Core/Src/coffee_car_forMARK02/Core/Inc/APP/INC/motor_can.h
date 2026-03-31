#ifndef __MOTOR_CAN_H__
#define __MOTOR_CAN_H__
#include "stdio.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"
void test_location_1(uint32_t ID, uint8_t dire);
void test_location_2(uint32_t ID, uint8_t dire);
void test_location_3(uint32_t ID, uint8_t dire);
void test_location_22(uint32_t ID, uint8_t dire);
void soft_scram(void);
void test_speed(uint32_t ID, uint8_t dire);
void test_origin(uint32_t ID);
void origin_1(uint32_t ID);
void origin_2(uint32_t ID);
void origin_3(uint32_t ID);
void origin_11(uint32_t ID);
void origin_22(uint32_t ID);
void origin_33(uint32_t ID);
void test_front_origin(uint32_t ID, uint8_t dire);
void test_contrary_origin(uint32_t ID, uint8_t dire);
void test_speed_acc(uint32_t ID, uint8_t dire);
void test_limit(uint8_t dire);
void test_remove_alarm(void);
void read_motor_state_4(void);
void read_motor_state_5(void);
uint8_t test_save(uint32_t ID, uint8_t dire);
uint8_t door_control(uint32_t ID, uint8_t dire);
uint8_t triangle_control(uint32_t ID, uint8_t dire);
uint8_t big_bin_control(uint32_t ID, uint8_t dire);
void deliver_from_godown(void);
void door_go_back(uint32_t ID);
void big_bin_and_triangle_back(void);
extern uint8_t jerk_process2;
extern uint8_t jerk_process3;
uint8_t location_control_run(uint32_t ID, int acceleration, int stop_time, int maximum_time, int pulse, int work_pattern);
#endif /*__MOTOR_CAN_H__ */
