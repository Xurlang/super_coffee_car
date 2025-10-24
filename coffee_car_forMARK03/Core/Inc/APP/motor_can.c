#include "motor_can.h"
#include "controler.h"
extern uint8_t rising_falling_flag;
extern uint16_t test_state;
extern uint16_t test_flag;
extern uint32_t ID;
extern uint32_t ID1;
extern uint32_t ID2;
extern uint32_t ID3;
extern uint8_t ID1_Rx_Data_4;
extern uint8_t ID2_Rx_Data_4;
extern uint8_t ID3_Rx_Data_4;
extern uint8_t ID1_Rx_Data_5;
extern uint8_t ID2_Rx_Data_5;
extern uint8_t ID3_Rx_Data_5;
uint8_t jerk_process2 = 0;
uint8_t jerk_process3 = 0;
extern uint8_t jerk_process4;
extern uint8_t jerk_door;
extern uint8_t jerk_triangle;
extern uint8_t jerk_bia_bin;
extern uint8_t flag_door;
extern uint8_t flag_triangle;
extern uint8_t flag_big_bin;
extern uint8_t flag_door_goback;
extern uint8_t flag_big_bin_and_triangle;

bool flag_location_run;
bool flag_runrun;
bool flag_coffee;
union position
{
    int d;
    uint8_t c[4];
};
// ��ͣ
void soft_scram(void) // 刹车
{
    CAN_Tx_Data[0] = 0x2B;
    CAN_Tx_Data[1] = 0x40;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x02;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(0x601, CAN_Tx_Data);
    HAL_Delay(10);
    printf("door_scram!\r\n");
    CAN1_Tx(0x602, CAN_Tx_Data);
    HAL_Delay(10);
    printf("triangle_scram!\r\n");
    CAN1_Tx(0x603, CAN_Tx_Data);
    HAL_Delay(10);
    printf("big_bin_scram!\r\n");
}
// 控制速度模式/位置模式
uint8_t location_control_run(uint32_t ID, int acceleration, int stop_time, int maximum_time, int pulse, int work_pattern)
{
    union position acce;
    union position stop;
    union position maximum;
    union position pul;
    union position work;
    acce.d = acceleration;
    stop.d = stop_time;
    maximum.d = maximum_time;
    pul.d = pulse;
    work.d = work_pattern;
    flag_location_run = true;
    jerk_process4 = 1;
    while (flag_location_run)
    {
        switch (jerk_process4)
        {
        case 1: // 使能直线模组
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x00; // 0xD0
            CAN_Tx_Data[5] = 0x00; // 0x19
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 2;
            break;
        }
        case 2: // acceleration time 2000ms 6083h
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = acce.c[0];
            CAN_Tx_Data[5] = acce.c[1];
            CAN_Tx_Data[6] = acce.c[2];
            CAN_Tx_Data[7] = acce.c[3];
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 3;
            break;
        }
        case 3: // stop time 100ms 6084h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = stop.c[0];
            CAN_Tx_Data[5] = stop.c[1];
            CAN_Tx_Data[6] = stop.c[2];
            CAN_Tx_Data[7] = stop.c[3];
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 4;
            break;
        }
        case 4: // maximum speed  (800rpm 0x320)(480r/min 0x01E0) (240r/min 0xF0)6081
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = maximum.c[0];
            CAN_Tx_Data[5] = maximum.c[1];
            CAN_Tx_Data[6] = maximum.c[2];
            CAN_Tx_Data[7] = maximum.c[3];
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 5; // ˳ʱ�� ��
            break;
        }

        case 5: // ������  607A  107000(0x1A1F8) 160000(0x27100)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = pul.c[0];
            CAN_Tx_Data[5] = pul.c[1];
            CAN_Tx_Data[6] = pul.c[2];
            CAN_Tx_Data[7] = pul.c[3];
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 7;
            break;
        }
        case 7: // work pattern ����ģʽ 01 λ��
        {
            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = work.c[0];
            CAN_Tx_Data[5] = work.c[1];
            CAN_Tx_Data[6] = work.c[2];
            CAN_Tx_Data[7] = work.c[3];
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 8;
            break;
        }
        case 8: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 9;
            break;
        }
        case 9:
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
            HAL_Delay(10);
            jerk_process4 = 10;
            break;
        }
        case 10:
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x0F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 11;
            break;
        }
        case 11: // relative movement 1
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 12;
            break;
        }
        case 12: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 0;
            flag_location_run = false;
            break;
        }
            //	default:
            //		break;
        }
    }
    return 0;
}

// ��������
uint8_t test_save(uint32_t ID, uint8_t dire) // SAVE
{
    switch (jerk_process)
    {
    case 1:
    {
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x00; //					0x64   0xE8  0xDC
        CAN_Tx_Data[5] = 0x00; //          0x03  0x05
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 2;
        break;
    }
    case 2: // acceleration time 2000ms 6083h
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x5A;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x06; //					0x64   0xE8  0xDC
        CAN_Tx_Data[5] = 0x00; //          0x03  0x05
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 3;
        break;
    }
    case 3: // stop time 100ms 6084h
    {
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x10;
        CAN_Tx_Data[2] = 0x20;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x02;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 0;
        printf("W succeed!\r\n");
        break;
    }
        //	default:
        //		break;
    }
    return 0;
}
uint8_t door_control(uint32_t ID, uint8_t dire) // �۵���
{
    do
    {
        switch (jerk_process4)
        {
        case 1:
        {
            read_motor_state_5();
            if (0x40 != ((ID2_Rx_Data_5 | ID3_Rx_Data_5) & 0x40))
            { // 2�ţ�3�ŵ�����ھ�ֹ״̬
                // �ջ�
                if (dire == 2)
                {
                    read_motor_state_4();
                    if (0x01 == ID1_Rx_Data_4) // ��ԭ��λ�ã���������
                    {
                        jerk_process4 = 0;
                        flag_door = 3;
                    }
                    else
                    { // ����ԭ��λ�ã�ִ���ջض���
                        jerk_process4 = 13;
                    }
                }
                // �Ƴ�
                if (dire == 1)
                {
                    read_motor_state_4();
                    if (0x01 == ID2_Rx_Data_4 || 0x02 == ID2_Rx_Data_4) // ���ǲֹ�λ
                    {
                        if (0x02 == ID1_Rx_Data_4 || (!(0x02 == ID3_Rx_Data_4))) // �ڲտ�λ��or��ֲ����¶�λ�ã���������
                        {
                            jerk_process4 = 0;
                            flag_door = 3;
                        }
                        else
                        { // ���ڲտ�λ�ã�ִ���Ƴ�����
                            jerk_process4 = 2;
                        }
                    }
                    else
                    { // ���ǲ�δ��λ
                        jerk_process4 = 0;
                        flag_door = 3;
                    }
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_door = 3;
            }
            break;
        }
        case 2: // acceleration time 2000ms 6083h
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xD0; //					0x64   0xE8  0xDC
            CAN_Tx_Data[5] = 0x07; //          0x03  0x05
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 3;
            break;
        }
        case 3: // stop time 100ms 6084h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x64;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 4;
            break;
        }
        case 4: // maximum speed  (800rpm 0x320)(480r/min 0x01E0) (240r/min 0xF0)6081
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x20; // F0 0X3C;  0xE0
            CAN_Tx_Data[5] = 0x03; //          0x01
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 5; // ˳ʱ�� ��
            break;
        }

        case 5: // ������  607A  107000(0x1A1F8) 160000(0x27100)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xF8; // 0xD0
            CAN_Tx_Data[5] = 0xA1; // 0x19
            CAN_Tx_Data[6] = 0x01;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 7;
            break;
        }
        case 7: // work pattern ����ģʽ 01 λ��
        {

            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 8;
            break;
        }
        case 8: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 9;
            break;
        }
        case 9:
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
            HAL_Delay(10);
            jerk_process4 = 10;
            break;
        }
        case 10:
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x0F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 11;
            break;
        }
        case 11: // relative movement 1
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 12;
            break;
        }
        case 12: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 23;
            break;
        }
        case 13: // RETURN
        {        // acceleration time 100ms 6083h

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x64; // 0x64   0xE8  0xDC
            CAN_Tx_Data[5] = 0x00; //          0x03  0x05
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 14;
            break;
        }
        case 14: // stop time 500ms 6084h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xF4;
            CAN_Tx_Data[5] = 0x01;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 15;
            break;
        }
        case 15: // maximum speed  (800rpm 0x320)(480rpm 0x1E0)��ԭ��1000rpm 0x3E8 6081��
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xE8; // F0 0X3C;  0xE0
            CAN_Tx_Data[5] = 0x03; //          0x01
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 16;
            break;
        }

        case 16: //-107000
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x08; // 0xD0
            CAN_Tx_Data[5] = 0x5E; // 0x19
            CAN_Tx_Data[6] = 0xFE;
            CAN_Tx_Data[7] = 0xFF;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 17;
            break;
        }
        case 17: // work pattern ����ģʽ 01 λ��
        {

            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 18;
            break;
        }
        case 18: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 19;
            break;
        }
        case 19:
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
            HAL_Delay(10);
            jerk_process4 = 20;
            break;
        }
        case 20:
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
            HAL_Delay(10);
            jerk_process4 = 21;
            break;
        }
        case 21: // relative movement 1
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 22;
            break;
        }
        case 22: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 23;
            break;
        }
        case 23: // �ж��˶�����
        {
            if (1 == dire)
            {
                jerk_process4 = 24;
            }
            if (2 == dire)
            {
                jerk_process4 = 25;
            }
            break;
        }
        case 24:
        {
            read_motor_state_5();
            if (0x40 == (ID1_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (2 == ID1_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("door over\r\n");
                    jerk_process4 = 0;
                    flag_door = 2;
                }
                else
                {
                    jerk_process4 = 24;
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_door = 3;
            }
            break;
        }
        case 25:
        {
            read_motor_state_5();
            if (0x40 == (ID1_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (1 == ID1_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("door over\r\n");
                    jerk_process4 = 0;
                    flag_door = 1;
                }
                else
                {
                    jerk_process4 = 25;
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_door = 3;
            }
            break;
        }
            //	default:
            //		break;
        }
    } while (flag_door == 0);
    return flag_door;
}
// 这是干什么用的？？？
uint8_t triangle_control(uint32_t ID, uint8_t dire) // ���ǲ�
{
    do
    {
        switch (jerk_process4)
        {
        case 1:
        {
            read_motor_state_5();
            if (0x40 != (ID1_Rx_Data_5 & 0x40))
            { // 1�ŵ����ֹ״̬
                // ���ǲ�����(�����Ƴ�)
                if (dire == 1)
                {
                    read_motor_state_4();
                    if (0x01 == ID1_Rx_Data_4) // �۵�����ԭ��
                    {
                        if (0x02 == ID2_Rx_Data_4) // ���ǲ�λ���ϼ���λ�ã���������
                        {
                            jerk_process4 = 0;
                            flag_triangle = 3;
                        }
                        else
                        { // ���ǲֲ����ϼ���λ�ã�ִ����������
                            jerk_process4 = 2;
                        }
                    }
                    else
                    { // �۵���δ��λ
                        jerk_process4 = 0;
                        flag_triangle = 3;
                    }
                }

                // ���ǲ��½��������ջأ�
                if (dire == 2)
                {
                    read_motor_state_4();
                    if (0x01 == ID1_Rx_Data_4) // �۵�����ԭ��
                    {
                        if (0x01 == ID2_Rx_Data_4) // ���ǲ�λ���¼���λ�ã���������
                        {
                            jerk_process4 = 0;
                            flag_triangle = 3;
                        }
                        else
                        { // ���ǲֲ����¼���λ�ã�ִ����������
                            jerk_process4 = 3;
                        }
                    }
                    else
                    { // �۵���δ��λ
                        jerk_process4 = 0;
                        flag_triangle = 3;
                    }
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_triangle = 3;
            }
            break;
        }

        case 2: // ������  607A  160000(0x27100)  248000(0x3C8C0) now240000(0x3A980)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x80; // 0xD0
            CAN_Tx_Data[5] = 0xA9; // 0x19
            CAN_Tx_Data[6] = 0x03;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 4;
            break;
        }
        case 3: //-160000(0xFFFD8F00)  -248000(0xFFFC3740) now-240000(0xFC5680)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x80;
            CAN_Tx_Data[5] = 0x56;
            CAN_Tx_Data[6] = 0xFC;
            CAN_Tx_Data[7] = 0xFF;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 4;
            break;
        }
        case 4: // acceleration time (500ms 0x1F4)(1000ms 0x3E8) 6083h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xF4; //					0x64   0xE8  0xDC
            CAN_Tx_Data[5] = 0x01; //          0x03  0x05
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 5;
            break;
        }
        case 5: // stop time (1500ms 0x5DC)(2000ms 0x7D0) 6084h (1000ms 0x3E8)
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xDC;
            CAN_Tx_Data[5] = 0x05;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 6;
            break;
        }
        case 6: // maximum speed  (480r/min 1E0) (ԭ��1000rpm 3E8) (1500ms 0x5DC)(1800rpm 708)(2000rpm 7D0)(3000rpm BB8)6081
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xA0; // F0 0X3C;  0xE0
            CAN_Tx_Data[5] = 0x05; //          0x01
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 7;
            break;
        }

        case 7: // work pattern ����ģʽ 01 λ��
        {

            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 8;
            break;
        }
        case 8: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 9;
            break;
        }
        case 9:
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
            HAL_Delay(10);
            jerk_process4 = 10;
            break;
        }
        case 10:
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
            HAL_Delay(10);
            jerk_process4 = 11;
            break;
        }
        case 11: // relative movement 1
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 12;
            break;
        }
        case 12: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 23;
            break;
        }
        case 23: // �ж��˶�����
        {
            if (1 == dire)
            {
                jerk_process4 = 24;
            }
            if (2 == dire)
            {
                jerk_process4 = 25;
            }
            break;
        }
        case 24:
        {
            read_motor_state_5();
            if (0x40 == (ID2_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (2 == ID2_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("triangle over\r\n");
                    jerk_process4 = 0;
                    flag_triangle = 2;
                }
                else
                {
                    jerk_process4 = 24;
                }
            }
            else
            {
                jerk_process4 = 0;
            }
            break;
        }
        case 25:
        {
            read_motor_state_5();
            if (0x40 == (ID2_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (1 == ID2_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("triangle over\r\n");
                    jerk_process4 = 0;
                    flag_triangle = 1;
                }
                else
                {
                    jerk_process4 = 25;
                }
            }
            else
            {
                jerk_process4 = 0;
            }
            break;
        }
            //	default:
            //		break;
        }
    } while (flag_triangle == 0);
    return flag_triangle;
}
uint8_t big_bin_control(uint32_t ID, uint8_t dire) // ���
{
    do
    {
        switch (jerk_process4)
        {
        case 1:
        {
            read_motor_state_5();
            if (0x40 != (ID1_Rx_Data_5 & 0x40))
            { // 1��,2�ŵ�����ھ�ֹ״̬
                // ������У������ջأ�
                if (dire == 1)
                {
                    read_motor_state_4();
                    if (0x01 == ID3_Rx_Data_4 || (0x01 != ID1_Rx_Data_4)) // ��ԭ��λ��or�۵���û�ջأ���������
                    {
                        jerk_process4 = 0;
                        flag_big_bin = 3;
                    }
                    else
                    { // ����ԭ��λ�ã�ִ�����ж���
                        jerk_process4 = 13;
                    }
                }
                // ������У������Ƴ���
                if (dire == 2)
                {
                    read_motor_state_4();
                    if (0x02 == ID3_Rx_Data_4) // �ڵ׶�λ�ã���������
                    {
                        jerk_process4 = 0;
                        flag_big_bin = 3;
                    }
                    else
                    { // ���ڵ׶�λ�ã�ִ�����ж���
                        jerk_process4 = 2;
                    }
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_big_bin = 3;
            }
            break;
        }
        case 2: // acceleration time (1000ms 0x1F4)6083h
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xE8; //					0x64   0xE8  0xDC
            CAN_Tx_Data[5] = 0x03; //          0x03  0x05
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 3;
            break;
        }
        case 3: // stop time (2000ms 0x7D0)6084h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xD0;
            CAN_Tx_Data[5] = 0x07;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 4;
            break;
        }
        case 4: // maximum speed  (ԭ��1000rpm 0x3E8)��600rpm 0x258��(480r/min 0x01E0) (240r/min 0xF0)6081
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x58; // F0 0X3C;  0xE0
            CAN_Tx_Data[5] = 0x02; //          0x01
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 5; // ˳ʱ�� ��
            break;
        }

        case 5: // ������  607A 210000(0x33450)(ԭ��190000 0x2E630)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x50; // 0xD0
            CAN_Tx_Data[5] = 0x34; // 0x19
            CAN_Tx_Data[6] = 0x03;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 7;
            break;
        }
        case 7: // work pattern ����ģʽ 01 λ��
        {

            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 8;
            break;
        }
        case 8: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 9;
            break;
        }
        case 9:
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
            HAL_Delay(10);
            jerk_process4 = 10;
            break;
        }
        case 10:
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x0F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 11;
            break;
        }
        case 11: // relative movement 1
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data); // 50RPM
            HAL_Delay(10);
            jerk_process4 = 12;
            break;
        }
        case 12: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 23;
            break;
        }
        case 13: // RETURN
        {        // acceleration time 2000ms 6083h

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x83;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xD0; // 0x64   0xE8  0xDC
            CAN_Tx_Data[5] = 0x07; //          0x03  0x055
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 14;
            break;
        }
        case 14: // stop time 1000ms 6084h
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x84;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xE8;
            CAN_Tx_Data[5] = 0x03;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 15;
            break;
        }
        case 15: // maximum speed   600rpm(0x258) 6081
        {
            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x81;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x58; // F0 0X3C;  0xE0
            CAN_Tx_Data[5] = 0x02; //          0x01
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 16;
            break;
        }

        case 16: //-210000(0xFCCBB0) -190000(0xFD19D0)
        {

            CAN_Tx_Data[0] = 0x23;
            CAN_Tx_Data[1] = 0x7A;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0xB0; // 0xD0
            CAN_Tx_Data[5] = 0xCB; // 0x19
            CAN_Tx_Data[6] = 0xFC;
            CAN_Tx_Data[7] = 0xFF;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 17;
            break;
        }
        case 17: // work pattern ����ģʽ 01 λ��
        {

            CAN_Tx_Data[0] = 0x2F;
            CAN_Tx_Data[1] = 0x60;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x01;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 18;
            break;
        }
        case 18: // switchover state machine �л�״̬��
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
            HAL_Delay(10);
            jerk_process4 = 19;
            break;
        }
        case 19:
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
            HAL_Delay(10);
            jerk_process4 = 20;
            break;
        }
        case 20:
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
            HAL_Delay(10);
            jerk_process4 = 21;
            break;
        }
        case 21: // relative movement 1
        {

            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x4F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 22;
            break;
        }
        case 22: // relative movement 2
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x5F;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            jerk_process4 = 23;
            break;
        }
        case 23: // �ж��˶�����
        {
            if (1 == dire)
            {
                jerk_process4 = 24;
            }
            if (2 == dire)
            {
                jerk_process4 = 25;
            }
            break;
        }
        case 24:
        {
            read_motor_state_5();
            if (0x40 == (ID3_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (1 == ID3_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("big_bin over\r\n");
                    jerk_process4 = 0;
                    flag_big_bin = 1;
                }
                else
                {
                    jerk_process4 = 24;
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_big_bin = 3;
            }
            break;
        }
        case 25:
        {
            read_motor_state_5();
            if (0x40 == (ID3_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (2 == ID3_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("big_bin over\r\n");
                    jerk_process4 = 0;
                    flag_big_bin = 2;
                }
                else
                {
                    jerk_process4 = 25;
                }
            }
            else
            {
                jerk_process4 = 0;
                flag_big_bin = 3;
            }
            break;
        }
            //	default:
            //		break;
        }
    } while (flag_big_bin == 0);
    return flag_big_bin;
}
void deliver_from_godown(void)
{
    do
    {
        switch (jerk_process)
        {
        //	test_location_3(ID3,2);//�����
        case 1:
        {
            if (flag_door == 1 && flag_triangle == 2 && flag_big_bin == 1 && jerk_process == 1)
            {
                jerk_process4 = 1;
                big_bin_control(ID3, 2);
                jerk_process = 2;
            }
            break;
        }
        case 2:
        {
            if (flag_door == 1 && flag_triangle == 2 && flag_big_bin == 2 && jerk_process == 2)
            {
                jerk_process4 = 1;
                door_control(ID1, 1);
                jerk_process = 3;
            }
            break;
        }
        case 3:
        {
            if (flag_door == 2 && flag_triangle == 2 && flag_big_bin == 2 && jerk_process == 3)
            {
                jerk_process4 = 1;
                door_control(ID1, 2);
                jerk_process = 4;
            }
            break;
        }
        case 4:
        {
            if (flag_door == 1 && flag_triangle == 2 && flag_big_bin == 2 && jerk_process == 4)
            {
                jerk_process4 = 1;
                triangle_control(ID2, 2);
                jerk_process = 5;
            }
            break;
        }
        case 5:
        {
            if (flag_door == 1 && flag_triangle == 1 && flag_big_bin == 2 && jerk_process == 5)
            {
                jerk_process4 = 1;
                door_control(ID1, 1);
                jerk_process = 6;
            }
            break;
        }
        case 6:
        {
            if (flag_door == 2 && flag_triangle == 1 && flag_big_bin == 2 && jerk_process == 6)
            {
                jerk_process4 = 1;
                door_control(ID1, 2);
                jerk_process = 7;
            }
            break;
        }
        case 7:
        {
            if (flag_door == 1 && flag_triangle == 1 && flag_big_bin == 2 && jerk_process == 7)
            {
                jerk_process4 = 1;
                big_bin_control(ID3, 1);
                triangle_control(ID2, 1);
                jerk_process = 8;
            }
            break;
        }
        }
    } while (jerk_process != 8);
}

void door_go_back(uint32_t ID)
{
    flag_door_goback = 0;
    do
    {
        switch (jerk_process)
        {
        case 1:
        {
            read_motor_state_5();
            read_motor_state_4();
            if (0x40 != ((ID2_Rx_Data_5 | ID3_Rx_Data_5) & 0x40))
            {                                                       // 2�ţ�3�ŵ�����ھ�ֹ״̬
                if (0x01 == ID2_Rx_Data_4 || 0x02 == ID2_Rx_Data_4) // ���ǲֹ�λ
                {
                    if (!(0x02 == ID3_Rx_Data_4)) // ��ֲ����¶�λ�ã���������
                    {
                        jerk_process = 0;
                        flag_door_goback = 1;
                        //										printf("********************big_bin_no_back_door_error*******************\r\n");
                    }
                    else if (0x02 == ID1_Rx_Data_4) // �ڲֿ�λ�ã�ִ���ջض���
                    {
                        jerk_process = 13;
                        //										printf("********************jerk_process = 13;*******************\r\n");
                    }
                    else
                    {
                        jerk_process = 2;
                        //										printf("********************jerk_process = 2;*******************\r\n");
                    }
                }
                else
                {
                    jerk_process = 0;
                    flag_door_goback = 1;
                    //									printf("********************triangle_no_back_door_error*******************\r\n");
                }
            }
            else
            {
                jerk_process = 0;
                flag_door_goback = 1;
                //								printf("********************go_door_error*******************\r\n");
            }
            break;
        }
        case 2: // acceleration time 2000ms 6083h
        {
            location_control_run(ID, 2000, 100, 800, 104000, 1);
            jerk_process = 24;
            printf("********************jerk_process = 12;*******************\r\n");
            break;
        }
        case 13: // RETURN
        {        // acceleration time 100ms 6083h
            location_control_run(ID, 100, 1500, 1000, -104000, 1);
            jerk_process = 25;
            break;
        }
        case 24:
        {
            read_motor_state_5();
            if (0x40 == (ID1_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (2 == ID1_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("********************door_go_over**********************\r\n");
                    jerk_process = 13;
                }
                else
                {
                    jerk_process = 24;
                }
            }
            else
            {
                jerk_process = 13;
                printf("********************jerk_process = 24 to 13*******************\r\n");
            }
            break;
        }
        case 25:
        {
            read_motor_state_5();
            if (0x40 == (ID1_Rx_Data_5 & 0x40))
            {
                read_motor_state_4();
                if (1 == ID1_Rx_Data_4)
                {
                    CAN_Tx_Data[0] = 0x2B;
                    CAN_Tx_Data[1] = 0x40;
                    CAN_Tx_Data[2] = 0x60;
                    CAN_Tx_Data[3] = 0x00;
                    CAN_Tx_Data[4] = 0x02;
                    CAN_Tx_Data[5] = 0x00;
                    CAN_Tx_Data[6] = 0x00;
                    CAN_Tx_Data[7] = 0x00;
                    CAN1_Tx(ID, CAN_Tx_Data);
                    HAL_Delay(10);
                    printf("********************door_goback_over*******************\r\n");
                    jerk_process = 0;
                    flag_door_goback = 1;
                }
                else
                {
                    jerk_process = 25;
                }
            }
            else
            {
                jerk_process = 0;
                flag_door_goback = 1;
            }
            break;
        }
        }
    } while (flag_door_goback != 1);
}
void big_bin_and_triangle_back(void)
{
    do
    {
        switch (jerk_process)
        {
        case 1:
        {
            read_motor_state_5();
            if (0x40 != (ID1_Rx_Data_5 & 0x40))
            { // 1��,2�ŵ�����ھ�ֹ״̬
                // ������У������ջأ�
                read_motor_state_4();
                if (0x01 != ID1_Rx_Data_4) // �۵���û�ջأ���������
                {
                    jerk_process = 0;
                    flag_big_bin_and_triangle = 1;
                    printf("********************door_no_back_error*******************\r\n");
                }
                else if (0x01 == ID3_Rx_Data_4)
                {
                    jerk_process = 13;
                    printf("********************jerk_process = 13;*******************\r\n");
                }
                else
                {
                    jerk_process = 2;
                    printf("********************one_jerk_process = 2;*******************\r\n");
                }
                //							break;
            }
            else
            {
                jerk_process = 0;
                flag_big_bin_and_triangle = 1;
                printf("********************other_go_error*******************\r\n");
            }
            break;
        }
        case 2: // acceleration time 2000ms 6083h
        {
            location_control_run(ID3, 2000, 1000, 900, -210000, 1);
            jerk_process = 12;
            printf("********************jerk_process = 12;*******************\r\n");
            break;
        }
        case 12:
        {
            read_motor_state_4();
            read_motor_state_5();
            if (0x02 != ID2_Rx_Data_4)
            {
                if (0x40 != (ID2_Rx_Data_5 & 0x40))
                {
                    jerk_process = 13;
                    printf("********************jerk_process = 13;*******************\r\n");
                }
                else
                {
                    jerk_process = 24;
                }
            }
            else
            {
                jerk_process = 24;
                printf("********************jerk_process = 24;*******************\r\n");
            }
            break;
        }
        case 13: // triangle up
        {
            location_control_run(ID2, 500, 1500, 1440, 240000, 1);
            jerk_process = 24;
            printf("********************jerk_process = 24;*******************\r\n");
            break;
        }
        case 24: // ��ͣ
        {
            read_motor_state_4();
            read_motor_state_5();
            if (0x40 == ((ID3_Rx_Data_5 & ID2_Rx_Data_5) & 0x40))
            {
                if ((1 == ID3_Rx_Data_4) && (2 == ID2_Rx_Data_4))
                {
                    jerk_process = 0;
                    flag_big_bin_and_triangle = 1;
                    printf("********************(1 == ID3_Rx_Data_4) && (2 == ID2_Rx_Data_4)*******************\r\n");
                }
                else
                {
                    if (1 == ID3_Rx_Data_4)
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
                        HAL_Delay(10);
                        printf("(1 == ID3_Rx_Data_4) && (2 == ID2_Rx_Data_4) -> big_bin over\r\n");
                    }
                    if (2 == ID2_Rx_Data_4)
                    {
                        CAN_Tx_Data[0] = 0x2B;
                        CAN_Tx_Data[1] = 0x40;
                        CAN_Tx_Data[2] = 0x60;
                        CAN_Tx_Data[3] = 0x00;
                        CAN_Tx_Data[4] = 0x02;
                        CAN_Tx_Data[5] = 0x00;
                        CAN_Tx_Data[6] = 0x00;
                        CAN_Tx_Data[7] = 0x00;
                        CAN1_Tx(ID2, CAN_Tx_Data);
                        HAL_Delay(10);
                        printf("(1 == ID3_Rx_Data_4) && (2 == ID2_Rx_Data_4) -> triangle over\r\n");
                    }
                    //									jerk_process = 24;
                    printf("********************!((1 == ID3_Rx_Data_4) || (2 == ID2_Rx_Data_4))*******************\r\n");
                }
            }
            else
            {
                if ((0x40 != (ID3_Rx_Data_5 & 0x40)) && (0x40 != (ID2_Rx_Data_5 & 0x40)))
                {
                    if ((1 == ID3_Rx_Data_4) && (2 == ID2_Rx_Data_4))
                    {
                        printf("big_bin_and_triangle over\r\n");
                        jerk_process = 0;
                        flag_big_bin_and_triangle = 1;
                    }
                    else
                    {
                        if ((1 != ID3_Rx_Data_4) || (2 != ID2_Rx_Data_4))
                        {
                            jerk_process = 1;
                        }
                        else
                        {
                            printf("big_bin_and_triangle erroe !!!\r\n");
                            jerk_process = 0;
                            flag_big_bin_and_triangle = 1;
                        }
                    }
                }
                if ((0x40 != (ID3_Rx_Data_5 & 0x40)) && (0x40 == (ID2_Rx_Data_5 & 0x40)))
                {
                    if (1 == ID3_Rx_Data_4)
                    {
                        printf("(0x40 != (ID3_Rx_Data_5 & 0x40)) && (0x40 == (ID2_Rx_Data_5 & 0x40)) -> big_bin over\r\n");
                    }
                    else
                    {
                        if (2 == ID2_Rx_Data_4)
                        {
                            CAN_Tx_Data[0] = 0x2B;
                            CAN_Tx_Data[1] = 0x40;
                            CAN_Tx_Data[2] = 0x60;
                            CAN_Tx_Data[3] = 0x00;
                            CAN_Tx_Data[4] = 0x02;
                            CAN_Tx_Data[5] = 0x00;
                            CAN_Tx_Data[6] = 0x00;
                            CAN_Tx_Data[7] = 0x00;
                            CAN1_Tx(ID2, CAN_Tx_Data);
                            HAL_Delay(10);
                            printf("(0x40 != (ID3_Rx_Data_5 & 0x40)) && (0x40 == (ID2_Rx_Data_5 & 0x40)) -> triangle over\r\n");
                        }
                    }
                }
                if ((0x40 == (ID3_Rx_Data_5 & 0x40)) && (0x40 != (ID2_Rx_Data_5 & 0x40)))
                {
                    if (1 == ID3_Rx_Data_4)
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
                        HAL_Delay(10);
                        printf("(0x40 == (ID3_Rx_Data_5 & 0x40)) && (0x40 != (ID2_Rx_Data_5 & 0x40)) -> big_bin over\r\n");
                    }
                }
            }
            break;
        }
        }
    } while (0 == flag_big_bin_and_triangle);
    printf("********************while(0 == flag_big_bin_and_triangle)*******************\r\n");
    flag_big_bin_and_triangle = 0;
    test_flag = 0;
}
// location
void test_location_1(uint32_t ID, uint8_t dire) // �۵���
{
    switch (jerk_process)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != ((ID2_Rx_Data_5 | ID3_Rx_Data_5) & 0x40))
        { // 2�ţ�3�ŵ�����ھ�ֹ״̬
            // �ջ�
            if (dire == 2)
            {
                read_motor_state_4();
                if (0x01 == ID1_Rx_Data_4) // ��ԭ��λ�ã���������
                {
                    jerk_process = 0;
                    test_flag = 0;
                }
                else
                { // ����ԭ��λ�ã�ִ���ջض���
                    jerk_process = 13;
                }
            }
            // �Ƴ�
            if (dire == 1)
            {
                read_motor_state_4();
                if (0x01 == ID2_Rx_Data_4 || 0x02 == ID2_Rx_Data_4) // ���ǲֹ�λ
                {
                    if (0x02 == ID1_Rx_Data_4 || (!(0x02 == ID3_Rx_Data_4))) // �ڲտ�λ��or��ֲ����¶�λ�ã���������
                    {
                        jerk_process = 0;
                        test_flag = 0;
                    }
                    else
                    { // ���ڲտ�λ�ã�ִ���Ƴ�����
                        jerk_process = 2;
                    }
                }
                else
                { // ���ǲ�δ��λ
                    jerk_process = 0;
                    test_flag = 0;
                }
            }
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
        }
        break;
    }
    case 2:
    {
        location_control_run(ID, 2000, 100, 800, 104000, 1);
        jerk_process = 23;
        break;
    }
    case 13: // RETURN
    {
        location_control_run(ID, 100, 500, 1000, 104000, 1);
        jerk_process = 23;
        break;
    }
    case 23: // �ж��˶�����
    {
        if (1 == dire)
        {
            jerk_process = 24;
        }
        if (2 == dire)
        {
            jerk_process = 25;
        }
        break;
    }
    case 24:
    {
        read_motor_state_5();
        if (0x40 == (ID1_Rx_Data_5 & 0x40))
        {
            read_motor_state_4();
            if (2 == ID1_Rx_Data_4)
            {
                jerk_process = 26;
            }
            else
            {
                jerk_process = 24;
            }
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
        }
        break;
    }
    case 25:
    {
        read_motor_state_5();
        if (0x40 == (ID1_Rx_Data_5 & 0x40))
        {
            read_motor_state_4();
            if (1 == ID1_Rx_Data_4)
            {
                jerk_process = 26;
            }
            else
            {
                jerk_process = 25;
            }
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
        }
        break;
    }
    case 26: // ��ͣ
    {
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x02;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        printf("door over\r\n");
        jerk_process = 0;
        test_flag = 0;
        break;
    }
        //	default:
        //		break;
    }
}
void test_location_2(uint32_t ID, uint8_t dire) // coffee control
{
    flag_coffee = true;
    while (flag_coffee)
    {
        read_motor_state_4();
        read_motor_state_5();
        switch (jerk_process)
        {
        case 1:
        {
            if (0x40 != (ID3_Rx_Data_5 & 0x40))
            {
                if (dire == 1)
                {
                    if (0x01 == ID3_Rx_Data_4) // upper limit
                    {
                        printf("********************coffee_up_no_over*******************\r\n");
                        jerk_process = 0;
                        flag_coffee = false;
                        test_flag = 0;
                    }
                    else
                    { //
                        jerk_process = 2;
                    }
                }
                if (dire == 2)
                {
                    if (0x02 == ID3_Rx_Data_4)
                    {
                        jerk_process = 0;
                        flag_coffee = false;
                        test_flag = 0;
                    }
                    else
                    { // �۵���δ��λ
                        jerk_process = 3;
                    }
                }
            }
            else
            {
                jerk_process = 0;
                flag_coffee = false;
                test_flag = 0;
            }
            break;
        }

        case 2:
        {
            if (2 == ID3_Rx_Data_4)
            {
                jerk_process = 26;
                break;
            }

            location_control_run(ID, 500, 1500, 200, 240000, 1);
            jerk_process = 23;
            break;
        }
        case 3:
        {
            if (1 == ID3_Rx_Data_4)
            {
                jerk_process = 26;
                break;
            }
            location_control_run(ID, 500, 1500, 200, -240000, 1);
            jerk_process = 23;
            break;
        }
        case 23: // �ж��˶�����
        {
            if (1 == dire)
            {
                jerk_process = 24;
            }
            if (2 == dire)
            {
                jerk_process = 25;
            }
            break;
        }
        case 24:
        {
            if (0x40 == (ID3_Rx_Data_5 & 0x40))
            {
                coffee_motor_control();
                if (test_flag != 111)
                {
                    if (1 == ID3_Rx_Data_4)
                    {
                        jerk_process = 26;
                    }
                    else
                    {
                        jerk_process = 24;
                    }
                }
                else
                {
                    jerk_process = 26;
                }
            }
            else
            {
                jerk_process = 0;
                flag_coffee = false;
                test_flag = 0;
            }
            break;
        }
        case 25:
        {
            if (0x40 == (ID3_Rx_Data_5 & 0x40))
            {
                coffee_motor_control();
                if (test_flag != 111)
                {
                    if (2 == ID3_Rx_Data_4)
                    {
                        jerk_process = 26;
                    }
                    else
                    {
                        jerk_process = 25;
                    }
                }
                else
                {
                    jerk_process = 26;
                }
            }
            else
            {
                jerk_process = 0;
                flag_coffee = false;
                test_flag = 0;
            }
            break;
        }
        case 26: // ��ͣ
        {
            CAN_Tx_Data[0] = 0x2B;
            CAN_Tx_Data[1] = 0x40;
            CAN_Tx_Data[2] = 0x60;
            CAN_Tx_Data[3] = 0x00;
            CAN_Tx_Data[4] = 0x02;
            CAN_Tx_Data[5] = 0x00;
            CAN_Tx_Data[6] = 0x00;
            CAN_Tx_Data[7] = 0x00;
            CAN1_Tx(ID, CAN_Tx_Data);
            HAL_Delay(10);
            printf("triangle over\r\n");
            jerk_process = 0;
            flag_coffee = false;
            test_flag = 0;
            break;
        }

            //	default:
            //		break;
        }
    }
}
void test_location_3(uint32_t ID, uint8_t dire) // ���
{
    switch (jerk_process)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != ((ID1_Rx_Data_5 | ID2_Rx_Data_5) & 0x40))
        { // 1��,2�ŵ�����ھ�ֹ״̬
            // ������У������ջأ�
            if (dire == 1)
            {
                read_motor_state_4();
                if (0x01 == ID3_Rx_Data_4 || 0x01 != ID1_Rx_Data_4) // ��ԭ��λ��or�۵���û�ջأ���������
                {
                    jerk_process = 0;
                    test_flag = 0;
                }
                else
                { // ����ԭ��λ�ã�ִ�����ж���
                    jerk_process = 13;
                }
            }
            // ������У������Ƴ���
            if (dire == 2)
            {
                read_motor_state_4();
                if (0x02 == ID3_Rx_Data_4) // �ڵ׶�λ�ã���������
                {
                    jerk_process = 0;
                    test_flag = 0;
                }
                else
                { // ���ڵ׶�λ�ã�ִ�����ж���
                    jerk_process = 2;
                }
            }
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
        }
        break;
    }
    case 2:
    {
        location_control_run(ID, 1000, 2000, 480, 220000, 1);
        jerk_process = 23;
        break;
    }
    case 13: // RETURN
    {        // acceleration time 2000ms 6083h
        location_control_run(ID, 2000, 1000, 900, -210000, 1);
        jerk_process = 23;
        break;
    }
    case 23: // �ж��˶�����
    {
        if (1 == dire)
        {
            jerk_process = 24;
        }
        if (2 == dire)
        {
            jerk_process = 25;
        }
        break;
    }
    case 24:
    {
        read_motor_state_5();
        if (0x40 == (ID3_Rx_Data_5 & 0x40))
        {
            read_motor_state_4();
            if (1 == ID3_Rx_Data_4)
            {
                jerk_process = 26;
            }
            else
            {
                jerk_process = 24;
            }
        }
        else
        {
            if (1 == ID3_Rx_Data_4)
            {
                jerk_process = 0;
                test_flag = 0;
                printf("1  0x40 != (ID3_Rx_Data_5 & 0x40) -> big_bin over\r\n");
            }
            else
            {
                jerk_process = 1;
                printf("1  big_bin over go case 1\r\n");
            }
        }
        break;
    }
    case 25:
    {
        read_motor_state_5();
        read_motor_state_4();
        if (0x40 == (ID3_Rx_Data_5 & 0x40))
        {
            if (2 == ID3_Rx_Data_4)
            {
                jerk_process = 26;
            }
            else
            {
                jerk_process = 25;
            }
        }
        else
        {
            if (2 == ID3_Rx_Data_4)
            {
                jerk_process = 0;
                test_flag = 0;
                printf("2  0x40 != (ID3_Rx_Data_5 & 0x40) -> big_bin over\r\n");
            }
            else
            {
                jerk_process = 1;
                printf("2  big_bin over go case 1\r\n");
            }
        }
        break;
    }
    case 26: // ��ͣ
    {
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x02;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        printf("big_bin over\r\n");
        jerk_process = 0;
        test_flag = 0;
        break;
    }

        //	default:
        //		break;
    }
}

// origin ע����ID
void test_origin(uint32_t ID)
{
    switch (jerk_process)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != ((ID1_Rx_Data_5 | ID2_Rx_Data_5 | ID3_Rx_Data_5) & 0x40))
        {
            read_motor_state_4();
            if (0x01 == ID3_Rx_Data_4)
            {
                jerk_process = 0;
                test_flag = 0;
            }
            else
            {
                jerk_process = 11;
            }
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
        }
        break;
    }
    case 11: // ��X0Ϊԭ���ź�

    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        // HAL_Delay(1000);
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40; // 不是40吗？
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x00;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data); // X0ԭ���ź�
        HAL_Delay(10);
        jerk_process = 2;
        break;
    }
    case 2: // �����Ҹ���λģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x98;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x12;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 3;
        break;
    }
    case 3: // ��ԭ���ٶ�480r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x01;
        CAN_Tx_Data[4] = 0x78;
        CAN_Tx_Data[5] = 0x01;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 4;
        break;
    }
    case 4: // ��ѯ�ٶ�30r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x02;
        CAN_Tx_Data[4] = 0x3C;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 5;
        break;
    }

    case 5: // ����ʱ��30ms
    {

        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x9A;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x64;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 6;
        break;
    }
    case 6: // ��ԭ��ģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x60;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x06;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 7;
        break;
    }
    case 7: // �л�������״̬��
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
        HAL_Delay(10);
        jerk_process = 8;
        break;
    }
    case 8:
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
        HAL_Delay(10);
        jerk_process = 9;
        break;
    }
    case 9:
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
        HAL_Delay(10);
        jerk_process = 10;
        break;
    }
    case 10: // ���ͻ�ԭ���˶�ָ��
    {
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1F;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 0;
        test_flag = 0;
        break;
    }
    }
}

void origin_1(uint32_t ID)
{
    switch (jerk_process)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != ((ID2_Rx_Data_5 | ID3_Rx_Data_5) & 0x40))
        {
            jerk_process = 11;
        }
        else
        {
            jerk_process = 0;
            test_flag = 0;
            printf("back fail!\r\n");
        }
        break;
    }
    case 11: // ��X0Ϊ����λ�ź�
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x30;
        CAN_Tx_Data[2] = 0x20;
        CAN_Tx_Data[3] = 0x02; // X0����
        CAN_Tx_Data[4] = 0x03; // ����λ�ź�
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data); // X0����λ�ź�
        HAL_Delay(10);
        jerk_process = 2;
        break;
    }
    case 2: // �����Ҹ���λģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x98;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x12;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 3;
        break;
    }
    case 3: // ��ԭ���ٶ�60r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x01;
        CAN_Tx_Data[4] = 0x3C;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 4;
        break;
    }
    case 4: // ��ѯ�ٶ�100RPM
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x64;
        CAN_Tx_Data[4] = 0x00;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 5;
        break;
    }

    case 5: // ����ʱ��30ms
    {

        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x9A;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1E;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 6;
        break;
    }
    case 6: // ��ԭ��ģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x60;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x06;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 7;
        break;
    }
    case 7: // �л�������״̬��
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
        HAL_Delay(10);
        jerk_process = 8;
        break;
    }
    case 8:
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
        HAL_Delay(10);
        jerk_process = 9;
        break;
    }
    case 9:
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
        HAL_Delay(10);
        jerk_process = 10;
        break;
    }
    case 10: // ���ͻ�ԭ���˶�ָ��
    {
        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1F;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process = 0;
        jerk_process2 = 1;
        jerk_process3 = 1;
        //					test_flag = 0;
        printf("door_over\r\n");
        break;
    }
    }
}
void origin_2(uint32_t ID)
{
    switch (jerk_process2)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != (ID1_Rx_Data_5 & 0x40))
        {
            read_motor_state_4();
            if (0x01 == ID1_Rx_Data_4)
            {
                jerk_process2 = 11;
            }
            else
            {
                jerk_process = 1;
                jerk_process2 = 0;
            }
        }
        else
        {
            jerk_process2 = 1;
        }
        break;
    }
    case 11: // ��X1Ϊ����λ�ź�
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x30;
        CAN_Tx_Data[2] = 0x20;
        CAN_Tx_Data[3] = 0x03; // ѡ��X1����
        CAN_Tx_Data[4] = 0x02; // ѡ������λ�ź�
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data); // X1����λ�ź�
        HAL_Delay(10);
        jerk_process2 = 2;
        break;
    }
    case 2: // ����������λģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x98;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x11;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 3;
        break;
    }
    case 3: // ��ԭ���ٶ�100r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x01;
        CAN_Tx_Data[4] = 0x64;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 4;
        break;
    }
    case 4: // ��ѯ�ٶ�30r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x02;
        CAN_Tx_Data[4] = 0x1E;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 5;
        break;
    }

    case 5: // ����ʱ��30ms
    {

        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x9A;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1E;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 6;
        break;
    }
    case 6: // ��ԭ��ģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x60;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x06;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 7;
        break;
    }
    case 7: // �л�������״̬��
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
        HAL_Delay(10);
        jerk_process2 = 8;
        break;
    }
    case 8:
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
        HAL_Delay(10);
        jerk_process2 = 9;
        break;
    }
    case 9:
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
        HAL_Delay(10);
        jerk_process2 = 10;
        break;
    }
    case 10: // ���ͻ�ԭ���˶�ָ��
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1F;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process2 = 0;
        jerk_process3 = 1;
        printf("triangle over\r\n");
        break;
    }
    }
}
void origin_3(uint32_t ID)
{
    jerk_process3 = 11;
    switch (jerk_process3)
    {
    case 1:
    {
        read_motor_state_5();
        if (0x40 != (ID1_Rx_Data_5 & 0x40))
        {
            read_motor_state_4();
            if (0x01 == ID1_Rx_Data_4)
            {
                jerk_process3 = 11;
                //								test_flag = 0;
            }
            else
            {
                jerk_process3 = 0;
                jerk_process = 1;
            }
        }
        else
        {
            jerk_process3 = 1;
        }
        break;
    }
    case 11: // ��X0Ϊ����λ�ź�
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x30;
        CAN_Tx_Data[2] = 0x20;
        CAN_Tx_Data[3] = 0x02; // X0����
        CAN_Tx_Data[4] = 0x03; // ����λ�ź�
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data); // X0����λ�ź�
        HAL_Delay(10);
        jerk_process3 = 2;
        break;
    }
    case 2: // �����Ҹ���λģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x98;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x12;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 3;
        break;
    }
    case 3: // ��ԭ���ٶ�60r/min
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x01;
        CAN_Tx_Data[4] = 0x3C;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 4;
        break;
    }
    case 4: // ��ѯ�ٶ�100RPM
    {
        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x99;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x64;
        CAN_Tx_Data[4] = 0x00;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 5;
        break;
    }

    case 5: // ����ʱ��30ms
    {

        CAN_Tx_Data[0] = 0x23;
        CAN_Tx_Data[1] = 0x9A;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1E;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 6;
        break;
    }
    case 6: // ��ԭ��ģʽ
    {

        CAN_Tx_Data[0] = 0x2F;
        CAN_Tx_Data[1] = 0x60;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x06;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 7;
        break;
    }
    case 7: // �л�������״̬��
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
        HAL_Delay(10);
        jerk_process3 = 8;
        break;
    }
    case 8:
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
        HAL_Delay(10);
        jerk_process3 = 9;
        break;
    }
    case 9:
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
        HAL_Delay(10);
        jerk_process3 = 10;
        break;
    }
    case 10: // ���ͻ�ԭ���˶�ָ��
    {

        CAN_Tx_Data[0] = 0x2B;
        CAN_Tx_Data[1] = 0x40;
        CAN_Tx_Data[2] = 0x60;
        CAN_Tx_Data[3] = 0x00;
        CAN_Tx_Data[4] = 0x1F;
        CAN_Tx_Data[5] = 0x00;
        CAN_Tx_Data[6] = 0x00;
        CAN_Tx_Data[7] = 0x00;
        CAN1_Tx(ID, CAN_Tx_Data);
        HAL_Delay(10);
        jerk_process3 = 0;
        test_flag = 0;
        printf("big_bin_over\r\n");
        break;
    }
    }
}
// ��������
void test_remove_alarm(void)
{
    CAN_Tx_Data[0] = 0x2B;
    CAN_Tx_Data[1] = 0x40;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x80;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    CAN1_Tx(0x601, CAN_Tx_Data);
    HAL_Delay(10);
    CAN1_Tx(0x602, CAN_Tx_Data);
    HAL_Delay(10);
    CAN1_Tx(0x603, CAN_Tx_Data);
    HAL_Delay(10);
    jerk_process = 0;
}
/*****************
��ȡ����˶�״̬
CANx_Rx_Data[4] == 0x01,X0��Ӧ���ź�
X0,X1�ź�
********************/
void read_motor_state_4(void)
{
    CAN_Tx_Data[0] = 0x40;
    CAN_Tx_Data[1] = 0x03;
    CAN_Tx_Data[2] = 0x20;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x00;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    //		CAN1_Tx(0x601, CAN_Tx_Data);//��Xi
    //		HAL_Delay(10);
    //		ID1_Rx_Data_4 = CAN_Rx_Data[4];
    //		printf("ID1_Rx_Data_4 = %x\r\n",ID1_Rx_Data_4);

    //		CAN1_Tx(0x602, CAN_Tx_Data);//��Xi
    //		HAL_Delay(10);
    //		ID2_Rx_Data_4 = CAN_Rx_Data[4];
    //		printf("ID2_Rx_Data_4 = %x\r\n",ID2_Rx_Data_4);

    CAN1_Tx(0x603, CAN_Tx_Data); // ��Xi
    HAL_Delay(10);
    ID3_Rx_Data_4 = CAN_Rx_Data[4];
    //		printf("ID3_Rx_Data_4 = %x\r\n",ID3_Rx_Data_4);
}
/*****************
��ȡ����˶�״̬
CANx_Rx_Data[5] == 0x40,�˶���
CANx_Rx_Data   bit8 == 0,��ֹ
********************/
void read_motor_state_5(void)
{
    CAN_Tx_Data[0] = 0x40;
    CAN_Tx_Data[1] = 0x41;
    CAN_Tx_Data[2] = 0x60;
    CAN_Tx_Data[3] = 0x00;
    CAN_Tx_Data[4] = 0x00;
    CAN_Tx_Data[5] = 0x00;
    CAN_Tx_Data[6] = 0x00;
    CAN_Tx_Data[7] = 0x00;
    //		CAN1_Tx(0x601, CAN_Tx_Data);//��Xi
    //		HAL_Delay(10);
    //		ID1_Rx_Data_5 = CAN_Rx_Data[5];
    //		printf("ID1_Rx_Data_5 = %x\r\n",ID1_Rx_Data_5);

    //		CAN1_Tx(0x602, CAN_Tx_Data);//��Xi
    //		HAL_Delay(10);
    //		ID2_Rx_Data_5 = CAN_Rx_Data[5];
    //		printf("ID2_Rx_Data_5 = %x\r\n",ID2_Rx_Data_5);

    CAN1_Tx(0x603, CAN_Tx_Data); // ��Xi
    HAL_Delay(10);
    ID3_Rx_Data_5 = CAN_Rx_Data[5];
    //		printf("ID3_Rx_Data_5 = %x\r\n",ID3_Rx_Data_5);
}
