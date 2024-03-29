#include "CAN_Receive.h"

#include "stm32f4xx.h"
// #include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

// #include "Detect_Task.h"

//电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//机器人ID和比赛开始指令读取
#define get_referee_Id_Game_measuer(ptr, rx_message)                                                 \
    {                                                                                                \
        (ptr)->robot_id = (uint8_t)((rx_message)->Data[0]);                                          \
        (ptr)->game_start = (uint8_t)((rx_message)->Data[1]);                                        \
    }

//机器人枪口热量及枪口热量限制
#define get_referee_shooter_measuer(ptr, rx_message)                                                 \
    {                                                                                                \
        (ptr)->shooter_1_limit = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);     \
        (ptr)->shooter_1_heat = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);      \
        (ptr)->shooter_2_limit = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->shooter_2_heat = (uint16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);      \
    }

//机器人底盘功率限制相关
#define get_referee_chassiser_measuer(ptr, rx_message)                                               \
    {                                                                                                 \
        (ptr)->chassis_power.byte_data[0]=(rx_message)->Data[0];                                      \
        (ptr)->chassis_power.byte_data[1]=(rx_message)->Data[1];                                         \
        (ptr)->chassis_power.byte_data[2]=(rx_message)->Data[2];                                        \
        (ptr)->chassis_power.byte_data[3]=(rx_message)->Data[3];                                       \
        (ptr)->chassis_power_buffer = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->chassis_power_limit = (uint16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);      \
    }

//统一处理can接收函数
static void CAN_hook(CanRxMsg *rx_message);
//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4];

static CanTxMsg GIMBAL_TxMessage;

static referee_Id_Game_Data_t referee_Id_Game;

static referee_chassis_Data_t referee_chassis;

static referee_shooter_Data_t referee_shooter;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(&rx1_message);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(&rx2_message);
    }
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_solve(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif

//发送云台控制命令，其中rev为保留字节
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif

}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//发送底盘电机控制命令
/**
  * @brief          发送转矩电流到底盘电机
  * @author         pxx
  * @param[in]      motor1：0x201(M3508)   [-16384:16384]
  * @param[in]      motor2：0x202(M3508)   [-16384:16384]
  * @param[in]      motor3：0x203(M3508)   [-16384:16384]
  * @param[in]      motor4：0x204(M3508)   [-16384:16384]
  * @retval         返回空
  */
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
uint8_t get_robot_id(void)
{
    
    return referee_Id_Game.robot_id;
}

uint8_t get_game_start(void)
{
    return referee_Id_Game.game_start;
}
//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_yaw, rx_message);
        //记录时间
        //DetectHook(YawGimbalMotorTOE);
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_pit, rx_message);
        //DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case CAN_TRIGGER_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_trigger, rx_message);
        //记录时间
        //DetectHook(TriggerMotorTOE);
        break;
    }
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure(&motor_chassis[i], rx_message);
        //记录时间
        //DetectHook(ChassisMotor1TOE + i);
        break;
    }
    case CAN_ID_GAME_REFEREE_ID:
    {
        //处理裁判系统传来的机器人ID和比赛结束指令
        
        get_referee_Id_Game_measuer(&referee_Id_Game,rx_message);
        
        break;
    }
    case CAN_SHOOTER_REFEREE_ID:
    {
        get_referee_shooter_measuer(&referee_shooter, rx_message);
        
        break;
    }
    case CAN_CHASSIS_REFEREE_ID:
    {
        get_referee_chassiser_measuer(&referee_chassis, rx_message);
        
        break;
    }

    default:
    {
        break;
    }
    }
}
