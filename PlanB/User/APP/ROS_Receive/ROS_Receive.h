#ifndef ROS_RECEIVE_H
#define ROS_RECEIVE_H

#include "main.h"

#define ROS_START_BYTE 0x5A
#define ROS_RX_BUF_NUM 48u
#define ROS_FRAME_LENGTH 24u

//陀螺仪数据发送周期 ms
#define IMU_SEND_TIME 20

typedef struct
{
	//自瞄使用数据
	fp32 shoot_yaw;
	fp32 shoot_pitch;
	fp32 shoot_depth;
	
	//导航使用数据
	Float_Byte vx_set;
	Float_Byte vy_set;
	Float_Byte wz_set;
	uint8_t mode;
	uint8_t state;
	
}ROS_Msg_t;

void imuSendTask(void *pvParameters);
extern void Get_Chassis_Msg(fp32 *vx_set,fp32 *vy_set,fp32 *angle_set);
void ROS_Init(void);
const ROS_Msg_t *get_ROS_Msg_point(void);
void Get_Gimbal_Angle(fp32 *yaw_add,fp32 *pitch_add);

#endif
