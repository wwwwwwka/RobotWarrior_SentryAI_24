#ifndef _Referee_Two_Task_H
#define _Referee_Two_Task_H

#include "main.h"

#define CHASSISER_SHOOTER_CAN CAN1
#define CAN_CHASSISER_ID 0X302
#define CAN_SHOOTER_ID 0X301

typedef struct
{
    Float_Byte referee_chassiser_power;
    uint16_t referee_chassiser_power_buffer;
    uint16_t referee_chassiser_power_limit;
} Referee_Chassiser_t;

typedef struct
{
    uint16_t heat1_limit;
    uint16_t heat1;
    uint16_t heat0_limit;
    uint16_t heat0;
} Referee_Shooter_t;

void Referee_Two_Task(void);

#endif