#include "Referee_Two_Task.h"
#include "protocol.h"
#include "referee.h"
#include "referee_usart.h"
#include "main.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

CanTxMsg TxMessage;
uint8_t referee_Send_Flag=0;
Referee_Chassiser_t Referee_Chassiser;
Referee_Shooter_t Referee_Shooter;

static void CAN_Shooter_Chassiser_Process(Referee_Chassiser_t* Chassiser,Referee_Shooter_t* Shooter);
static void CAN_Shooter_Chassiser_Translate(uint32_t Strand_ID,uint8_t* Message);

void Referee_Two_Task(void)
{
   referee_Send_Flag=1;
   while(1)
   {
     CAN_Shooter_Chassiser_Process(&Referee_Chassiser,&Referee_Shooter);
     vTaskDelay(10);
   }
}  
        
static void CAN_Shooter_Chassiser_Process(Referee_Chassiser_t* Chassiser,Referee_Shooter_t* Shooter)
{
   uint8_t Message[8]={0};
   uint32_t Strand_ID=0;
    
   if(referee_Send_Flag==1)
   {
     Chassiser->referee_chassiser_power.float_data = get_chassis_power();
     Chassiser->referee_chassiser_power_buffer=get_chassis_buffer();
     Chassiser->referee_chassiser_power_limit=get_chassis_power_limit();

     Message[0]=Chassiser->referee_chassiser_power.byte_data[0];
     Message[1]=Chassiser->referee_chassiser_power.byte_data[1];
     Message[2]=Chassiser->referee_chassiser_power.byte_data[2];
     Message[3]=Chassiser->referee_chassiser_power.byte_data[3];
     Message[4]=(Chassiser->referee_chassiser_power_buffer)>>8;
     Message[5]=(Chassiser->referee_chassiser_power_buffer);
     Message[6]=(Chassiser->referee_chassiser_power_limit)>>8;
     Message[7]=(Chassiser->referee_chassiser_power_limit);

     Strand_ID=CAN_CHASSISER_ID;
     referee_Send_Flag=2;
   }else if(referee_Send_Flag==2)
   {
     Shooter->heat0_limit=get_shoot_heat0_limit();
     Shooter->heat0=get_shoot_heat0();
     Shooter->heat1_limit=get_shoot_heat1_limit();
     Shooter->heat1=get_shoot_heat1();

     Message[0]=(Shooter->heat0_limit)>>8;
     Message[1]=Shooter->heat0_limit;
     Message[2]=(Shooter->heat0)>>8;
     Message[3]=Shooter->heat0;
     Message[4]=(Shooter->heat1_limit)>>8;
     Message[5]=Shooter->heat1_limit;
     Message[6]=(Shooter->heat1)>>8;
     Message[7]=Shooter->heat1;
     
     Strand_ID=CAN_SHOOTER_ID;
     referee_Send_Flag=1;
   }

   CAN_Shooter_Chassiser_Translate(Strand_ID,Message);
}

static void CAN_Shooter_Chassiser_Translate(uint32_t Strand_ID,uint8_t* Message)
{
    TxMessage.StdId = Strand_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = Message[0];
    TxMessage.Data[1] = Message[1];
    TxMessage.Data[2] = Message[2];
    TxMessage.Data[3] = Message[3];
    TxMessage.Data[4] = Message[4];
    TxMessage.Data[5] = Message[5];
    TxMessage.Data[6] = Message[6];
    TxMessage.Data[7] = Message[7];

    CAN_Transmit(CHASSISER_SHOOTER_CAN, &TxMessage);
}