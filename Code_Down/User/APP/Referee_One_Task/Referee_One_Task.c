#include "Referee_One_Task.h"
#include "protocol.h"
#include "referee.h"
#include "referee_usart.h"
#include "main.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

CanTxMsg TxMessage_ID_Game;

static void CAN_ID_Game_Process(void);

void Referee_One_Task(void)
{
   while(1)
   {
    CAN_ID_Game_Process();
    vTaskDelay(200);
   }
}

static void CAN_ID_Game_Process(void)
{
    TxMessage_ID_Game.StdId = CAN_ID_GAME_ID;
    TxMessage_ID_Game.IDE = CAN_ID_STD;
    TxMessage_ID_Game.RTR = CAN_RTR_DATA;
    TxMessage_ID_Game.DLC = 0x08;
    TxMessage_ID_Game.Data[0] = get_robot_id();
    TxMessage_ID_Game.Data[1] = get_game_start();
    TxMessage_ID_Game.Data[2] = 0;
    TxMessage_ID_Game.Data[3] = 0;
    TxMessage_ID_Game.Data[4] = 0;
    TxMessage_ID_Game.Data[5] = 0;
    TxMessage_ID_Game.Data[6] = 0;
    TxMessage_ID_Game.Data[7] = 0;

    CAN_Transmit(ID_GAME_CAN, &TxMessage_ID_Game);
}