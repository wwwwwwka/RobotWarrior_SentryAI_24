/**
  ****************************RM Warrior 2023****************************
  * @file       start_task.c/h
  * @brief      一个普通的心跳程序，如果程序没有问题，蓝灯以1Hz的频率闪烁
  *             同时也用来发送相关数值到上位机调参。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "User_Task.h"
#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"
#include "buzzer.h"

//#include "Detect_Task.h"
#include "referee.h"

#include "Kalman_Filter.h"
#include "bluetooth.h"
//#define user_is_error() toe_is_error(errorListLength)



void UserTask(void *pvParameters)
{
    while (1)
    {
        led_blue_on();
        led_red_on();
        led_green_on();
        vTaskDelay(500);
        led_blue_off();
        led_red_off();
        led_green_off();
        vTaskDelay(500);
    }
}


/**
  * @brief          蜂鸣器报警
  * @param[in]      num:响声次数
  * @retval         none
  */
void buzzer_warn(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(64, 20);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

