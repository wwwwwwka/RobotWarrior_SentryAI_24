/**
  ****************************RM Warrior 2023****************************
  * @file       start_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/1/4         pxx              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"

#include "user_task.h"

#include "referee_usart_task.h"
#include "Referee_One_Task.h"
#include "Referee_Two_Task.h"


#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define User_TASK_PRIO 4
#define User_STK_SIZE 512
static TaskHandle_t UserTask_Handler;

#define REFEREE_ONE_TASK_PRIO 16
#define REFEREE_ONE_STK_SIZE 512
static TaskHandle_t Referee_One_Task_Handler;

#define REFEREE_TASK_PRIO 15
#define REFEREE_STK_SIZE 512
static TaskHandle_t RefreeTask_Handler;

#define REFEREE_TWO_TASK_PRIO 17
#define REFEREE_TWO_STK_SIZE  512
static TaskHandle_t Referee_Two_Task_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();//进入临界区

    xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);


    xTaskCreate((TaskFunction_t)referee_usart_task,
                (const char *)"RefereeTask",
                (uint16_t)REFEREE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REFEREE_TASK_PRIO,
                (TaskHandle_t *)&RefreeTask_Handler);

    xTaskCreate((TaskFunction_t)Referee_One_Task,
                (const char *)"Referee_One_Task",
                (uint16_t)REFEREE_ONE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REFEREE_ONE_TASK_PRIO,
                (TaskHandle_t *)&Referee_One_Task_Handler);

    xTaskCreate((TaskFunction_t)Referee_Two_Task,
                (const char *)"Referee_Two_Task",
                (uint16_t)REFEREE_TWO_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REFEREE_TWO_TASK_PRIO,
                (TaskHandle_t *)&Referee_Two_Task_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
