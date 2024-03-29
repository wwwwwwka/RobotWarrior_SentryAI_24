#include "uart6.h"
#include "stm32f4xx.h"

//UART 读数据缓冲区
__IO uint8_t uartBuf[128];
__IO int head = 0;
__IO int tail  = 0;

void Uart_Flush(void)
{
	head = tail = 0;
}

int16_t Uart_Read(void)
{
	if(head!=tail){
		uint8_t Data = uartBuf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}


//UART6_Tx PG14    UART6_Rx PG9
void Actuator_uart6_init(uint32_t baudRate)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    USART_DeInit(USART6);

    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART_InitStructure);

    //中断配置
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_NVIC;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART6, ENABLE);

}


void USART6_IRQHandler(void)
{
	uartBuf[tail] = USART_ReceiveData(USART6);
	tail = (tail+1)%128;
}

void Uart_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	for(i=0; i<len; i++){
		USART_SendData(USART6, buf[i]);
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
	}
	while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
}

