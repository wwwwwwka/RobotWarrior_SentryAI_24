/*
 * SCServo.c
 * ���ض��Ӳ���ӿڲ����
 * ����: 2022.3.29
 * ����: 
 */

#include "stm32f4xx.h"
#include "uart6.h"

uint32_t IOTimeOut = 5000;//���������ʱ
uint8_t wBuf[128];
uint8_t wLen = 0;

int readSCSTimeOut(unsigned char *nDat, int nLen, uint32_t TimeOut)
{
	int Size = 0;
	int ComData;
	uint32_t t_user = 0;
	while(1){
		ComData = Uart_Read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
		}
		if(Size>=nLen){
			break;
		}
		t_user++;
		if(t_user>TimeOut){
			break;
		}
	}
	return Size;
}

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
	int Size = 0;
	int ComData;
	uint32_t t_user = 0;
	while(1){
		ComData = Uart_Read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_user = 0;
		}
		if(Size>=nLen){
			break;
		}
		t_user++;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int writeByteSCS(unsigned char bDat)
{
	if(wLen<sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

//UART �������ݽӿ�
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

//�ȴ���������л�(Լ20us)
void nopDelay(void)
{
	uint16_t i = 300;
	while(i--);
}

//���ջ�����ˢ��
void rFlushSCS()
{
	nopDelay();
	Uart_Flush();
}

//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		Uart_Send(wBuf, wLen);
		wLen = 0;
	}
}
