#include "qrcode.h"

uint8_t QRRxbuff = 0;
uint8_t QRsave[3] = {0};
uint8_t qrRecsta = 0;
void qrcode_init(void)
{
	HAL_UART_Receive_IT(&QRCODE_UART,&QRRxbuff,1);
}

void QR_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	QRsave[2] = QRsave[1];
	QRsave[1] = QRsave[0];
	QRsave[0] = QRRxbuff;
	qrRecsta = 1;
	HAL_UART_Receive_IT(&QRCODE_UART,&QRRxbuff,1);
}

uint8_t *qrcode_getsta(void)
{
	if(qrRecsta)
	{
		qrRecsta = 0;
		return QRsave;
	}
	else
	{
		return NULL;
	}
}

uint8_t qrcode_getdata(void)
{
	if(QRsave[2] == QRsave[1]&&QRsave[1] == QRsave[0])
	{
		return QRsave[0];
	}
	else
	{
		for(uint8_t i = 0;i < 3;i++)
		{
			QRsave[i] = 0;
		}
		return 0;
	}
}

uint8_t qrcode_getRxbuff(void)
{
	return QRsave[0];
}
