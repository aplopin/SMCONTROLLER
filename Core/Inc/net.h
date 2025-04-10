#ifndef INC_UDP_CLIENT_H_
#define INC_UDP_CLIENT_H_

#include "stdio.h"
#include "string.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

/* Структура UDP протокола */
struct udp_pcb *upcb;

/* Буфер принимаемых данных */
char rxBuf[128] = {0};

/* Счетчик принятых сообщений */
int counter = 0;

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

void udpSocketInit(void);
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpReceiveHandler(void);
void udpClientSend(char *buf);

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

/** Функция инициализации UDP сетевого интерфейса
 */
void udpSocketInit(void)
{
	/* Переменная ошибок работы UDP интерфейса */
	err_t err;

	/* Новый UDP управляющий блок */
	upcb = udp_new();

	/* Связать IP адрес и порт */
	ip_addr_t myIPaddr;

	/* IP адрес устройства */
	IP_ADDR4(&myIPaddr, 192, 168, 1, 156);
	udp_bind(upcb, &myIPaddr, 1556);

	/* Конфигурация IP адреса и порта назначения - ПК */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 1, 2);
	err = udp_connect(upcb, &DestIPaddr, 1555);

	if (err == ERR_OK)
	{
		/* Передача указателя на функцию обработчика входящих пакетов в структуру UDP интерфейса */
		udp_recv(upcb, udpReceiveCallback, NULL);
	}
}

/** Функция обработки принятых пакетов UDP
 */
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	strncpy(rxBuf, (char *)p->payload, p->len);
	counter ++;
	pbuf_free(p);

	char data[256];
	sprintf(data, "STM32: number of message received = %d\n", counter);

	udpClientSend(data);

	/* Вызов обработчик принятных сообщений */
	udpReceiveHandler();
}

/** Функция отправки сообщения по UDP
 */
void udpClientSend(char *buf)
{
	struct pbuf *txBuf;
	char data[256];

	int len = sprintf(data, "%s\n", (char *)buf);

	/* Выделить память под данные в буфере */
	txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

	if (txBuf != NULL)
	{
		pbuf_take(txBuf, data, len);
		udp_send(upcb, txBuf);
    	pbuf_free(txBuf);
	}
}

//void udp_receive_handler(void)
//{
//	char txBuf[128]; // локальный буфер для передачи по UDP
//	char hlBuf[128]; // буфер обработчика прерывания принятых сообщений по UDP
//	strncpy(hlBuf, rxBuf, sizeof rxBuf);
//
//	udp_client_send(rxBuf);
//
//	if(hlBuf[0] == 'G') // Если поступили g-команды
//	{
//		strcpy(TaskBuffer[counter_gcommand], hlBuf);
//		counter_gcommand ++;
//	}
//	if(hlBuf[0] == 'B') // Символ начала исполнения программы
//	{
//		if(counter_gcommand == 0) // Проверка на начилие пустового буфера пришедших команд
//		{
//			snprintf(txBuf, sizeof txBuf, "\nERROR!\nNumber of g-commands - 0\n");
//			udp_client_send(txBuf);
//		}
//		else
//		{
//			flag_START = 1;
//			snprintf(txBuf, sizeof txBuf, "\nThe process has started!\nNumber of g-commands - %d\n", counter_gcommand);
//			udp_client_send(txBuf);
//		}
//	}
//	if(hlBuf[0] == 'E')
//	{
//		HAL_TIM_Base_Stop_IT(&htim1);
//		snprintf(txBuf, sizeof txBuf, "\nThe process is stopped!\n");
//		udp_client_send(txBuf);
//
//		spi_pilot_switch();
//
//		memset(TaskBuffer[0], 0, 128);
//		counter_gcommand = 0;
//	}
//}

#endif /* INC_UDP_CLIENT_H_ */
