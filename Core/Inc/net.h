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

void udpSocketInit(void);
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpClientSend();

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
	sprintf(data, "UDP: Message has been delivered. Echo: counter = %d;\n", counter);

	udpClientSend(data);
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




#endif /* INC_UDP_CLIENT_H_ */
