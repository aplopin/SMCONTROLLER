/**
  ******************************************************************************
  * @Файл    	net.h
  * @Автор  	PromisLab
  * @Описание   Этот файл описывает прототипы функций сетевого интерфейса UDP
  * 		 	с использованием библиотека lwip.h
  ******************************************************************************
  *
  */

#ifndef INC_NET_H_
#define INC_NET_H_

#include <stdio.h>
#include <string.h>

/* LwIP */
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "fifo.h"

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

void udpSocketInit(void);
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpClientSend(char *buf);

/* Функция описана в файле main.c */
void udpReceiveHandler(void);

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

#endif /* INC_NET_H_ */
