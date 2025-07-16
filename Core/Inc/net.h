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

/* Структура UDP команды, принятой по сети */
typedef struct
{
	/* Буфер принимаемых данных UDP команды */
	uint8_t buf[128];

	/* Размера блока принимаемых данных */
	uint8_t len;

	/* Код принятой команды */
	uint8_t code;

	/* Контрольная сумма принятого пакета */
	uint8_t checksum;

} UDP_RECEIVE_DATA_StructDef;

/* Структура UDP команд для отладки работы контроллера */
typedef struct
{
	/* Команды для переинициализации буфера G - кода */
	char* test1;
	char* test2;
	char* test3;
	char* test4;
	char* circle;
	char* example;

	/* Команды для запуска/паузы/возобновления/остановки работы станка */
	char* start;
	char* pause;
	char* resume;
	char* stop;

	/* Команды задания параметров планировщика - скорости/ускорения */
	char* setPlannerMaxSpeed; // 18 символов
	char* setPlannerAcceleration; // 22 символа

	/* Команды для ручного управления*/
	char* setDriverRunMode;

	char* setDriverAcceleration;
	char* setDriverAccelerationDeg;
	char* setDriverAccelerationMm;

	char* setDriverMaxSpeed;
	char* setDriverMaxSpeedDeg;
	char* setDriverMaxSpeedMm;

	char* setDriverTargetPos;
	char* setDriverTargetPosDeg;
	char* setDriverTargetPosMm;

} UDP_COMMANDS_StructDef;

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

void UDP_Init(void);
void udpReceiveDataStructClear(void);
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpClientSendResponse(uint8_t *buf, uint8_t len);
void udpClientSendResponseChar(char *buf);
uint8_t udpCalculateChecksum(uint8_t* buf, uint8_t len);

void udpReceiveHandler(void);

/* Функция должна быть описана в файле main.c */
void udpReceiveHandlerEcho(void);

void udpCommandFreeRun(void);
void udpCommandSendSoftVersion(void);
void udpCommandSetAxisParam(void);
void udpCommandGetAxisParam(void);
void udpCommandGetAxesState(void);
void udpCommandSetCurrentPosZero(void);
void udpCommandStartAxesZeroing(void);
void udpCommandStartProcess(void);
void udpCommandStopProcess(void);
void udpCommandPauseProcess(void);
void udpCommandResumeProcess(void);

/* --------------------------------------- Прототипы функций библиотеки net.h --------------------------------------- */

#endif /* INC_NET_H_ */
