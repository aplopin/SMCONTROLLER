/**
  ******************************************************************************
  * @Файл    	net.с
  * @Автор  	PromisLab
  * @Описание   Этот файл описывает реализацию функций сетевого интерфейса UDP
  * 		 	с использованием библиотека lwip.h, а также содержит
  * 		 	определение переменных и структур, необходимых для работы
  * 		 	библиотеки
  ******************************************************************************
  *
  */

#include "net.h"

/* Структура UDP протокола */
struct udp_pcb *upcb;

/* -------------------------------- ГЛОБАЛЬНЫЕ СЕТЕВЫЕ БУФЕРЫ ПРИНЯТЫХ ДАННЫХ -------------------------------- */

/* Буфер принимаемых строковых данных (отладка) */
char rxUdpCharBuf[256] = {0};

/* -------------------------------- ГЛОБАЛЬНЫЕ СЕТЕВЫЕ БУФЕРЫ ПРИНЯТЫХ ДАННЫХ -------------------------------- */

/* Счетчик принятых сообщений по UDP интерфейсу */
int32_t counter = 0;

/* Структура FIFO буфера сетевого интерфейса UDP */
FIFO_StructDef fifoNetBuf;

/* Статический буфер сетевого интерфейса UDP */
int8_t netBuf[FIFO_NET_SIZE];

/* Экземпляр структуры UDP команды */
UDP_RECEIVE_DATA_StructDef rxData;

/* Экземпляр структуры UDP команд (отладка) */
UDP_COMMANDS_StructDef udpcommands = {"test1", "test2", "test3", "test4", "circle", "example", \
				 	 	 	 	 	  "start", "pause", "resume", "stop", \
									  "setPlannerMaxSpeed", "setPlannerAcceleration", \
									  "setDriverRunMode", \
									  "setDriverAcceleration", "setDriverAccelerationDeg", "setDriverAccelerationMm", \
									  "setDriverMaxSpeed", "setDriverMaxSpeedDeg" , "setDriverMaxSpeedMm", \
									  "setDriverTargetPos", "setDriverTargetPosDeg", "setDriverTargetPosMm", \
									 };

/** Функция инициализации UDP сетевого интерфейса
 */
void UDP_Init(void)
{
	/* Переменная ошибок работы UDP интерфейса */
	err_t err;

	/* Новый UDP управляющий блок */
	upcb = udp_new();

	/* Связать IP адрес и порт */
	ip_addr_t myIPaddr;

	/* IP адрес устройства */
	IP_ADDR4(&myIPaddr, 192, 168, 1, 156);
	udp_bind(upcb, &myIPaddr, 1555);

	/* Конфигурация IP адреса и порта назначения - ПК */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 1, 2);
	err = udp_connect(upcb, &DestIPaddr, 1555);

	if (err == ERR_OK)
	{
		/* Передача указателя на функцию обработчика входящих пакетов в структуру UDP интерфейса */
		udp_recv(upcb, udpReceiveCallback, NULL);
	}

	/* Инициализация сткрутуры принимаемых данных */
	udpReceiveDataStructClear();

	/* Инициализация буфера сетевого интерфейса с размеров 64 * 2 байта = 128 байт*/
	fifoInit(&fifoNetBuf, netBuf, FIFO_NET_SIZE);
}

/** Инициализация структуры принимаемых сетевых данных
 *
 */
void udpReceiveDataStructClear()
{
	/* Очистка буфера принимаемых данных */
	for(uint8_t i = 0; i < 128; i ++)
	{
		rxData.buf[i] = 0;
	}

	/* Очистка полей структуры принимаемых данных */
	rxData.len = 0;
	rxData.code = 0;
	rxData.checksum = 0;
}

/** Функция обработки принятых пакетов UDP
 */
void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	uint8_t* ptr = (uint8_t*)p->payload;

	/* Копирование принимаемых данных в структуру данных */
	for(uint8_t i = 0; i < p->len; i ++)
	{
		rxData.buf[i] = *(ptr + i);
	}

	/* Копирование данных для работы со строками (отладка) */
	strncpy(rxUdpCharBuf, (char *)p->payload, p->len);

	counter ++;
	pbuf_free(p);

	/* Вызов обработчик принятных сообщений */
	udpReceiveHandler();
}

/** Функция отправки ответных сообщений по UDP в виде строки
 */
void udpClientSendResponseChar(char *buf)
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

/** Функция отправки ответов на сетевые команды команды
 */
void udpClientSendResponse(uint8_t *buf, uint8_t len)
{
	struct pbuf *txBuf;

	/* Выделить память под данные в буфере */
	txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

	if (txBuf != NULL)
	{
		pbuf_take(txBuf, buf, len);
		udp_send(upcb, txBuf);
    	pbuf_free(txBuf);
	}
}

/** Функция расчета контрольной суммы сетевого пакета данных
 */
uint8_t udpCalculateChecksum(uint8_t* buf, uint8_t len)
{
	uint8_t checksum = 0;

	for(uint8_t i = 0; i < len; i ++)
	{
		checksum += buf[i];
	}

	return 256 - checksum % 256;
}

void udpReceiveHandler(void)
{
	rxData.len = rxData.buf[0];
	rxData.code = rxData.buf[1];

	/* Запись принятого значения КС */
	rxData.checksum = rxData.buf[rxData.len - 1];

	switch(rxData.code)
	{
		/* Выдать версию ПО (код A0h) */
		case 0xA0:
		{
			udpCommandSendSoftVersion();
			break;
		}

		/* Задать параметры выбранной оси (код A1h) */
		case 0xA1:
		{
			udpCommandSetAxisParam();
			break;
		}

		/* Выдать параметры выбранной оси (код A2h) */
		case 0xA2:
		{
			udpCommandGetAxisParam();
			break;
		}

		/* Выдать текущее состояние осей устройства (код A3h) */
		case 0xA3:
		{
			udpCommandGetAxesState();
			break;
		}

		/* Обнулить значение текущей координаты осей (код 30h) */
		case 0x30:
		{
			udpCommandSetCurrentPosZero();
			break;
		}

		/* Начать движение к датчикам 0 для выбранных осей (код 31h) */
		case 0x31:
		{
			udpCommandStartAxesZeroing();
			break;
		}

		/* Ручной режим управления осями FreeRun (код 33h) */
		case 0x33:
		{
			udpCommandFreeRun();
			break;
		}

		/* Запустить рабочий цикл устройства (код – 10h) */
		case 0x10:
		{
			udpCommandStartProcess();
			break;
		}

		/* Полностью остановить цикл работы устройства (код – 11h) */
		case 0x11:
		{
			udpCommandStopProcess();
			break;
		}

		/* Приостановить рабочий цикл устройства (команда пауза) (код – 12h) */
		case 0x12:
		{
			udpCommandPauseProcess();
			break;
		}

		/* Продолжить выполнение рабочего цикла устройства (код – 13h) */
		case 0x13:
		{
			udpCommandResumeProcess();
			break;
		}

		default:
			udpReceiveHandlerEcho();
	}

	/* Очистка сткрутуры принимаемых данных */
	udpReceiveDataStructClear();
}

/**	Функция обработки UDP команды:
 * 	Ручной режим управления осями FreeRun (код 33h)
 */
void udpCommandFreeRun(void)
{
	uint8_t axisNum = rxData.buf[2];
	int32_t speed = 0;

	for(uint8_t i = 0; i < 4; i ++)
	{
		speed += rxData.buf[6 - i] << 8 * i;
	}

	/* Массив отправляемых серверу данных */
	uint8_t txData[5];

	txData[0] = 5;
	txData[1] = 0x33;
	txData[2] = axisNum;
	txData[3] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[4] = udpCalculateChecksum(txData, 5);

	udpClientSendResponse(txData, 5);
}

/**	Функция обработки UDP команды:
 * 	Выдать версию ПО (код A0h)
 */
void udpCommandSendSoftVersion(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[16];

	txData[0] = 16;
	txData[1] = 0xA0;
	txData[2] = 1; // Номер версии ПО 1 - пример

	char* date = {"Jul 15 2025\0"};

	for(uint8_t i = 3; i < 15; i ++)
	{
		txData[i] = date[i - 3];
	}

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[15] = udpCalculateChecksum(txData, 16);

	udpClientSendResponse(txData, 16);
}

/**	Функция обработки UDP команды:
 * 	Задать параметры выбранной оси (код A1h)
 */
void udpCommandSetAxisParam(void)
{
	uint8_t axisNum = rxData.buf[2];

	/* Массив отправляемых серверу данных */
	uint8_t txData[5];

	txData[0] = 16;
	txData[1] = 0xA1;
	txData[2] = axisNum;
	txData[3] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[4] = udpCalculateChecksum(txData, 5);

	udpClientSendResponse(txData, 5);
}

/**	Функция обработки UDP команды:
 * 	Выдать параметры выбранной оси (код A2h)
 */
void udpCommandGetAxisParam(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[32];

	txData[0] = 32;
	txData[1] = 0xA2;

	/* ДАННЫЕ ДЛЯ ОТЛАДКИ СВЯЗИ */
	for(uint8_t i = 2; i < 31; i ++)
	{
		txData[i] = i;
	}

	txData[31] = udpCalculateChecksum(txData, 32);

	udpClientSendResponse(txData, 32);
}

/**	Функция обработки UDP команды:
 * 	Выдать текущее состояние осей устройства (код A3h)
 */
void udpCommandGetAxesState(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[56];

	txData[0] = 56;
	txData[1] = 0xA3;

	/* ДАННЫЕ ДЛЯ ОТЛАДКИ СВЯЗИ */
	for(uint8_t i = 2; i < 55; i ++)
	{
		txData[i] = i;
	}

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[55] = udpCalculateChecksum(txData, 56);

	udpClientSendResponse(txData, 56);
}

/**	Функция обработки UDP команды:
 * 	Обнулить значение текущей координаты осей (код 30h)
 */
void udpCommandSetCurrentPosZero(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[5];

	txData[0] = 5;
	txData[1] = 0x30;
	txData[2] = 0b00000011;
	txData[3] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[4] = udpCalculateChecksum(txData, 5);

	udpClientSendResponse(txData, 5);
}

/**	Функция обработки UDP команды:
 * 	Начать движение к датчикам 0 для выбранных осей (код 31h)
 */
void udpCommandStartAxesZeroing(void)
{

}

/**	Функция обработки UDP команды:
 * 	Запустить рабочий цикл устройства (код – 10h)
 */
void udpCommandStartProcess(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[4];

	txData[0] = 4;
	txData[1] = 0x10;
	txData[2] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[3] = udpCalculateChecksum(txData, 4);

	udpClientSendResponse(txData, 4);
}

/**	Функция обработки UDP команды:
 * 	Полностью остановить цикл работы устройства (код – 11h)
 */
void udpCommandStopProcess(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[38];

	txData[0] = 38;
	txData[1] = 0x11;

	/* ДАННЫЕ ДЛЯ ОТЛАДКИ СВЯЗИ */
	for(uint8_t i = 2; i < 37; i ++)
	{
		txData[i] = i;
	}

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[37] = udpCalculateChecksum(txData, 38);

	udpClientSendResponse(txData, 38);
}

/**	Функция обработки UDP команды:
 * 	Приостановить рабочий цикл устройства (команда пауза) (код – 12h)
 */
void udpCommandPauseProcess(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[5];

	txData[0] = 5;
	txData[1] = 0x12;
	txData[2] = 1;
	txData[3] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[4] = udpCalculateChecksum(txData, 5);

	udpClientSendResponse(txData, 5);
}

/**	Функция обработки UDP команды:
 * 	Продолжить выполнение рабочего цикла устройства (код – 13h)
 */
void udpCommandResumeProcess(void)
{
	/* Массив отправляемых серверу данных */
	uint8_t txData[4];

	txData[0] = 4;
	txData[1] = 0x13;
	txData[2] = 1;

	/* Расчет контрольной суммы отправляемого пакета данных */
	txData[3] = udpCalculateChecksum(txData, 4);

	udpClientSendResponse(txData, 4);
}
