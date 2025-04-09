/*
 * spi_laser.c
 *
 *  Created on: Jul 30, 2024
 *      Author: aplopin
 */
#include "laser.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

#define SPI_TXBUF_SIZE 10

uint8_t spi_txbuf[SPI_TXBUF_SIZE]; // буфер передачи по SPI


// Дополнительные функции
// функция передачи по SPI пакета данных
void spi_transmit_package(uint8_t *str)
{
	HAL_SPI_Transmit(&hspi3, str, 1, 100); // переслать длину блока данных
	HAL_SPI_Transmit_IT(&hspi3, str + 1, str[0] - 1); // переслать оставшуюся часть
}

// Протокол общения с контроллером лазером по SPI
// 1. код команды - 00h
void spi_set_device_type(uint8_t device_type) // задать тип устройства 00h - FFh
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для установки кода типа устройства
	spi_txbuf[0] = 3; 				// длина блока данных
	spi_txbuf[1] = 0x00; 			// код команд
	spi_txbuf[2] = device_type; 	// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 2. код команды - 01h
void spi_set_laser_param(void) // установить параметры лазера
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // инициализация TX буфера SPI

	// пакет данных для инициализации лазера с нужной мощностью
	spi_txbuf[0] = 9; 			// длина блока данных
	spi_txbuf[1] = 0x01; 		// код команды
	spi_txbuf[2] = 0x00; 		// тип устройства
	spi_txbuf[3] = 30; 			// ток начаки
	spi_txbuf[4] = 50; 			// частота модуляции

	// количество импульсов в пачке - 32000
	spi_txbuf[5] = 0b01111101;
	spi_txbuf[6] = 0b00000000;

	// количество импульсов в паузе - 0
	spi_txbuf[7] = 0;
	spi_txbuf[8] = 0;

	spi_transmit_package(spi_txbuf);
}

// 3. код команды - 02h
void spi_get_laser_param(void) // получить параметры от контроллера лазера
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для получения параметров лазера
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0x02; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 4. код команды - 03h
void spi_set_power_laser(uint8_t power) // задать мощность лазера
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для установки мощности лазера 0 - 100%
	spi_txbuf[0] = 4; 			// длина блока данных
	spi_txbuf[1] = 0x03; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h
	spi_txbuf[3] = power; 		// ток начаки

	spi_transmit_package(spi_txbuf);
}

// 5. код команды - 04h
void spi_get_current_param(void) // получить текущее состояние лазера
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для получения текущего состояния лазера
	spi_txbuf[0] = 4; 			// длина блока данных
	spi_txbuf[1] = 0x04; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 6. код команды - 05h
void spi_set_work_mode(void) // перейти в рабочий режим
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для перехода в рабочий режим лазера
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0x05; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 7. код команды - 06h
void spi_set_duty_mode(void) // перейти в дежурный режим
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для перехода в дежурный режим лазера
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0x06; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 8. код команды - 42h
void spi_pilot_switch(void) // включить/выключить пилот
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для включения/выключения пилота
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0x42; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 9. код команды - F0h
void spi_get_operating_time_counter(void) // получить значения счётчиков наработки
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для приема значения счётчиков наработки
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0xF0; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 10. код команды - F1h
void spi_reset_operating_time_counter(void) // обнулить текущий счётчик наработки
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // очистка TX буфера SPI

	// пакет данных для обнуление текущего счётчика наработки
	spi_txbuf[0] = 3; 			// длина блока данных
	spi_txbuf[1] = 0xF1; 		// код команд
	spi_txbuf[2] = 0x00; 		// тип устройства по-умолчанию 00h

	spi_transmit_package(spi_txbuf);
}

// 11. код команды - F2h
void spi_set_current_time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t date, uint8_t month, uint8_t year) // установить текущее время
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // инициализация TX буфера SPI

	// пакет данных для установки текущего времени
	spi_txbuf[0] = 9; 				// длина блока данных
	spi_txbuf[1] = 0xF2; 			// код команды
	spi_txbuf[2] = 0x00; 			// тип устройства
	spi_txbuf[3] = sec; 			// секунды
	spi_txbuf[4] = min; 			// минуты
	spi_txbuf[5] = hour; 			// часы
	spi_txbuf[6] = date; 			// число
	spi_txbuf[7] = month; 			// месяц
	spi_txbuf[8] = year; 			// год, начиная с 2000, пример 2001 - 1

	spi_transmit_package(spi_txbuf);
}

// 12. код команды - F3h
void spi_get_current_time(void) // получить текущее время
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // инициализация TX буфера SPI

	// пакет данных для получения текущего времени
	spi_txbuf[0] = 3; 				// длина блока данных
	spi_txbuf[1] = 0xF3; 			// код команды
	spi_txbuf[2] = 0x00; 			// тип устройства

	spi_transmit_package(spi_txbuf);
}

// 13. код команды - FFh
void spi_send_emergency_message(uint8_t emergency_status) // аварийное сообщение контроллеру лазера
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // инициализация TX буфера SPI

	// пакет данных для отсылки аварийного сообщения контроллеру лазера
	spi_txbuf[0] = 3; 					// длина блока данных
	spi_txbuf[1] = 0xFF; 				// код команды
	spi_txbuf[2] = 0x00; 				// тип устройства
	spi_txbuf[3] = emergency_status; 	// статус аварии: 1 — аварийное состояние; 0 — авария устранена

	spi_transmit_package(spi_txbuf);
}

// 14. код команды - 50h
void spi_get_last_logs(uint8_t logs) // получить последние n логов, кол-во ошибок, если n = 0, то только кол-во ошибок
{
	memset(spi_txbuf, 0, SPI_TXBUF_SIZE); // инициализация TX буфера SPI

	// пакет данных для получения последних n логов
	spi_txbuf[0] = 3; 					// длина блока данных
	spi_txbuf[1] = 0x50; 				// код команды
	spi_txbuf[2] = 0x00; 				// тип устройства
	spi_txbuf[3] = logs; 				// число последних n логов

	spi_transmit_package(spi_txbuf);
}
