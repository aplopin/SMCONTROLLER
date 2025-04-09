/*
 * spi_laser.h
 *
 *  Created on: Jul 30, 2024
 *      Author: aplopin
 */

#ifndef INC_SPI_LASER_H_
#define INC_SPI_LASER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdio.h>
#include "string.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define SPI_TXBUF_SIZE 10

// Прототипы функций общения с контроллером лазером по SPI
void spi_transmit_package(uint8_t *str);			// переслать пакет данных по SPI
void spi_set_device_type(uint8_t device_type);		// задать тип устройства - 00h
void spi_set_laser_param(void);						// установить параметры лазера - 01h
void spi_get_laser_param(void);						// получить параметры от контроллера лазера - 02h
void spi_set_power_laser(uint8_t power);			// задать мощность лазера - 03h
void spi_get_current_param(void);					// получить текущее состояние лазера - 04h
void spi_set_work_mode(void);						// перейти в рабочий режим - 05h
void spi_set_duty_mode(void);						// перейти в дежурный режим - 06h
void spi_pilot_switch(void);						// включить/выключить пилот - 42h
void spi_get_operating_time_counter(void);				// получить значения счётчиков наработки - F0h
void spi_reset_operating_time_counter(void); 			// обнулить текущий счётчик наработки - F1h
void spi_set_current_time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t date, uint8_t month, uint8_t year); // установить текущее время - F2h
void spi_get_current_time(void); 						// получить текущее время - F3h
void spi_send_emergency_message(uint8_t emergency_status); 	// аварийное сообщение контроллеру лазера - FFh
void spi_get_last_logs(uint8_t logs); 					// получить последние n логов, кол-во ошибок, если n = 0, то только кол-во ошибок - 50h

#ifdef __cplusplus
}
#endif

#endif /* INC_SPI_LASER_H_ */
