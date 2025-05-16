/**
  ******************************************************************************
  * @file    	stepper.h
  * @author  	PromisLab
  * @brief   	Этот файл описывает прототипы функций работы с шаговым мотором
  *
  ******************************************************************************
  */

#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "types.h"

#define STEP_TIME 5 		//< Время переключения состояния пина STEP в мкс (по-умолчанию 5 мкс)

/* Определение структуры порта DIO из стандартной библиотеки HAL_STM32
 * для использования указателей GPIO_TypeDef *GPIOx
 */
typedef struct
{
	volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */

} GPIO_StructDef_custom;

/* Определение структуры пинов шагового мотора STEP - DIR - EN */
typedef struct
{
	/* Порт и номер пина STEP */
	GPIO_StructDef_custom* GPIOx_step;
	uint16_t GPIO_Pin_step;

	/* Порт и номер пина DIR */
	GPIO_StructDef_custom* GPIOx_dir;
	uint16_t GPIO_Pin_dir;

	/* Порт и номер пина EN */
	GPIO_StructDef_custom* GPIOx_en;
	uint16_t GPIO_Pin_en;

} STEPPER_PINS_StructDef;


/* Определение структуры шагового мотора */
typedef struct
{
	/* Экземпляр структуры пинов */
	STEPPER_PINS_StructDef stepper_pins;

	/* Текущая позиция мотора в шагах */
	volatile int32_t pos;

	/* Текущее направление вращения мотора (1 или -1)
	 * 1 - по часовой стрелки
	 * -1 - против часовой стрелки
	 * Направление вращения определяется со стороны задней части мотора
	 */
	volatile int8_t dir;

	/* Статус включения мотора */
	statusEn_t en;

	/* Переменная - флаг для определения инвертированного состояния пина EN */
	bool _globEn;

	/* Переменная - флаг для определения инвертированного состояния пина DIR */
	bool _globDir;

} STEPPER_StructDef;

/* Определение пустого указателя на функию с параметрами */
typedef void (*writePinFunction_void_ptr)(GPIO_StructDef_custom*, uint16_t, PinState_custom);

/* --------------------------------------- Прототипы функций библиотеки stepper.h --------------------------------------- */

void stepperFunctionsInit(writePinFunction_void_ptr function);
void stepperInit(STEPPER_StructDef* stepper, STEPPER_PINS_StructDef* pins);
void step(STEPPER_StructDef* stepper);
void setDir(STEPPER_StructDef* stepper, int8_t dir);
void enableStepper(STEPPER_StructDef* stepper);
void disableStepper(STEPPER_StructDef* stepper);
void invertPinEn(STEPPER_StructDef* stepper);
void invertPinDir(STEPPER_StructDef* stepper);

/* --------------------------------------- Прототипы функций библиотеки stepper.h --------------------------------------- */

#endif // INC_STEPPER_H
