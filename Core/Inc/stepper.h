#ifndef STEPPER_H
#define STEPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "types.h"
#include "dwt.h"

/* Время переключения состояния пина STEP в мкс, по-умолчанию 4 мкс */
#define DRIVER_STEP_TIME 50

/* Определение структуры порта DIO из стандартной библиотеки HAL STM32
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

/* Указатель на функцию для изменения состояния пина */
static writePinFunction_void_ptr setPin;

/* --------------------------------------- Прототипы функций библиотеки driver.h --------------------------------------- */

void stepperFunctionsInit(writePinFunction_void_ptr function);
void stepperInit(STEPPER_StructDef* stepper, STEPPER_PINS_StructDef* pins);
void doStep(STEPPER_StructDef* stepper);
void setDir(STEPPER_StructDef* stepper, int8_t dir);
void enableStepper(STEPPER_StructDef* stepper);
void disableStepper(STEPPER_StructDef* stepper);
void invertPinEn(STEPPER_StructDef* stepper);
void invertPinDir(STEPPER_StructDef* stepper);

/* --------------------------------------- Прототипы функций библиотеки driver.h --------------------------------------- */

/**	Функция инициализации указателей на функции из других областей программы
 * 	для использования в данной библиотеке
 */
void stepperFunctionsInit(writePinFunction_void_ptr function)
{
	setPin = function;
}

/** Функция инициализации шагового мотора
 * 	на вход передается указатель на экземпляр структуры STEPPER_StructDef
 * 	и указатель на экземпляр структуры STEPPER_PINS_StructDef
 *
 * 	Функция определяет пины шагового мотора и инициализирует начальные данные мотора -
 * 	позицию, направление, статус включения, глобальное определение поведения пинов DIR, EN
 */
void stepperInit(STEPPER_StructDef* stepper, STEPPER_PINS_StructDef* pins)
{
	stepper->stepper_pins.GPIOx_step = pins->GPIOx_step;
	stepper->stepper_pins.GPIO_Pin_step = pins->GPIO_Pin_step;

	stepper->stepper_pins.GPIOx_dir = pins->GPIOx_dir;
	stepper->stepper_pins.GPIO_Pin_dir = pins->GPIO_Pin_dir;

	/* ------------------ НЕ ИСПОЛЬЗУЕТСЯ ----------------- */

//	stepper->pins.GPIOx_en = pins->GPIOx_en;
//	stepper->pins.GPIO_Pin_en = pins->GPIO_Pin_en;

	/* ------------------ НЕ ИСПОЛЬЗУЕТСЯ ----------------- */

	stepper->stepper_pins.GPIOx_en = 0x0;
	stepper->stepper_pins.GPIO_Pin_en = 0x0;

	stepper->pos = 0;
	stepper->dir = 1;
	stepper->en = OFF;

	stepper->_globEn = false;
	stepper->_globDir = false;
}

/** Сделать шаг мотором
 */
void doStep(STEPPER_StructDef* stepper)
{
	stepper->pos += stepper->dir;
	setDir(stepper, stepper->dir); /* Установить пин DIR в нужное состояние в соответствии с направлением вращения */

	setPin(stepper->stepper_pins.GPIOx_step, stepper->stepper_pins.GPIO_Pin_step, PIN_SET);
	DWT_usDelay(DRIVER_STEP_TIME);
	setPin(stepper->stepper_pins.GPIOx_step, stepper->stepper_pins.GPIO_Pin_step, PIN_RESET);
}

/** Задать направление вращения мотора в соответствии с величиной _globDir
 * 	если _globDir = false, то GPIO_PIN_SET -> dir = 1,
 * 	PIN_RESET -> dir = -1,
 * 	если _globDir = true, то GPIO_PIN_SET -> dir = -1,
 * 	PIN_RESET -> dir = 1,
 * 	dir = 1 - по часовой стрелки независимо от _globDir
 * 	dir = -1 - против часовой стрелки независимо от _globDir
 * 	Направление вращения определяется со стороны задней части мотора,
 * 	т.е. вал мотора смотрит от нас!
 */
void setDir(STEPPER_StructDef* stepper, int8_t dir)
{
	if(stepper->_globDir == false)
	{
		if(dir == 1)
		{
			stepper->dir = 1;
			setPin(stepper->stepper_pins.GPIOx_dir, stepper->stepper_pins.GPIO_Pin_dir, PIN_SET);
		}
		else
		{
			stepper->dir = -1;
			setPin(stepper->stepper_pins.GPIOx_dir, stepper->stepper_pins.GPIO_Pin_dir, PIN_RESET);
		}
	}
	else if(dir == 1)
	{
		stepper->dir = 1;
		setPin(stepper->stepper_pins.GPIOx_dir, stepper->stepper_pins.GPIO_Pin_dir, PIN_RESET);
	}
		else
		{
			stepper->dir = -1;
			setPin(stepper->stepper_pins.GPIOx_dir, stepper->stepper_pins.GPIO_Pin_dir, PIN_SET);
		}
}

/* ------------------ НЕ ИСПОЛЬЗУЕТСЯ ----------------- */

/** Включение мотора
 */
void enableStepper(STEPPER_StructDef* stepper)
{
	stepper->en = ON;

	if(stepper->_globEn == false)
	{
		setPin(stepper->stepper_pins.GPIOx_en, stepper->stepper_pins.GPIO_Pin_en, PIN_SET);
	}
	else setPin(stepper->stepper_pins.GPIOx_en, stepper->stepper_pins.GPIO_Pin_en, PIN_RESET);
}

/** Выключение мотора
 */
void disableStepper(STEPPER_StructDef* stepper)
{
	stepper->en = OFF;

	if(stepper->_globEn == false)
	{
		setPin(stepper->stepper_pins.GPIOx_en, stepper->stepper_pins.GPIO_Pin_en, PIN_RESET);
	}
	else setPin(stepper->stepper_pins.GPIOx_en, stepper->stepper_pins.GPIO_Pin_en, PIN_SET);
}

/** Инвертировать поведение пина EN
 * 	_globEn определяет поведение пина EN
 * 	если _globEn = false, то PIN_SET -> en = ON,
 * 	PIN_RESET -> en = OFF,
 * 	если _globEn = true, то PIN_SET -> en = OFF,
 * 	PIN_RESET -> en = ON,
 */
void invertPinEn(STEPPER_StructDef* stepper)
{
	stepper->_globEn = !stepper->_globEn;
}

/* ------------------ НЕ ИСПОЛЬЗУЕТСЯ ----------------- */

/** Инвертировать поведение пина DIR
 * 	_globDir определяет поведение пина DIR
 * 	если _globDir = false, то GPIO_PIN_SET -> dir = 1,
 * 	PIN_RESET -> dir = -1,
 * 	если _globDir = true, то GPIO_PIN_SET -> dir = -1,
 * 	PIN_RESET -> dir = 1,
 * 	dir = 1 - по часовой стрелки независимо от _globDir
 * 	dir = -1 - против часовой стрелки независимо от _globDir
 * 	Направление вращения определяется со стороны задней части мотора,
 * 	т.е. вал мотора смотрит от нас!
 */
void invertPinDir(STEPPER_StructDef* stepper)
{
	stepper->_globDir = !stepper->_globDir;
}

#endif // STEPPER_H
