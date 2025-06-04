#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define STEP_TIME 5 		//< Время переключения состояния пина STEP в мкс (по-умолчанию 5 мкс)

/* Тип данных - состояние пина DIO из стандартной библиотеки HAL STM32
 * для использования в данной библиотеке
 */
typedef enum
{
	PIN_RESET = 0,
	PIN_SET

} pin_state_custom_t;

/* Тип данных - состояние пина EN
 * OFF - мотор выключен
 * ON - мотор включен
 */
typedef enum
{
	OFF = 0,
	ON

} statusEn_t;

/* Определение структуры порта DIO из стандартной библиотеки HAL STM32
 * для использования указателей GPIO_TypeDef *GPIOx
 */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

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
	volatile statusEn_t en;

	/* Переменная - флаг для определения инвертированного состояния пина EN */
	volatile bool _globEn;

	/* Переменная - флаг для определения инвертированного состояния пина DIR */
	volatile bool _globDir;

} STEPPER_StructDef;

/* Определение пустого указателя на функию с параметрами */
typedef void (*writePinFunction_void_ptr)(GPIO_StructDef_custom*, uint16_t, pin_state_custom_t);

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
