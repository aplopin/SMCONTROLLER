#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "pin.h"

#define STEPPER_STEP_TIME 	5 		//< Время переключения состояния пина STEP (мкс)

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
	STEPPER_PINS_StructDef* stepper_pins;

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

void stepperInit(STEPPER_StructDef* stepper, STEPPER_PINS_StructDef* pins);
void step(STEPPER_StructDef* stepper);
void setStepperDir(STEPPER_StructDef* stepper, int8_t dir);
void enableStepper(STEPPER_StructDef* stepper);
void disableStepper(STEPPER_StructDef* stepper);
void invertStepperPinEn(STEPPER_StructDef* stepper);
void invertStepperPinDir(STEPPER_StructDef* stepper);

/* --------------------------------------- Прототипы функций библиотеки stepper.h --------------------------------------- */

#endif // INC_STEPPER_H
