#ifndef INC_GCODE_H_
#define INC_GCODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "interpolator.h"

/* Тип данных - состояние работы обработчика g - кода
 * HANDLER_GCODE_ERROR - обработчик в ошибке
 * HANDLER_GCODE_END - обработчик закончил обработку g - кода в буфере
 * HANDLER_GCODE_DONE - обработчик g - кода инициализирован
 * HANDLER_GCODE_READY - обработчик готов обработать следующую g - команду
 * HANDLER_GCODE_WAIT - обработчик ждет пока планировщик обработает текущую g - команду
 */
typedef enum
{
	HANDLER_GCODE_ERROR = -1,
	HANDLER_GCODE_END = 0,
	HANDLER_GCODE_DONE = 1,
	HANDLER_GCODE_READY,
	HANDLER_GCODE_WAIT

} handler_gcode_state_t;

/** Тип данных - g - команды
 * 	NONE - нет команды
 * 	G00 - неинтерполяционное движение
 * 	G01 - интерполяционное линейное движение
 * 	G02 - интерполяционное круговое движение по часовой стрелки
 * 	G03 - интерполяционное круговое движение против часовой стрелки
 */
typedef enum
{
	NONE = -1,
	G00 = 0,
	G01 = 1,
	G02 = 2,
	G03 = 3

} gcommand_t;

/** Структура обработчика g - кода
 */
typedef struct
{
	/* Указатель на структуру интерполятора */
	INTERPOLATOR_StructDef* interpolator;

	/* Состояние работы обработчика g - кода */
	handler_gcode_state_t _workState;

	/* Текущая обрабатываемая g - команда */
	gcommand_t _command;


} HANDLER_GCODE_StructDef;

void tickGcodeHandler(HANDLER_GCODE_StructDef* ghandler);

void handlerGcodeInit(HANDLER_GCODE_StructDef* ghandler);
void handlerGcode(HANDLER_GCODE_StructDef* ghandler);
void handlerGcommand(HANDLER_GCODE_StructDef* ghandler);
void handlerEndState(HANDLER_GCODE_StructDef* ghandler);

#ifdef __cplusplus
}
#endif

#endif /* INC_GCODE_H_ */
