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

/* Тип данных - состояние работы обработчика g - кода */
typedef enum
{
	HANDLER_GCODE_ERROR = -1,
	HANDLER_GCODE_END = 0,
	HANDLER_GCODE_DONE = 1,
	HANDLER_GCODE_READY,
	HANDLER_GCODE_WAIT

} handler_gcode_state_t;

typedef enum
{
	NONE = -1,
	G00 = 0,
	G01 = 1,
	G02 = 2,
	G03 = 3

} gcommand_t;

typedef struct
{
	INTERPOLATOR_StructDef* interpolator;

	handler_gcode_state_t _workState;
	gcommand_t _command;


} HANDLER_GCODE_StructDef;

void handlerGcodeInit(HANDLER_GCODE_StructDef* ghandler);
void handlerGcode(HANDLER_GCODE_StructDef* ghandler);
void handlerGcommand(HANDLER_GCODE_StructDef* ghandler);
void handlerStateCalculate(HANDLER_GCODE_StructDef* ghandler);


#ifdef __cplusplus
}
#endif

#endif /* INC_GCODE_H_ */
