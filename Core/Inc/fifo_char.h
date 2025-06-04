#ifndef INC_FIFO_CHAR_H_
#define INC_FIFO_CHAR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#include "fifo.h"

#define FIFO_GCODE_BUF_SIZE			256 				//< 256 байт - размер FIFO буфера g - команд

typedef struct
{
	/* Буфер данных известного заранее размера, память уже
	 * выделенна для статического буфера.
	 * FIFO буфер хранит указатели на статический массив/структуру данных
	 */
	char** data;

	/* Указатель на позицию считывания - "голова" */
	volatile uint32_t tail;

	/* Указатель на позицию записи - "хвост" */
	volatile uint32_t head;

	/* Размер буфера */
	uint32_t size;

} FIFO_CHAR_StructDef;

fifo_state_t fifoInitChar(FIFO_CHAR_StructDef* buf, char** data, uint32_t size);
fifo_state_t fifoClearChar(FIFO_CHAR_StructDef* buf);
fifo_state_t fifoWriteChar(FIFO_CHAR_StructDef* buf, char* str);
fifo_state_t fifoReadChar(FIFO_CHAR_StructDef* buf, char* str);
uint16_t cellsForReadChar(FIFO_CHAR_StructDef* buf);
uint16_t cellsForWriteChar(FIFO_CHAR_StructDef* buf);
fifo_state_t availableForReadChar(FIFO_CHAR_StructDef* buf);
fifo_state_t availableForWriteChar(FIFO_CHAR_StructDef* buf);


#ifdef __cplusplus
}
#endif

#endif /* INC_FIFO_CHAR_H_ */
