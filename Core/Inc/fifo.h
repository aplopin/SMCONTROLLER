/**
  ******************************************************************************
  * @Файл    	fifo.h
  * @Автор  	PromisLab
  * @Описание   Этот файл описывает прототипы функций FIFO буфера, который
  * 			использует указатель на статически выделенную область памяти
  *
  ******************************************************************************
  *
  */

#ifndef INC_FIFO_H_
#define INC_FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

	/** Максимальное число ячеек FIFO буфера
	 * 	Нужно подумать не о количестве ячеек, а о выделении
	 * 	нужного количества памяти
	 */
#define FIFO_SIZE_MAX			128 * 2			//< 250 кБайт - максимальный размер буфера шагов
#define FIFO_STEPS_SIZE 		1024 * 5 		//< 100 кБайт - размер буфера шагов интерполятора
#define FIFO_NET_SIZE			64 				//< 1 кБайт - размер буфера сетевого интерфейса UDP

typedef enum
{
	FIFO_ERR = -1,
	FIFO_OK = 0,
	FIFO_INIT,
	FIFO_EMPTY,
	FIFO_OVERFLOW,
	FIFO_CLEARED

} fifo_state_t;

typedef struct
{
	/* Буфер данных известного заранее размера, память уже
	 * выделенна для статического буфера.
	 * FIFO буфер хранит указатели на статический массив/структуру данных
	 */
	int8_t* data;

	/* Указатель на позицию считывания - "голова" */
	uint32_t tail;

	/* Указатель на позицию записи - "хвост" */
	uint32_t head;

	/* Размер буфера */
	uint32_t size;

} FIFO_StructDef;

fifo_state_t fifoInit(FIFO_StructDef* buf, int8_t* data, uint32_t size);
fifo_state_t fifoClear(FIFO_StructDef* buf);
fifo_state_t fifoWrite(FIFO_StructDef* buf, int8_t value);
fifo_state_t fifoRead(FIFO_StructDef* buf, int8_t* value);
uint16_t cellsForRead(FIFO_StructDef* buf);
uint16_t cellsForWrite(FIFO_StructDef* buf);
fifo_state_t availableForRead(FIFO_StructDef* buf);
fifo_state_t availableForWrite(FIFO_StructDef* buf);


#ifdef __cplusplus
}
#endif

#endif /* INC_FIFO_H_ */
