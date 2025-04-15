#ifndef INC_FIFO_H_
#define INC_FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

#define FIFO_SIZE_MAX	1024	// Максимальный размер FIFO буфера

typedef struct
{
	/* Буфер данных неизместного размера, память выделяется динамически
	 * в инициализации если размер отличен от FIFO_SIZE_MAX
	 */
	int* data;

	/* Указатель на позицию считывания - "голова" */
    volatile uint16_t tail;

    /* Указатель на позицию записи - "хвост" */
    volatile uint16_t head;

    /* Размер буфера */
    volatile uint16_t size;

} FIFO_StructDef;

/* --------------------------------------- Прототипы функций библиотеки fifo.h --------------------------------------- */

void fifoInit(uint16_t size, FIFO_StructDef* buf);
void clear(FIFO_StructDef* buf);
uint16_t availableForRead(FIFO_StructDef* buf);
uint16_t availableForWrite(FIFO_StructDef* buf);
void fifoWrite(FIFO_StructDef* buf);
void fifoRead(FIFO_StructDef* buf);

/* --------------------------------------- Прототипы функций библиотеки fifo.h --------------------------------------- */

/** Функция инициализации fifo буфера
 * 	при инициализации указывается размер буфера
 */
void fifoInit(uint16_t size, FIFO_StructDef* buf)
{
	buf->size = size;

	if(size <= FIFO_SIZE_MAX)
	{
		buf->data = (int*)calloc(size, sizeof(uint16_t));
	}
	else buf->data = (int*)calloc(FIFO_SIZE_MAX, sizeof(uint16_t));

	buf->head = 0;
	buf->tail = 0;
}

/** Очистка буфера
 */
void clear(FIFO_StructDef* buf)
{
	for(uint16_t i = 0; i < buf->size; i ++)
	{
		buf->data[i] = 0;
	}

	buf->head = 0;
	buf->tail = 0;
}

/** Число ячеек доступных для ЧТЕНИЯ (заполненное пространство)
 */
uint16_t availableForRead(FIFO_StructDef* buf)
{
    if (buf->head <= buf->tail)  return buf->size - buf->tail + buf->head;
    else return buf->head - buf->tail;
}

/** Число ячеек доступных для ЗАПИСИ (доступное пространство)
 */
uint16_t availableForWrite(FIFO_StructDef* buf)
{
	return buf->size - availableForRead(buf);
}

/** Записать ячейку данных в буфер и переместить указатель "голова" на одну позицию
 */
void fifoWrite(FIFO_StructDef* buf)
{
	if (buf->head >= buf->size) buf->head = 0;

	buf->head++;
}

/** Прочитать ячейку данных из буфера и переместить указатель "хвост" на одну позицию
 */
void fifoRead(FIFO_StructDef* buf)
{
	buf->tail++;

    if (buf->tail >= buf->size) buf->tail = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_FIFO_H_ */
