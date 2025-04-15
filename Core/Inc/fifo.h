#ifndef INC_FIFO_H_
#define INC_FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>

/** Максимальное число ячеек FIFO буфера
 * 	Нужно подумать не о количестве ячеек, а о выделении
 * 	нужного количества памяти
 */
#define FIFO_SIZE_MAX	1024

typedef enum
{
	FIFO_ERR = 0,
	FIFO_EMPTY,
	FIFO_OVERFLOW,
	FIFO_OK

} fifo_state_t;

typedef struct
{
	/* Буфер данных неизвестного размера, память выделяется
	 * динамически в инициализации.
	 * Ограничевается количеством ячеек FIFO_SIZE_MAX
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
uint16_t cellsForRead(FIFO_StructDef* buf);
uint16_t cellsForWrite(FIFO_StructDef* buf);
fifo_state_t fifoWrite(FIFO_StructDef* buf, int32_t value);
fifo_state_t fifoRead(FIFO_StructDef* buf, int32_t* ptr_value);

/* --------------------------------------- Прототипы функций библиотеки fifo.h --------------------------------------- */

/** Функция инициализации fifo буфера
 * 	при инициализации указывается размер буфера
 */
void fifoInit(uint16_t size, FIFO_StructDef* buf)
{
	buf->size = size;

	if(size <= FIFO_SIZE_MAX)
	{
		buf->data = (int*)calloc(size, sizeof(int16_t));
	}
	else buf->data = (int*)calloc(FIFO_SIZE_MAX, sizeof(int16_t));

	buf->head = 1;
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

	buf->head = 1;
	buf->tail = 0;
}

/** Число ячеек доступных для ЧТЕНИЯ (заполненное пространство)
 */
uint16_t cellsForRead(FIFO_StructDef* buf)
{
    if (buf->head < buf->tail)  return buf->size - buf->tail + buf->head - 1;
    else return buf->head - buf->tail - 1;
}

/** Функция определения наличия доступных ячеек для ЧТЕНИЯ (заполненное пространство)
 */
bool availableForRead(FIFO_StructDef* buf)
{
	if(cellsForRead(buf) > 0) return true;
	else return false;
}

/** Число ячеек доступных для ЗАПИСИ (доступное пространство)
 */
uint16_t cellsForWrite(FIFO_StructDef* buf)
{
	return buf->size - cellsForRead(buf) - 2;
}

/** Функция определения наличия доступных ячеек для ЗАПИСИ (доступное пространство)
 */
bool availableForWrite(FIFO_StructDef* buf)
{
	if(cellsForWrite(buf) == 0) return false;
	else return true;
}

/** Записать ячейку данных в буфер и переместить указатель "голова" на одну позицию
 */
fifo_state_t fifoWrite(FIFO_StructDef* buf, int32_t value)
{
	if(availableForWrite(buf) == false) return FIFO_OVERFLOW;

	if (buf->head >= buf->size) buf->head = 0;

	buf->data[buf->head] = value;
	buf->head++;

	return FIFO_OK;
}

/** Прочитать ячейку данных из буфера и переместить указатель "хвост" на одну позицию
 * 	Функция принимает указатель на переменную, в которую записывается значение последней
 * 	ячейки fifo буфера
 */
fifo_state_t fifoRead(FIFO_StructDef* buf, int32_t* ptr_value)
{
	if(ptr_value == NULL) return FIFO_ERR;

	if(availableForRead(buf) == false) return FIFO_EMPTY;

	buf->tail++;
	if (buf->tail >= buf->size) buf->tail = 0;

	*ptr_value = buf->data[buf->tail];

	return FIFO_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_FIFO_H_ */
