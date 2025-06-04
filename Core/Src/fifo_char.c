#include "fifo_char.h"

#include <string.h>

/** Функция инициализации fifo буфера
 * При инициализации передается указатель на
 * статический массив/структуру данных и его длина
 */
fifo_state_t fifoInitChar(FIFO_CHAR_StructDef* buf, char** data, uint32_t size)
{
	buf->size = size;
	buf->data = data;

	buf->tail = 0;
	buf->head = 1;

	if (buf->data == NULL) return FIFO_ERR;

	return FIFO_INIT;
}

/** Очистка буфера
 */
fifo_state_t fifoClearChar(FIFO_CHAR_StructDef* buf)
{
	/*for (uint32_t i = 0; i < buf->size; i++)
	{
		buf->data[i] = 0;
	}*/

	buf->tail = 0;
	buf->head = 1;

	return FIFO_CLEARED;
}

/** Число ячеек доступных для ЧТЕНИЯ (заполненное пространство)
 */
uint16_t cellsForReadChar(FIFO_CHAR_StructDef* buf)
{
	if (buf->head < buf->tail)  return buf->size - buf->tail + buf->head - 1;
	else return buf->head - buf->tail - 1;
}

/** Функция определения наличия доступных ячеек для ЧТЕНИЯ (заполненное пространство)
 */
fifo_state_t availableForReadChar(FIFO_CHAR_StructDef* buf)
{
	if (cellsForReadChar(buf) > 0) return FIFO_OK;
	else return FIFO_EMPTY;
}

/** Число ячеек доступных для ЗАПИСИ (доступное пространство)
 */
uint16_t cellsForWriteChar(FIFO_CHAR_StructDef* buf)
{
	return buf->size - cellsForReadChar(buf) - 2;
}

/** Функция определения наличия доступных ячеек для ЗАПИСИ (доступное пространство)
 */
fifo_state_t availableForWriteChar(FIFO_CHAR_StructDef* buf)
{
	if (cellsForWriteChar(buf) == 0) return FIFO_OVERFLOW;
	else return FIFO_OK;
}

/** Записать ячейку данных в буфер и переместить указатель "голова" на одну позицию
 */
fifo_state_t fifoWriteChar(FIFO_CHAR_StructDef* buf, char* str)
{
	if (buf->head >= buf->size) buf->head = 0;

	strcpy(buf->data[buf->head], str);
	buf->head++;

	return FIFO_OK;
}

/** Прочитать ячейку данных из буфера и переместить указатель "хвост" на одну позицию
 * 	Функция принимает указатель на переменную, в которую записывается значение последней
 * 	ячейки fifo буфера
 */
fifo_state_t fifoReadChar(FIFO_CHAR_StructDef* buf, char* str)
{
	if (str == NULL) return FIFO_ERR;

	buf->tail++;
	if (buf->tail >= buf->size) buf->tail = 0;

	strcpy(str, buf->data[buf->tail]);

	return FIFO_OK;
}
