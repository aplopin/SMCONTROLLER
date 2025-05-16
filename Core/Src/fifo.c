/**
  ******************************************************************************
  * @Файл    	fifo.с
  * @Автор  	PromisLab
  * @Описание	Этот файл описывает реализацию функций FIFO буфера, который
  * 			использует указатель на статически выделенную область памяти
  *
  ******************************************************************************
  *
  */

#include "fifo.h"

  /** Функция инициализации fifo буфера
	* При инициализации передается указатель на
	* статический массив/структуру данных и его длина
	*/
fifo_state_t fifoInit(FIFO_StructDef* buf, int8_t* data, uint32_t size)
{
	buf->size = size;
	buf->data = data;

	if (buf->data == NULL) return FIFO_ERR;

	return FIFO_INIT;
}

/** Очистка буфера
 */
fifo_state_t fifoClear(FIFO_StructDef* buf)
{
	for (uint32_t i = 0; i < buf->size; i++)
	{
		buf->data[i] = 0;
	}

	buf->head = 1;
	buf->tail = 0;

	return FIFO_CLEARED;
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
fifo_state_t availableForRead(FIFO_StructDef* buf)
{
	if (cellsForRead(buf) > 0) return FIFO_OK;
	else return FIFO_EMPTY;
}

/** Число ячеек доступных для ЗАПИСИ (доступное пространство)
 */
uint16_t cellsForWrite(FIFO_StructDef* buf)
{
	return buf->size - cellsForRead(buf) - 2;
}

/** Функция определения наличия доступных ячеек для ЗАПИСИ (доступное пространство)
 */
fifo_state_t availableForWrite(FIFO_StructDef* buf)
{
	if (cellsForWrite(buf) == 0) return FIFO_OVERFLOW;
	else return FIFO_OK;
}

/** Записать ячейку данных в буфер и переместить указатель "голова" на одну позицию
 */
fifo_state_t fifoWrite(FIFO_StructDef* buf, int8_t value)
{
	if (buf->head >= buf->size) buf->head = 0;

	buf->data[buf->head] = value;
	buf->head++;

	return FIFO_OK;
}

/** Прочитать ячейку данных из буфера и переместить указатель "хвост" на одну позицию
 * 	Функция принимает указатель на переменную, в которую записывается значение последней
 * 	ячейки fifo буфера
 */
fifo_state_t fifoRead(FIFO_StructDef* buf, int8_t* value)
{
	if (value == NULL) return FIFO_ERR;

	buf->tail++;
	if (buf->tail >= buf->size) buf->tail = 0;

	*value = buf->data[buf->tail];

	return FIFO_OK;
}
