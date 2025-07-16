#include "pin.h"

/* Указатель на функцию для изменения состояния пина */
static writePinFunction_void_ptr GPIO_WritePin;

/**	Функция инициализации указателя на функцию из другой области программы
 * 	для использования в данной библиотеке
 */
void pinFunctionsInit(writePinFunction_void_ptr function)
{
	GPIO_WritePin = function;
}

/** Функция задания состояния пина GPIO
 */
void setPin(GPIO_StructDef_custom* GPIOx, uint16_t GPIO_Pin, pin_state_custom_t GPIO_State)
{
	GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_State);
}
