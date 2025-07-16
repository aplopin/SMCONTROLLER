#ifndef INC_PIN_H_
#define INC_PIN_H_

#include <stdint.h>

/* Тип данных - состояние пина DIO из стандартной библиотеки HAL STM32
 * для использования в данной библиотеке
 */
typedef enum
{
	PIN_RESET = 0,
	PIN_SET

} pin_state_custom_t;

/* Тип данных - состояние пина включения/отключения объекта
 * OFF - объект выключен
 * ON - объект включен
 */
typedef enum
{
	OFF = 0,
	ON

} statusEn_t;

/* Определение структуры порта DIO из стандартной библиотеки HAL STM32
 * для использования указателей GPIO_TypeDef *GPIOx
 */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

} GPIO_StructDef_custom;

/* Определение пустого указателя на функию с параметрами */
typedef void (*writePinFunction_void_ptr)(GPIO_StructDef_custom*, uint16_t, pin_state_custom_t);

/* --------------------------------------- Прототипы функций библиотеки pin.h --------------------------------------- */

void pinFunctionsInit(writePinFunction_void_ptr function);
void setPin(GPIO_StructDef_custom* GPIOx, uint16_t GPIO_Pin, pin_state_custom_t GPIO_State);

/* --------------------------------------- Прототипы функций библиотеки pin.h --------------------------------------- */

#endif /* INC_PIN_H_ */
