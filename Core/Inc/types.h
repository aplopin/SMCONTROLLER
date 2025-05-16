#ifndef INC_TYPES_H_
#define INC_TYPES_H_

typedef enum
{
	ABSENT = 0,
	NORMALLY_CLOSED,
	NORMALLY_OPEN

} limit_switch_t;

typedef enum
{
	PIN_RESET = 0,
	PIN_SET

} PinState_custom;

/* Тип данных - состояние пина EN
 * OFF - мотор выключен (если пин EN используется)
 * ON - мотор включен (если пин EN используется)
 */
typedef enum
{
	OFF = 0,
	ON

} statusEn_t;

#endif // INC_TYPES_H_
