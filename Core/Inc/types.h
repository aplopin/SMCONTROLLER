#ifndef INC_TYPES_H_
#define INC_TYPES_H_

typedef enum
{
	LINEAR,
	ROTATIONAL

} movemen_type_t;

/* Тип данных - состояние работы драйвера
 * INIT - драйвер инициализирован
 * READY - драйвер готов к работе, мотор не двигается
 * BUSY - драйвер в работе, мотор двигается
 * BRAKE - драйвера был остановлен
 * ERR - драйвер в ошибке
 */
typedef enum
{
	DRIVER_INIT = 0,
	DRIVER_READY,
	DRIVER_BUSY,
	DRIVER_BRAKE,
	DRIVER_ERR

} work_state_t;

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
