/**
  ******************************************************************************
  * @file    	driver.h
  * @author  	PromisLab
  * @brief   	Этот файл описывает прототипы функций драйвера шагового мотора
  * 			Для работы библиотеки требуется передать функцию для получения
  * 			реального времени с дискретностью 1 мкс от настроенного таймера микроконтроллера
  * 			указатель на функцию получения времени - micros
  *
  ******************************************************************************
  */

#ifndef INC_DRIVER_H_
#define INC_DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "stepper.h"

#define _sign(x) ((x) >= 0 ? 1 : -1)  							//< Знак числа целого
#define _signf(x) ((x) > 1e-12 ? 1 : -1)  						//< Знак числа с плавающей точкой

#define MIN_SPEED_POS_MODE 5  									//< Минимальная скорость для движения в POSITION_MODE с ускорением в шаг/с
#define MAX_PERIOD_POS_MODE (1000000L / MIN_SPEED_POS_MODE)
#define MIN_STEP_SPEED (1.0 / 3600)  							//< Ограничение минимальной скорости в 1 шаг/ч

/* Тип данных - состояние работы драйвера
 * ERR - драйвер в ошибке
 * INIT - драйвер инициализирован
 * READY - драйвер готов к работе, мотор не двигается
 * RUN - драйвер в работе, мотор двигается
 * BRAKE - драйвера был остановлен
 */
typedef enum
{
	DRIVER_ERR = -1,
	DRIVER_OK = 0,
	DRIVER_INIT,
	DRIVER_READY,
	DRIVER_RUN,
	DRIVER_BRAKE

} driver_state_t;

typedef enum
{
	PARAM_CHANGE_ERR = -1,
	PARAM_CHANGE_OK = 0

} param_change_t;

/** Типы драверов шаговых моторов
 * 	LINEAR - линейное движение
 * 	ROTATIONAL - круговое движение
 */
typedef enum
{
	LINEAR = 0,
	ROTATIONAL

} movement_type_t;

/** Режимы работы драйвера:
 * 	POSITION_MODE - управление по позиции
 * 	VELOCITY_MODE - управление по скорости
 */
typedef enum
{
    POSITION_MODE = 0,
    VELOCITY_MODE

} run_mode_t;

/* Определение структуры пинов концевых переключателей и датчика нуля драйвера шагового мотора
 * концевик отрицательного направления - NEG_LIMIT_SWITCH
 * нулевик - ZERO_SENSOR
 * концевик положительного направления - POS_LIMIT_SWITCH
 */
typedef struct
{
	/* Порт и номер пина NEG_LIMIT_SWITCH */
	GPIO_StructDef_custom* GPIOx_neg_lim_sw;
	uint16_t GPIO_Pin_neg_lim_sw;
	limit_switch_t type_neg_lim_sw;

	/* Порт и номер пина ZERO_SENSOR */
	GPIO_StructDef_custom* GPIOx_zero_sens;
	uint16_t GPIO_Pin_zero_sens;
	limit_switch_t type_zero_sens;

	/* Порт и номер пина POS_LIMIT_SWITCH */
	GPIO_StructDef_custom* GPIOx_pos_lim_sw;
	uint16_t GPIO_Pin_pos_lim_sw;
	limit_switch_t type_pos_lim_sw;

} DRIVER_LIMIT_SWITCH_PINS_StructDef;

/* Структура драйвера шагового мотора */
typedef struct
{
	/* Структура шагового мотора */
	STEPPER_StructDef* stepper;

	/* Структура пинов концевых переключателей и датчика нуля драйвера шагового мотора */
	DRIVER_LIMIT_SWITCH_PINS_StructDef* driver_pins;

	/* ------------- Общие переменные движения ----------- */

	/* Текущее время драйвера с момента начала движения */
	uint32_t tickUs;

	/* Время предыдущего вхождения в главную функцию драйвера мотора tickDriver */
	uint32_t _prevTime;

	/* Время между шагами при постоянной скорости мотора */
	uint32_t stepTime;

	/* Тип оси - линейная или круговая ось */
	movement_type_t _axisType;

	/* Текущий режим работы драйвера */
	run_mode_t _runMode;

	/* Переменная - флаг работы режима автоотключения драйвера после завершения движения */
	bool _autoPower;

	/* Переменная состояния драйвера */
	driver_state_t _workState;

	/* Количество шагов для поворота на 1 градус */
	float _stepsPerDeg;

	/* Ускорение мотора */
	uint16_t _accel;

	/* ------------ Переменные POSITION_MODE ------------- */

	/* Отсчет времени в планировщике скорости в режиме POSITION_MODE */
	uint32_t _positionPlannerTime;

	/* Максимальная скорость мотора для режима POSITION_MODE */
	float _maxSpeed;

	/* Целевая координата */
	int32_t _targetPosition;

	/* Количетсво шагов разгона */
	uint32_t _s1;

	/* Количетсво шагов равномерного движения */
	uint32_t _s2;

	/* Количество шагов торможения */
	uint32_t _s3;

	/* Счетчик шагов планировщика скорости в режиме POSITION_MODE */
	uint32_t _k;

	/* ------------ Переменные VELOCITY_MODE ------------- */

	/* Отсчет времени в планировщике скорости в режиме VELOCITY_MODE */
	uint32_t _speedPlannerTime;

	/* Текущая скорость мотора */
	float _curSpeed;

	/* Скорость мотора для режима VELOCITY_MODE */
	float _targetSpeed;

	/* Стоп - флаг */
	bool _stopFlag;

	/* ------ Переменные алгоритма плавного разгона ------ */

	/* Начальное время планировщика в мкс */
	float _c0;

	/* n-тое время планировщика в мкс */
	float _cn;

	/* Счетчик шагов планировщика скорости */
	int32_t _n;

	/* Количество шагов до целевой  скорости */
	uint32_t N;

	/* Минимальное время планировщика скорости, вычисляется из максимальной скорости */
	float _cmin;

} DRIVER_StructDef;

/* Определение uint32_t указателя на функию без параметров */
typedef uint32_t (*timeFunction_uint32_t_ptr)(void);

/* Определение void указателя на функию без параметров */
typedef void (*timeFunction_void_ptr)(void);

/* --------------------------------------- Инициализация --------------------------------------- */

void driverFunctionsInit(timeFunction_void_ptr function1, timeFunction_void_ptr function2, timeFunction_uint32_t_ptr function3, timeFunction_void_ptr function4);
void driverInit(DRIVER_StructDef* driver, STEPPER_StructDef* stepper, DRIVER_LIMIT_SWITCH_PINS_StructDef* pins, uint32_t stepsPerRev, movement_type_t type);
void setAutoPowerMode(DRIVER_StructDef* driver, bool mode);
void setRunMode(DRIVER_StructDef* driver, run_mode_t mode);
void resetDriverTimers(DRIVER_StructDef* driver);
void resetMotor(DRIVER_StructDef* driver); 								// подумать о том - нужна ли данная функция
void setDriverDir(DRIVER_StructDef* driver, uint8_t dir);
void invertDriverPinDir(DRIVER_StructDef* driver);
void invertDriverPinEn(DRIVER_StructDef* driver);
uint16_t getMinPeriod(DRIVER_StructDef* driver);

/* --------------------------------------- Инициализация --------------------------------------- */

/* ------------------------------------ Управляющие функции ------------------------------------ */

driver_state_t tickDriver(DRIVER_StructDef* driver);
void enableDriver(DRIVER_StructDef* driver);
void disableDriver(DRIVER_StructDef* driver);
void stop(DRIVER_StructDef* driver); 												//требует доработки
void brakeDriver(DRIVER_StructDef* driver);
void resetDriver(DRIVER_StructDef* driver);
void stepDriver(DRIVER_StructDef* driver);
driver_state_t getStatusDriver(DRIVER_StructDef* driver);

/* ------------------------------------ Управляющие функции ------------------------------------ */

/* --------------------------------------- POSITION_MODE --------------------------------------- */

void plannerPositionMode(DRIVER_StructDef* driver);
param_change_t setTargetPos(DRIVER_StructDef* driver, int32_t target_pos);
param_change_t setTargetPosDeg(DRIVER_StructDef* driver, float target_pos_deg);
int32_t getTargetPos(DRIVER_StructDef* driver);
float getTargetPosDeg(DRIVER_StructDef* driver);
param_change_t setMaxSpeed(DRIVER_StructDef* driver, float speed);
param_change_t setMaxSpeedDeg(DRIVER_StructDef* driver, float speed);
param_change_t setAcceleration(DRIVER_StructDef* driver, uint16_t accel);
param_change_t setAccelerationDeg(DRIVER_StructDef* driver, float accel);
param_change_t setCurrentPos(DRIVER_StructDef* driver, int32_t pos);
param_change_t setCurrentPosDeg(DRIVER_StructDef* driver, float pos);
int32_t getCurrentPos(DRIVER_StructDef* driver);
float getCurrentPosDeg(DRIVER_StructDef* driver);

/* --------------------------------------- POSITION_MODE --------------------------------------- */

/* --------------------------------------- VELOCITY_MODE --------------------------------------- */

void plannerVelocityMode(DRIVER_StructDef* driver);
param_change_t setTargetSpeed(DRIVER_StructDef* driver, float speed);
param_change_t setTargetSpeedDeg(DRIVER_StructDef* driver, float speed);
float getTargetSpeed(DRIVER_StructDef* driver);
float getTargetSpeedDeg(DRIVER_StructDef* driver);
float getCurrentSpeed(DRIVER_StructDef* driver);
float getCurrentSpeedDeg(DRIVER_StructDef* driver);

/* --------------------------------------- VELOCITY_MODE --------------------------------------- */


/* ---------------------------------- Дополнительные функции ----------------------------------- */

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t constrain(int32_t x, int32_t in_min, int32_t in_max);

/* ---------------------------------- Дополнительные функции ----------------------------------- */

#endif /* INC_DRIVER_H_ */
