/** Библиотека драйвера шагового мотора
 * 	Файл: driver.h
 *
 * 	Для работы библиотеки требуется передать функцию для получения
 * 	реального времени с дискретностью 1 мкс от настроенного таймера микроконтроллера
 * 	указатель на функцию получения времени - micros
 *
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

/** Режимы работы драйвера:
 * 	POSITION_MODE - управление по позиции
 * 	VELOCITY_MODE - управление по скорости
 */
typedef enum
{
    POSITION_MODE,
    VELOCITY_MODE

} runMode_t;

/* Определение структуры пинов драйвера шагового мотора NEG_LIMIT_SWITCH - ZERO_SENSOR - POS_LIMIT_SWITCH */
typedef struct
{
	/* Порт и номер пина NEG_LIMIT */
	GPIO_StructDef_custom* GPIOx_neg_lim_sw;
	uint16_t GPIO_Pin_neg_lim_sw;
	limit_switch_t type_neg_lim_sw;

	/* Порт и номер пина ZERO_SENSOR */
	GPIO_StructDef_custom* GPIOx_zero_sens;
	uint16_t GPIO_Pin_zero_sens;
	limit_switch_t type_zero_sens;

	/* Порт и номер пина POW_LIMIT */
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

	/* Тип оси движения - линейная или круговая ось */
	movement_type_t _moveType;

	/* Текущий режим работы драйвера */
	runMode_t _moveMode;

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

/* Указатель на функцию запуска таймера реального времени (мкс)*/
static timeFunction_void_ptr startTimer;

/* Указатель на функцию остановки таймера реального времени (мкс) */
static timeFunction_void_ptr stopTimer;

/* Указатель на функцию для измерения времени */
static timeFunction_uint32_t_ptr getMicros;

/* Указатель на функцию сброса счетчика таймера реального времени (мкс) */
static timeFunction_void_ptr resetTimer;

/* --------------------------------------- Прототипы функций библиотеки driver.h --------------------------------------- */

/* --------------------------------------- Инициализация --------------------------------------- */

void driverFunctionsInit(timeFunction_void_ptr function1, timeFunction_void_ptr function2, timeFunction_uint32_t_ptr function3, timeFunction_void_ptr function4);
void driverInit(DRIVER_StructDef* driver, STEPPER_StructDef* stepper, DRIVER_LIMIT_SWITCH_PINS_StructDef* pins, uint32_t stepsPerRev, movement_type_t type);
void setAutoPowerMode(DRIVER_StructDef* driver, bool mode);
void setMoveMode(DRIVER_StructDef* driver, runMode_t mode);
void resetTimers(DRIVER_StructDef* driver);
void resetMotor(DRIVER_StructDef* driver);
void setDriverDir(DRIVER_StructDef* driver, uint8_t dir);
void invertDriverPinDir(DRIVER_StructDef* driver);
void invertDriverPinEn(DRIVER_StructDef* driver);
uint16_t getMinPeriod(DRIVER_StructDef* driver);

/* --------------------------------------- Инициализация --------------------------------------- */

/* ------------------------------------ Управляющие функции ------------------------------------ */

driver_state_t tickDriver(DRIVER_StructDef* driver);
void enableDriver(DRIVER_StructDef* driver);
void disableDriver(DRIVER_StructDef* driver);
void stop(DRIVER_StructDef* driver); //требует доработки
void brake(DRIVER_StructDef* driver);
void reset(DRIVER_StructDef* driver);
void step(DRIVER_StructDef* driver);
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
param_change_t setSpeed(DRIVER_StructDef* driver, float speed);
param_change_t setSpeedDeg(DRIVER_StructDef* driver, float speed);
float getTargetSpeed(DRIVER_StructDef* driver);
float getTargetSpeedDeg(DRIVER_StructDef* driver);
float getCurrentSpeed(DRIVER_StructDef* driver);
float getCurrentSpeedDeg(DRIVER_StructDef* driver);

/* --------------------------------------- VELOCITY_MODE --------------------------------------- */

/* ---------------------------------------- РАЗРАБОТКА ----------------------------------------- */

//--

/* ---------------------------------------- РАЗРАБОТКА ----------------------------------------- */

/* ---------------------------------- Дополнительные функции ----------------------------------- */

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t constrain(int32_t x, int32_t in_min, int32_t in_max);

/* ---------------------------------- Дополнительные функции ----------------------------------- */

/* --------------------------------------- Прототипы функций библиотеки driver.h --------------------------------------- */

/* -------------------------------------------- Инициализация и параметры ---------------------------------------------- */

/** Функция приема указателя на функцию получения времени в микросекундах.
 *  Используется таймер микроконтроллера и его регистр счетчика - TIMx->CNT
 *  function(1...n) ОБЯЗАТЕЛЬНО должны быть определены в файле main.c
 */
void driverFunctionsInit(timeFunction_void_ptr function1, timeFunction_void_ptr function2, timeFunction_uint32_t_ptr function3, timeFunction_void_ptr function4)
{
	startTimer = function1;
	stopTimer = function2;
	getMicros = function3;
	resetTimer = function4;
}

/** Функция инициализации драйвера шагового мотора
 */
void driverInit(DRIVER_StructDef* driver, STEPPER_StructDef* stepper, DRIVER_LIMIT_SWITCH_PINS_StructDef* pins, uint32_t stepsPerRev, movement_type_t type)
{
	/* Структура шагового мотора */
	driver->stepper = stepper;

	/* Структура пинов концевых переключателей и датчика нуля драйвера шагового мотора */
	driver->driver_pins = pins;

	/* ------------- Общие переменные движения ----------- */

	driver->tickUs = 0;
	driver->_prevTime = 0;
	driver->stepTime = 0;
	driver->_moveType = type;
	driver->_moveMode = POSITION_MODE;
	driver->_autoPower = false;
	driver->_workState = DRIVER_INIT;
	driver->_stepsPerDeg = stepsPerRev / 360.0;
	driver->_accel = 0;

	/* ------------ Переменные POSITION_MODE ------------- */

	driver->_positionPlannerTime = 0;
	driver->_maxSpeed = 0;
	driver->_targetPosition = 0;

	driver->_s1 = 0;
	driver->_s2 = 0;
	driver->_s3 = 0;
	driver->_k = 0;

	/* ------------ Переменные VELOCITY_MODE ------------- */

	driver->_speedPlannerTime = 0;
	driver->_curSpeed = 0;
	driver->_targetSpeed = 0;
	driver->_stopFlag = false;

	/* ------ Переменные алгоритма плавного разгона ------ */

	driver->_c0 = 0;
	driver->_cn = 0;
	driver->_n = 0;
	driver->N = 0;
	driver->_cmin = 1.0;
}

/** Режим автоотключения мотора при достижении позиции - true (по умолч. false)
 */
void setAutoPowerMode(DRIVER_StructDef* driver, bool mode)
{
	driver->_autoPower = mode;
}

/** Установка режима работы, mode:
 * 	FOLLOW_POS - следование к позиции setTarget(...)
 * 	KEEP_SPEED - удержание скорости setSpeed(...)
 */
void setMoveMode(DRIVER_StructDef* driver, runMode_t mode)
{
	driver->_moveMode = mode;
}

/** Функция сброса всех таймеров
 */
void resetTimers(DRIVER_StructDef* driver)
{
	driver->_speedPlannerTime = driver->_positionPlannerTime = driver->_prevTime = getMicros();
}

/** Установка текущей скорости мотора в ноль
 */
void resetMotor(DRIVER_StructDef* driver)
{
	driver->_curSpeed = 0;
}

/** Задать направление мотора
 * 	dir = 1 - по часовой стрелки независимо
 * 	dir = -1 - против часовой стрелки независимо
 */
void setDriverDir(DRIVER_StructDef* driver, uint8_t dir)
{
	setDir(driver->stepper, dir);
}

/** Инвертировать поведение пина DIR мотора - true (по умолч. false)
 */
void invertDriverPinDir(DRIVER_StructDef* driver)
{
	invertPinDir(driver->stepper);
}

/** Инвертировать поведение EN пина - true (по умолч. false)
 */
void invertDriverPinEn(DRIVER_StructDef* driver)
{
	invertPinEn(driver->stepper);
}

/** Получить минимальный период, с которым нужно вызывать tick при заданной макс. скорости
 */
uint16_t getMinPeriod(DRIVER_StructDef* driver)
{
	float curSpeed;

	if (driver->_moveMode == VELOCITY_MODE)
	{
		curSpeed = fabs(driver->_targetSpeed);
		if (fabs(driver->_curSpeed) > curSpeed) curSpeed = fabs(driver->_curSpeed);
	}
	else curSpeed = driver->_maxSpeed;

	return (1000000.0 / curSpeed);
}

/* -------------------------------------------- Инициализация и параметры ---------------------------------------------- */

/* ------------------------------------------- Главные управляющие функции --------------------------------------------- */

/** Здесь происходит движение мотора, функция должна вызываться как можно чаще,
 * 	имеется встроенный таймер на тиках микропроцессора
 * 	Возвращает BUSY, если мотор запущен в режиме POSITION_MODE или VELOCITY_MODE
 */
driver_state_t tickDriver(DRIVER_StructDef* driver)
{
	if (driver->_workState == DRIVER_BUSY)
	{
		driver->tickUs = getMicros();

		/* POSITION_MODE */
		if (driver->_moveMode == POSITION_MODE && driver->_accel != 0) plannerPositionMode(driver);

		/* VELOCITY_MODE */
		if (driver->_moveMode == VELOCITY_MODE && driver->_accel != 0) plannerVelocityMode(driver);

		/* Основной таймер степпера */
		if (driver->stepTime != 0 && driver->tickUs - driver->_prevTime >= driver->stepTime)
		{
			driver->_prevTime = driver->tickUs;

			/* Проверка достижения целевой координаты, проверка остановки для быстрого планировщика, а также работы без ускорения */
			if (driver->_moveMode == POSITION_MODE && driver->_targetPosition == driver->stepper->pos)
            {
				brake(driver);
				return driver->_workState;
            }

			step(driver);  // двигаем мотор
		}
	}

	return driver->_workState;
}

/** Включить мотор (пин EN)
 */
void enableDriver(DRIVER_StructDef* driver)
{
	driver->_workState = DRIVER_READY;
	resetTimers(driver);
	enableStepper(driver->stepper);
}

/** Выключить мотор (пин EN)
 */
void disableDriver(DRIVER_StructDef* driver)
{
	driver->_workState = DRIVER_BRAKE;
	disableStepper(driver->stepper);
}

/** Плавная остановка с заданным ускорением
 */
void stop(DRIVER_StructDef* driver)
{
	if (driver->_workState == DRIVER_BUSY)
	{
		resetTimers(driver);

	    if (driver->_moveMode == POSITION_MODE)
	    {
	    	if (driver->_accel != 0)
	    	{
	    		brake(driver);
	    		return;
	    	}

	        driver->_curSpeed = 1000000.0f / driver->stepTime * driver->stepper->dir;
	        setTargetPos(driver, driver->stepper->pos + (float)driver->_curSpeed * driver->_curSpeed * driver->_accel / 1000000.0 * driver->stepper->dir);
//	        setMaxSpeed(driver, fabs(driver->_accelSpeed));
//	        driver->_stopSpeed = fabs(driver->_curSpeed);

//#ifdef SMOOTH_ALGORITHM
//	        _n = (float)_accelSpeed * _accelSpeed * _accelInv;
//#endif

	    }
	    else
	    {
	    	setSpeed(driver, 0);
	    }
	}
}

/* Резкая остановка + выключение мотора
 */
void brake(DRIVER_StructDef* driver)
{
	driver->_workState = DRIVER_READY;
	driver->stepTime = 0;
	driver->_curSpeed = 0;
	driver->_n = 0;

	resetMotor(driver);

	if(driver->_autoPower == true) disableDriver(driver);
}

/** Резкая остановка + сброс позиции в 0 (функция для работы с концевиками)
 */
void reset(DRIVER_StructDef* driver)
{
	brake(driver);
	setCurrentPos(driver, 0);
}

/** Сделать шаг мотором
 */
void step(DRIVER_StructDef* driver)
{
	doStep(driver->stepper);
}

/** Возвращает статус драйвера мотора:
 * 	INIT - в состоянии инициализации
 * 	READY - готов принять новую команду
 * 	BUSY - в движении
 * 	BRAKE - был сброшен
 * 	ERR - в ошибке
 */
driver_state_t getStatusDriver(DRIVER_StructDef* driver)
{
	return driver->_workState;
}

/* ------------------------------------------- Главные управляющие функции --------------------------------------------- */

/* -------------------------------------- Функции для работы режима POSITION_MODE -------------------------------------- */

/**	Планировщик скорости для POSITION_MODE с разгоном и торможением
 */
void plannerPositionMode(DRIVER_StructDef* driver)
{
	int8_t dir = driver->stepper->dir;
	int8_t err = _sign((int32_t)(driver->_s1 - driver->_k));

	if (driver->tickUs - driver->_positionPlannerTime >= driver->stepTime)
	{
		/* Обновление переменной времени планировщика */
		driver->_positionPlannerTime = driver->tickUs;

		/* Основная логика разгона и торможения */
		if(driver->_k < driver->_s1 || driver->_k > driver->_s2)
		{
			if(driver->_n == 0) driver->_cn = driver->_c0;
			else if(driver->_n == 1) driver->_cn = 0.4056 * driver->_c0;
			else if(driver->_n > 1)
			{
				driver->_cn = driver->_cn * (1 - err * 2.0 / (4.0 * driver->_n + err));
			}

			driver->stepTime = (uint32_t)(driver->_cn) - STEP_TIME;
			driver->_curSpeed = dir * 1000000.0 / driver->_cn;

			driver->_n += err;
		}

		/* Условие достижения максимальной скорости */
		if(driver->_k == driver->_s1 && driver->_s1 != driver->_s2)
		{
			driver->_curSpeed = dir * driver->_maxSpeed;
			driver->stepTime = 1000000.0 / driver->_maxSpeed;
		}

		/* Особый случай первого шага при торможении */
		if(driver->_k == driver->_s2)
		{
			driver->stepTime = (uint32_t)(driver->_cn) - STEP_TIME;
			driver->_curSpeed = dir * 1000000.0 / driver->_cn;

			driver->_n --;
		}

		driver->_k ++;
	}
}

/** Установка целевой позиции в шагах (для режима POSITION_MODE)
 * 	и движение к указанной цели с максимальной скоростью
 */
param_change_t setTargetPos(DRIVER_StructDef* driver, int32_t target_pos)
{
	uint32_t steps = abs(target_pos - driver->stepper->pos);

	/* Если драйвер в режиме POSITION_MODE */
	if(driver->_moveMode != POSITION_MODE) return PARAM_CHANGE_ERR;

	/* Если мотор в движении */
	if(driver->_workState == DRIVER_BUSY) return PARAM_CHANGE_ERR;

	driver->_targetPosition = target_pos;

	if(driver->_targetPosition != driver->stepper->pos)
	{
		driver->stepper->dir = (driver->_targetPosition > driver->stepper->pos) ? 1 : -1;

		if (driver->_accel == 0 || driver->_maxSpeed < MIN_SPEED_POS_MODE)
		{
			driver->stepTime = 1000000.0 / driver->_maxSpeed;
		}
		else
		{
			driver->N = driver->_maxSpeed * driver->_maxSpeed / (2.0 * driver->_accel);

			if(steps > 2 * driver->N)
			{
				driver->_s1 = driver->N;
				driver->_s2 = driver->_s1 + steps - 2 * driver->N;
				driver->_s3 = driver->_s2 + driver->N;
			}
			else
			{
				driver->_s1 = steps / 2;
				driver->_s2 = driver->_s1 + (steps % 2 == 0) ? 0 : 1;
				driver->_s3 = driver->_s2 + driver->_s1;
			}

			driver->_k = 0;
		}

		driver->_workState = DRIVER_BUSY;
	}

	return PARAM_CHANGE_OK;
}

/* Установка целевой позиции в градусах
 */
param_change_t setTargetPosDeg(DRIVER_StructDef* driver, float target_pos_deg)
{
	return setTargetPos(driver, target_pos_deg * driver->_stepsPerDeg);
}

/** Получение целевой позиции в шагах
 */
int32_t getTargetPos(DRIVER_StructDef* driver)
{
	return driver->_targetPosition;
}

/** Получение целевой позиции в градусах
 */
float getTargetPosDeg(DRIVER_StructDef* driver)
{
	return (float)driver->_targetPosition / driver->_stepsPerDeg;
}

/** Установка максимальной скорости (по модулю) в шагах/секунду (для режима POSITION_MODE)
 *  по умолчанию 300
 */
param_change_t setMaxSpeed(DRIVER_StructDef* driver, float speed)
{
	if(driver->_workState == DRIVER_BUSY) return PARAM_CHANGE_ERR;

	/* Ограничения минимальной скорости - 1 шаг/час */
	driver->_maxSpeed = fmax(fabs(speed), MIN_STEP_SPEED);

	return PARAM_CHANGE_OK;
}

/** Установка максимальной скорости (по модулю) в градусах/секунду (для режима POSITION_MODE)
 */
param_change_t setMaxSpeedDeg(DRIVER_StructDef* driver, float speed)
{
	return setMaxSpeed(driver, fabs(speed) * driver->_stepsPerDeg);
}

/** Установка ускорения в шагах в секунду^2 (для режима POSITION_MODE)
 * 	при значении 0 ускорение отключается и мотор работает
 * 	по профилю постоянной максимальной скорости setMaxSpeed().
 * 	по умолчанию 500 ш/с^2
 */
param_change_t setAcceleration(DRIVER_StructDef* driver, uint16_t accel)
{
	if(driver->_workState == DRIVER_BUSY) return PARAM_CHANGE_ERR;

	driver->_accel = abs(accel);

	/* Считаем значение _c0 и _N по алгоритму плавного старта */
	if(accel != 0)
	{
		driver->_c0 = 1000000.0 * sqrt(2.0 / driver->_accel);
	}
	else driver->_c0 = 0;

	return PARAM_CHANGE_OK;
}

/** Установка ускорения в градусах в секунду^2 (для режима POSITION_MODE)
 * 	при значении 0 ускорение отключается и мотор работает
 * 	по профилю постоянной максимальной скорости setMaxSpeedDeg().
 */
param_change_t setAccelerationDeg(DRIVER_StructDef* driver, float accel)
{
	return setAcceleration(driver, accel * driver->_stepsPerDeg);
}

/** Установка текущей позиции мотора в шагах
 */
param_change_t setCurrentPos(DRIVER_StructDef* driver, int32_t pos)
{
	if(driver->_workState == DRIVER_BUSY) return PARAM_CHANGE_ERR;

	driver->stepper->pos = pos;
	driver->_curSpeed = 0;

	return PARAM_CHANGE_OK;
}

/** Установка текущей позиции мотора в градусах
 */
param_change_t setCurrentPosDeg(DRIVER_StructDef* driver, float pos)
{
	return setCurrentPos(driver, (float)pos * driver->_stepsPerDeg);
}

/** Чтение текущей позиции мотора в шагах
 */
int32_t getCurrentPos(DRIVER_StructDef* driver)
{
	return driver->stepper->pos;
}

/** Чтение текущей позиции мотора в градусах
 */
float getCurrentPosDeg(DRIVER_StructDef* driver)
{
	return ((float)driver->stepper->pos) / driver->_stepsPerDeg;
}

/* -------------------------------------- Функции для работы режима POSITION_MODE -------------------------------------- */

/* -------------------------------------- Функции для работы режима VELOCITY_MODE -------------------------------------- */

/** Планировщик скорости для VELOCITY_MODE
 * 	Планировщик не вызывается, если ускорение _accel = 0;
 * 	Если ускорение _accel = 0, то setSpeed(...) задает сразу скорость мотора
 * 	и старт будет резким!
 * 	Поддерживает горячую смену скорости
 */
void plannerVelocityMode(DRIVER_StructDef* driver)
{
	float err = driver->_targetSpeed - driver->_curSpeed;
	int8_t dir = driver->stepper->dir;

	if (driver->_stopFlag == true && fabs(driver->_curSpeed) <= MIN_STEP_SPEED)
	{
		brake(driver);
		return;
	}

	if (driver->tickUs - driver->_speedPlannerTime >= driver->stepTime)
	{
		/* Обновление переменной времени планировщика */
		driver->_speedPlannerTime = driver->tickUs;

		/* Основная логика разгона/торможения/смены скорости */
		if(driver->_n == 0) driver->_cn = driver->_c0;
		else if(driver->_n == 1) driver->_cn = 0.4056 * driver->_c0;
		else if(driver->_n > 1)
		{
			driver->_cn = driver->_cn * (1 - _signf(err * dir) * 2.0 / (4.0 * driver->_n + _signf(err * dir)));
		}
		else if(driver->_n < 0)
		{
			driver->_curSpeed = 0;

			if(driver->_stopFlag == true)
			{
				brake(driver);
				return;
			}
			else
			{
				driver->stepper->dir = _signf(driver->_targetSpeed);
			}
		}

		driver->stepTime = (uint32_t)(driver->_cn) - STEP_TIME;
		driver->_curSpeed = dir * 1000000.0 / driver->_cn;

		driver->_n += _signf(err * dir);
	}
}

/** Установка целевой скорости в шагах/секунду (для режима VELOCITY_MODE)
 * 	В соответствии с минимальной скоростью, определенной в макросах
 */
param_change_t setSpeed(DRIVER_StructDef* driver, float speed)
{
	/* Если драйвер в режиме POSITION_MODE */
	if(driver->_moveMode != VELOCITY_MODE) return PARAM_CHANGE_ERR;

	driver->_targetSpeed = speed;
	driver->_stopFlag = (driver->_targetSpeed == 0);

	if(driver->_targetSpeed == 0 && driver->_accel == 0)
	{
		driver->stepTime = 0;
		brake(driver);

		return PARAM_CHANGE_OK;
	}

	driver->stepper->dir = (speed > 0) ? 1 : -1;

	/* Ограничение минимальной скорости */
	if (fabs(speed) < MIN_STEP_SPEED) driver->_targetSpeed = MIN_STEP_SPEED * driver->stepper->dir;

	if(driver->_accel == 0)
	{
		driver->_curSpeed = driver->_targetSpeed;
		driver->stepTime = fabs(1000000.0 / driver->_targetSpeed) - STEP_TIME;
	}
	else
	{
		if(driver->_curSpeed == 0)
		{
			driver->stepper->dir = _signf(driver->_targetSpeed);
		}
		else driver->stepper->dir = _signf(driver->_curSpeed);
	}

	driver->_workState = DRIVER_BUSY;

	return PARAM_CHANGE_OK;
}

/** Установка целевой скорости в градусах/секунду (для режима VELOCITY_MODE)
 */
param_change_t setSpeedDeg(DRIVER_StructDef* driver, float speed)
{
	return setSpeed(driver, speed * driver->_stepsPerDeg);
}

/** Получение целевой скорости в шагах/секунду (для режима VELOCITY_MODE)
 */
float getTargetSpeed(DRIVER_StructDef* driver)
{
	return driver->_targetSpeed;
}

/** Получение целевой скорости в градусах/секунду (для режима VELOCITY_MODE)
 */
float getTargetSpeedDeg(DRIVER_StructDef* driver)
{
	return ((float)getTargetSpeed(driver) / driver->_stepsPerDeg);
}

/** Получение текущей скорости в шагах/секунду (для режима VELOCITY_MODE)
 */
float getCurrentSpeed(DRIVER_StructDef* driver)
{
	return driver->_curSpeed;
}

/** Получение текущей скорости в градусах/секунду (для режима VELOCITY_MODE)
 */
float getCurrentSpeedDeg(DRIVER_StructDef* driver)
{
	return ((float)getCurrentSpeed(driver) / driver->_stepsPerDeg);
}

/* -------------------------------------- Функции для работы режима VELOCITY_MODE -------------------------------------- */

/* ---------------------------------------------- Дополнительные функции ----------------------------------------------- */

/** Функция преобразования числа из заданного диапазона [in_min, in_max]
 * 	в диапазон [out_min, out_max], при этом конечный диапазон может быть
 * 	любой, out_min не обязательно меньше out_max
 * 	на выходе преобразованная величина не обязана принадлежать
 * 	конечному отрезку [out_min, out_max]
 */
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** Функция определения принадлежности значения числа указанному отрезку
 * 	отрезок [in_min, in_max], в функцие необязательно in_min < in_max
 * 	возможна и обратная ситуация
 */
int32_t constrain(int32_t x, int32_t in_min, int32_t in_max)
{
	if(in_min < in_max)
	{
		return (x < in_min) ? in_min : ((x > in_max) ? in_max : x);
	}
	else return (x < in_max) ? in_max : ((x > in_min) ? in_min : x);
}

/* ---------------------------------------------- Дополнительные функции ----------------------------------------------- */

/* ---------------------------------------------------- РАЗРАБОТКА ----------------------------------------------------- */

//--

/* ---------------------------------------------------- РАЗРАБОТКА ----------------------------------------------------- */

#endif /* INC_DRIVER_H_ */
