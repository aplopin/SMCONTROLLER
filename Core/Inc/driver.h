/** Библиотека драйвера шагового мотора
 * 	Файл: driver.h
 *
 * 	Для работы библиотеки требуется передать функцию для получения
 * 	реального времени с дискретностью 1 мкс от настроенного таймера микроконтроллера
 * 	указательность на функцию получения времени - micros
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

#define degPerMinute(x) ((x) / 60.0f)
#define degPerHour(x) ((x) / 3600.0f)
#define _sign(x) ((x) >= 0 ? 1 : -1)  // знак числа

#define MIN_SPEED_POS_MODE 5  // мин. скорость для движения в POSITION_MODE с ускорением
#define MAX_PERIOD_POS_MODE (1000000L / MIN_SPEED_POS_MODE)
#define MIN_STEP_SPEED (1.0 / 3600)  // мин. скорость 1 шаг в час

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

	/* Переменные для планировщика скорости */
	float _accelTime;
	uint16_t _speedPlannerPrd; // Период планировщика скорости
	uint32_t _speedPlannerTime;
	uint32_t _plannerTime;

	/* Стоп - флаг */
	bool _stopFlag;

	/* Количество шагов для поворота на 360 градусов */
	float _stepsPerDeg;

	/*  */
	uint32_t _prevTime;

	/* Текущая скорость при разгоне */
	float _accelSpeed;

	/* Целевая координата */
	int32_t _target;

	/* Текущее время драйвера с момента начала движения */
	volatile uint32_t tickUs;

	/* Переменная состояния драйвера */
	workState_t _workState;

	/* Переменная - флаг работы режима автоотключения драйвера после завершения движения */
	bool _autoPower;

	/* Текущая скорость при ториожении */
	float _stopSpeed;

	/* Максимальная скорость мотора для режима POSITION_MODE */
	float _maxSpeed;

	/* Скорость мотора для режима VELOCITY_MODE */
	float _speed;

	/* Ускорение мотора */
	uint16_t _accel;

	/*  */
	float _accelInv;

	/* Текущий режим работы драйвера */
	runMode_t _curMode;

	/* Время между шагами при постоянной скорости мотора */
	uint32_t stepTime;

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
void driverInit(DRIVER_StructDef* driver, STEPPER_StructDef* stepper, DRIVER_LIMIT_SWITCH_PINS_StructDef* pins, uint32_t stepsPerRev);
void setAutoPowerMode(DRIVER_StructDef* driver, bool mode);
void setRunMode(DRIVER_StructDef* driver, runMode_t mode);
void resetTimers(DRIVER_StructDef* driver);
void resetMotor(DRIVER_StructDef* driver);
void setDriverDir(DRIVER_StructDef* driver, uint8_t dir);
void invertDriverPinDir(DRIVER_StructDef* driver);
void invertDriverPinEn(DRIVER_StructDef* driver);
uint16_t getMinPeriod(DRIVER_StructDef* driver);

/* --------------------------------------- Инициализация --------------------------------------- */

/* ------------------------------------ Управляющие функции ------------------------------------ */

workState_t tickDriver(DRIVER_StructDef* driver);
void enableDriver(DRIVER_StructDef* driver);
void disableDriver(DRIVER_StructDef* driver);
void stop(DRIVER_StructDef* driver);
void brake(DRIVER_StructDef* driver);
void reset(DRIVER_StructDef* driver);
void step(DRIVER_StructDef* driver);
workState_t getStatusDriver(DRIVER_StructDef* driver);

/* ------------------------------------ Управляющие функции ------------------------------------ */

/* --------------------------------------- POSITION_MODE --------------------------------------- */

void plannerPositionMode(DRIVER_StructDef* driver);						// Дописана, требует анализа!
void setTarget(DRIVER_StructDef* driver, int32_t target_pos);
void setTargetDeg(DRIVER_StructDef* driver, float target_pos_deg);
int32_t getTarget(DRIVER_StructDef* driver);
float getTargetDeg(DRIVER_StructDef* driver);
void setMaxSpeed(DRIVER_StructDef* driver, float speed);
void setMaxSpeedDeg(DRIVER_StructDef* driver, float speed);
void setAcceleration(DRIVER_StructDef* driver, uint16_t accel);
void setAccelerationDeg(DRIVER_StructDef* driver, float accel);
void setCurrent(DRIVER_StructDef* driver, int32_t pos);
void setCurrentDeg(DRIVER_StructDef* driver, float pos);
int32_t getCurrent(DRIVER_StructDef* driver);
float getCurrentDeg(DRIVER_StructDef* driver);

/* --------------------------------------- POSITION_MODE --------------------------------------- */

/* --------------------------------------- VELOCITY_MODE --------------------------------------- */

void plannerVelocityMode(DRIVER_StructDef* driver);						// Дописана, требует анализа!
void setSpeed(DRIVER_StructDef* driver, float speed);					// Дописана, требует анализа!
void setSpeedDeg(DRIVER_StructDef* driver, float speed);
float getSpeed(DRIVER_StructDef* driver);
float getSpeedDeg(DRIVER_StructDef* driver);

/* --------------------------------------- VELOCITY_MODE --------------------------------------- */

/* ---------------------------------- Дополнительные функции ----------------------------------- */

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t constrain(int32_t x, int32_t in_min, int32_t in_max);

/* ---------------------------------- Дополнительные функции ----------------------------------- */

/* --------------------------------------- Прототипы функций библиотеки driver.h --------------------------------------- */

/* -------------------------------------------- Инициализация и параметры ---------------------------------------------- */

/** Функция приема указателя на функцию получения времени в микросекундах.
 *  Используется таймер микроконтроллера и его регистр счетчика - TIMx->CNT
 *  function ОБЯЗАТЕЛЬНО должна быть определена в файле main.c
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
void driverInit(DRIVER_StructDef* driver, STEPPER_StructDef* stepper, DRIVER_LIMIT_SWITCH_PINS_StructDef* pins, uint32_t stepsPerRev)
{
	driver->stepper = stepper;

	driver->driver_pins = pins;

	driver->_accelTime = 0;
	driver->_speedPlannerPrd = 15000;
	driver->_speedPlannerTime = 0;
	driver->_plannerTime = 0;

	driver->_stopFlag = false;
	driver->_stepsPerDeg = stepsPerRev / 360.0;
	driver->_prevTime = 0;
	driver->_accelSpeed = 0;
	driver->_target = 0;
	driver->tickUs = 0;
	driver->_workState = INIT;
	driver->_autoPower = false;
	driver->_stopSpeed = 0;
	driver->_maxSpeed = 300;
	driver->_speed = 0;
	driver->_accel = 300;
	driver->_accelInv = 0;
	driver->_curMode = POSITION_MODE;

	driver->stepTime = 10000;
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
void setRunMode(DRIVER_StructDef* driver, runMode_t mode)
{
	driver->_curMode = mode;
}

/** Функция сброса всех таймеров
 */
void resetTimers(DRIVER_StructDef* driver)
{
	driver->_speedPlannerTime = driver->_plannerTime = driver->_prevTime = getMicros();
}

/** Установка текущей скорости мотора в ноль
 */
void resetMotor(DRIVER_StructDef* driver)
{
	driver->_accelSpeed = 0;
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

	if (driver->_curMode == VELOCITY_MODE)
	{
		curSpeed = fabs(driver->_speed);
		if (fabs(driver->_accelSpeed) > curSpeed) curSpeed = fabs(driver->_accelSpeed);
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
workState_t tickDriver(DRIVER_StructDef* driver)
{
	if (driver->_workState == BUSY)
	{
		driver->tickUs = getMicros();

		/* POSITION_MODE */
		if (driver->_curMode == POSITION_MODE && driver->_accel != 0 && driver->_maxSpeed >= MIN_SPEED_POS_MODE) plannerPositionMode(driver);

		/* VELOCITY_MODE */
		if (driver->_curMode == VELOCITY_MODE && driver->_accel != 0) plannerVelocityMode(driver);

		/* Основной таймер степпера */
		if (driver->stepTime != 0 && driver->tickUs - driver->_prevTime >= driver->stepTime)
		{
			driver->_prevTime = driver->tickUs;

//#ifdef SMOOTH_ALGORITHM
//                // плавный планировщик вызывается каждый шаг. Проверка остановки
//                if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP && !plannerSmooth()) {
//                    brake();
//                    return false;
//                }
//#endif
			/* Проверка достижения целевой координаты, проверка остановки для быстрого планировщика, а также работы без ускорения */
			if (driver->_curMode == POSITION_MODE && driver->_target == driver->stepper->pos)
            {
				brake(driver);
				return BRAKE;
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
	driver->_workState = BUSY;
	driver->_stopSpeed = 0;

	resetTimers(driver);

	enableStepper(driver->stepper);
}

/** Выключить мотор (пин EN)
 */
void disableDriver(DRIVER_StructDef* driver)
{
	disableStepper(driver->stepper);
}

/** Плавная остановка с заданным ускорением
 */
void stop(DRIVER_StructDef* driver)
{
	if (driver->_workState == BUSY)
	{
		resetTimers(driver);

	    if (driver->_curMode == POSITION_MODE)
	    {
	    	if (driver->_accel != 0)
	    	{
	    		brake(driver);
	    		return;
	    	}

	        driver->_accelSpeed = 1000000.0f / driver->stepTime * driver->stepper->dir;
	        setTarget(driver, driver->stepper->pos + (float)driver->_accelSpeed * driver->_accelSpeed * driver->_accel / 1000000.0 * driver->stepper->dir);
//	        setMaxSpeed(driver, fabs(driver->_accelSpeed));
	        driver->_stopSpeed = fabs(driver->_accelSpeed);

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
	driver->_workState = RESET;
	driver->_stopSpeed = 0;

	resetMotor(driver);

	if(driver->_autoPower == true) disableDriver(driver);
}

/** Резкая остановка + сброс позиции в 0 (функция для работы с концевиками)
 */
void reset(DRIVER_StructDef* driver)
{
	brake(driver);
	setCurrent(driver, 0);
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
workState_t getStatusDriver(DRIVER_StructDef* driver)
{
	return driver->_workState;
}

/* ------------------------------------------- Главные управляющие функции --------------------------------------------- */

/* -------------------------------------- Функции для работы режима POSITION_MODE -------------------------------------- */

uint16_t _plannerPrd = 15000;

/**	Планировщик скорости для POSITION_MODE с разгоном и торможением
 */
void plannerPositionMode(DRIVER_StructDef* driver)
{
	if (driver->tickUs - driver->_plannerTime >= _plannerPrd)
	{
		driver->_plannerTime += _plannerPrd; // ~110 us

		int32_t err = driver->_target - driver->stepper->pos; // "ошибка"

		bool thisDir = (driver->_accelSpeed * driver->_accelSpeed * driver->_accel / 1000000.0 >= abs(err)); // пора тормозить

		driver->_accelSpeed += (driver->_accel * (_plannerPrd / 1000000.0) * (thisDir ? -_sign(driver->_accelSpeed) : _sign(err))); // разгон/торможение

		if (driver->_stopSpeed == 0) driver->_accelSpeed = constrain(driver->_accelSpeed, -driver->_maxSpeed, driver->_maxSpeed); // ограничение
		else driver->_accelSpeed = constrain(driver->_accelSpeed, -driver->_stopSpeed, driver->_stopSpeed);

		/* Ограничение на мин. скорость */
		if (fabs(driver->_accelSpeed) > MIN_SPEED_POS_MODE)
		{
			driver->stepTime = fabs(1000000.0 / driver->_accelSpeed);
		}
		else driver->stepTime = MAX_PERIOD_POS_MODE;

		driver->stepper->dir = _sign(driver->_accelSpeed); /* Направление для шагов */
	}
}

/** Установка целевой позиции в шагах и градусах (для режима POSITION_MODE)
 * и движение к указанной цели с максимальной скоростью
 */
void setTarget(DRIVER_StructDef* driver, int32_t target_pos)
{
	driver->_target = target_pos;

	if(driver->_target != driver->stepper->pos)
	{
		if (driver->_accel == 0 || driver->_maxSpeed < MIN_SPEED_POS_MODE)
		{
			driver->stepTime = 1000000.0 / driver->_maxSpeed;
			driver->stepper->dir = (driver->_target > driver->stepper->pos) ? 1 : -1;
		}

		enableDriver(driver);
	}
}

/* Установка целевой позиции в градусах
 */
void setTargetDeg(DRIVER_StructDef* driver, float target_pos_deg)
{
	setTarget(driver, target_pos_deg * driver->_stepsPerDeg);
}

/** Получение целевой позиции в шагах
 */
int32_t getTarget(DRIVER_StructDef* driver)
{
	return driver->_target;
}

/** Получение целевой позиции в градусах
 */
float getTargetDeg(DRIVER_StructDef* driver)
{
	return (float)driver->_target / driver->_stepsPerDeg;
}

/** Установка максимальной скорости (по модулю) в шагах/секунду (для режима POSITION_MODE)
 *  по умолчанию 300
 */
void setMaxSpeed(DRIVER_StructDef* driver, float speed)
{
	/* Ограничения минимальной скорости - 1 шаг/час */
	driver->_maxSpeed = fmax(fabs(speed), MIN_STEP_SPEED);

	/* Cчитаем stepTime для низких скоростей или отключенного ускорения */
	if (driver->_accel == 0 || driver->_maxSpeed < MIN_SPEED_POS_MODE) driver->stepTime = 1000000.0 / driver->_maxSpeed;

//#ifdef SMOOTH_ALGORITHM
//        _cmin = 1000000.0 / _maxSpeed;
//        if (_n > 0) {
//            _n = (float)_accelSpeed * _accelSpeed * _accelInv;
//            plannerSmooth();
//        }
//#else
//        // период планировщка в зависимости от макс. скорости
//        _plannerPrd = map((int)_maxSpeed, 1000, 20000, 15000, 1000);
//        _plannerPrd = constrain(_plannerPrd, 15000, 1000);
//#endif

}

/** Установка максимальной скорости (по модулю) в градусах/секунду (для режима POSITION_MODE)
 */
void setMaxSpeedDeg(DRIVER_StructDef* driver, float speed)
{
	setMaxSpeed(driver, fabs(speed) * driver->_stepsPerDeg);
}

/** Установка ускорения в шагах в секунду^2 (для режима POSITION_MODE)
 * 	при значении 0 ускорение отключается и мотор работает
 * 	по профилю постоянной максимальной скорости setMaxSpeed().
 * 	по умолчанию 300
 */
void setAcceleration(DRIVER_StructDef* driver, uint16_t accel)
{
	driver->_accel = abs(accel);
	if (accel != 0) driver->_accelInv = 0.5f / accel;
	else driver->_accelInv = 0;

	//driver->_accelTime = accel / 1000000.0f; // пока эта переменная только путает!!!

//#ifdef SMOOTH_ALGORITHM
//        if (_accel) _c0 = 0.676 * sqrt(2.0 / _accel) * 1000000.0;
//        plannerSmooth();
//#endif
}

/** Установка ускорения в градусах в секунду^2 (для режима POSITION_MODE)
 * 	при значении 0 ускорение отключается и мотор работает
 * 	по профилю постоянной максимальной скорости setMaxSpeedDeg().
 * 	по умолчанию 300
 */
void setAccelerationDeg(DRIVER_StructDef* driver, float accel)
{
	setAcceleration(driver, accel * driver->_stepsPerDeg);
}

/** Установка текущей позиции мотора в шагах
 */
void setCurrent(DRIVER_StructDef* driver, int32_t pos)
{
	driver->stepper->pos = pos;
	driver->_accelSpeed = 0;
}

/** Установка текущей позиции мотора в градусах
 */
void setCurrentDeg(DRIVER_StructDef* driver, float pos)
{
	setCurrent(driver, (float)pos * driver->_stepsPerDeg);
}

/** Чтение текущей позиции мотора в шагах
 */
int32_t getCurrent(DRIVER_StructDef* driver)
{
	return driver->stepper->pos;
}

/** Чтение текущей позиции мотора в градусах
 */
float getCurrentDeg(DRIVER_StructDef* driver)
{
	return ((float)driver->stepper->pos) / driver->_stepsPerDeg;
}

/* -------------------------------------- Функции для работы режима POSITION_MODE -------------------------------------- */

/* -------------------------------------- Функции для работы режима VELOCITY_MODE -------------------------------------- */

/** Планировщик скорости для VELOCITY_MODE
 * 	Планировщик не вызывается, если ускорение _accel = 0;
 * 	Если ускорение _accel = 0, то setSpeed(...) задает сразу скорость мотора
 * 	и старт будет резким!
 */
void plannerVelocityMode(DRIVER_StructDef* driver)
{
	if (driver->tickUs - driver->_speedPlannerTime >= driver->_speedPlannerPrd)
	{
		/* Обновляем переменную времени планировщика на величину периода планировщика */
		driver->_speedPlannerTime = driver->tickUs;

		/* Формула конечной скорости с учетом знака ускорения (м/с) */
		driver->_accelSpeed += (driver->_accel * (driver->_speedPlannerPrd / 1000000.0) * _sign(driver->_speed - driver->_accelSpeed));

		driver->stepper->dir = _sign(driver->_accelSpeed);
		driver->stepTime = fabs(1000000.0 / driver->_accelSpeed);

        if (driver->_stopFlag == true && fabs(driver->_accelSpeed) <= MIN_STEP_SPEED) brake(driver);
	}
}

/** Установка целевой скорости в шагах/секунду (для режима VELOCITY_MODE)
 * 	В соответствии с минимальной скоростью, определенной в макросах
 */
void setSpeed(DRIVER_StructDef* driver, float speed)
{
	driver->_speed = speed;
	driver->_stopFlag = (speed == 0);

	if (speed == 0 && driver->_accelSpeed == 0) return; /* Если нулевая скорость */

	driver->stepper->dir = (speed > 0) ? 1 : -1;

	if (fabs(speed) < MIN_STEP_SPEED) driver->_speed = MIN_STEP_SPEED * driver->stepper->dir; /* Ограничение минимальной скорости */

	if (driver->_accel != 0) /* Плавный старт */
	{
		if (driver->_accelSpeed != driver->_speed)
		{
			int32_t speed1 = (int32_t)fabs(driver->_speed);
			int32_t speed2 = (int32_t)fabs(driver->_accelSpeed);

	        driver->_speedPlannerPrd = map(fmax(speed1, speed2), 1000, 20000, 15000, 2000);
	        driver->_speedPlannerPrd = constrain(driver->_speedPlannerPrd, 15000, 2000);

	        driver->stepTime = fabs(1000000.0 / driver->_accelSpeed);
		}
	}
	else /* Резкий старт, ускорение 0 */
	{
		if (speed == 0) /* Если задана нулевая скорость */
		{
			brake(driver);
		    return;
		}

		/* Текущая скорость мотора и время одного шага */
		driver->_accelSpeed = driver->_speed;
		driver->stepTime = fabs(1000000.0 / driver->_speed);
	}

	enableDriver(driver);
}

/** Установка целевой скорости в градусах/секунду (для режима VELOCITY_MODE)
 */
void setSpeedDeg(DRIVER_StructDef* driver, float speed)
{
	setSpeed(driver, speed * driver->_stepsPerDeg);
}

/** Получение целевой скорости в шагах/секунду (для режима VELOCITY_MODE)
 */
float getSpeed(DRIVER_StructDef* driver)
{
	return 1000000.0 / driver->stepTime * driver->stepper->dir;
}

/** Получение целевой скорости в градусах/секунду (для режима VELOCITY_MODE)
 */
float getSpeedDeg(DRIVER_StructDef* driver)
{
	return ((float)getSpeed(driver) / driver->_stepsPerDeg);
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

#endif /* INC_DRIVER_H_ */
