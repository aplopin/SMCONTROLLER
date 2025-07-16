#ifndef INC_PLANNER_H_
#define INC_PLANNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "driver.h"

#define GP_MIN_US 300000  			//< период, длиннее которого мотор можно резко тормозить или менять скорость
#define AXES 2 						//< число осей интерполятора

/* Тип данных - состояние работы интерполяционного планировщика
 * PLANNER_ERR - планировщик в ошибке
 * PLANNER_OK - планировщика закончил работу без ошибки
 * PLANNER_INIT - планировщик инициализирован
 * PLANNER_READY - планировщик готов к работе, моторы не запущены
 * PLANNER_RUN - планировщик в работе, моторы запущены
 */
typedef enum
{
	PLANNER_ERR = -1,
	PLANNER_OK = 0,
	PLANNER_INIT,
	PLANNER_READY,
	PLANNER_RUN,
	PLANNER_PAUSE,
	PLANNER_END,
	PLANNER_STOP

} planner_state_t;

typedef enum
{
	BRAKING = -1,
	UNIFORM = 0,
	ACCELERATION = 1,
	SPECIAL = 2,
	STAND

} planner_phase_t;

/** Тип данных - статус изменения параметров драйвера шагового мотора
 * 	PLANNER_PARAM_CHANGE_ERR - параметры не изменены, ошибка
 * 	PLANNER_PARAM_CHANGE_OK - параметры изменены, обишок нет
 */
typedef enum
{
	PLANNER_PARAM_CHANGE_ERR = -1,
	PLANNER_PARAM_CHANGE_OK = 0

} planner_param_change_t;

/* Структура планировщика */
typedef struct
{
	/* Указатели на экземляры структуры драйверов моторов
	 * После инициализации необходмо вызвать функцию addDriver(),
	 * чтобы сопоставить указатель на нужный драйвер с номером оси
	 * планировщика
	 */
	DRIVER_StructDef* driver[AXES];

	/* Отсчет времени в планировщике скорости */
	uint32_t _PlannerTime;

	/* ------------ Переменные интерполятора ------------- */

	/* Текущая скорость интерполятора */
	int32_t _curSpeed;

	/* Максимальная скорость интерполятора */
	int32_t _maxSpeed;

	/* Ускорение интерполятора */
	int32_t _accel;

	/* Стоп - флаг */
	bool _stopFlag;

	/* Пауза - флаг*/
	bool _pauseFlag;

	/* Фаза движения */
	planner_phase_t _phase;

	/* Переменная состояния драйвера */
	planner_state_t _workState;

	/* Текущее время драйвера с момента начала движения */
	uint32_t tickUs;

	/* Время предыдущего вхождения в главную функцию драйвера мотора tickPlanner */
	uint32_t _prevTime;

	/* Время между шагами при постоянной скорости мотора */
	uint32_t stepTime;

	/* ------ Переменные алгоритма плавного разгона ------ */

	/* Начальное время планировщика скорости (мкс) */
	float _c0;

	/* n-тое время планировщика (мкс) */
	float _cn;

	/* Счетчик шагов планировщика скорости */
	int32_t _n;

	/* Количество шагов до целевой скорости */
	uint32_t N;

	/* Минимальное время планировщика скорости, вычисляется из максимальной скорости */
	float _cmin;

	/* Количетсво шагов разгона */
	uint32_t _s1;

	/* Флаг шагов равномерного движения
	 * -1 - нужно сделать много шагов
	 * 0 - нужно сделать 0 шагов
	 * 1 - нужно сделать один шаг
	 */
	int8_t _s2;

	/* Количество шагов торможения */
	uint32_t _s3;

	/* Счетчик шагов планировщика скорости в режиме POSITION_MODE */
	uint32_t _k;

	/* Период шагов */
	uint32_t us;

	/* Цель, переменная Брезенхема, смещение по оси */
	int32_t tar[AXES], nd[AXES], dS[AXES];

	/* Шаги на текущем участке, дробные шаги, длина участка, (s1,s2,so1) - для расчёта трапеций */
	int32_t step, substep, S, s1, s2, so1;

	/* Таймер тика, время первого шага, мин. период шага, сдвинутый на 10 us */
	uint32_t tmr, us0, usMin, us10;

	/* Ускорение, буфер ускорения для применения после остановки */
	uint16_t a, na;

	/* Шагов до остановки */
	int16_t stopStep;

	/* Скорость, буфер скорости для применения после остановки */
	float V, nV;

	/* Переменная статуса */
	uint8_t status;

	/* Cтатус, ось в режиме скорости */
	uint8_t speedAxis;

	/* Сдвиг для повышения разрешения Брезенхема */
	uint8_t shift;

	/* Флаг готовности ЧЕГО-ТО? */
	bool readyF;

	/* Флаг изменения настроек */
	bool changeSett;

	/* --- Переменные алгоритма линейной интерполяции ---- */


} PLANNER_StructDef;

/* --------------------------------------- Инициализация --------------------------------------- */

void plannerInit(PLANNER_StructDef* planner);
void plannerFunctionsInit(timeFunction_uint32_t_ptr function1);
void addDriver(PLANNER_StructDef* planner, DRIVER_StructDef* driver, uint8_t axis);

/* ------------------------------------ Управляющие функции ------------------------------------ */

void tickPlanner(PLANNER_StructDef* planner);
void plannerSteps(PLANNER_StructDef* planner);
void plannerVelocity(PLANNER_StructDef* planner);
void plannerPhase(PLANNER_StructDef* planner);

planner_param_change_t setPlannerAcceleration(PLANNER_StructDef* planner, float accel);
planner_param_change_t setPlannerMaxSpeed(PLANNER_StructDef* planner, float speed);

void brakePlanner(PLANNER_StructDef* planner);
void pausePlanner(PLANNER_StructDef* planner);
void resumePlanner(PLANNER_StructDef* planner);
void stopPlanner(PLANNER_StructDef* planner);
void calculatePlannerInitialParam(PLANNER_StructDef* planner);


#endif /* INC_PLANNER_H_ */
