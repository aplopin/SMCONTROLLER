#include "planner.h"

#include "interpolator.h"

/* Статический буфер - массив шагов */
int8_t BufSteps[FIFO_STEPS_SIZE];

/* Структура FIFO буфера шагов для интерполяции оценочной функцией */
extern FIFO_StructDef fifoBufSteps;

/* --------------------------------------- Импортированные переменные из библиотеки interpolator.c --------------------------------------- */

/* ------------ Глобальные координаты станка (шаги) ------------- */

/* Координаты линейных осей станка */
extern volatile int64_t x;
extern volatile int64_t y;
extern volatile int64_t z;

/* Координаты поворотных осей станка */
extern volatile int64_t a;
extern volatile int64_t b;
extern volatile int64_t c;

/* ------------ Глобальные координаты станка (мм) ------------- */

/* Координаты линейных осей станка */
extern volatile double X;
extern volatile double Y;
extern volatile double Z;

/* Координаты поворотных осей станка */
extern volatile double A;
extern volatile double B;
extern volatile double C;

/* --------------------------------------- Импортированные переменные из библиотеки interpolator.c --------------------------------------- */

/* Указатель на функцию для измерения времени */
static timeFunction_uint32_t_ptr getPlannerMicros;

void plannerFunctionsInit(timeFunction_uint32_t_ptr function1)
{
	getPlannerMicros = function1;
}

/** Инициализация планироващика
 *
 */
void plannerInit(PLANNER_StructDef* planner)
{
	/* Указатели на структуры драйверов моторов */
	for(uint8_t i = 0; i < AXES; i ++)
	{
		planner->driver[i] = 0;
	}

	planner->_PlannerTime = 0;

	planner->_curSpeed = 0;
	planner->_maxSpeed = 20 * 160; /* 20 мм/c --> 20 * 160 шаг/c */
	planner->_accel = 10 * 200; /* Ускорение по умолчанию 10 мм/c^2 */

	planner->tickUs = 0;
	planner->stepTime = 0;
	planner->_prevTime = 0;

	planner->_stopFlag = false;
	planner->_pauseFlag = false;
	planner->_workState = PLANNER_INIT;
	planner->_phase = STAND;

	planner->_c0 = 0;
	planner->_cn = 0;
	planner->_n = 0;
	planner->N = 0;
	planner->_cmin = 1.0;

	planner->_s1 = 0;
	planner->_s2 = 0;
	planner->_s3 = 0;
	planner->_k = 0;
}

/** Здесь происходит движение мотора, функция должна вызываться как можно чаще,
 * 	имеется встроенный таймер на тиках микропроцессора
 * 	Возвращает PLANNER_RUN, процесс движения запущен
 */
void tickPlanner(PLANNER_StructDef* planner)
{
	if (planner->_workState == PLANNER_RUN)
	{
		/* Сравнение ускорения с нулем */
		if (planner->_accel > EPS) plannerVelocity(planner);

		planner->tickUs = getPlannerMicros();

		/* Основной таймер планировщика */
		if (planner->tickUs - planner->_prevTime >= planner->stepTime)
		{
			planner->_prevTime = planner->tickUs;

			/* Чтение ячейки FIFO буфера шагов и воспроизведение шагов */
			int8_t rx_bin;
			fifoRead(&fifoBufSteps, &rx_bin);

			for(uint8_t i = 0; i < AXES; i ++)
			{
				if((rx_bin & 3) == 1)
				{
					/* Устанавливаем направление и производим шаг мотором */
					planner->driver[i]->stepper->dir = 1;
					step(planner->driver[i]->stepper);

					break;
				}
				else if((rx_bin & 3) == 3)
				{
					/* Устанавливаем направление и производим шаг мотором */
					planner->driver[i]->stepper->dir = -1;
					step(planner->driver[i]->stepper);

					break;
				}

				rx_bin >>= (i + 1) * 2;
			}
		}
	}
}

static planner_phase_t phase;

/** Планировщик скорости
 */
void plannerVelocity(PLANNER_StructDef* planner)
{
	if (planner->tickUs - planner->_PlannerTime >= planner->stepTime)
	{
		if(planner->_pauseFlag == false)
		{
			/* Планировщик фазы движения */
			plannerPhase(planner);
		}

		phase = planner->_phase;

		/* Обновление переменной времени планировщика */
		planner->_PlannerTime = planner->tickUs;

		/* Основная логика разгона и торможения */
		if(phase == ACCELERATION || phase == BRAKING)
		{
			if(planner->_n == 0) planner->_cn = planner->_c0;
			else if(planner->_n == 1) planner->_cn = 0.4056 * planner->_c0;
			else if(planner->_n > 1)
			{
				planner->_cn = planner->_cn * (1 - phase * 2.0 / (4.0 * planner->_n + phase));
			}

			planner->stepTime = (uint32_t)(planner->_cn) - STEP_TIME;
			planner->_curSpeed = 1000000.0 / planner->_cn;

			planner->_n += phase;
		}

		if(planner->_n < 0) planner->_workState = PLANNER_PAUSE;
	}
}

void plannerPhase(PLANNER_StructDef* planner)
{
	if(planner->_n == planner->N && planner->_phase == ACCELERATION)
	{
		planner->_phase = UNIFORM;
		return;
	}

	if(cellsForRead(&fifoBufSteps) == planner->N)
	{
		planner->_phase = BRAKING;
		return;
	}
}

/** Запуск планировщика (запуск движения)
 */
void calculatePlannerInitialParam(PLANNER_StructDef* planner)
{
	uint32_t steps;

	if (planner->_accel == 0 || planner->_maxSpeed < MIN_SPEED_POS_MODE)
	{
		planner->stepTime = 1000000.0 / planner->_maxSpeed;
		planner->_phase = UNIFORM;
	}
	else
	{
		steps = cellsForRead(&fifoBufSteps);

		planner->_n = 0;

		planner->_phase = ACCELERATION;
		planner->N = planner->_maxSpeed * planner->_maxSpeed / (2.0 * planner->_accel);

		if(steps < 2 * planner->N)
		{
			planner->N = steps / 2;
		}
	}

	planner->_workState = PLANNER_RUN;
}

/** Установить ускорение планировщика (мм/c^2)
 */
param_change_t setPlannerAcceleration(PLANNER_StructDef* planner, float accel)
{
	if(planner->_workState == PLANNER_RUN) return PARAM_CHANGE_ERR;

	/* Перевод единиц измерения ускорения в шаги/с^2 для 800 шагов на оборот мотора */
	planner->_accel = fabs(accel) * 160;

	/* Считаем значение _c0 и _N по алгоритму плавного старта */
	if(accel > EPS)
	{
		planner->_c0 = 1000000.0 * sqrt(2.0 / planner->_accel);
	}
	else planner->_c0 = 0;

	return PARAM_CHANGE_OK;
}

/** Установка максимальной скорости планировщика (мм/c^2)
 *
 */
param_change_t setPlannerMaxSpeed(PLANNER_StructDef* planner, float speed)
{
	if(planner->_workState == PLANNER_RUN) return PARAM_CHANGE_ERR;

	/* Перевод максимальной скорости в шаги/с^2 */
	planner->_maxSpeed = fabs(speed) * 160;

	return PARAM_CHANGE_OK;
}

/** Подключить драйвер мотора driver на ось axis к планировщику
 */
void addDriver(PLANNER_StructDef* planner, DRIVER_StructDef* driver, uint8_t axis)
{
	planner->driver[axis] = driver;
}

/** Включить моторы
 */
void enablePlanner(PLANNER_StructDef* planner)
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		enableDriver(planner->driver[i]);
	}
}

/** Выключить моторы
 */
void disablePlanner(PLANNER_StructDef* planner)
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		disableDriver(planner->driver[i]);
	}
}

/** Переключить питание
 *
 */
void power(bool v)
{

}

// НАСТРОЙКИ




// ПЛАНИРОВЩИК
/** Возвращает время в мкс до следующего вызова tick/tickManual
 *
 */
uint32_t getPeriod();

/** True - готов принять следующую точку маршрута
 *
 */
bool ready();

/** Функция паузы движения
 *
 */
void pausePlanner(PLANNER_StructDef* planner)
{
	if(cellsForRead(&fifoBufSteps) > planner->N)
	{
		planner->_pauseFlag = true;
		planner->_phase = BRAKING;
	}
}

/** Продолжить после остановки/паузы
 *
 */
void resumePlanner(PLANNER_StructDef* planner)
{
	calculatePlannerInitialParam(planner);
	planner->_pauseFlag = false;

	planner->_workState = PLANNER_RUN;
}

/** Функция резкой остановки
 *
 */
void stopPlanner(PLANNER_StructDef* planner)
{
	planner->_workState = PLANNER_STOP;
}

/* Резкая остановка + выключение мотора
 */
void brakePlanner(PLANNER_StructDef* planner)
{
	planner->_workState = PLANNER_INIT;
//	planner->stepTime = 0;
//	planner->_curSpeed = 0;
	planner->_n = 0;
}

/** Сбросить счётчики всех моторов в 0
 *
 */
void resetDriver();

/** Отправить в 0 по всем осям
 *
 */
void home();

/** Текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 -крутимся со скоростью, 4 - тормозим
 *
 */
uint8_t getStatusPlanner();

// СКОРОСТЬ
/** Режим постоянного вращения для оси axis со скоростью speed шаг/сек (м.б. отрицателеьной)
 *
 */
void setSpeedPlanner(uint8_t axis, float speed);   //

/** Установить текущее положение всех осей!
 *
 */
param_change_t setCurrentAxes(PLANNER_StructDef* planner, int32_t pos[])
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		if(setDriverCurrentPos(planner->driver[i], pos[i]) != PARAM_CHANGE_OK);
		return PARAM_CHANGE_ERR;
	}

	return PARAM_CHANGE_OK;
}

/** Установить текущую позицию одной оси!
 */
param_change_t setCurrentPosAxis(PLANNER_StructDef* planner, uint8_t axis, int32_t pos)
{
	return setDriverCurrentPos(planner->driver[axis], pos);
}

/* ---------------------------------------------------- РАЗРАБОТКА ----------------------------------------------------- */

/** Получить текущую позицию по оси axis
 *
 */
int32_t getCurrentPosAxis(PLANNER_StructDef* planner, uint8_t axis)
{
	return getDriverCurrentPos(planner->driver[axis]);
}

/** Установить цель в шагах и начать движение
 *
 */
bool setTargetAxis(int32_t target[]);

/** Получить цель в шагах на оси axis
 *
 */
int32_t getTargetAxis(int axis);

// ручной тикер для вызова в прерывании или где то ещё. Выполняется 20..50 us
bool tickManual();

// ОСОБЕННОСТИ
//- Планировщик не поддерживает горячую смену цели с плавным изменением скорости

/* ---------------------------------------------------- РАЗРАБОТКА ----------------------------------------------------- */
