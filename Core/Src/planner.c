#include "planner.h"
#include "driver.h"
#include "interpolator.h"

/* Статический буфер - массив шагов */
int8_t BufSteps[FIFO_STEPS_SIZE];

/* Структура FIFO буфера шагов для интерполяции оценочной функцией */
extern FIFO_StructDef fifoBufSteps;

/* --------------------------------------- Импортированные переменные из библиотеки interpolator.c --------------------------------------- */

/* ------------ Глобальные координаты станка (шаги) ------------- */

/* Координаты линейных осей станка */
extern int64_t x;
extern int64_t y;
extern int64_t z;

/* Координаты поворотных осей станка */
extern int64_t a;
extern int64_t b;
extern int64_t c;

/* ------------ Глобальные координаты станка (мм) ------------- */

/* Координаты линейных осей станка */
extern double X;
extern double Y;
extern double Z;

/* Координаты поворотных осей станка */
extern double A;
extern double B;
extern double C;

/* --------------------------------------- Импортированные переменные из библиотеки interpolator.c --------------------------------------- */

/* Указатель на функцию для измерения времени */
static timeFunction_uint32_t_ptr getTime;

void plannerFunctionsInit(timeFunction_uint32_t_ptr function1)
{
	getTime = function1;
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
	planner->stepTime = 0;

	planner->_stopFlag = false;
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
//		planner->tickUs = getMicros();

		/*  */
//		if (availableForRead(&fifoBufSteps) == true && planner->_accel != 0) plannerVelocity(planner);

		/* Основной таймер степпера */
		if (planner->stepTime != 0 && planner->tickUs - planner->_prevTime >= planner->stepTime)
		{
			planner->_prevTime = planner->tickUs;

			/* Проверка завершения работы планировщика по флагу доступа к ячейкам fifo буфера шагов */
//			if (availableForRead(&fifoBufSteps) == false)
//			{
//				brakePlanner(planner);
//			}

			/* Чтение ячейки FIFO буфера шагов и воспроизведение шагов */
			int8_t rx_bin;
			fifoRead(&fifoBufSteps, &rx_bin);

			for(uint8_t i = 0; i < AXES; i ++)
			{
				if((rx_bin & 3) == 1)
				{
					/* Устанавливаем направление и производим шаг мотором */
					setDriverDir(planner->driver[i], 1);
					stepDriver(planner->driver[i]);

					break;
				}
				else if((rx_bin & 3) == 3)
				{
					/* Устанавливаем направление и производим шаг мотором */
					setDriverDir(planner->driver[i], -1);
					stepDriver(planner->driver[i]);

					break;
				}

				rx_bin >>= (i + 1) * 2;
			}
		}
	}
}

/** Планировщик шагов
 */
void plannerTickTest(PLANNER_StructDef* planner)
{
	if (planner->_workState == PLANNER_RUN)
	{
		planner->tickUs = getTime();

		/* Основной таймер степпера */
		if (planner->tickUs - planner->_prevTime >= planner->stepTime)
		{
			planner->_prevTime = planner->tickUs;

			/* Проверка завершения работы планировщика по флагу доступа к ячейкам fifo буфера шагов
			if (availableForRead(&fifoBufSteps) == FIFO_EMPTY)
			{
				brakePlanner(planner);
			}*/

			/* Чтение ячейки FIFO буфера шагов и воспроизведение шагов */
			int8_t rx_bin;
			fifoRead(&fifoBufSteps, &rx_bin);

			for(uint8_t i = 0; i < AXES; i ++)
			{
				if((rx_bin & 3) == 1)
				{
					/* Устанавливаем направление и производим шаг мотором */
					setDriverDir(planner->driver[i], 1);
					stepDriver(planner->driver[i]);

					break;
				}
				else if((rx_bin & 3) == 3)
				{
					/* Устанавливаем направление и производим шаг мотором */
					setDriverDir(planner->driver[i], -1);
					stepDriver(planner->driver[i]);

					break;
				}

				rx_bin >>= (i + 1) * 2;
			}
		}
	}
}

/** Планировщик скорости
 */
void plannerVelocity(PLANNER_StructDef* planner)
{
	planner_phase_t phase = planner->_phase;

	if(planner->_accel <= EPS)
	{
		planner->_curSpeed = planner->_maxSpeed;
		planner->stepTime = 1000000.0 / planner->_maxSpeed;

		return;
	}

	if (planner->tickUs - planner->_PlannerTime >= planner->stepTime)
	{
		/* Обновление переменной времени планировщика */
		planner->_PlannerTime = planner->tickUs;

		if(planner->_n >= planner->N)
		{
			planner->_phase = UNIFORM;
		}

		/* Основная логика разгона и торможения */
		if(planner->_phase == ACCELERATION || planner->_phase == BRAKING)
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

		/* Условие достижения максимальной скорости */
		if(planner->_phase == UNIFORM)
		{
			planner->_curSpeed = planner->_maxSpeed;
			planner->stepTime = 1000000.0 / planner->_maxSpeed;
		}

		/* Особый случай первого шага при торможении */
		if(planner->_k == planner->_s2)
		{
			planner->stepTime = (uint32_t)(planner->_cn) - STEP_TIME;
			planner->_curSpeed = 1000000.0 / planner->_cn;

			planner->_n --;
		}

		planner->_k ++;
	}
}

/** Запуск планировщика (запуск движения)
 */
void startPlanner(PLANNER_StructDef* planner)
{
	uint32_t steps;

//	if(availableForRead(&fifoBufSteps) == false)
//	{
//		planner->_workState = PLANNER_BRAKE;
//		planner->_phase = STAND;
//		return;
//	}

	if (planner->_accel == 0 || planner->_maxSpeed < MIN_SPEED_POS_MODE)
	{
		planner->stepTime = 1000000.0 / planner->_maxSpeed;
		planner->_phase = UNIFORM;
	}
	else
	{
		steps = cellsForRead(&fifoBufSteps);

		planner->_phase = ACCELERATION;
		planner->N = planner->_maxSpeed * planner->_maxSpeed / (2.0 * planner->_accel);

		if(steps > 2 * planner->N)
		{
			planner->_s1 = planner->N;
			planner->_s2 = -1;
			planner->_s3 = planner->N;
		}
		else
		{
			planner->_s1 = steps / 2;
			planner->_s2 = (steps % 2 == 0) ? 0 : 1;
			planner->_s3 = planner->_s1;
		}

		planner->_k = 0;
	}

	planner->_workState = PLANNER_RUN;
}

/** Установить ускорение планировщика (мм/c^2)
 */
param_change_t setAccelerationPlanner(PLANNER_StructDef* planner, float accel)
{
	if(planner->_workState == PLANNER_RUN) return PARAM_CHANGE_ERR;

	planner->_accel = fabs(accel);

	/* Считаем значение _c0 и _N по алгоритму плавного старта */
	if(accel > EPS)
	{
		planner->_c0 = 1000000.0 * sqrt(2.0 / planner->_accel);
	}
	else planner->_c0 = 0;

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
void enable(PLANNER_StructDef* planner)
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		enableDriver(planner->driver[i]);
	}
}

/** Выключить моторы
 */
void disable(PLANNER_StructDef* planner)
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

/** Установка максимальной скорости планировщика в шаг/сек
 *
 */
void setMaxSpeedPlanner(float nV);


// ПЛАНИРОВЩИК
/** Возвращает время в мкс до следующего вызова tick/tickManual
 *
 */
uint32_t getPeriod();

/** True - готов принять следующую точку маршрута
 *
 */
bool ready();

/** Пауза (доехать до заданной точки и ждать). ready() не вернёт true, пока ты на паузе
 *
 */
void pause();

/** Остановить плавно (с заданным ускорением)
 *
 */
void stop();

/* Резкая остановка + выключение мотора
 */
void brakePlanner(PLANNER_StructDef* planner)
{
	planner->_workState = PLANNER_BRAKE;
	planner->stepTime = 0;
	planner->_curSpeed = 0;
	planner->_n = 0;
}

/** Продолжить после остановки/паузы
 *
 */
void resume();

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
		if(setCurrentPos(planner->driver[i], pos[i]) != PARAM_CHANGE_OK);
		return PARAM_CHANGE_ERR;
	}

	return PARAM_CHANGE_OK;
}

/** Установить текущую позицию одной оси!
 */
param_change_t setCurrentPosAxis(PLANNER_StructDef* planner, uint8_t axis, int32_t pos)
{
	return setCurrentPos(planner->driver[axis], pos);
}

/* ---------------------------------------------------- РАЗРАБОТКА ----------------------------------------------------- */

/** Получить текущую позицию по оси axis
 *
 */
int32_t getCurrentPosAxis(PLANNER_StructDef* planner, uint8_t axis)
{
	return getCurrentPos(planner->driver[axis]);
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
