#ifndef INC_PLANNER_H_
#define INC_PLANNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "driver.h"

#define GP_MIN_US 300000  // период, длиннее которого мотор можно резко тормозить или менять скорость
#define AXES 3 // число осей интерполятора

/* Структура планировщика */
typedef struct
{
	/* Указатели на экземляры структуры драйверов моторов
	 * После инициализации необходмо вызвать функцию addDriver(),
	 * чтобы сопоставить указатель на нужный драйвер с номером оси
	 * планировщика
	 */
	DRIVER_StructDef* driver[AXES];

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

} PLANNER_StructDef;

/** Инициализация планироващика
 *
 */
void plannerInit(PLANNER_StructDef* planner)
{
	planner->status = 0;
	planner->speedAxis = 0;
	planner->shift = 0;
	planner->readyF = true;
	planner->changeSett = 0;
}

/** Подключить драйвер мотора driver на ось axis к планировщику
 *
 */
void addDriver(PLANNER_StructDef* planner, DRIVER_StructDef* driver, uint8_t axis)
{
	planner->driver[axis] = driver;
}

/** Включить моторы
 *
 */
void enable(PLANNER_StructDef* planner)
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		enableDriver(planner->driver[i]);
	}
}

/** Выключить моторы
 *
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

/** Установка ускорения планировщика в шаг/сек^2
 *
 */
void setAccelerationPlanner(uint16_t nA);

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

/** Резко остановить моторы из любого режима
 *
 */
void brake();

/** Продолжить после остановки/паузы
 *
 */
void resume();

/** Сбросить счётчики всех моторов в 0
 *
 */
void reset();

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

// ПОЗИЦИЯ
/** Установить текущее положение моторов
 *
 */
void setCurrentAxes(PLANNER_StructDef* planner, int32_t cur[])
{
	for(uint8_t i = 0; i < AXES; i ++)
	{
		planner->driver[i]->stepper->pos = cur[i];
	}
}

/** Получить текущую позицию по оси axis
 *
 */
int32_t getCurrentAxis(PLANNER_StructDef* planner, uint8_t axis)
{
	return planner->driver[axis]->stepper->pos;
}

/** Установить цель в шагах и начать движение
 *
 */
bool setTargetAxis(int32_t target[]);

/** Получить цель в шагах на оси axis
 *
 */
int32_t getTargetAxis(int axis);

// ТИКЕР
// тикер, вызывать как можно чаще. Вернёт true, если мотор крутится
// здесь делаются шаги как для движения по точкам, так и для вращения по скорости
bool tickPlanner();

// ручной тикер для вызова в прерывании или где то ещё. Выполняется 20..50 us
bool tickManual();

// ОСОБЕННОСТИ
//- Планировщик не поддерживает горячую смену цели с плавным изменением скорости

#endif /* INC_PLANNER_H_ */
