#pragma once

#ifndef INC_INTERPOLATOR_H_
#define INC_INTERPOLATOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#include "fifo.h"

#define MM_PER_STEP 	0.003125F					//< Расстояние в мм на один шаг мотора - 1600 шагов за оборот
//#define MM_PER_STEP 	0.00625F					//< Расстояние в мм на один шаг мотора - 800 шагов за оборот
#define EPS             1e-09                       //< Бесконечно малое число для сравнения с 0 чисел double

/* Тип данных - состояние работы планировщика траектории */
typedef enum
{
	INTERPOLATOR_ERROR = -1,
	INTERPOLATOR_DONE = 0,
	INTERPOLATOR_READY = 1,
	INTERPOLATOR_PROCESSING

} handler_interpolator_state_t;

/* Структура работы интерполятора */
typedef struct
{
	handler_interpolator_state_t _workState;

} INTERPOLATOR_StructDef;

/* Структура параметров обрабатываемого отрезка */
typedef struct
{
    /* Начальные координаты отрезка (мм) */
    double X0;
    double Y0;

    /* Конечные координаты отрезка (мм) */
    double Xk;
    double Yk;
    
    /* Смещение по осям (мм) */
    double dX;
    double dY;
    
    /* Координаты станка в относительной системе координат (мм) */
    double x_rel;
    double y_rel;

    /* Оценочная функция прямой */
    double F;
    
    /* Шаги по осям */
    int8_t dx;
    int8_t dy;

    /* Угол между прямой и осью OX (радианы) */
    double alfa;

    /* Дополнительная переменная для оптимизации
     * (определяется в блоке вычисления четверти положения отрезка)
     */
    int8_t s;
    
} LINE_StructDef;

/* Структура параметров обрабатываемой дуги окружности */
typedef struct
{
    /* Начальные координаты дуги (мм) */
    double X0;
    double Y0;

    /* Конечные координаты дуги (мм) */
    double Xk;
    double Yk;

    /* Координаты центра окружности (мм) */
    double Xc;
    double Yc;

    /* Координаты станка в относительной системе координат (мм) */
    double x_rel;
    double y_rel;

    /* Квадрат радиуса дуги (мм^2) */
    double R2;

    /* Оценочная функция приямой */
    double F;

    /* Шаги по осям */
    int8_t dx;
    int8_t dy;

    /* Дополнительная переменная для оптимизации
     * (определяется в блоке вычисления четверти положения точки дуги)
     */
    int8_t s;

    /* Направление вращения по дуге 1 - по часовой стрелки, -1 - против */
    int8_t dir;

} ARC_StructDef;

void interpolatorInit(INTERPOLATOR_StructDef* interpolator);
handler_interpolator_state_t setLine(LINE_StructDef* line, double Xk, double Yk);
handler_interpolator_state_t setArc(ARC_StructDef* arc, double Xk, double Yk, double I, double J, int8_t dir);
handler_interpolator_state_t handlerLine(INTERPOLATOR_StructDef* interpolator, FIFO_StructDef* buf, LINE_StructDef* line, uint8_t axis1, uint8_t axis2);
handler_interpolator_state_t handlerArc(INTERPOLATOR_StructDef* interpolator, FIFO_StructDef* buf, ARC_StructDef* arc, uint8_t axis1, uint8_t axis2);
uint8_t setBinarySteps(uint8_t axis, int8_t step);

#ifdef __cplusplus
}
#endif

#endif /* INC_INTERPOLATOR_H_ */
