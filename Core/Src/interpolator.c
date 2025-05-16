#include <math.h>
#include <stdio.h>

#include "interpolator.h"

/* ------------ Глобальные координаты станка (шаги) ------------- */

/* Координаты линейных осей станка */
int64_t x = 0;
int64_t y = 0;
int64_t z = 0;

/* Координаты поворотных осей станка */
int64_t a = 0;
int64_t b = 0;
int64_t c = 0;

/* ------------ Глобальные координаты станка (мм) ------------- */

/* Координаты линейных осей станка */
double X = 0;
double Y = 0;
double Z = 0;

/* Координаты поворотных осей станка */
double A = 0;
double B = 0;
double C = 0;

/* Буфер шагов интерполятора */
int8_t bufSteps[FIFO_STEPS_SIZE];

/* FIFO буфер шагов интерполятора */
FIFO_StructDef fifoBufSteps;

/** Инициализация структура интерполятора
 */
void interpolatorInit(INTERPOLATOR_StructDef* interpolator)
{
	interpolator->_workState = INTERPOLATOR_READY;

	fifoInit(&fifoBufSteps, bufSteps, FIFO_STEPS_SIZE);
}

/** Функция определения параметров отрезка
 */
handler_interpolator_state_t setLine(LINE_StructDef* line, double Xk, double Yk)
{
    /* Координаты отрезка в абсолютных координатах (мм) */
    line->X0 = X;
    line->Y0 = Y;
    line->Xk = Xk;
    line->Yk = Yk;

    /* Проверка существования прямой */
    if(fabs(X - Xk) <= MM_PER_STEP && fabs(Y - Yk) <= MM_PER_STEP) return INTERPOLATOR_ERROR;

    /* Определение смещения по осям */
    line->dX = line->Xk - line->X0;
    line->dY = line->Yk - line->Y0;

    /* Определение координат станка в относительной СК начала отрезка (мм) */
    line->x_rel = x * MM_PER_STEP - line->X0;
    line->y_rel = y * MM_PER_STEP - line->Y0;

    /* Начальное значения оценочной функции */
    line->F = line->dX * line->y_rel - line->dY * line->x_rel;

    /* Определение четверти, в которой расположен отрезок */
    if (line->dX >= 0 && line->dY > 0) { line->dx = 1; line->dy = 1; line->s = 1; }             	//< 1 четверть
    else if (line->dY <= 0 && line->dX > 0) { line->dx = 1; line->dy = -1; line->s = -1; }       	//< 4 четверть
    else if (line->dX <= 0 && line->dY < 0) { line->dx = -1; line->dy = -1; line->s = 1; }      	//< 3 четверть
    else if (line->dY >= 0 && line->dX < 0) { line->dx = -1; line->dy = 1; line->s = -1; }       	//< 2 четверть

    /* Определение угла между отрезком и осью OX (радианы) */
    if (fabs(line->dX) <= MM_PER_STEP)
    {
        line->alfa = M_PI_2;
    }
    else line->alfa = fabs(atan(line->dY / line->dX));

    return INTERPOLATOR_DONE;
}

/** Функция определения параметров дуги
 */
handler_interpolator_state_t setArc(ARC_StructDef* arc, double Xk, double Yk, double I, double J, int8_t dir)
{
    /* Координаты точек дуги в абсолютной СК (мм) */
    arc->X0 = X;
    arc->Y0 = Y;
    arc->Xk = Xk;
    arc->Yk = Yk;
    arc->Xc = arc->X0 + I;
    arc->Yc = arc->Y0 + J;

    /* Проверка существования дуги */
    if (fabs(X - Xk) <= MM_PER_STEP && fabs(Y - Yk) <= MM_PER_STEP) return INTERPOLATOR_ERROR;

    /* Переход в относительную систему координаты центра дуги (мм) */
    arc->x_rel = x * MM_PER_STEP - arc->Xc;
    arc->y_rel = y * MM_PER_STEP - arc->Yc;
    arc->X0 -= arc->Xc;
    arc->Y0 -= arc->Yc;
    arc->Xk -= arc->Xc;
    arc->Yk -= arc->Yc;

    /* Квадрат радиуса дуги (мм^2) */
    arc->R2 = I * I + J * J;

    /* Начальное значения оценочной функции */
    arc->F = arc->x_rel * arc->x_rel + arc->y_rel * arc->y_rel - arc->R2;

    /* Направление вращения по дуге 1 - по часовой стрелки, -1 - против */
    arc->dir = dir;

    /* Определение четверти, в которой расположено начало дуги */
    if (arc->x_rel >= 0 && arc->y_rel > 0) { arc->dx = 1 * arc->dir; arc->dy = -1 * arc->dir; arc->s = -1 * arc->dir; }             //< 1 четверть
    else if (arc->y_rel <= 0 && arc->x_rel > 0) { arc->dx = -1 * arc->dir; arc->dy = -1 * arc->dir; arc->s = 1 * arc->dir; }        //< 4 четверть
    else if (arc->x_rel <= 0 && arc->y_rel < 0) { arc->dx = -1 * arc->dir; arc->dy = 1 * arc->dir; arc->s = -1 * arc->dir; }        //< 3 четверть
    else if (arc->y_rel >= 0 && arc->x_rel < 0) { arc->dx = 1 * arc->dir; arc->dy = 1 * arc->dir; arc->s = 1 * arc->dir; }          //< 2 четверть

    return INTERPOLATOR_DONE;
}

/** Обработчик команды G01 - интерполяционное движение по прямой (оценочная функция)
 */
handler_interpolator_state_t handlerLine(INTERPOLATOR_StructDef* interpolator, FIFO_StructDef* buf, LINE_StructDef* line, uint8_t axis1, uint8_t axis2)
{
	if(interpolator->_workState == INTERPOLATOR_READY && availableForWrite(buf) == FIFO_OK)
	{
		interpolator->_workState = INTERPOLATOR_PROCESSING;

		/* Основные вычисления на основе анализа знака оценочной функции */
		if (fabs(line->F) <= EPS)
		{
			if (line->alfa < M_PI_4)
			{
				line->x_rel += line->dx * MM_PER_STEP;
				x += line->dx;

				fifoWrite(buf, setBinarySteps(axis1, line->dx));
			}
			else
			{
				line->y_rel += line->dy * MM_PER_STEP;
				y += line->dy;

				fifoWrite(buf, setBinarySteps(axis2, line->dy));
			}
		}
		else
		{
			if (line->F * line->s > 0)
			{
				line->x_rel += line->dx * MM_PER_STEP;
				x += line->dx;

				fifoWrite(buf, setBinarySteps(axis1, line->dx));
			}
		        else
		        {
		            line->y_rel += line->dy * MM_PER_STEP;
		            y += line->dy;

		            fifoWrite(buf, setBinarySteps(axis2, line->dy));
		        }
		}

//		printf("G01 X%.6f Y%.6f\n", x * MM_PER_STEP, y * MM_PER_STEP);

		/* Пересчет оценочной функции */
		line->F = line->dX * line->y_rel - line->dY * line->x_rel;

		/* Проверка достижения конечной точки */
		if ((fabs(line->x_rel - line->dX) < MM_PER_STEP) && (fabs(line->y_rel - line->dY) < MM_PER_STEP))
		{
			X = line->Xk;
			Y = line->Yk;

			interpolator->_workState = INTERPOLATOR_READY;
			return INTERPOLATOR_DONE;
		}

		interpolator->_workState = INTERPOLATOR_READY;
	}

	return INTERPOLATOR_PROCESSING;
}

/** Обработчик команд G02, G03 - интерполяционное движение по окружности по/против часовой стрелке (оценочная функция)
 */
handler_interpolator_state_t handlerArc(INTERPOLATOR_StructDef* interpolator, FIFO_StructDef* buf, ARC_StructDef* arc, uint8_t axis1, uint8_t axis2)
{
	if(interpolator->_workState == INTERPOLATOR_READY && availableForWrite(buf) == FIFO_OK)
	{
		interpolator->_workState = INTERPOLATOR_PROCESSING;

		/* Основные вычисления на основе анализа знака оценочной функции */
		if (arc->s == -1)
		{
			if (arc->F >= 0)
			{
				arc->y_rel += arc->dy * MM_PER_STEP;
				y += arc->dy;

				fifoWrite(buf, setBinarySteps(axis2, arc->dy));
			}
			else
			{
				arc->x_rel += arc->dx * MM_PER_STEP;
				x += arc->dx;

				fifoWrite(buf, setBinarySteps(axis1, arc->dx));
			}
		}
		else
		{
			if (arc->F >= 0)
			{
				arc->x_rel += arc->dx * MM_PER_STEP;
				x += arc->dx;

				fifoWrite(buf, setBinarySteps(axis1, arc->dx));

			}
			else
			{
				arc->y_rel += arc->dy * MM_PER_STEP;
				y += arc->dy;

				fifoWrite(buf, setBinarySteps(axis2, arc->dy));
			}
		}

//		printf("G01 X%.6f Y%.6f\n", x * MM_PER_STEP, y * MM_PER_STEP);

		/* Пересчет оценочной функции */
		arc->F = arc->x_rel * arc->x_rel + arc->y_rel * arc->y_rel - arc->R2;

		/* Проверка достижения конечной точки */
		if ((fabs(arc->x_rel - arc->Xk) < MM_PER_STEP) && (fabs(arc->y_rel - arc->Yk) < MM_PER_STEP))
		{
			X = arc->Xk + arc->Xc;
			Y = arc->Yk + arc->Yc;

			interpolator->_workState = INTERPOLATOR_READY;
			return INTERPOLATOR_DONE;
		}

		/* Условие прохождения оси координат */
		if (fabs(arc->x_rel) <= MM_PER_STEP || fabs(arc->y_rel) <= MM_PER_STEP)
		{
			/* Переопределение четверти, в которой расположена текущая точка дуги */
			if (arc->x_rel >= 0 && arc->y_rel > 0) { arc->dx = 1 * arc->dir; arc->dy = -1 * arc->dir; arc->s = -1 * arc->dir; }             //< 1 четверть
			else if (arc->y_rel <= 0 && arc->x_rel > 0) { arc->dx = -1 * arc->dir; arc->dy = -1 * arc->dir; arc->s = 1 * arc->dir; }        //< 4 четверть
			else if (arc->x_rel <= 0 && arc->y_rel < 0) { arc->dx = -1 * arc->dir; arc->dy = 1 * arc->dir; arc->s = -1 * arc->dir; }        //< 3 четверть
			else if (arc->y_rel >= 0 && arc->x_rel < 0) { arc->dx = 1 * arc->dir; arc->dy = 1 * arc->dir; arc->s = 1 * arc->dir; }          //< 2 четверть
		}

		interpolator->_workState = INTERPOLATOR_READY;

	}

    return INTERPOLATOR_PROCESSING;
}

/** Функция формирования двоичного представления шагов по осям
  * step - шаг по оси, может быть 1 или -1
  */
uint8_t setBinarySteps(uint8_t axis, int8_t step)
{
    int8_t bin = 0b00000000;

    if (step == 1)
    {
        bin |= 0b00000001 << axis * 2;
    }
    else if (step == -1)
    {
        bin |= 0b00000011 << axis * 2;
    }

    return bin;
}
