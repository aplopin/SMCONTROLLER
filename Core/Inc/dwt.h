/**
  ******************************************************************************
  * @Файл    	dwt.h
  * @Автор  	PromisLab
  * @Описание   Этот файл описывает прототипы функций FIFO буфера, который
  * 			использует указатель на статически выделенную область памяти
  *
  ******************************************************************************
  *
  */

/* ------------------------------ USAGE --------------------------------------*/
/**
 * To use the library functions, it's simply to add the file - "dwt.h" to the
 * directory "../Core/Inc" of the working project
 */

#ifndef INC_DWT_H_
#define INC_DWT_H_

#include <stdint.h>

#define    DWT_CYCCNT	*(volatile uint32_t*)0xE0001004
#define    DWT_CONTROL	*(volatile uint32_t*)0xE0001000
#define    SCB_DEMCR	*(volatile uint32_t*)0xE000EDFC

/* Debug Exception and Monitor Control Register Definitions */
#define CoreDebug_DEMCR_TRCENA_Pos         24U                                            /*!< CoreDebug DEMCR: TRCENA Position */
#define CoreDebug_DEMCR_TRCENA_Msk         (1UL << CoreDebug_DEMCR_TRCENA_Pos)            /*!< CoreDebug DEMCR: TRCENA Mask */

#define DWT_CTRL_CYCCNTENA_Pos              0U                                         /*!< DWT CTRL: CYCCNTENA Position */
#define DWT_CTRL_CYCCNTENA_Msk             (0x1UL /*<< DWT_CTRL_CYCCNTENA_Pos*/)       /*!< DWT CTRL: CYCCNTENA Mask */

/* --------------------------------------- Прототипы функций библиотеки dwt.h --------------------------------------- */

void DWT_Init(void);
double DWT_GetDuration(void (*function)(void));
void DWT_usDelay(uint32_t us);

/* --------------------------------------- Прототипы функций библиотеки dwt.h --------------------------------------- */

#endif /* INC_DWT_H_ */
