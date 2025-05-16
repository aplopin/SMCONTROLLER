/**
  ******************************************************************************
  * @Файл    	dwt.c
  * @Автор  	PromisLab
  * @Описание   Этот файл описывает прототипы функций FIFO буфера, который
  * 			использует указатель на статически выделенную область памяти
  *
  ******************************************************************************
  *
  */

#include "dwt.h"

extern uint32_t SystemCoreClock;

/**
  * @brief  This function initializes the DWT counter.
  * @param[in] None
  * @return None
  */
void DWT_Init()
{
	/* Allow the use of the counter DWT */
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	/* Turn on the counter */
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  This method returns the function time in us
  * @param[in] Function pointer
  * @return Time in us
  */
double DWT_GetDuration(void (*function)(void))
{
	/* Reset the counter */
	DWT_CYCCNT = 0;

	function();

	/* Return time to us */
	return (double) DWT_CYCCNT / SystemCoreClock * 1000000;
}

/**
  * @brief  This function produces a time delay in us.
  * @param[in] Time in us
  * @return None
  */
void DWT_usDelay(uint32_t us)
{
	/* Convert microseconds to processor ticks */
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000);
	/* Reset the counter */
	DWT_CYCCNT = 0;
	while(DWT_CYCCNT < us_count_tic);
}
