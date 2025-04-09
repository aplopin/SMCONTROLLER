/**
 * @file dwt.h
 * @brief A Library for using DWT timer for microsecond operations calculations
 * @version 0.1.0
 * @authors aplopin
 */

/* ------------------------------ USAGE --------------------------------------*/
/**
 * To use the library functions, it's simply to add the file - "dwt.h" to the
 * directory "../Core/Inc" of the working project
 */

#ifndef INC_DWT_H_
#define INC_DWT_H_

#pragma once

#define    DWT_CYCCNT	*(volatile uint32_t*)0xE0001004
#define    DWT_CONTROL	*(volatile uint32_t*)0xE0001000
#define    SCB_DEMCR	*(volatile uint32_t*)0xE000EDFC

/* Functions prototypes ------------------------------------------------------*/
void DWT_Init(void);
double DWT_GetDuration(void (*function)(void));
void DWT_usDelay(uint32_t us);

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

#endif /* INC_DWT_H_ */
