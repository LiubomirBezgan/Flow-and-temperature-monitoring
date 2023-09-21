/* Includes ------------------------------------------------------------------*/
#include <LB_time.h>
#include <stdio.h>

/* Local data type -----------------------------------------------------------*/


/* Program-specific declarations ---------------------------------------------*/


/* Local function prototype --------------------------------------------------*/


/* Function definitions ------------------------------------------------------*/

/**
  * @brief  initializes a Time_t data type (time)
  * @param  ptime points to the Time_t data type (time)
  * @retval None
  */
void LB_Init_Time(Time_t * ptime)
{
		ptime->seconds = 0;
}


/**
  * @brief  simulates a clock
  * @param  pdate points to an initialized Date_t data type (date)
  * @retval returns true if a new day has come.
  */
bool LB_Times_Ticking(Time_t * ptime)
{
	(ptime->seconds)++;

}

/* Local functions -----------------------------------------------------------*/

