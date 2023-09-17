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
	int i;
	for (i = 0; i < UNITS; i++)
	{
		ptime->time[i] = 0;
	}
}

/**
  * @brief  sets the next hour of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Hour(Time_t * ptime)
{
	if (HOURS == ++(ptime->time[2]) )
	{
		(ptime->time[2]) = 0;
	}
}

/**
  * @brief  sets the previous hour of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Hour(Time_t * ptime)
{
	if (0 == (ptime->time[2])--)
	{
		(ptime->time[2]) = HOURS - 1;
	}
}

/**
  * @brief  sets the next minute of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Minute(Time_t * ptime)
{
	if (MINUTES == ++(ptime->time[1]) )
	{
		(ptime->time[1]) = 0;
	}	
}

/**
  * @brief  sets the previous minute of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Minute(Time_t * ptime)
{
	if (0 == (ptime->time[1])--)
	{
		(ptime->time[1]) = MINUTES - 1;
	}	
}

/**
  * @brief  sets the next second of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Second(Time_t * ptime)
{
	if (SECONDS == ++(ptime->time[0]) )
	{
		(ptime->time[0]) = 0;
	}	
}

/**
  * @brief  sets the previous second of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Second(Time_t * ptime)
{
	if (0 == (ptime->time[0])--)
	{
		(ptime->time[0]) = SECONDS - 1;
	}
}

/**
  * @brief  simulates a clock
  * @param  pdate points to an initialized Date_t data type (date)
  * @retval returns true if a new day has come.
  */
bool LB_Times_Ticking(Time_t * ptime)
{
	++(ptime->time[0]);

	if (SECONDS == (ptime->time[0]))
	{
		(ptime->time[0]) = 0;
		++(ptime->time[1]);
	}
	if (MINUTES == (ptime->time[1]))
	{
		(ptime->time[1]) = 0;
		++(ptime->time[2]);
	}
	if (HOURS == (ptime->time[2]))
	{
		(ptime->time[2]) = 0;
		return true;
	}
	return false;
}
/**
  * @brief  sets the clock of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @param  hours specifies the number of hours to be set in Time_t data type (time)
  * @param  minutes specifies the number of minutes to be set in Time_t data type (time)
  * @param  seconds specifies the number of seconds to be set in Time_t data type (time)
  * @retval None
  */
void LB_Set_Time(Time_t * ptime, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	ptime->time[0] = seconds;
	ptime->time[1] = minutes;
	ptime->time[2] = hours;
}

/**
  * @brief  sends formatted output to a string pointed to by the second argument
  * @param  ptime points to an initialized Time_t data type (time)
  * @param  message points to the target string
  * @retval returns the total number of characters written excluding the null-character
  *     appended at the end of the string.
  */
int LB_Time_to_Str(Time_t * ptime, char * message)
{
	return sprintf(message, "%02u:%02u:%02u", ptime->time[2], ptime->time[1], ptime->time[0]);
}

/* Local functions -----------------------------------------------------------*/

