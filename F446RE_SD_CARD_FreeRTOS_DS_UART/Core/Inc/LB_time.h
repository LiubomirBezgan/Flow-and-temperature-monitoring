//  LB_time.h
//  v1.1

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LB_TIME_H_
#define _LB_TIME_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Program-specific declarations ---------------------------------------------*/
#define UNITS 3
#define SECONDS 60
#define MINUTES 60
#define HOURS 24
#define NEW_DAY true

/* General type definitions --------------------------------------------------*/
/* redefine Time_t as appropriate */
typedef struct hh_mm_ss {
//	uint8_t time[UNITS];               // 2 - hours, 1 - minutes, 0 - seconds
	uint16_t seconds;
} Time_t;

/* Function prototypes -------------------------------------------------------*/

/**
  * @brief  initializes a Time_t data type (time)
  * @param  ptime points to the Time_t data type (time)
  * @retval None
  */
void LB_Init_Time(Time_t * ptime);

/**
  * @brief  sets the next hour of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Hour(Time_t * ptime);

/**
  * @brief  sets the previous hour of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Hour(Time_t * ptime);

/**
  * @brief  sets the next minute of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Minute(Time_t * ptime);

/**
  * @brief  sets the previous minute of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Minute(Time_t * ptime);

/**
  * @brief  sets the next second of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Next_Second(Time_t * ptime);

/**
  * @brief  sets the previous second of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @retval None
  */
void LB_Set_Prev_Second(Time_t * ptime);

/**
  * @brief  simulates a clock
  * @param  pdate points to an initialized Date_t data type (date)
  * @retval returns true if a new day has come.
  */
bool LB_Times_Ticking(Time_t * ptime);

/**
  * @brief  sets the clock of Time_t data type (time)
  * @param  ptime points to an initialized Time_t data type (time)
  * @param  hours specifies the number of hours to be set in Time_t data type (time)
  * @param  minutes specifies the number of minutes to be set in Time_t data type (time)
  * @param  seconds specifies the number of seconds to be set in Time_t data type (time)
  * @retval None
  */
void LB_Set_Time(Time_t * ptime, uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
  * @brief  sends formatted output to a string pointed to by the second argument
  * @param  ptime points to an initialized Time_t data type (time)
  * @param  message points to the target string
  * @retval returns the total number of characters written excluding the null-character
  *     appended at the end of the string.
  */
int LB_Time_to_Str(Time_t * ptime, char * message);

#endif
