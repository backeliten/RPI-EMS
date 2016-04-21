/*
 * wheeldecoder.h
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */

#ifndef WHEELDECODER_H_
#define WHEELDECODER_H_

#define MAX_ERRORLIST		20
#define MAX_TOOTHLIST		60

typedef enum
{
	TRIGGER_NO_FAULT,
	TRIGGER_LOST_MISSING_TOOTH,
	TRIGGER_MISSING_TOOTH_WRONG_PLACE,
	TRIGGER_TO_MUCH_DIFFERENCE,
	TRIGGER_COUNTS_WRONG
}trigger_error_reason_e;

typedef enum
{
	SYNC_NOT_DEFINED,
	SYNC,
	SYNC_NOT_SYNC,
	SYNC_FIRST_TOOTH		//Will indicate if we are searching for first tooth
}trigger_sync_e;

typedef struct
{
	uint32_t					triggertooth[MAX_TOOTHLIST];		//Absolut time for every tooth
	uint32_t					triggertooth_diff[MAX_TOOTHLIST]; //First cell will contain diff between tooth 1 and 2

	trigger_error_reason_e 		trigggererror[MAX_ERRORLIST];

	trigger_sync_e				sync;

	uint32_t					count;

	uint8_t 					trigger_err_pos;

	uint16_t					rpm;

}triggerwheel_s;

/*Set functions*/
//int8_t set_inj0_time(uint32_t ns);
//int8_t set_inj1_time(uint32_t ns);
//int8_t set_ign0_time(uint32_t ns);
//int8_t set_ign1_time(uint32_t ns);

#endif /* WHEELDECODER_H_ */
