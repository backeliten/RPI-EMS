/*
 * communication.h
 *
 *  Created on: Jun 10, 2013
 *      Author: jonas
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "wheeldecoder.h"

typedef struct
{
	unsigned short 	rpm;
	trigger_sync_e	sync;

}ku_comm_s;


typedef struct
{
	/*This is for injector*/
	unsigned short 	pw_inj1;
	unsigned short 	pw_inj2;

	unsigned short 	angle_inj1;
	unsigned short 	angle_inj2;

	/*This is for ignition*/
	unsigned short  angle_ign1;
	unsigned short 	angle_ign2;

	unsigned short 	dwell_ign1;
	unsigned short 	dwell_ign2;
}uk_comm_s;


#endif /* COMMUNICATION_H_ */
