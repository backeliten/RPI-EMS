/*
 * fueltable.h
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */

#ifndef FUELTABLE_H_
#define FUELTABLE_H_

typedef struct
{
	float 		ms;
	float		map;
	float 		iat_temp;
	float 		clt_temp;
	float 		airden;
	float 		req_fuel;
	float 		enrichment;
	float		ve;
	float 		open_time;

	float		accel;

	unsigned short	rpm;

}fuel_inj_s;

typedef struct
{
	unsigned char 	warmup[16];
	unsigned char	warmup_temp[16];		//0 = -40 degrees  -> 255 = 215 degrees

}warmup_table_s;


#endif /* FUELTABLE_H_ */
