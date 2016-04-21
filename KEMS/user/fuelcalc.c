/*
 * fuelcalc.c
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <dirent.h>
#include <assert.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/mman.h>

#include <unistd.h>

#define uint16_t unsigned short;
#define uint8_t	unsigned char;

#include "../header/fueltable.h"

//AIRDEN(MAP, temp) = 1.2929 * 273.13/(T+273.13) * MAP/101.325
static float airden(float map, float temp)
{
	float value = 0;

	value = 1.2929 * 273.13/(temp+273.13) * map/101.325;

	return value;
}

//E = gamma_Enrich = (Warmup/100) * (O2_Closed Loop/100) * (AirCorr/100) * (BaroCorr/100)
static float calc_enrichment(fuel_inj_s *inj)
{
	inj->enrichment = 1.0;		//We dont have any enrichment right now
}

static float calc_warmup(warmup_table_s warmup, float clt_temp)		//Used in background thread, not that time critical
{
	float value = 0;

	//Check where in table to get correct value

	return value;
}

static float get_ve(unsigned short rpm, float map)
{
	float value = 0;

	//Function to get value from 3D table

	return value;
}

//PW = REQ_FUEL * VE * MAP * E + accel + Injector_open_time
float fuel_calc_main_pw(fuel_inj_s *inj)
{
	float value = 0;

	inj->airden = airden(inj->map, inj->iat_temp);
	inj->enrichment = calc_enrichment(inj);
	inj->ve = get_ve(inj->rpm, inj->map);

	value = inj->req_fuel * inj->ve * inj->map * inj->enrichment + inj->accel + inj->open_time;

	return value;
}

void fuel_set_rpm(fuel_inj_s *inj, unsigned short rpm)
{
	inj->rpm = rpm;
}

void fuel_set_map(fuel_inj_s *inj, float map)
{
	inj->map = map;
}

void fuel_set_iat_temp(fuel_inj_s *inj, float temp)
{
	inj->iat_temp = temp;
}

void fuel_set_clt_temp(fuel_inj_s *inj, float temp)
{
	inj->clt_temp = temp;
}

void fuel_set_req_fuel(fuel_inj_s *inj, float req_fuel)
{
	inj->req_fuel = req_fuel;
}


/* HELP FUNCTION */
float fuel_calc_req(fuel_inj_s *inj, float motor_deplacement, unsigned char ncyl, float afr, float injflow, unsigned char divide_pulse)
{
	float value = 0;

	//36,000,000 is the number of tenths of a millisecond in an hour,
	//used to get the pounds per 1/10 milllisecond from the pounds/hours rating of the injectors

	//60*1000*10  = 600000-> 60 seconds (cc / min) * milliseconds * tenth of milliseconds

	//REQ_FUEL*10 = 36,000,000 * CID * AIRDEN(100kPA, 70�F)/(NCYL*AFR*INJFLOW ) * 1/DIVIDE_PULSE

	value = 600000 * (float)motor_deplacement * airden(100.0, 21.0)/((float)ncyl*afr*injflow) * 1/divide_pulse;

	return value;
}



//For ignition

//calc_tooth

//calc_angle_to_time

//calc_diff_tooth


//For wheel decoder

//Add_event

//Event dispatcher

//Remove event

