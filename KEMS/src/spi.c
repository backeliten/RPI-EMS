/*
 * spi.c
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */



#include "../header/spi.h"

volatile unsigned *spi;
char *spi_mem , *spi_map;

void spi_init(int mem_fd)
{

}

int8_t	spi_write(uint8_t cs, uint8_t size, uint8_t *array)
{


	return 0;
}

int8_t	spi_read(uint8_t cs, uint8_t size, uint8_t *array)
{


	return 0;
}

int8_t	spi_rw(uint8_t cs, uint8_t size, uint8_t *in_array, uint8_t *out_array)
{


	return 0;
}
