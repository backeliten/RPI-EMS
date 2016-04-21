/*
 * mcp3208.c
 *
 *  Created on: 7 jun 2013
 *      Author: bkj1mll
 */


static int mcp3208_init(void)
{

	return 0;
}

static void mcp3208_deinit(void)
{

}

int mcp3208_read_channel(uint8_t channel)
{

	return 100;		//Should return value here

	//If -1, we got error
}

int mcp3208_read_all_channel(uint16_t *data_channel)
{
	data_channel[0] = 1;
	data_channel[1] = 2;
	data_channel[2] = 3;
	data_channel[3] = 4;
	data_channel[4] = 5;
	data_channel[5] = 6;
	data_channel[6] = 7;
	data_channel[7] = 8;

	return 100;		//Should return value here

	//If -1, we got error
}
