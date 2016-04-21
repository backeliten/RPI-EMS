/*
 * main.c
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

#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <native/cond.h>
#include <native/mutex.h>
#include <native/heap.h>
#include <native/alarm.h>
#include <native/intr.h>
#include <native/queue.h>

#include "../header/communication.h"
#include "../header/wheeldecoder.h"

#define VERSION 		1
#define MAJOR			3
#define MINOR			11

RT_QUEUE q_desc;

/*Function declaration*/
void main_loop(void *cookie);

/*RT Tasks and interrups*/
RT_TASK main_task;

void catch_signal()
{
	exit(1);
}

int main(void)
{
	int err;
	int i;
	unsigned int value;

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    rt_print_auto_init(1);

    /*Main Task Creation*/
    rt_task_create(&main_task, "RPi-EMS-Main", 0,99,0);
    rt_task_start(&main_task, &main_loop, NULL);

    getchar();		//Wait for keypress to exit

    rt_task_delete(&main_task);

    //FIXME: SHTUDOWN FUNCTION TO MAKE EVERYTHING TURN OFF

    return 0;
}
void main_loop(void *cookie)
{
    ssize_t len;
    ku_comm_s 		ku_comm;
    int err;

    printf("Startup of EMS for Raspberry PI:\n-User process and main calculation program by Jonas Back \n");
    printf("\n\tVersion: %u.%u.%u\n", VERSION,MAJOR,MINOR);
    printf("____________________________________________\n");

    printf("Try to bind to kernel queue...");
    err = rt_queue_bind(&q_desc,"KEMS-RTDATA",TM_INFINITE);
     if (err)
     {
    	 printf("Failed to get queue bind!");
    	 exit(-1);
     }
     else
     {
    	 printf("Success!\n");
     }
    	// fail();

//	uint8_t iomask = 0;
//	uint8_t count = 0;
   // rt_task_set_periodic(NULL, TM_NOW, MAIN_SLEEP);

     printf("Start to get data:\n");
     printf("____________________________________________\n");

    while ((len = rt_queue_read(&q_desc,&ku_comm,sizeof(ku_comm_s),TM_INFINITE)) > 0)
	  {
    	printf("RPM: %u", ku_comm.rpm);
    	if(ku_comm.sync == SYNC)
    	{
    		printf(" | SYNC \n");
    	}
    	else
    	{
    		printf(" | NOT_SYNC \n");
    	}
		//  printf("received message> %s \n", (const char *)msg);
		//FIXME: We got a message to calculate here
    	//rt_queue_free(&q_desc,msg);
	  }
}

