/*
 * proximity_detect.c
 *
 *  Created on: 22 avr. 2021
 *      Author: gaspe
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include "sensors/proximity.h"

#include <proximity_detect.h>
//define


//semaphore
static BSEMAPHORE_DECL(prox_det_sem, TRUE);
static THD_WORKING_AREA(waProxLeader, 1024);
//doit on vraiment créer un Thread?
static THD_FUNCTION(ProxLeader, arg) {
	(void)arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    proximity_msg_t prox_values;
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");


    while(1){
    	//get_calibrated_prox
    	time = chVTGetSystemTime();
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	for(uint8_t i = 0 ; i < 8 ; i+=1){
    		chprintf((BaseSequentialStream *)&SD3, "%4d,", prox_values.ambient[i]);
    		chprintf((BaseSequentialStream *)&SD3, "%4d,", prox_values.reflected[i]);
    		chprintf((BaseSequentialStream *)&SD3, "%4d\r\n", prox_values.delta[i]);

    	}
    	chprintf((BaseSequentialStream *)&SD3, "\r\n");

    	chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
    }
}

void proximity_lead(void){
	chThdCreateStatic(waProxLeader, sizeof(waProxLeader), NORMALPRIO, ProxLeader, NULL);
}


