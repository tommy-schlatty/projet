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
static THD_WORKING_AREA(waProxReader, 256);
static THD_FUNCTION(ProxReader, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	proximity_start();
    	calibrate_ir();
    	for(uint16_t i = 0 ; i < 8 ; i+=1){
    		printf("La valeur du capteur vaut",get_prox(i));
    	}
    }
}



