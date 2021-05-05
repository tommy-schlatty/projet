#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>


#define ROT_GOAL IMAGE_BUFFER_SIZE/2
#define ERROR_THRESHOLD 10.f
#define SUM_THRESHOLD 60
#define KP 2.4f
#define KI 0.12f
#define KD 0.0f
#define DT 0.001f

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

	int16_t speed = 0;
	float error, prev_error = 0., sum_error = 0., slope;


	while(1){
		time = chVTGetSystemTime();


    		error = get_line_position()-ROT_GOAL;

    		sum_error += error;

		slope = (error-prev_error)/DT;

		prev_error = error;

		if(fabs(error) < ERROR_THRESHOLD)
    		{
			error=0;
    			sum_error = 0;
    			slope = 0;
    		}
		else
		{
    			if(sum_error >= SUM_THRESHOLD)
    				sum_error = SUM_THRESHOLD;
    			else if(sum_error <= -SUM_THRESHOLD)
    				sum_error = -SUM_THRESHOLD;
		}

        speed = (int16_t)(KP*error + KI*sum_error + KD*slope);




         //applies the speed from the PI regulator
		if(error < 640)
		{
            	right_motor_set_speed(500-speed);
    			left_motor_set_speed(500+speed);
		}
		else
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}


		//1000Hz
		chThdSleepUntilWindowed(time, time + MS2ST(1));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
