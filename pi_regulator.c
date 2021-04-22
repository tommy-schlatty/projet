#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define DIST_OBJ 10.f
#define ROT_GOAL IMAGE_BUFFER_SIZE/2
#define ERROR_THRESHOLD 5.f
#define SUM_THRESHOLD 10
#define KP 2.f
#define KI 1.f

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    uint8_t motor_state;
    float error=0;
    float sum_error = 0;


    while(1){
        time = chVTGetSystemTime();

        /*
		*	To complete
		*/

        //error = get_distance_cm()-DIST_OBJ;


        error = get_line_position()-ROT_GOAL;

        if(fabs(error) < ERROR_THRESHOLD)
        {
        		error=0;
        		sum_error = 0;

        }

        sum_error += error;




        //if (abs(error) <= 2)
        	//	sum_error += error/abs(error)*20;

        if(sum_error >= SUM_THRESHOLD)
        		sum_error = SUM_THRESHOLD;
        else if(sum_error <= -SUM_THRESHOLD)
        		sum_error = -SUM_THRESHOLD;

        //chprintf((BaseSequentialStream *)&SD3, "sum_erreur=%f\r\n", sum_error);

        speed = (int16_t)(KP*error + KI*sum_error);
        /*
        chprintf((BaseSequentialStream *)&SD3, "speed=%d\r\n", speed);

        if (abs(speed) >= 4000)
        		speed = speed/abs(speed)*4000;

        if (abs(error) <= 0.8)
        		speed *= abs(error);

		*/


        //applies the speed from the PI regulator
        if(error < 640)
        {
		right_motor_set_speed(500-speed);
		left_motor_set_speed(500+speed);
		motor_state = 1;
        }
        else
        {	/*
        		if(motor_state)
        		{
        			right_motor_set_speed(500);
        			left_motor_set_speed(500);
        			chThdSleepMilliseconds(100);
        		}*/

        		right_motor_set_speed(0);
			left_motor_set_speed(0);
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
