#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "sensors/proximity.h"

#define ROT_GOAL IMAGE_BUFFER_SIZE/2
#define ERROR_THRESHOLD 10.f
#define SUM_THRESHOLD 60
#define KP 2.4f
#define KI 0.12f

#define FORWARD_COEFF  4
#define MIDDLE_F_COEFF 3
#define MIDDLE_B_COEFF 2
#define BACKWARD_COEFF 1
#define KP_DETECT 0.2f
#define KI_DETECT 0.1f
#define NB_CAPTEUR 8
#define NB_CAPTEUR_DROITE 4

#define M_SPEED 700

static bool stop = 0;

void finishing_sequence(void);

static THD_WORKING_AREA(waLineFollowRegulator, 256);
static THD_FUNCTION(LineFollowRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

	int16_t speed = 0;
	float error, sum_error = 0.;

	while(!stop){
		time = chVTGetSystemTime();
		if (get_line_position()<640){

			error = get_line_position()-ROT_GOAL;

			sum_error += error;

			if(fabs(error) < ERROR_THRESHOLD)
			{
				error=0;
				sum_error = 0;
			}
			else
			{
				if(sum_error >= SUM_THRESHOLD)
					sum_error = SUM_THRESHOLD;
				else if(sum_error <= -SUM_THRESHOLD)
					sum_error = -SUM_THRESHOLD;
			}

			speed = (int16_t)(KP*error + KI*sum_error);

			//applies the speed from the PI regulator
			right_motor_set_speed(M_SPEED-speed);
			left_motor_set_speed(M_SPEED+speed);

		}
		else{
			right_motor_set_speed(M_SPEED);
			left_motor_set_speed(M_SPEED);
		}
		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

static THD_WORKING_AREA(waProxRegulator, 256);
static THD_FUNCTION(ProxRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time_det;
    int16_t proximity_distance[NB_CAPTEUR];
    int16_t sum_error_det = 0;

    while(!stop)
    {
		int16_t speed_det, error_det;
		uint16_t prox_right, prox_left;

		time_det = chVTGetSystemTime();
		uint16_t sum_prox=0;

		for(uint8_t i = 0 ; i < NB_CAPTEUR ; i+=1){
			sum_prox+=get_prox(i);
		}

		if (sum_prox>400){

			for(uint8_t i = 0 ; i < NB_CAPTEUR ; i++){
				proximity_distance[i]=get_prox(i);
				// coefficient a integrer dans cette condition. cas ou le robot est de travers dans le labyrinthe
			}

			prox_right=FORWARD_COEFF*proximity_distance[0]+MIDDLE_F_COEFF*proximity_distance[1]+MIDDLE_B_COEFF*proximity_distance[2]+BACKWARD_COEFF*proximity_distance[3];
			prox_left=FORWARD_COEFF*proximity_distance[7]+MIDDLE_F_COEFF*proximity_distance[6]+MIDDLE_B_COEFF*proximity_distance[5]+BACKWARD_COEFF*proximity_distance[4];


			// on cree error_det avec les coeff pour estimer la difference des deux cotes
			error_det=(prox_right-prox_left);


			if (fabs(error_det) < 500)
				error_det=0;


			sum_error_det += error_det;

			if(sum_error_det >= SUM_THRESHOLD)
				sum_error_det = SUM_THRESHOLD;
			else if(sum_error_det <= -SUM_THRESHOLD)
				sum_error_det = -SUM_THRESHOLD;

			speed_det = (int16_t)(KP_DETECT*error_det+ KI_DETECT*sum_error_det );

			right_motor_set_speed(M_SPEED+speed_det);
			left_motor_set_speed(M_SPEED-speed_det);
		}
		else{
			right_motor_set_speed(M_SPEED);
			left_motor_set_speed(M_SPEED);
		}
		chThdSleepUntilWindowed(time_det, time_det + MS2ST(5)); // cela fonctionne avec 10 image 10 prox. Meilleur avec 5 prox pour eviter
    }
}

static THD_WORKING_AREA(waFinish, 256);
static THD_FUNCTION(Finish, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1)
    {
    		time = chVTGetSystemTime();
		stop = get_finish_line();
		if(stop)
			finishing_sequence();
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}

void finishing_sequence(void)
{

	//Robot avance tout droit pour depasser la ligne
	right_motor_set_speed(M_SPEED);
	left_motor_set_speed(M_SPEED);
	chThdSleepMilliseconds(1000000/M_SPEED);

	//Rotation du robot de 540deg sur lui meme
	right_motor_set_speed(1000);
	left_motor_set_speed(-1000);
	chThdSleepMilliseconds(1950);

	//Arret du robot
	right_motor_set_speed(0);
	left_motor_set_speed(0);

}

void finish_start(void){
	chThdCreateStatic(waFinish, sizeof(waFinish), NORMALPRIO, Finish, NULL);
}

void line_follow_regulator_start(void){
	chThdCreateStatic(waLineFollowRegulator, sizeof(waLineFollowRegulator), NORMALPRIO, LineFollowRegulator, NULL);
}
void detect_regulator_start(void){
	chThdCreateStatic(waProxRegulator, sizeof(waProxRegulator), NORMALPRIO, ProxRegulator, NULL);
}
