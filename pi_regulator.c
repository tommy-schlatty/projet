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
#define KD 0.0f
#define DT 0.001f
enum MODE{initialisation,pi_regulator, prox_regulator };
#define FORWARD_COEFF  4
#define MIDDLE_F_COEFF 3
#define MIDDLE_B_COEFF 2
#define BACKWARD_COEFF 1
#define KP_DETECT 0.2f
#define KI_DETECT 0.1f
#define NB_CAPTEUR 8
#define NB_CAPTEUR_DROITE 4
static int32_t proximity_distance[NB_CAPTEUR] = {0};

// permet de definir le mode dans lequel on se trouve
int8_t mode=0; // 1 appelle piregulator // 2 appelle proxregulator

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

	int16_t speed = 0;
	float error, prev_error = 0., sum_error = 0., slope;


	while(1){
		time = chVTGetSystemTime();
		if (get_line_position()<640){

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
							//right_motor_set_speed(100);
							//left_motor_set_speed(100);
						}

		}
		else{
			right_motor_set_speed(500);
					left_motor_set_speed(500);
		}
		//1000Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));

	}
}

static THD_WORKING_AREA(waProxRegulator, 256);
static THD_FUNCTION(ProxRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time_det;


    int16_t sum_error_det = 0;

    while(1){
		int16_t speed_det, error_det;
		uint16_t error_det_right = 0, error_det_left = 0;
		uint16_t prox_right, prox_left;

		time_det = chVTGetSystemTime();
		uint16_t sum_prox=0;
		    for(uint8_t i = 0 ; i < NB_CAPTEUR ; i+=1){
		    			sum_prox+=get_prox(i);
		    }
		    chprintf((BaseSequentialStream *)&SD3, "condition pro1=%d\r\n", sum_prox);
		    if (sum_prox>400){

		for(uint8_t i = 0 ; i < NB_CAPTEUR ; i+=1){
			proximity_distance[i]=get_prox(i);

			// coefficient a integrer dans cette condition. cas ou le robot est de travers dans le labyrinthe
		}
		prox_right=FORWARD_COEFF*proximity_distance[0]+MIDDLE_F_COEFF*proximity_distance[1]+MIDDLE_B_COEFF*proximity_distance[2]+BACKWARD_COEFF*proximity_distance[3];
		prox_left=FORWARD_COEFF*proximity_distance[7]+MIDDLE_F_COEFF*proximity_distance[6]+MIDDLE_B_COEFF*proximity_distance[5]+BACKWARD_COEFF*proximity_distance[4];
		//On compare capteurs gauches et droites pour savoir dans quel sens tourner
		chprintf((BaseSequentialStream *)&SD3, "right1=%d\r\n", prox_right);
		chprintf((BaseSequentialStream *)&SD3, "left1=%d\r\n", prox_left);

		// ces deux boucles for ne servent pas si l'on s'occupe uniquement des valeurs avec coeffs

		for(uint8_t i = 0 ; i < NB_CAPTEUR_DROITE ; i+=1){
			error_det_right+=proximity_distance[i];
		}

		for(uint8_t i = NB_CAPTEUR_DROITE ; i < NB_CAPTEUR ; i+=1){
			error_det_left+=proximity_distance[i];
		}

		chprintf((BaseSequentialStream *)&SD3, "right2=%d\r\n", error_det_right);
		chprintf((BaseSequentialStream *)&SD3, "left2=%d\r\n", error_det_left);

		// on cree error_det avec les coeff pour estimer la difference des deux cotes
		error_det=(prox_right-prox_left);


		if (fabs(error_det) < 500)
			error_det=0;


		sum_error_det += error_det;

		if(sum_error_det >= SUM_THRESHOLD)
			sum_error_det = SUM_THRESHOLD;
		else if(sum_error_det <= -SUM_THRESHOLD)
			sum_error_det = -SUM_THRESHOLD;

		speed_det = (int16_t)(KP_DETECT*error_det+ KI_DETECT*sum_error_det ); //+ KI_DETECT*sum_error_det terme KI a mettre apres

		right_motor_set_speed(500+speed_det);
		left_motor_set_speed(500-speed_det);
		    }
		    else{
		    	right_motor_set_speed(500);
		    			left_motor_set_speed(500);
		    }
		chThdSleepUntilWindowed(time_det, time_det + MS2ST(5)); // cela fonctionne avec 10 image 10 prox. Meilleur avec 5 prox pour eviter

    }
}


void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
void detect_regulator_start(void){
	chThdCreateStatic(waProxRegulator, sizeof(waProxRegulator), NORMALPRIO, ProxRegulator, NULL);
}
