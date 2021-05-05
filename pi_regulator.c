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
#define DIST_OBJ 10.f
#define ROT_GOAL IMAGE_BUFFER_SIZE/2
#define ERROR_THRESHOLD 5.f
#define SUM_THRESHOLD 10
#define FORWARD_COEFF 100
#define MIDDLE_F_COEFF 70
#define MIDDLE_B_COEFF 50
#define BACKWARD_COEFF 20
#define KP 2.f
#define KI 1.f
#define KP_DETECT 0.5f
#define KI_DETECT 0.1f
#define NB_CAPTEUR 8
#define NB_CAPTEUR_DROITE 4
static int32_t proximity_distance[NB_CAPTEUR] = {0};

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
		//right_motor_set_speed(500-speed);
		//left_motor_set_speed(500+speed);
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

        	//right_motor_set_speed(0);
			//left_motor_set_speed(0);
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

    int16_t speed_det = 0;
   // uint8_t motor_state;
    float error_det;
    float sum_error_det = 0;

    while(1){
    		error_det=0;
    		time_det = chVTGetSystemTime();
    		uint16_t prox_right=0;
    		uint16_t prox_left=0;
    		for(uint8_t i = 0 ; i < NB_CAPTEUR ; i+=1){
        	   proximity_distance[i]=get_prox(i);
        	   // coefficient à intégrer dans cette condition. cas ou le robot est de travers dans le labyrinthe
    		}
        	prox_right=FORWARD_COEFF*proximity_distance[0]+MIDDLE_F_COEFF*proximity_distance[1]+MIDDLE_B_COEFF*proximity_distance[2]+BACKWARD_COEFF*proximity_distance[3];
        	prox_left=FORWARD_COEFF*proximity_distance[7]+MIDDLE_F_COEFF*proximity_distance[6]+MIDDLE_B_COEFF*proximity_distance[5]+BACKWARD_COEFF*proximity_distance[4];
        	//On compare capteurs gauches et droites pour savoir dans quel sens tourner

        	if (prox_right>prox_left){
        	   for(uint8_t i = 0 ; i < NB_CAPTEUR_DROITE ; i+=1){
        		   error_det+=proximity_distance[i];
        	   }
           }

           else{
        	   for(uint8_t i = NB_CAPTEUR_DROITE ; i < NB_CAPTEUR ; i+=1){
        		   error_det-=proximity_distance[i];
        	   }
           }

        	if(abs(error_det)<500)
        		error_det=0;


           sum_error_det += error_det;
           if(sum_error_det >= SUM_THRESHOLD)
                   		sum_error_det = SUM_THRESHOLD;
                   else if(sum_error_det <= -SUM_THRESHOLD)
                   		sum_error_det = -SUM_THRESHOLD;

           speed_det = (int16_t)(KP_DETECT*error_det ); //+ KI_DETECT*sum_error_det terme KI a mettre apres

           right_motor_set_speed(200+speed_det);
           left_motor_set_speed(200-speed_det);
           chThdSleepUntilWindowed(time_det, time_det + MS2ST(10));
      }

}


void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
void detect_regulator_start(void){
	chThdCreateStatic(waProxRegulator, sizeof(waProxRegulator), NORMALPRIO, ProxRegulator, NULL);
}
