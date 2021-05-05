#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define LINE_SIZE 2f //2cm
#define CONV_FACT 1400.f


static float distance_cm = 0;
static float line_position=IMAGE_BUFFER_SIZE/2;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

uint8_t detection_black_line(uint8_t *image);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    //horizontal ligne basse
	po8030_advanced_config(FORMAT_RGB565, 0, 478, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    //vertical ligne milieu
    //int8_t state = po8030_advanced_config(FORMAT_RGB565, 320, 0, 2, 480, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();



    while(1){

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();

		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t send_to_computer = 1;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		/*
		*	To complete
		*/

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}


		//for (uint16_t i = 1; i< 2*IMAGE_BUFFER_SIZE; i+=2)
		//{
		//	image[(i-1)/2] = img_buff_ptr[i] & 0b00011111;
		//}



		if(send_to_computer)
		{
			//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
			send_to_computer = 0;
		}
		else
			send_to_computer = 1;

		detection_black_line(image);
    }
}

uint8_t detection_black_line(uint8_t *image)
{
	uint32_t average = 0;
	for (uint16_t i = 0; i<IMAGE_BUFFER_SIZE;i++)
		average += image[i];

	average /= IMAGE_BUFFER_SIZE;

	uint16_t start, stop;
	for(start = 0; start < IMAGE_BUFFER_SIZE && image[start] > 0.45*average; start++);
	for(stop = start; stop < IMAGE_BUFFER_SIZE && image[stop] <= 0.45*average; stop++);

	if (start < 640)
	{
		//chprintf((BaseSequentialStream *)&SD3, "start=%d\tstop=%d\r\n", start, stop);
		distance_cm = CONV_FACT/(stop-start);
		line_position = (start+(stop-start)/2);
	}
	else
	{
		//chprintf((BaseSequentialStream *)&SD3, "no line\r\n");
		distance_cm = 0.;
		line_position = 1000;
	}


	//chprintf((BaseSequentialStream *)&SD3, "dist to line=%f\r\n", distance_cm);

	return (stop-start);

}

float get_distance_cm(void){

	return distance_cm;
}

uint16_t get_line_position(void){

	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
