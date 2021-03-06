#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define LOW_FACTOR_LINE 0.35f
#define HIGH_FACTOR_LINE 0.60f
#define MIN_LINE_SIZE 20

#define LOW_FACTOR_FINISH 0.45f
#define HIGH_FACTOR_FINISH 0.65f
#define MIN_STEP_FINISH 7


static float line_position=IMAGE_BUFFER_SIZE/2;
static bool finish_line = false;



//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

void detection_black_line(uint8_t *image);
bool detection_finish_line(uint8_t *image);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 478 and 479 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 478, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

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

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		finish_line = detection_finish_line(image);
		detection_black_line(image);
    }
}

void detection_black_line(uint8_t *image)
{
	uint32_t average = 0;
	for (uint16_t i = 0; i<IMAGE_BUFFER_SIZE;i++)
		average += image[i];

	average /= IMAGE_BUFFER_SIZE;

	uint16_t start, stop;
	//determine pixel de depart de lq ligne
	for(start = 0; start < IMAGE_BUFFER_SIZE && image[start] > LOW_FACTOR_LINE*average; start++);
	//determine pixel de fin de lq ligne
	for(stop = start; stop < IMAGE_BUFFER_SIZE && image[stop] <= HIGH_FACTOR_LINE*average; stop++);

	//verification si valeurs sont correctes
	if (start < IMAGE_BUFFER_SIZE && !finish_line && (stop-start) >= MIN_LINE_SIZE)
		line_position = (start+(stop-start)/2);
	else
		line_position = IMAGE_BUFFER_SIZE;
}

bool detection_finish_line(uint8_t *image)
{
	uint32_t average = 0;
	uint8_t step = 0;

	for (uint16_t i = 0; i<IMAGE_BUFFER_SIZE;i++)
		average += image[i];

	average /= IMAGE_BUFFER_SIZE;

	bool started = 0;
	//determine le nombre sauts de luminosit?? vus par la camera
	for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
	{
		if(image[i] <= LOW_FACTOR_FINISH*average && started == false && image[i+1] <= LOW_FACTOR_FINISH*average && image[i+2] <= LOW_FACTOR_FINISH*average && image[i+5] <= LOW_FACTOR_FINISH*average)
			started = true;
		else if(image[i] > HIGH_FACTOR_FINISH*average && started == true && image[i+1] > HIGH_FACTOR_FINISH*average && image[i+2] > HIGH_FACTOR_FINISH*average && image[i+5] > HIGH_FACTOR_FINISH*average)
		{
			step +=2;
			started = false;
		}
	}

	if (step >= MIN_STEP_FINISH)
		return true;

	return false;
}

uint16_t get_line_position(void)
{
	return line_position;
}

bool get_finish_line(void)
{
	return finish_line;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
