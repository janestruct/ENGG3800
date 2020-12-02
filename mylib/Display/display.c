/*
 * display.c
 *
 *  Created on: Sep 25, 2019
 *      Author: ainajane
 */

#include "display.h"
#include "DEV_Config.h"
#include "GUI_Paint.h"
#include "ImageData.h"
#include "EPD_2in9.h"
#include "Debug.h"
#include "mock.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */
uint8_t byte = 0;
uint32_t nLoop=0;
UBYTE *BlackImage;
int med = 0;
int uv_index;

/* USER CODE END PV */

int ink_init(void);							//initialise display
void ink_clear(void);						//clear entire display
void ink_update(void);						//clear reading every update
void ink_display(void);						//display reading
void ink_datetime(void);					//display date and time
void ink_skintype(void);					//display skintype
void ink_med(void);							//display user med
void ink_sed(void);							//display cumulative sed
void ink_uvindex(void);						//display uv index
void ink_bargraph(void);					//display bar graph
void ink_datapoint(void);					//display data point
int get_skintype(void);
int getUV(void);
int get_med(void);
int get_sed(void);
int get_datapoint(void);


int ink_init(void) {

    UWORD Imagesize = ((EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1)) * EPD_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
	  Debug("Failed to apply for black memory...\r\n");
	  return -1;
    }

    if (EPD_Init(lut_full_update) != 0)
    {
      Debug("e-Paper init failed\r\n");
    }
    EPD_Clear();

    Paint_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, 270, WHITE);
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    Paint_DrawString_EN(80, 40, "ENGG3800", &Font24, WHITE, BLACK);
    Paint_DrawString_EN(80, 60, "Initialising", &Font16, WHITE, BLACK);
    EPD_Display(BlackImage);
    Paint_Clear(WHITE);

    return 0;
}

void ink_clear(void){

	static int counter = 0;
	if(counter > 4){

		Paint_Clear(WHITE);
		Paint_DrawString_EN(0, 15, "    ", &Font12, WHITE, BLACK);
		EPD_Display(BlackImage);
		Paint_Clear(WHITE);

	    counter = 0;
	}
	counter++;
}

void ink_update(void){

    Paint_ClearWindows(165, 20, 165 + Font12.Width * 15, 20 + Font12.Height, WHITE);		//skin type
    Paint_ClearWindows(75, 33, 75 + Font12.Width * 7, 33 + Font12.Height, WHITE);		//user MED
    Paint_ClearWindows(77, 45, 77 + Font12.Width * 7, 45 + Font12.Height, WHITE);		//MED dosage
    Paint_ClearWindows(72, 57, 72 + Font12.Width * 7, 57 + Font12.Height, WHITE);		//uv index
    Paint_ClearWindows(260, 110, 260 + Font12.Width * 3, 110 + Font12.Height, WHITE);	//data points

}

void ink_display(void) {

	if(EPD_Init(lut_full_update) != 0) {
		Debug("e-Paper init failed\r\n");
	}

  ink_clear();
  ink_datetime();
  ink_skintype();
  ink_med();
  ink_sed();
  ink_uvindex();
  ink_bargraph();
  ink_datapoint();

  EPD_Display(BlackImage);
  ink_update();

}

void ink_datetime(void) {

	EPD_Init(lut_partial_update);

	//top info bar
    Paint_DrawString_EN(0, 0, "ENGG3800", &Font12, WHITE, BLACK);						//ENGG3800
    Paint_DrawString_EN(70, 0, "GROUP 50", &Font12, WHITE, BLACK);						//team number


    Paint_DrawString_EN(150, 0, "HH:MM", &Font12, WHITE, BLACK);						//time setup
    Paint_DrawString_EN(200, 0, "DD/MM/YYYY", &Font12, WHITE, BLACK);					//date setup

    Paint_DrawRectangle(0, 12, 294, 14, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);			//outer frame
    Paint_DrawRectangle(0, 12, 2, 122, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);			//outer frame
    Paint_DrawRectangle(292, 14, 294, 124, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);			//outer frame
    Paint_DrawRectangle(0, 122, 294, 124, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);			//outer frame


}

void ink_skintype(void){

	int skin_type = get_skintype();
	char val[7];

	if(skin_type == 1){
		strcpy(val, "I");
	}else if(skin_type == 2){
		strcpy(val, "II");
	}else if(skin_type == 3){
		strcpy(val, "III");
	}else if(skin_type == 4){
		strcpy(val, "IV");
	}else if(skin_type == 5){
		strcpy(val, "V");
	}else if(skin_type == 6){
		strcpy(val, "VI");
	}

	EPD_Init(lut_partial_update);
	Paint_DrawString_EN(8,20, "Fitzpatrick skintype:", &Font12, WHITE, BLACK); 			//skintype setup

	if(skin_type < 7){

		Paint_DrawString_EN(165,20, val , &Font12, WHITE, BLACK);

	}

	else {
		Paint_DrawString_EN(165,20, "not specified" , &Font12, WHITE, BLACK);				//if skintype invalid or not specified
	}

}


void ink_med(void) {

	med = get_med();
	char val[5];
	itoa(med, val, 10);
	EPD_Init(lut_partial_update);

	Paint_DrawString_EN(8,33, "USER MED:", &Font12, WHITE, BLACK); 						//user MED setup
	Paint_DrawString_EN(75,33, val , &Font12, WHITE, BLACK);					//print value
	Paint_DrawString_EN(97,33, "SED" , &Font12, WHITE, BLACK);							//SED unit for MED

	if(med >= 440){
	  med = 0;
	}

}

void ink_sed(void) {

	int sed = get_sed();
	char val[5];
	itoa(sed, val, 10);

	EPD_Init(lut_partial_update);
	Paint_DrawString_EN(8,45, "TOTAL SED:", &Font12, WHITE, BLACK);						//total MED dosage setup
	Paint_DrawString_EN(98,45, "SED" , &Font12, WHITE, BLACK);						//SED unit for MED



	if (sed < 1.4*med){																//check value for warning

		Paint_DrawString_EN(77,45, val , &Font12, WHITE, BLACK);			//print value
	}
	else {

		Paint_DrawString_EN(77,45, val , &Font12, WHITE, BLACK);			//print value
		Paint_DrawString_EN(155,45, "[MED >40% WARNING]" , &Font12, WHITE, BLACK);		//print warning
	}

}


void ink_uvindex(void) {

	EPD_Init(lut_partial_update);

	char val[5];

	uv_index = getUV();

	Paint_DrawString_EN(8,57, "UV Index:", &Font12, WHITE, BLACK);						//uv index setup


	if (uv_index < 12){																							//check index value

		itoa(uv_index, val, 10);
																									//loop incerement only use for demo 2
		Paint_DrawString_EN(72,57, val , &Font12, WHITE, BLACK);										//print value
	}
	else {

		Paint_DrawString_EN(72,57, "11+" , &Font12, WHITE, BLACK);													//print value
	}

	if (uv_index > 3){

		Paint_DrawString_EN(155,57, "[USE SUNSCREEN]" , &Font12, WHITE, BLACK);				//print warning

	}

}

void ink_bargraph(void){
	EPD_Init(lut_partial_update);

	//bar graph indicator
	    Paint_DrawRectangle(6, 93, 285, 109, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);		//indicator outer frame
	    Paint_DrawRectangle(7, 94, 284, 108, WHITE, DRAW_FILL_FULL, DOT_PIXEL_1X1);		//indicator inner frame

	 //indicator for bar graph
	    Paint_DrawString_EN(10, 94, "0 |1 |2 |3 |4 |5 |6 |7 |8 |9 |10|11|11+", &Font12, WHITE, BLACK);

		int spacing = 20;


		if (uv_index < 12){																							//check index value
																							//loop incerement only use for demo 2
			Paint_DrawRectangle(6, 87, spacing * (uv_index+1) + 8, 92, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);			//print bar graph

		}
			else {

			Paint_DrawRectangle(6, 87, spacing * 13 + 18, 92, BLACK, DRAW_FILL_FULL, DOT_PIXEL_1X1);					//print bar graph

		}

}


void ink_datapoint(void) {

	int data = get_datapoint();
	char val[5];
	itoa(data, val, 10);
	EPD_Init(lut_partial_update);

	Paint_DrawString_EN(170,110, "Data Points:", &Font12, WHITE, BLACK);					//SD card data points setup
	Paint_DrawString_EN(260,110, val, &Font12, WHITE, BLACK);			//print value

}


