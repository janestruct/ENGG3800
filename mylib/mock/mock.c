/*
 * display.c
 *
 *  Created on: Sep 25, 2019
 *      Author: ainajane
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int skin = 0 ;
int gmed = 25 ;
int gsed = 10 ;
int data = 0 ;

int get_skintype(void);						//clear entire display
int get_med(void);
int get_sed(void);
int get_datapoint(void);


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


int get_skintype(void){

	skin++;
	return skin;
}

int get_med(void){

	gmed++;
	return gmed;
}

int get_sed(void){

	gsed += 10;
	return gsed;
}

int get_datapoint(void){

	data++;
	return data;
}



/* USER CODE BEGIN PV */



