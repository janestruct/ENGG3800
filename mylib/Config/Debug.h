/*****************************************************************************
* | File      	:	Debug.h
* | Author      :   Waveshare team
* | Function    :	debug with printf
* | Info        :
*   Image scanning
*      Please use progressive scanning to generate images or fonts
*----------------
* |	This version:   V1.0
* | Date        :   2018-01-11
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H
#include <stdio.h>

#define DEBUG 1 //set to 0 to turn off debug printing

#if DEBUG
	#define Debug(__info,...) printf("Debug: " __info,##__VA_ARGS__)
#else
	#define Debug(__info,...) (void) 0
#endif

//PRINTF Wrapper function for actual data you want to send to device
#define OUTPUT 1 //set to 0 to turn off debug printing

#if OUTPUT
	#define Output(__info,...) printf(__info,##__VA_ARGS__)
#else
	#define Output(__info,...) (void) 0
#endif

#endif
