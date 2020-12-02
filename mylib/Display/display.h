/*
 * display.h
 *
 *  Created on: Sep 25, 2019
 *      Author: ainajane
 */

#ifndef DISPLAY_DISPLAY_H_
#define DISPLAY_DISPLAY_H_


void ink_clear(void);						//clear entire display
void ink_update(void);						//clear reading every update
int ink_init(void);							//initialise display
void ink_display(void);						//display all reading
void ink_datetime(void);					//display date and time
void ink_skintype(void);					//display skintype
void ink_med(void);							//display user med
void ink_sed(void);							//display cumulative sed
void ink_uvindex(void);						//display uv index
void ink_bargraph(void);					//display bar graph
void ink_datapoint(void);					//display data point


#endif /* DISPLAY_DISPLAY_H_ */
