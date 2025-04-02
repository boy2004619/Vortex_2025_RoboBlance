#ifndef __PC_CTRL_H
#define __PC_CTRL_H

#include "main.h"

typedef __packed struct
{

int16_t mouse_x; 
int16_t mouse_y; 
int16_t mouse_z; 
int8_t  left_button_down; 
int8_t  right_button_down; 
uint16_t keyboard_value; 
uint16_t reserved; 

}remote_control_t; 


void imagelinkReadData(uint8_t *buff);
#endif
