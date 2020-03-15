/* 
 * File: timer.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <math.h>
#include "constant.h"
#include "GPIO.h"
#include "UART.h"


void initTimer();
void initTimer1();
void initTimer2();

void delay_us(uint32_t delay);
void delay_ms(uint32_t delay);
uint32_t micros();
uint32_t millis();

#endif	/* TIMER_H */