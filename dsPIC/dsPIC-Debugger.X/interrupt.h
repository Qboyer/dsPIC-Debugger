/* 
 * File: interrupt.h  
 * Author: Quentin BOYER
 * Comments: check most appropriate variable type <-> speed of execution 
 *              optimization : remove prevProcessVariable
 * Revision history: 1
 */

#ifndef INTERRUPT_H
#define	INTERRUPT_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "constant.h"
#include <stdint.h>
#define _ISR_PSV        __attribute__((__interrupt__, __auto_psv__))

void initInt();

#endif	/* INTERRUPT_H */