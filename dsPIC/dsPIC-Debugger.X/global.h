/*
 * File:   global.h
 * Author: QB
 *
 * Created on 4 janvier 2020, 11:54
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#include <xc.h>
#include <stdint.h>

#define SIZE_GLOBAL_VAR_ARRAY   255
#define NB_DEBUG_VAR    2

#define ID_VAR_TEST_UI8		100
#define ID_VAR_TEST_I8		101
#define ID_VAR_TEST_UI16	102
#define ID_VAR_TEST_I16		103
#define ID_VAR_TEST_UI32	104
#define ID_VAR_TEST_I32		105
#define ID_VAR_TEST_LD		106

#define ID_VAR_VERBOSE      110


void initPtrArray();

typedef struct debugVar debugVar;
struct debugVar
{
    uint8_t on;
    uint8_t id;
    uint32_t period;
    uint8_t type;
    uint8_t onTimeStamp;
    uint32_t time;
    uint8_t nb;
};

extern void *globalVar[SIZE_GLOBAL_VAR_ARRAY];
extern debugVar debugVarTab[NB_DEBUG_VAR];

extern volatile uint8_t verbose;

extern uint8_t testUint8;
extern int8_t testInt8;

extern uint16_t testUint16;
extern int16_t testInt16;

extern uint32_t testUint32;
extern int32_t testInt32;

extern long double testLD;

#endif	/* GLOBAL_H */

