/*
 * File:   global.c
 * Author: QB
 *
 * Created on 4 janvier 2020, 11:54
 */

#include "global.h"

void *globalVar[SIZE_GLOBAL_VAR_ARRAY];

uint8_t testUint8   = 0;
int8_t testInt8     = 0;

uint16_t testUint16 = 0;
int16_t testInt16   = 0;

uint32_t testUint32 = 0;
int32_t testInt32   = 0;

long double testLD  = 0;

debugVar debugVarTab[NB_DEBUG_VAR];

void initPtrArray() {
    
    globalVar[ID_VAR_TEST_UI8]  = (void*)&testUint8;
    globalVar[ID_VAR_TEST_I8]	= (void*)&testInt8;
    globalVar[ID_VAR_TEST_UI16]	= (void*)&testUint16;
    globalVar[ID_VAR_TEST_I16]	= (void*)&testInt16;
    globalVar[ID_VAR_TEST_UI32]	= (void*)&testUint32;
    globalVar[ID_VAR_TEST_I32]	= (void*)&testInt32;
    globalVar[ID_VAR_TEST_LD]	= (void*)&testLD;
    
    globalVar[ID_VAR_VERBOSE]   =  (void*)&verbose;
}
