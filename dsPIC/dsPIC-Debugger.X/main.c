/*
 * File:   main.c
 * Author: Quentin BOYER
 *
 * Created on March 13, 2020, 20:02 PM
 */

#include "constant.h"



#include <xc.h>
#include <math.h>
#include <p33EP512GM310.h>
#include <stdint.h>
#include "clock.h"
#include "GPIO.h"
#include "timer.h"
#include "UART.h"
#include "interrupt.h"
#include "DMA.h"
#include "global.h"


//Global variables
//char TX[TX_SIZE];
char RX[RX_SIZE];
char unsigned TX_i;
char unsigned RX_i;

extern volatile uint8_t timer2Overflow;
int main(){
    initClock(); //Clock 140 MHz
    initGPIO();
    initDMA();
    initUART();
    initInt();
    initPtrArray();
    initTimer();
    
    // <editor-fold defaultstate="collapsed" desc="Reset led">
    TRISFbits.TRISF7 = 0;
    LATFbits.LATF7 = 1;
    delay_ms(50);
    LATFbits.LATF7 = 0;
    delay_ms(50);
    LATFbits.LATF7 = 1;
    delay_ms(50);
    LATFbits.LATF7 = 0;
    delay_ms(50); // </editor-fold>
    
    uint32_t t1 = millis();
    uint8_t tic = 1;
    
    //debugVar debugVarTab[NB_DEBUG_VAR];
    uint8_t i_debugVar;
    for(i_debugVar = 0; i_debugVar < NB_DEBUG_VAR; i_debugVar++){
        debugVarTab[i_debugVar].id = 0;
        debugVarTab[i_debugVar].nb = 0;
        debugVarTab[i_debugVar].on = 0;
        debugVarTab[i_debugVar].onTimeStamp = 0;
        debugVarTab[i_debugVar].period = 0;
        debugVarTab[i_debugVar].time = 0;
        debugVarTab[i_debugVar].type = 0;
    }

    while(1){
        uint32_t t = millis();
        testUint32 = t;
        
        uint32_t deltaT = t - t1;
        if(deltaT > 1000){
            LED_PLATINE = !LED_PLATINE;
            if(tic){
                sendLog("Tic !");
                tic = 0;   
            }
            else{
                sendLog("Tac !");
                tic = 1;   
            }
            if(deltaT > 1100)
                sendLog("ERROR !");
            t1 = t;
            testUint8++;
        }
        
        // <editor-fold defaultstate="collapsed" desc="Debugger">
        for (i_debugVar = 0; i_debugVar < NB_DEBUG_VAR; i_debugVar++) {
            if (debugVarTab[i_debugVar].on) {
                if (t - debugVarTab[i_debugVar].time >= debugVarTab[i_debugVar].period) {
                    debugVarTab[i_debugVar].time = t;
                    switch (debugVarTab[i_debugVar].type) {
                        case VAR_8b:
                        {
                            uint8_t *ptr = (uint8_t*)globalVar[debugVarTab[i_debugVar].id];
                            uint8_t var = *ptr;
                            sendVar8(debugVarTab[i_debugVar].id,var);
                        }
                            break;
                        case VAR_16b:
                            break;
                        case VAR_32b:
                        {
                            uint32_t *ptr = (uint32_t*)globalVar[debugVarTab[i_debugVar].id];
                            uint32_t var = *ptr;
                            sendVar32(debugVarTab[i_debugVar].id,var);
                        }
                            break;
                        case VAR_64b:
                            //
                            break;
                        case VAR_LD_64b:
                            sendLongDoubleCode(debugVarTab[i_debugVar].id);
                            break;
                        default:
                            break;
                    }
                }
            }
        }// </editor-fold>
        
        CheckMessages();
    }
    return 0;
}