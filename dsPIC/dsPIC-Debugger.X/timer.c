/*
 * File:   timer.c
 * Author: Quentin BOYER
 *
 * Created on 19 septembre 2017, 17:17
 */

#include "timer.h"
#include "global.h"

// <editor-fold defaultstate="collapsed" desc="Variables">
volatile uint8_t timer2Overflow = 0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Init">
void initTimer() {
    initTimer1();
    initTimer2();
}
void initTimer1() { //Timer 1 -> asserv' interrupt
    T1CONbits.TON = 0; //disable timer
    TMR1 = 0; // Clear timer register
    PR1 = 1094; //10937;     //period 10 937 -> ~10ms
    T1CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T1CONbits.TCS = 0; //internal clock
    T1CONbits.TCKPS = 0b10; //prescaler : 1:64
    T1CONbits.TGATE = 0; //Gated time accumulation is disabled
    IFS0bits.T1IF = 0; //Clear flag
    //IEC0bits.T1IE = 1; //Enable interrupt
    IEC0bits.T1IE = 0; //Disable interrupt
    T1CONbits.TON = 1; //enable Timer1
}
void initTimer2() { //Timer 2 -> timing (delay_ms,delay_us,millis,micros)
    /*32bits mode*/
    /*TMR2 = LSB & TMR3 = MSB*/
    /*period PR2 = LSB & PR3 = MSB*/

    T2CONbits.TON = 0; //disable timer
    TMR2 = 0; // Clear timer register
    TMR3 = 0;
    /*Fp = 70Mhz -> 1tick = (100/7)ns -> 4200000000tick = 60s -> 4199999999 = 0xFA56E9FF */
    PR2 = 0xE9FF; //Period value
    PR3 = 0xFA56;
    T2CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T2CONbits.TCS = 0; //internal clock
    //T2CONbits.TCKPS = 0b10; //prescaler : 1:64
    T2CONbits.TCKPS = 0b00; //prescaler : 1:1
    //T2CONbits.TCKPS = 0b01; //prescaler : 1:8
    T2CONbits.TGATE = 0; //Gated time accumulation is disabled
    T2CONbits.T32 = 1; //Timer2 & Timer 3 form a single 32-bit timer
    IFS0bits.T2IF = 0; //Clear flag
    //IEC0bits.T2IE = 1;      //Enable interrupt
    IEC0bits.T2IE = 0;

    IFS0bits.T3IF = 0; //Clear flag
    IEC0bits.T3IE = 1;

    T2CONbits.TON = 1; //enable Timer1
}

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Timer interrupts">
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0; //Clear Timer1 interrupt flag
    timer2Overflow++;
}
// </editor-fold>


uint32_t micros(){
    IEC0bits.T3IE = 0;  //disable interrupt on timer 3 overflow
    uint32_t saveTMR2 = TMR2;
    if(IFS0bits.T3IF){      //timer overflow while reading -> read again
        //sendLog("erreur potentielle ms");
        IEC0bits.T3IE = 1;
        asm("NOP");         //let 1 cycle to trigger interrupt
        IEC0bits.T3IE = 0;
        if(!IFS0bits.T3IF){ //if interrupt has been triggered read again
            //sendLog("interrupt triggered");
            saveTMR2 = TMR2;
        }
    }
    uint32_t ret = TMR3HLD;
    ret = ret << 16;
    ret = ret + saveTMR2;
    ret = ret / 70;
    uint32_t t2of = timer2Overflow;
    t2of = t2of * 60000000UL;
    ret = ret + t2of;
    
    IEC0bits.T3IE = 1;  //enable interrupt on timer 3 overflow
    return ret;
}
uint32_t millis(){
    IEC0bits.T3IE = 0;  //disable interrupt on timer 3 overflow
    uint32_t saveTMR2 = TMR2;
    if(IFS0bits.T3IF){      //timer overflow while reading -> read again
        //sendLog("erreur potentielle ms");
        IEC0bits.T3IE = 1;
        asm("NOP");         //let 1 cycle to trigger interrupt
        IEC0bits.T3IE = 0;
        if(!IFS0bits.T3IF){ //if interrupt has been triggered read again
            //sendLog("interrupt triggered");
            saveTMR2 = TMR2;
        }
    }
    uint32_t ret = TMR3HLD;
    ret = ret << 16;
    ret = ret + saveTMR2;
    ret = ret / 70000; //140MHz
    //ret = ret / 3684;   //7.3728Mhz
    uint32_t t2of = timer2Overflow;
    t2of = t2of * 60000UL;    //140Mhz
    //t2of = t2of * 3157;     //7.3728Mhz
    ret = ret + t2of;
    
    IEC0bits.T3IE = 1;  //enable interrupt on timer 3 overflow
    return ret;
}
void delay_us(uint32_t delay){
   uint32_t tick = micros();
   while(micros() - tick < delay);
}
void delay_ms(uint32_t delay){
    uint32_t tick = millis();
    while(millis() - tick < delay);
}