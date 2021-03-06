/*
 * File:   UART.c
 * Author: Quentin BOYER
 *
 * Created on 31 octobre 2017, 21:10
 * 
 * Notes : Parfois des erreurs lors de l'envoi de plusieurs octets en meme temps depui la RPi, � v�rifier en augmentant la clock
 */

#include "UART.h"
#include "global.h"

// <editor-fold defaultstate="collapsed" desc="Variables">
//extern long double xc;

volatile uint8_t verbose = 1;

char TxLoopBuffer[TX_SIZE];
char TxLoopBuffer2[TX_SIZE];
char RxBuffer[RX_SIZE];
char RxBuffer2[RX_SIZE];
volatile uint8_t RxDMABuffer[RX_DMA_SIZE];
uint16_t iD, iF, iD2, iF2; //index of first data and of first free element
unsigned char TxOn, TxOn2;
uint16_t iRx, iRx2;
uint16_t start = 0;

// </editor-fold>				 
// <editor-fold defaultstate="collapsed" desc="Init">

void initUART() {
    //initUART1();
    initUART2();
}

void initUART1() { //Bluetooth
    IEC0bits.U1TXIE = 0; //Disable UART1 Tx interrupt
    IEC0bits.U1RXIE = 0; //Disable UART1 Rx interrupt

    TRISBbits.TRISB6 = 0; //TX
    TRISBbits.TRISB5 = 1; //RX

    RPINR18 = 0b0100101;
    RPOR2bits.RP38R = 0b000001;

    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U1MODEbits.IREN = 0; // Bit12 No IR translation
    U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    U1MODEbits.UEN = 0b00;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0;

    U1STAbits.UTXBRK = 0; //Bit11 Disabled
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0; //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U1STAbits.UTXINV = 0;
    U1STAbits.ADDEN = 0;
    U1STAbits.URXISEL = 0;

    U1BRG = BRGVAL;

    IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1; // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts

    U1MODEbits.UARTEN = 1; //Enable the module
    U1STAbits.UTXEN = 1;

    iD = 0;
    iF = 0;
    TxOn = 0;
    iRx = 0;
}

void initUART2() { //Raspberry Pi
    IEC1bits.U2TXIE = 0; //Disable UART2 Tx interrupt
    IEC1bits.U2RXIE = 0; //Disable UART2 Rx interrupt

    //RPINR19 = 0b01010000; //RPI80(pin52-RE0) tied to UART2 RX
    //RPOR9bits.RP81R = 0b000011; //RP81 (pin53-RE1) tied to UART2 TX
    
    RPINR19 = 27; //RPI27(pin21-RA11) tied to UART2 RX
    RPOR2bits.RP38R = 0b000011; //RP38 (pin70-RB6) tied to UART2 TX

    U2MODEbits.UARTEN = 0;
    U2MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U2MODEbits.IREN = 0; // Bit12 No IR translation
    U2MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    U2MODEbits.UEN = 0b00;
    U2MODEbits.LPBACK = 0;
    U2MODEbits.ABAUD = 0;
    U2MODEbits.URXINV = 0;
    U2MODEbits.BRGH = UART2_HIGH_SPEED;
    U2MODEbits.PDSEL = 0b00;
    U2MODEbits.STSEL = 0;

    U2STAbits.UTXBRK = 0; //Bit11 Disabled
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0; //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U2STAbits.UTXINV = 0;
    U2STAbits.ADDEN = 0;
    U2STAbits.URXISEL = 0;

    U2BRG = BRGVAL2;

    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 1;
    IEC1bits.U2RXIE = 0;

    U2MODEbits.UARTEN = 1; //Enable the module
    U2STAbits.UTXEN = 1;

    iD2 = 0;
    iF2 = 0;
    TxOn2 = 0;
    iRx2 = 0;
}// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Interrupts">
/*void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
}*/

/*void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void){
    IFS1bits.U2RXIF = 0;
}*/

/*void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if(iD != iF){
        U1TXREG = pop();
    }
    else
        TxOn = 0;
}*/

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag
    if (iD2 != iF2) {
        U2TXREG = pop2();
    } else
        TxOn2 = 0;
}// </editor-fold>

void push(char c){
    TxLoopBuffer[iF] = c;
    iF++;
    if(iF == TX_SIZE)
        iF = 0;
}
void push2(char c){
    TxLoopBuffer2[iF2] = c;
    iF2++;
    if(iF2 == TX_SIZE)
        iF2 = 0;
}
char pop(){
    char r = TxLoopBuffer[iD];
    iD++;
    if(iD == TX_SIZE)
        iD = 0;
    return r;
}
char pop2(){
    char r = TxLoopBuffer2[iD2];
    iD2++;
    if(iD2 == TX_SIZE)
        iD2 = 0;
    return r;
}

char *itoa(int value) {
     static char buffer[12];        // 12 bytes is big enough for an INT32
     int original = value;        // save original value
 
     int c = sizeof(buffer)-1;
 
     buffer[c] = 0;                // write trailing null in last byte of buffer    
 
     if (value < 0)                 // if it's negative, note that and take the absolute value
         value = -value;
     
     do                             // write least significant digit of value that's left
     {
         buffer[--c] = (value % 10) + '0';    
         value /= 10;
     } while (value);
 
     if (original < 0) 
         buffer[--c] = '-';
 
     return &buffer[c];
 }
char *dtoa(double value){
    static char buffer[10];
    char tempBuffer[10];
    /*int*/
    int8_t i;
    int8_t j = 0;
    int integerPart = (int)value;
    double dec = value;
    if(value < 0){
        integerPart = -integerPart;
        dec = -dec;
        buffer[0] = '-';
        j++;
    }
    dec = dec - integerPart;
    for(i = 0; i < 10; i++){
        tempBuffer[i] = (integerPart % 10) + '0';
        integerPart /= 10;
        if(!integerPart)
            break;
    }

    /*reverse string*/
    while(i >= 0){
        buffer[j] = tempBuffer[i];
        i--;
        j++;
    }
    /*decimal*/
    buffer[j] = '.';
    j++;
    while(j < 9){
        dec *= 10;
        buffer[j] = ((int)dec)%10 + '0';
        dec = dec - (int)dec;
        j++;
    }
    buffer[9] = '\0';
    return buffer;
}
void CheckMessages(){
    while(1){
        
        // <editor-fold defaultstate="collapsed" desc="Get message">
        uint16_t nextDMAWriteAdress = DMA1STAL;
        uint16_t writeIndex = nextDMAWriteAdress - (uint16_t) & RxDMABuffer;
        int16_t nBytesToRead = writeIndex - start;

        if (nBytesToRead < 0) //RX Buffer overflow
            nBytesToRead += RX_DMA_SIZE;

        if (nBytesToRead == 0) //no data in the buffer
            return;

        uint8_t size = RxDMABuffer[start];
        if (nBytesToRead < size + 1) //the message is incomplete
            return;

        /*Circular buffer index*/
        uint16_t iCode = (start + 1) % RX_DMA_SIZE;
        uint16_t iArg1 = (start + 2) % RX_DMA_SIZE;
        uint16_t iArg2 = (start + 3) % RX_DMA_SIZE;
        uint16_t iArg3 = (start + 4) % RX_DMA_SIZE;
        uint16_t iArg4 = (start + 5) % RX_DMA_SIZE;
        uint16_t iArg5 = (start + 6) % RX_DMA_SIZE;
        uint16_t iArg6 = (start + 7) % RX_DMA_SIZE;
        uint16_t iArg7 = (start + 8) % RX_DMA_SIZE;
        uint16_t iArg8 = (start + 9) % RX_DMA_SIZE;
        uint16_t iArg9 = (start + 10) % RX_DMA_SIZE;
        uint16_t iArg10 = (start + 11) % RX_DMA_SIZE;
        uint16_t iChecksum = (start + size) % RX_DMA_SIZE;

        uint16_t i, j;
        uint8_t checksum = 0;
        for (i = 0; i < size; i++) {
            j = (start + i) % RX_DMA_SIZE;
            checksum += RxDMABuffer[j];
        }
        start += RxDMABuffer[start] + 1;
        start = start % RX_DMA_SIZE;
        if (checksum != RxDMABuffer[iChecksum]) {
            uint8_t saveVerbose = verbose;
            verbose = 1;
            sendLog("Checksum error !\n");
            verbose = saveVerbose;
            return;
            //cout << "cheksum error : " << (int)checksum << " =/= " << (int)(inBuf[start + size]) << endl;
        }// </editor-fold>
        switch (RxDMABuffer[iCode]) {

                // <editor-fold defaultstate="collapsed" desc="Reset">
            case RX_CODE_RESET:
                if (size != RX_SIZE_RESET)
                    return;
                sendLog(("RESET !\n"));
                asm("RESET");
                //delay_ms(10);
                //asm("MOV 0x1000, W15");
                //asm("GOTO 0x0");
                //__asm__ volatile ("reset");
                break; // </editor-fold>
                
                // <editor-fold defaultstate="collapsed" desc="Set">
            case RX_CODE_SET:
            {
                uint8_t var = RxDMABuffer[iArg1];
                uint8_t type = RxDMABuffer[iArg2];
				// <editor-fold defaultstate="collapsed" desc="8bits">													  
                if (type == VAR_8b) {
                    uint8_t value = RxDMABuffer[iArg3];
                    uint8_t *ptr = (uint8_t*)globalVar[var];
                    *ptr = value;
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="16bits">										   
                else if (type == VAR_16b) {
                    int16_t value = (RxDMABuffer[iArg3] << 8) + RxDMABuffer[iArg4]; //or ptr / union / memcpy
                    uint16_t *ptr = (uint16_t*)globalVar[var];
                    *ptr = value;
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="32bits">
                else if (type == VAR_32b) {
                    uint32_t value = ((uint32_t)RxDMABuffer[iArg3] << 24) + ((uint32_t)RxDMABuffer[iArg4] << 16) + ((uint32_t)RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                    uint32_t *ptr = (uint32_t*)globalVar[var];
                    *ptr = value;
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="long double 64bits">
                else if (type == VAR_LD_64b) {
                    uint8_t *ptr = globalVar[var];
                    ptr[0] = RxDMABuffer[iArg3];
                    ptr[1] = RxDMABuffer[iArg4];
                    ptr[2] = RxDMABuffer[iArg5];
                    ptr[3] = RxDMABuffer[iArg6];
                    ptr[4] = RxDMABuffer[iArg7];
                    ptr[5] = RxDMABuffer[iArg8];
                    ptr[6] = RxDMABuffer[iArg9];
                    ptr[7] = RxDMABuffer[iArg10];
                }// </editor-fold>

            } // </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Get">
            case RX_CODE_GET_BIS:
            {
                if (size != RX_SIZE_GET_BIS)
                    return;

                uint8_t type = RxDMABuffer[iArg1];
                uint8_t varCode = RxDMABuffer[iArg2];
                // <editor-fold defaultstate="collapsed" desc="8bits">		
                if (type == VAR_8b) {
                    sendVar8Code(varCode);
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="16bits">										   
                else if (type == VAR_16b) {
                    sendVar16Code(varCode);
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="32bits">
                else if (type == VAR_32b) {
                    sendVar32Code(varCode);
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="long double 64bits">
                else if (type == VAR_LD_64b) {
                    sendLongDoubleCode(varCode);
                }// </editor-fold>
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Configure Debugger Var">
            case RX_CODE_CONFIG_DEBUG_VAR:
                if (size != RX_SIZE_CONFIG_DEBUG_VAR)
                    return;
                uint8_t row = RxDMABuffer[iArg1];
                uint8_t id = RxDMABuffer[iArg2];
                uint8_t type = RxDMABuffer[iArg3];
                uint8_t on = RxDMABuffer[iArg4];
                uint32_t period = ((uint32_t)RxDMABuffer[iArg5] << 24) + ((uint32_t)RxDMABuffer[iArg6] << 16) + ((uint32_t)RxDMABuffer[iArg7] << 8) + RxDMABuffer[iArg8];
                uint8_t nb = RxDMABuffer[iArg9];
                uint8_t onTimeStamp = RxDMABuffer[iArg10];
                
                debugVarTab[row].id = id;
                debugVarTab[row].type = type;
                debugVarTab[row].on = on;
                debugVarTab[row].period = period;
                debugVarTab[row].nb = nb;
                debugVarTab[row].onTimeStamp = onTimeStamp;
                debugVarTab[row].time = millis();
                
                break; // </editor-fold>
                
            default:
                break;
        }
    }
}

void send(uint8_t *str,uint16_t size){
    if(verbose == 0)
        return;
    IEC1bits.U2TXIE = 0;    //disable Tx interrupt
    uint16_t i = 0;
    //uint8_t saveTxOn2 = TxOn2;  //TxOn2 could change during an interrupt
    if(TxOn2 == 1)
        push2(str[0]);
    for(i = 1; i < size; i++){
        push2(str[i]);
    }
    if(TxOn2 == 0){
        TxOn2 = 1;
        U2TXREG = str[0];
    }
    IEC1bits.U2TXIE = 1;    //reenable TX interrupt
}
void sendLog(char *str){
    if(verbose == 0)
        return;
    int i;
    int len = strlen(str);
    uint8_t header[2];
    header[0] = 2 + len;
    header[1] = TX_CODE_LOG;
    uint8_t checksum = header[0] + header[1];
    for(i = 0; i < len; i++){
        checksum += str[i];
    }   
    send(header,2);
    send((uint8_t*)str,len);
    send(&checksum,1);
}

void sendVar8(uint8_t varCode, uint8_t var){
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_8B + 1];
    buffer[0] = TX_SIZE_VAR_8B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = var;
    buffer[4] = 0;
    for(i = 0; i < TX_SIZE_VAR_8B; i++){
        buffer[4] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_8B + 1);
}
void sendVar16(uint8_t varCode, uint16_t var){
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_16B + 1];
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = (uint8_t)(var >> 8);
    buffer[4] = (uint8_t)(var & 0xFF);
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_16B + 1);
}
void sendVar32(uint8_t varCode, uint32_t var){
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_32B + 1];
    buffer[0] = TX_SIZE_VAR_32B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = (uint8_t)(var >> 24);
    buffer[4] = (uint8_t)(var >> 16);
    buffer[5] = (uint8_t)(var >> 8);
    buffer[6] = (uint8_t)(var & 0xFF);
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_32B; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_32B + 1);
}
void sendDouble(uint8_t varCode, double *ptrVar){
    uint8_t *ptrChar = (uint8_t*)ptrVar;
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_DOUBLE + 1];
    buffer[0] = TX_SIZE_VAR_DOUBLE;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = ptrChar[0];
    buffer[4] = ptrChar[1];
    buffer[5] = ptrChar[2];
    buffer[6] = ptrChar[3];
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_DOUBLE; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_DOUBLE + 1);
}


void sendVar8Code(uint8_t varCode){
    uint8_t *ptr = (uint8_t*)globalVar[varCode];
    uint8_t var = *ptr;
    
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_8B + 1];
    buffer[0] = TX_SIZE_VAR_8B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = var;
    buffer[4] = 0;
    for(i = 0; i < TX_SIZE_VAR_8B; i++){
        buffer[4] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_8B + 1);
}
void sendVar16Code(uint8_t varCode){
    uint16_t *ptr = (uint16_t*)globalVar[varCode];
    uint16_t var = *ptr;
    
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_16B + 1];
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = (uint8_t)(var >> 8);
    buffer[4] = (uint8_t)(var & 0xFF);
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_16B + 1);
}
void sendVar32Code(uint8_t varCode){
    uint32_t *ptr = (uint32_t*)globalVar[varCode];
    uint32_t var = *ptr;
    
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_32B + 1];
    buffer[0] = TX_SIZE_VAR_32B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = (uint8_t)(var >> 24);
    buffer[4] = (uint8_t)(var >> 16);
    buffer[5] = (uint8_t)(var >> 8);
    buffer[6] = (uint8_t)(var & 0xFF);
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_32B; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_32B + 1);
}
/*void sendDouble(uint8_t varCode, double *ptrVar){
    uint8_t *ptrChar = (uint8_t*)ptrVar;
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_DOUBLE + 1];
    buffer[0] = TX_SIZE_VAR_DOUBLE;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = ptrChar[0];
    buffer[4] = ptrChar[1];
    buffer[5] = ptrChar[2];
    buffer[6] = ptrChar[3];
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_DOUBLE; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_DOUBLE + 1);
}*/
void sendLongDoubleCode(uint8_t varCode){
    long double *ptrVar = (long double*)globalVar[varCode];
    uint8_t *ptrChar = (uint8_t*)ptrVar;
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_LONG_DOUBLE + 1];
    buffer[0] = TX_SIZE_VAR_LONG_DOUBLE;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = ptrChar[0];
    buffer[4] = ptrChar[1];
    buffer[5] = ptrChar[2];
    buffer[6] = ptrChar[3];
    buffer[7] = ptrChar[4];
    buffer[8] = ptrChar[5];
    buffer[9] = ptrChar[6];
    buffer[10] = ptrChar[7];
    buffer[11] = 0;
    for(i = 0; i < TX_SIZE_VAR_LONG_DOUBLE; i++){
        buffer[11] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_LONG_DOUBLE + 1);
}
