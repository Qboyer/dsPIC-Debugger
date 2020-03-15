#ifndef CONSTANT_H
#define	CONSTANT_H

// <editor-fold defaultstate="collapsed" desc="Clock">
#define CLOCK_M     35
#define CLOCK_N1    2
#define CLOCK_N2    2
#define CLOCK_SOURCE_FREQ_HZ   16000000

//#define CLOCK_FREQ  CLOCK_SOURCE_FREQ * CLOCK_M / (CLOCK_N1 * CLOCK_N2)
#define CLOCK_FREQ_HZ   140000000
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART">
#define BAUDRATE 9600
#define BRGVAL 455  //((CLOCK_FREQ_HZ/2/BAUDRATE)/16) - 1   9600
#define BRGVAL2 34//7 //34//6//34//37   //  500 000
//#define BRGVAL2 37//7 //34//6//34//37   //  500 000
#define UART2_HIGH_SPEED 1  //coef *4 : BRGH

#define TX_SIZE 1000     //size of Tx buffer
#define RX_SIZE 100     //size of Rx buffer
#define RX_DMA_SIZE 1000 //


#define RX_CODE_SET                     3
#define RX_CODE_GET                     4

#define RX_CODE_RESET                   66

#define RX_CODE_GET_BIS                 104

#define RX_CODE_CONFIG_DEBUG_VAR    14

#define RX_SIZE_SET               // var,type,value
#define RX_SIZE_GET             3 // var
#define RX_SIZE_GET_BIS         4 // type, var


#define RX_SIZE_RESET           2

#define RX_SIZE_CONFIG_DEBUG_VAR 12

#define TX_CODE_VAR             1
#define TX_CODE_LOG             2

#define TX_SIZE_VAR      //ça dépend
#define TX_SIZE_VAR_8B  4
#define TX_SIZE_VAR_16B 5
#define TX_SIZE_VAR_32B 7
#define TX_SIZE_VAR_DOUBLE 7
#define TX_SIZE_VAR_LONG_DOUBLE 11


#define VAR_8b      0
#define VAR_16b     1
#define VAR_32b     2
#define VAR_64b     3
#define VAR_LD_64b  4


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I/O">

#define LED_PLATINE     LATFbits.LATF7

// </editor-fold>



#endif	/* CONSTANT_H */
