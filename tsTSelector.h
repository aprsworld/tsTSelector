#include <18F26K22.h>
#device ADC=10
#device *=16

#include <stdlib.h>
#FUSES INTRC_IO
#FUSES NOPROTECT
#FUSES PUT
#FUSES NOLVP
#FUSES BROWNOUT
#FUSES BORV29
#FUSES NOMCLR
#FUSES WDT
//#FUSES WDT512 // 512 = 10 seconds
#FUSES WDT1024 // 1024 = 18 seconds
//#FUSES WDT2048 // 2048 = 32 seconds
#FUSES NOPLLEN
#FUSES NOFCMEN
#FUSES NOIESO
#FUSES NOXINST
#FUSES NODEBUG
#use delay(clock=4000000, restart_wdt)

/* 
Parameters are stored in EEPROM
*/
#define PARAM_CRC_ADDRESS  0x002
#define PARAM_ADDRESS      PARAM_CRC_ADDRESS+2


/* UART1 - connection to two TriStar charge controllers */
#use rs232(UART1,stream=STREAM_TRISTAR,baud=9600,xmit=PIN_C6,rcv=PIN_C7,errors)	


#byte TXSTA=GETENV("SFR:txsta1")
#bit  TRMT=TXSTA.1

#byte TXSTA2=GETENV("SFR:txsta2")
#bit  TRMT2=TXSTA2.1

#byte PORTB=GETENV("SFR:portb")
#byte INTCON2=GETENV("SFR:intcon2")
#bit RBPU=INTCON2.7

/* UART2 - RS-485 MODBUS network */
#use rs232(UART2, stream=MODBUS_SERIAL, baud=9600,errors)	




#use standard_io(A)
#use standard_io(B)
#use standard_io(C)
#use standard_io(E)

#define CYCLES_BEFORE_CONTACTOR_RESET 100

#define RS485_DE                 PIN_A4
#define RS232_TX_EN              PIN_A7
#define RS232_RX_NEN             PIN_A6
#define LED_A                    PIN_C0
#define LED_B                    PIN_C1
#define BRIDGE_B                 PIN_C2
#define BRIDGE_A                 PIN_C3
#define LED_C                    PIN_C4
#define LED_D                    PIN_C5
#define SER_TO_TS                PIN_C6
#define SER_FROM_TS              PIN_C7

#define ROTARY_SW_1              PIN_B2
#define ROTARY_SW_2              PIN_B3
#define ROTARY_SW_4              PIN_B4
#define ROTARY_SW_8              PIN_B5

#define SER_TO_NET               PIN_B6
#define SER_FROM_NET             PIN_B7


/* analog inputs ADC channels*/
#define AN_CH_T4	10	// RB1 / AN10 (on board NTC)
#define AN_CH_T3	0	// RA0 / AN0
#define AN_CH_T2	1	// RA1 / AN1
#define AN_CH_T1	2	// RA2 / AN2
#define AN_CH_T0	3	// RA3 / AN3
#define AN_IN_VOLTS	4	// RA5 / AN4  (on board 1:6 divider)


typedef union {
	int16 l[2];
    int8 b[4];
    int32 word;
} u_lblock;

#byte port_b=GETENV("SFR:portb")
#byte port_c=GETENV("SFR:portc")
#BIT ADFM=GETENV("BIT:ADFM") 
