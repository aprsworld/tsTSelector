

//////////////////////////////////////////////////////////////////////////////////////////
////                                      modbus.c                                    ////
////                                                                                  ////
////                 MODBUS protocol driver for serial communications.                ////
////                                                                                  ////
////  Refer to documentation at http://www.modbus.org for more information on MODBUS. ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////
////                                                                                  ////
//// DEFINES:                                                                         ////
////                                                                                  ////
////  MODBUS_TYPE                   MODBUS_TYPE_MASTER or MODBUS_TYPE_SLAVE           ////
////  MODBUS_SERIAL_INT_SOURCE      Source of interrupts                              ////
////                                (MODBUS_INT_EXT,MODBUS_INT_RDA,MODBUS_INT_RDA2)   ////
////  MODBUS_SERIAL_BAUD            Valid baud rate for serial                        ////
////  MODBUS_SERIAL_RX_PIN          Valid pin for serial receive                      ////
////  MODBUS_SERIAL_TX_PIN          Valid pin for serial transmit                     ////
////  MODBUS_SERIAL_ENABLE_PIN      Valid pin for serial enable, rs485 only           ////
////  MODBUS_SERIAL_RX_ENABLE       Valid pin for serial rcv enable, rs485 only       ////
////  MODBUS_SERAIL_RX_BUFFER_SIZE  Size of the receive buffer                        ////
////                                                                                  ////
////                                                                                  ////
//// SHARED API:                                                                      ////
////                                                                                  ////
////  modbus_init()                                                                   ////
////    - Initialize modbus serial communication system                               ////
////                                                                                  ////
////  modbus_serial_send_start(address,func)                                          ////
////    - Setup serial line to begin sending.  Once this is called, you can send data ////
////      using modbus_serial_putc().  Should only be used for custom commands.       ////
////                                                                                  ////
////  modbus_serial_send_stop()                                                       ////
////    - Must be called to finalize the send when modbus_serial_send_start is used.  ////
////                                                                                  ////
////  modbus_kbhit()                                                                  ////
////    - Used to check if a packet has been received.                                ////
////                                                                                  ////
//// MASTER API:                                                                      ////
////  All master API functions return 0 on success.                                   ////
////                                                                                  ////
////  exception modbus_read_coils(address,start_address,quantity)                     ////
////    - Wrapper for function 0x01(read coils) in the MODBUS specification.          ////
////                                                                                  ////
////  exception modbus_read_discrete_input(address,start_address,quantity)            ////
////    - Wrapper for function 0x02(read discret input) in the MODBUS specification.  ////
////                                                                                  ////
////  exception modbus_read_holding_registers(address,start_address,quantity)         ////
////    - Wrapper for function 0x03(read holding regs) in the MODBUS specification.   ////
////                                                                                  ////
////  exception modbus_read_input_registers(address,start_address,quantity)           ////
////    - Wrapper for function 0x04(read input regs) in the MODBUS specification.     ////
////                                                                                  ////
////  exception modbus_write_single_coil(address,output_address,on)                   ////
////    - Wrapper for function 0x05(write single coil) in the MODBUS specification.   ////
////                                                                                  ////
////  exception modbus_write_single_register(address,reg_address,reg_value)           ////
////    - Wrapper for function 0x06(write single reg) in the MODBUS specification.    ////
////                                                                                  ////
////  exception modbus_read_exception_status(address)                                 ////
////    - Wrapper for function 0x07(read void status) in the MODBUS specification.    ////
////                                                                                  ////
////  exception modbus_diagnostics(address,sub_func,data)                             ////
////    - Wrapper for function 0x08(diagnostics) in the MODBUS specification.         ////
////                                                                                  ////
////  exception modbus_get_comm_event_counter(address)                                ////
////    - Wrapper for function 0x0B(get comm event count) in the MODBUS specification.////
////                                                                                  ////
////  exception modbus_get_comm_event_log(address)                                    ////
////    - Wrapper for function 0x0C(get comm event log) in the MODBUS specification.  ////
////                                                                                  ////
////  exception modbus_write_multiple_coils(address,start_address,quantity,*values)   ////
////    - Wrapper for function 0x0F(write multiple coils) in the MODBUS specification.////
////    - Special Note: values is a pointer to an int8 array, each byte represents 8  ////
////                    coils.                                                        ////
////                                                                                  ////
////  exception modbus_write_multiple_registers(address,start_address,quantity,*values)///
////    - Wrapper for function 0x10(write multiple regs) in the MODBUS specification. ////
////    - Special Note: values is a pointer to an int8 array                          ////
////                                                                                  ////
////  exception modbus_report_slave_id(address)                                       ////
////    - Wrapper for function 0x11(report slave id) in the MODBUS specification.     ////
////                                                                                  ////
////  exception modbus_read_file_record(address,byte_count,*request)                  ////
////    - Wrapper for function 0x14(read file record) in the MODBUS specification.    ////
////                                                                                  ////
////  exception modbus_write_file_record(address,byte_count,*request)                 ////
////    - Wrapper for function 0x15(write file record) in the MODBUS specification.   ////
////                                                                                  ////
////  exception modbus_mask_write_register(address,reference_address,AND_mask,OR_mask)////
////    - Wrapper for function 0x16(read coils) in the MODBUS specification.          ////
////                                                                                  ////
////  exception modbus_read_write_multiple_registers(address,read_start,read_quantity,////
////                            write_start,write_quantity, *write_registers_value)   ////
////    - Wrapper for function 0x17(read write mult regs) in the MODBUS specification.////
////                                                                                  ////
////  exception modbus_read_FIFO_queue(address,FIFO_address)                          ////
////    - Wrapper for function 0x18(read FIFO queue) in the MODBUS specification.     ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////
////                (C) Copyright 1996, 2006 Custom Computer Services                 ////
////        This source code may only be used by licensed users of the CCS            ////
////        C compiler.  This source code may only be distributed to other            ////
////        licensed users of the CCS C compiler.  No other use,                      ////
////        reproduction or distribution is permitted without written                 ////
////        permission.  Derivative programs created using this software              ////
////        in object code form are not restricted in any way.                        ////
//////////////////////////////////////////////////////////////////////////////////////////



#define RCV_OFF() {disable_interrupts(INT_RDA);}


#define MODBUS_SERIAL_RX_BUFFER_SIZE  255      //size of send/rcv buffer

void wait_for_hw_buffer() {
  while(!TRMT);
}

int1 modbus_serial_new=0;

/********************************************************************
These exceptions are defined in the MODBUS protocol.  These can be
used by the slave to communicate problems with the transmission back
to the master who can also use these to easily check the exceptions.  
The first exception is the only one that is not part of the protocol 
specification.  The TIMEOUT exception is returned when no slave 
responds to the master's request within the timeout period.
********************************************************************/
typedef enum _exception{ILLEGAL_FUNCTION=1,ILLEGAL_DATA_ADDRESS=2, 
ILLEGAL_DATA_VALUE=3,SLAVE_DEVICE_FAILURE=4,ACKNOWLEDGE=5,SLAVE_DEVICE_BUSY=6, 
MEMORY_PARITY_ERROR=8,GATEWAY_PATH_UNAVAILABLE=10,GATEWAY_TARGET_NO_RESPONSE=11,
TIMEOUT=12} exception;

/********************************************************************
These functions are defined in the MODBUS protocol.  These can be
used by the slave to check the incomming function.  See 
ex_modbus_slave.c for example usage.
********************************************************************/
typedef enum _function{FUNC_READ_COILS=0x01,FUNC_READ_DISCRETE_INPUT=0x02,
FUNC_READ_HOLDING_REGISTERS=0x03,FUNC_READ_INPUT_REGISTERS=0x04,
FUNC_WRITE_SINGLE_COIL=0x05,FUNC_WRITE_SINGLE_REGISTER=0x06,
FUNC_READ_EXCEPTION_STATUS=0x07,FUNC_DIAGNOSTICS=0x08,
FUNC_GET_COMM_EVENT_COUNTER=0x0B,FUNC_GET_COMM_EVENT_LOG=0x0C,
FUNC_WRITE_MULTIPLE_COILS=0x0F,FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
FUNC_REPORT_SLAVE_ID=0x11,FUNC_READ_FILE_RECORD=0x14,
FUNC_WRITE_FILE_RECORD=0x15,FUNC_MASK_WRITE_REGISTER=0x16,
FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,FUNC_READ_FIFO_QUEUE=0x18} function;
    
/*Stages of MODBUS reception.  Used to keep our ISR fast enough.*/
enum {MODBUS_GETADDY=0, MODBUS_GETFUNC=1, MODBUS_GETDATA=2} modbus_serial_state = 0;

/*Global value holding our current CRC value.*/
union
{
   int8 b[2];
   int16 d;
} modbus_serial_crc;

/********************************************************************
Our receive struct.  This is used when receiving data as a master or
slave.  Once a message is sent to you with your address, you should
begin processing that message.  Refer to ex_modbus_slave.c to see 
how to properly use this structure.
********************************************************************/
struct
{
   int8 address;
   int8 len;                                //number of bytes in the message received
   function func;                           //the function of the message received
   exception error;                         //error recieved, if any
   int8 data[MODBUS_SERIAL_RX_BUFFER_SIZE]; //data of the message received
} modbus_rx;

/* Table of CRC values for high–order byte */
const unsigned char modbus_auchCRCHi[] = {
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40
};

/* Table of CRC values for low–order byte */
const char modbus_auchCRCLo[] = {
0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
0x40
};

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None
void RCV_ON(void) {
	// Clear RX buffer. Clear RDA interrupt flag. Clear overrun error flag.
	while ( kbhit(MODBUS_SERIAL) ) {
		fprintf(DEBUG,"# reading starting ...");
		fgetc(MODBUS_SERIAL);
		fprintf(DEBUG," complete");
	}

	clear_interrupt(INT_RDA);
	enable_interrupts(INT_RDA);
}

// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
void modbus_init() {

//	fprintf(DEBUG,"# rcv_on() starting ...\r\n");
	RCV_ON();
//	fprintf(DEBUG,"# rcv_on() complete\r\n");

//	setup_timer_2(T2_DIV_BY_16,249,5);  //~4ms interrupts
//	setup_timer_0(RTCC_INTERNAL | RTCC_DIV_32 | RTCC_8_BIT); /* ~1.024 ms @ 8 MHz ... 0.686 ms @ 12 MHz */

	fprintf(DEBUG,"# setup_timer_0 starting ...\r\n");
	setup_timer_0(T0_INTERNAL | T0_DIV_128 | T0_8_BIT); /* ~4.096 ms @ 8 MHz ... 2.73 ms @ 12 MHz */
	fprintf(DEBUG,"# setup_timer_0 complete\r\n");

	fprintf(DEBUG,"# enable_interrupts(GLOBAL) starting ...\r\n");
	enable_interrupts(GLOBAL);
	fprintf(DEBUG,"# enable interrupts complete\r\n");
}

// Purpose:    Start our timeout timer
// Inputs:     Enable, used to turn timer on/off
// Outputs:    None
void modbus_enable_timeout(int1 enable) {
	disable_interrupts(INT_TIMER0);
	if (enable) {
		set_timer0(0);
		clear_interrupt(INT_TIMER0);
		enable_interrupts(INT_TIMER0);
	}
}

// Purpose:    Check if we have timed out waiting for a response
// Inputs:     None
// Outputs:    None
#int_timer0
void modbus_timeout_now(void)
{
   if((modbus_serial_state == MODBUS_GETDATA) && (modbus_serial_crc.d == 0x0000) && (!modbus_serial_new))
   {
      modbus_rx.len-=2;
      modbus_serial_new=TRUE;
   }
   else
      modbus_serial_new=FALSE;

   modbus_serial_crc.d=0xFFFF;
   modbus_serial_state=MODBUS_GETADDY;
   modbus_enable_timeout(FALSE);
}

// Purpose:    Calculate crc of data and updates global crc
// Inputs:     Character
// Outputs:    None
void modbus_calc_crc(char data)
{
  unsigned int8 uIndex ; // will index into CRC lookup table

  uIndex = (modbus_serial_crc.b[1]) ^ data; // calculate the CRC
  modbus_serial_crc.b[1] = (modbus_serial_crc.b[0]) ^ modbus_auchCRCHi[uIndex];
  modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];
}

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None
void modbus_serial_putc(int8 c) {
	fputc(c, MODBUS_SERIAL);
	modbus_calc_crc(c);

	/* one stop bit delay */
	delay_us(9);

	//delay_us(1000000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact
}


// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum
void modbus_serial_send_start(int8 to, int8 func)
{
   modbus_serial_crc.d=0xFFFF;
   modbus_serial_new=FALSE;

   RCV_OFF();
   
//	output_high(MODBUS_SERIAL_RX_ENABLE); // JJJ
 //  output_high(MODBUS_SERIAL_ENABLE_PIN);

	/* 3.5 character delay (3500000/baud) */
	delay_us(31); /* 115200 */

   modbus_serial_putc(to);
   modbus_serial_putc(func);
}

void modbus_serial_send_stop()
{
   int8 crc_low, crc_high;

   crc_high=modbus_serial_crc.b[1];
   crc_low=modbus_serial_crc.b[0];

   modbus_serial_putc(crc_high);
   modbus_serial_putc(crc_low);

   WAIT_FOR_HW_BUFFER();
    
	/* 3.5 character delay (3500000/baud) */
	delay_us(31); /* 115200 */


   RCV_ON();

//   output_low(MODBUS_SERIAL_ENABLE_PIN);
//	output_low(MODBUS_SERIAL_RX_ENABLE); // JJJ

   modbus_serial_crc.d=0xFFFF;
}

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     None
// Outputs:    TRUE if a message was received
//             FALSE if no message is available
// Note:       Data will be filled in at the modbus_rx struct:
int1 modbus_kbhit()
{
   if(!modbus_serial_new)
      return FALSE;
   else if(modbus_rx.func & 0x80)           //did we receive an error?
   {
      modbus_rx.error = modbus_rx.data[0];  //if so grab the error and return true
      modbus_rx.len = 1;
   }
   modbus_serial_new=FALSE;
   return TRUE;
}

/*MODBUS Slave Functions*/

/********************************************************************
The following structs are used for read/write_sub_request_rsp.  These
functions take in one of these structs.  Please refer to the MODBUS
protocol specification if you do not understand the members of the
structure.
********************************************************************/
typedef struct _modbus_read_sub_request_rsp
{
   int8 record_length;
   int8 reference_type;
   int16 data[((MODBUS_SERIAL_RX_BUFFER_SIZE)/2)-3];
} modbus_read_sub_request_rsp;

typedef struct _modbus_write_sub_request_rsp
{
   int8 reference_type;
   int16 file_number;
   int16 record_number;
   int16 record_length;
   int16 data[((MODBUS_SERIAL_RX_BUFFER_SIZE)/2)-8];
} modbus_write_sub_request_rsp;


/********************************************************************
The following slave functions are defined in the MODBUS protocol.
Please refer to http://www.modbus.org for the purpose of each of
these.  All functions take the slaves address as their first
parameter.
********************************************************************/

/*
read_coils_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      coil_data          Pointer to an array of data to send
Output:    void
*/
void modbus_read_coils_rsp(int8 address, int8 byte_count, int8* coil_data)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_COILS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*coil_data);
      coil_data++;
   }

   modbus_serial_send_stop();
}

/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_discrete_input_rsp(int8 address, int8 byte_count, 
                                    int8 *input_data)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*input_data);
      input_data++;
   }

   modbus_serial_send_stop();
}

/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
*/
void modbus_read_holding_registers_rsp(int8 address, int8 byte_count, 
                                        int8 *reg_data)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*reg_data);
      reg_data++;
   }

   modbus_serial_send_stop();
}

/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_input_registers_rsp(int8 address, int8 byte_count, 
                                        int8 *input_data)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*input_data);
      input_data++;
   }

   modbus_serial_send_stop();
}

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
*/
void modbus_write_single_coil_rsp(int8 address, int16 output_address, 
                                    int16 output_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

   modbus_serial_putc(make8(output_address,1));
   modbus_serial_putc(make8(output_address,0));

   modbus_serial_putc(make8(output_value,1));
   modbus_serial_putc(make8(output_value,0));

   modbus_serial_send_stop();
}

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
*/
void modbus_write_single_register_rsp(int8 address, int16 reg_address, 
                                        int16 reg_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);

   modbus_serial_putc(make8(reg_address,1));
   modbus_serial_putc(make8(reg_address,0));

   modbus_serial_putc(make8(reg_value,1));
   modbus_serial_putc(make8(reg_value,0));

   modbus_serial_send_stop();
}

/*
read_exception_status_rsp
Input:     int8       address            Slave Address
Output:    void
*/
void modbus_read_exception_status_rsp(int8 address, int8 data)
{
   modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
   modbus_serial_send_stop();
}

/*
diagnostics_rsp
Input:     int8       address            Slave Address
           int16      sub_func           Echo of sub function received
           int16      data               Echo of data received
Output:    void
*/
void modbus_diagnostics_rsp(int8 address, int16 sub_func, int16 data)
{
   modbus_serial_send_start(address, FUNC_DIAGNOSTICS);

   modbus_serial_putc(make8(sub_func,1));
   modbus_serial_putc(make8(sub_func,0));

   modbus_serial_putc(make8(data,1));
   modbus_serial_putc(make8(data,0));

   modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
Output:    void
*/
void modbus_get_comm_event_counter_rsp(int8 address, int16 status, 
                                        int16 event_count)
{
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_COUNTER);

   modbus_serial_putc(make8(status, 1));
   modbus_serial_putc(make8(status, 0));

   modbus_serial_putc(make8(event_count, 1));
   modbus_serial_putc(make8(event_count, 0));

   modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
           int16      message_count      Count of messages
           int8*      events             Pointer to event data
           int8       events_len         Length of event data in bytes
Output:    void
*/
void modbus_get_comm_event_log_rsp(int8 address, int16 status,
                                    int16 event_count, int16 message_count, 
                                    int8 *events, int8 events_len)
{
   int8 i;
    
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_LOG);

   modbus_serial_putc(events_len+6);

   modbus_serial_putc(make8(status, 1));
   modbus_serial_putc(make8(status, 0));

   modbus_serial_putc(make8(event_count, 1));
   modbus_serial_putc(make8(event_count, 0));

   modbus_serial_putc(make8(message_count, 1));
   modbus_serial_putc(make8(message_count, 0));

   for(i=0; i < events_len; ++i)
   {
      modbus_serial_putc(*events);
      events++;
   }

   modbus_serial_send_stop();
}

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to
Output:    void
*/
void modbus_write_multiple_coils_rsp(int8 address, int16 start_address, 
                                        int16 quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
}

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
*/
void modbus_write_multiple_registers_rsp(int8 address, int16 start_address, 
                                            int16 quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
}

/*
report_slave_id_rsp
Input:     int8       address            Slave Address
           int8       slave_id           Slave Address
           int8       run_status         Are we running?
           int8*      data               Pointer to an array holding the data
           int8       data_len           Length of data in bytes
Output:    void
*/
void modbus_report_slave_id_rsp(int8 address, int8 slave_id, int1 run_status,
                              int8 *data, int8 data_len)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_REPORT_SLAVE_ID);

   modbus_serial_putc(data_len+2);
   modbus_serial_putc(slave_id);

   if(run_status)
    modbus_serial_putc(0xFF);
   else
    modbus_serial_putc(0x00);

   for(i=0; i < data_len; ++i)
   {
      modbus_serial_putc(*data);
      data++;
   }

   modbus_serial_send_stop();
}

/*
read_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Number of bytes to send
           read_sub_request_rsp*    request            Structure holding record/data information
Output:    void
*/
void modbus_read_file_record_rsp(int8 address, int8 byte_count, 
                                    modbus_read_sub_request_rsp *request)
{
   int8 i=0,j;

   modbus_serial_send_start(address, FUNC_READ_FILE_RECORD);

   modbus_serial_putc(byte_count);

   while(i < byte_count);
   {
      modbus_serial_putc(request->record_length);
      modbus_serial_putc(request->reference_type);

      for(j=0; (j < request->record_length); j+=2)
      {
         modbus_serial_putc(make8(request->data[j], 1));
         modbus_serial_putc(make8(request->data[j], 0));
      }

      i += (request->record_length)+1;
      request++;
   }

   modbus_serial_send_stop();
}

/*
write_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Echo of number of bytes sent
           write_sub_request_rsp*   request            Echo of Structure holding record information
Output:    void
*/
void modbus_write_file_record_rsp(int8 address, int8 byte_count, 
                                    modbus_write_sub_request_rsp *request)
{
   int8 i, j=0;

   modbus_serial_send_start(address, FUNC_WRITE_FILE_RECORD);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=(7+(j*2)))
   {
      modbus_serial_putc(request->reference_type);
      modbus_serial_putc(make8(request->file_number, 1));
      modbus_serial_putc(make8(request->file_number, 0));
      modbus_serial_putc(make8(request->record_number, 1));
      modbus_serial_putc(make8(request->record_number, 0));
      modbus_serial_putc(make8(request->record_length, 1));
      modbus_serial_putc(make8(request->record_length, 0));

      for(j=0; (j < request->record_length); j+=2)
      {
         modbus_serial_putc(make8(request->data[j], 1));
         modbus_serial_putc(make8(request->data[j], 0));
      }
   }

   modbus_serial_send_stop();
}

/*
mask_write_register_rsp
Input:     int8        address            Slave Address
           int16       reference_address  Echo of reference address
           int16       AND_mask           Echo of AND mask
           int16       OR_mask            Echo or OR mask
Output:    void
*/
void modbus_mask_write_register_rsp(int8 address, int16 reference_address,
                           int16 AND_mask, int16 OR_mask)
{
   modbus_serial_send_start(address, FUNC_MASK_WRITE_REGISTER);

   modbus_serial_putc(make8(reference_address,1));
   modbus_serial_putc(make8(reference_address,0));

   modbus_serial_putc(make8(AND_mask,1));
   modbus_serial_putc(make8(AND_mask,0));

   modbus_serial_putc(make8(OR_mask, 1));
   modbus_serial_putc(make8(OR_mask, 0));

   modbus_serial_send_stop();
}

/*
read_write_multiple_registers_rsp
Input:     int8        address            Slave Address
           int16*      data               Pointer to an array of data
           int8        data_len           Length of data in bytes
Output:    void
*/
void modbus_read_write_multiple_registers_rsp(int8 address, int8 data_len, 
                                                int16 *data)
{
   int8 i;

   modbus_serial_send_start(address, FUNC_READ_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(data_len*2);

   for(i=0; i < data_len*2; i+=2)
   {
      modbus_serial_putc(make8(data[i], 1));
      modbus_serial_putc(make8(data[i], 0));
   }

   modbus_serial_send_stop();
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       FIFO_len           Length of FIFO in bytes
           int16*      data               Pointer to an array of data
Output:    void
*/
void modbus_read_FIFO_queue_rsp(int8 address, int16 FIFO_len, int16 *data)
{
   int8 i;
   int16 byte_count;

   byte_count = ((FIFO_len*2)+2);

   modbus_serial_send_start(address, FUNC_READ_FIFO_QUEUE);

   modbus_serial_putc(make8(byte_count, 1));
   modbus_serial_putc(make8(byte_count, 0));

   modbus_serial_putc(make8(FIFO_len, 1));
   modbus_serial_putc(make8(FIFO_len, 0));

   for(i=0; i < FIFO_len; i+=2)
   {
      modbus_serial_putc(make8(data[i], 1));
      modbus_serial_putc(make8(data[i], 0));
   }

   modbus_serial_send_stop();
}

void modbus_exception_rsp(int8 address, int16 func, exception error)
{
   modbus_serial_send_start(address, func|0x80);
   modbus_serial_putc(error);
   modbus_serial_send_stop();
}

