#include "tsTSelector.h"

typedef struct {
	int8 modbus_address;
	int8 modbus_mode;

	int8 serial_prefix;
	int16 serial_number;

	int16 adc_sample_ticks;

	int16 t_setpoints[16];
	int16 v_contactor_on_above;
	int16 v_contactor_off_below;
} struct_config;



typedef struct {
	/* most recent valid */
	/* circular buffer for ADC readings */
	int16 adc_buffer[6][16];
	int8  adc_buffer_index;

	int16 modbus_our_packets;
	int16 modbus_other_packets;
	int16 modbus_last_error;

	int16 sequence_number;

	int8 factory_unlocked;

	int8 restart_cause;
	int8 rotary_switch_value;
} struct_current;

typedef struct {
	int8 led_on_a;
	int8 led_on_b;
	int8 led_on_c;
	int8 led_on_d;
} struct_time_keep;


/* global structures */
struct_config config;
struct_current current;
struct_time_keep timers;

#include "adc_tsTSelector.c"
#include "param_tsTSelector.c"

#include "modbus_slave_tsTSelector.c"
#include "modbus_handler_tsTSelector.c"

#include "interrupt_tsTSelector.c"

int8 read_rotary_switch(void) {
	int8 value=0;

	/* turn on port b pull up resistors on rotary switch pins */
	port_b_pullups(0b00111100);
	delay_ms(1);


	if ( 0 == input(ROTARY_SW_1) )
		value+=1;
	if ( 0 == input(ROTARY_SW_2) )
		value+=2;
	if ( 0 == input(ROTARY_SW_4) )
		value+=4;
	if ( 0 == input(ROTARY_SW_8) )
		value+=8;

	/* shut off all pullups to save power */
//	port_b_pullups(0b00000000);

	return value;
}

void contactor_off(void) {
	/* check to see if we need to set */
	if ( 0 == read_eeprom(EE_CONTACTOR_STATE) ) 
		return;

	/* energize coil to open contactor for 100 milliseconds */
	output_high(BRIDGE_A);
	output_low(BRIDGE_B);
	delay_ms(100);

	/* de-energize coil */
	output_high(BRIDGE_A);
	output_high(BRIDGE_B);

	/* update our EEPROM register with the contactor now being off */
	write_eeprom(EE_CONTACTOR_STATE,0);
}

void contactor_on(void) {
	/* always set on. If we are turning on, then we are going to be dumping power,
	so we can spend a little making sure the dump loads are on */
	/* energize coil to open contactor for 100 milliseconds */

	output_low(BRIDGE_A);
	output_high(BRIDGE_B);
	delay_ms(100);

	/* de-energize coil */
	output_high(BRIDGE_A);
	output_high(BRIDGE_B);


	if ( 1 != read_eeprom(EE_CONTACTOR_STATE) ) {
		/* update our EEPROM register with the contactor now being on, since it wasn't already */
		write_eeprom(EE_CONTACTOR_STATE,1);
	}
}


void init() {
	setup_oscillator(OSC_4MHZ);	
	setup_wdt(WDT_ON); /* 32 second watchdog interval */

	setup_adc(ADC_CLOCK_INTERNAL);
	setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN10, VSS_VDD);
	ADFM=1; /* right justify ADC results */

	setup_dac(DAC_OFF);
	setup_vref(VREF_OFF);
	setup_spi(SPI_DISABLED);


#if 0
	set_tris_a(0b00101111);
	set_tris_b(0b10111110);
	set_tris_c(0b10000000);
#endif

	/* data structure initialization */
	timers.led_on_a=0;
	timers.led_on_b=0;
	timers.led_on_c=0;
	timers.led_on_d=0;

	current.modbus_our_packets=0;
	current.modbus_other_packets=0;
	current.modbus_last_error=0;
	current.sequence_number=0;
	current.adc_buffer_index=0;
	current.factory_unlocked=0;
}


void main(void) {
	int8 i;
	int16 adc;

	/* record restart cause before it gets lost */
	current.restart_cause=restart_cause();

	/* setup hardware */
	init();

	/* turn on blue LED (D6) */
	output_high(LED_A);

	/* read parameter file */
	read_param_file();

	/* read input voltage */
	adc=read_adc_average16(4);

	if ( adc > config.v_contactor_on_above ) {
		contactor_on();
	
		/* now check temperatures and enable / disable tristars */

	}  else if ( adc < config.v_contactor_off_below ) {
		contactor_off();
	}
	

	/* read rotary switch */
	current.rotary_switch_value=read_rotary_switch();

#if 1
	/* turn on RS-232 port */
	output_low(RS232_RX_NEN);
	output_high(RS232_TX_EN);
	delay_ms(10);
	fprintf(STREAM_TRISTAR,"# tsTSelector.c %s\r\n",__DATE__);
#endif




	for ( i=0 ; i<18 ; i++ ) {
		adc_update();

		fprintf(STREAM_TRISTAR,"# i=%u sw=%u ",i,read_rotary_switch());
		fprintf(STREAM_TRISTAR,"\r\n");
#if 1
		fprintf(STREAM_TRISTAR,"(T3)=%04lu ",adc_get(0));
		fprintf(STREAM_TRISTAR,"(T2)=%04lu ",adc_get(1));
		fprintf(STREAM_TRISTAR,"(T1)=%04lu ",adc_get(2));
		fprintf(STREAM_TRISTAR,"(T0)=%04lu ",adc_get(3));
		fprintf(STREAM_TRISTAR,"(IN)=%04lu ",adc_get(4));
		fprintf(STREAM_TRISTAR,"(T4)=%04lu ",adc_get(5));
		fprintf(STREAM_TRISTAR,"<- filtered \r\n");

		set_adc_channel(0); delay_us(10);
		fprintf(STREAM_TRISTAR,"(T3)=%04lu ",read_adc());
		set_adc_channel(1); delay_us(10);
		fprintf(STREAM_TRISTAR,"(T2)=%04lu ",read_adc());
		set_adc_channel(2); delay_us(10);
		fprintf(STREAM_TRISTAR,"(T1)=%04lu ",read_adc());
		set_adc_channel(3); delay_us(10);
		fprintf(STREAM_TRISTAR,"(T0)=%04lu ",read_adc());
		set_adc_channel(4); delay_us(10);
		fprintf(STREAM_TRISTAR,"(IN)=%04lu ",read_adc());
		set_adc_channel(10); delay_us(10);
		fprintf(STREAM_TRISTAR,"(T4)=%04lu ",read_adc());
#endif
		fprintf(STREAM_TRISTAR,"<- un-filtered\r\n");

		output_bit(LED_A,input(ROTARY_SW_1));
		output_bit(LED_B,input(ROTARY_SW_2));
		output_bit(LED_C,input(ROTARY_SW_4));
		output_bit(LED_D,input(ROTARY_SW_8));

		delay_ms(100);


		restart_wdt();
	}

	fprintf(STREAM_TRISTAR,"# restart_cause()=%u ",current.restart_cause);
	switch ( current.restart_cause ) {
		case WDT_TIMEOUT: fprintf(STREAM_TRISTAR,"WDT_TIMEOUT"); break;
		case MCLR_FROM_SLEEP: fprintf(STREAM_TRISTAR,"MCLR_FROM_SLEEP"); break;
		case MCLR_FROM_RUN: fprintf(STREAM_TRISTAR,"MCLR_FROM_RUN"); break;
		case NORMAL_POWER_UP: fprintf(STREAM_TRISTAR,"NORMAL_POWER_UP"); break;
		case BROWNOUT_RESTART: fprintf(STREAM_TRISTAR,"BROWNOUT_RESTART"); break;
		case WDT_FROM_SLEEP: fprintf(STREAM_TRISTAR,"WDT_FROM_SLEEP"); break;
		case RESET_INSTRUCTION: fprintf(STREAM_TRISTAR,"RESET_INSTRUCTION"); break;
		default: fprintf(STREAM_TRISTAR,"unknown!");
	}
	fprintf(STREAM_TRISTAR,"\r\n");

	/* turn off RS-232 port */
	output_high(RS232_RX_NEN);
	output_low(RS232_TX_EN);

	sleep();



	/* read rotary switch ... if pos = 0 then come up full speed, stay awake, and be modbus slave */

	/* turn on  and init */

	/* read input voltage and set latching contactor if different than EEPROM value */

	/* if contactor off, sleep */

	/* if contactor on, check temperatures to determine if any exceed set point */
	/* read set point for rotary switch as -20C + 5*ROTARY SWITCH VALUE */


#if 0
	/* debugging messages sent on RS-485 port ... so we will start transmitting */
	output_high(RS485_DE);

	fprintf(DEBUG,"# tsTSelector %s\r\n",__DATE__);
	fprintf(DEBUG,"# restart_cause()=%u ",i);
	switch ( i ) {
		case WDT_TIMEOUT: fprintf(DEBUG,"WDT_TIMEOUT"); break;
		case MCLR_FROM_SLEEP: fprintf(DEBUG,"MCLR_FROM_SLEEP"); break;
		case MCLR_FROM_RUN: fprintf(DEBUG,"MCLR_FROM_RUN"); break;
		case NORMAL_POWER_UP: fprintf(DEBUG,"NORMAL_POWER_UP"); break;
		case BROWNOUT_RESTART: fprintf(DEBUG,"BROWNOUT_RESTART"); break;
		case WDT_FROM_SLEEP: fprintf(DEBUG,"WDT_FROM_SLEEP"); break;
		case RESET_INSTRUCTION: fprintf(DEBUG,"RESET_INSTRUCTION"); break;
		default: fprintf(DEBUG,"unknown!");
	}
	fprintf(DEBUG,"\r\n");
#endif

//	fprintf(DEBUG,"# read_param_file() starting ...");
	read_param_file();
//	fprintf(DEBUG," complete\r\n");


	if ( config.modbus_address != 255 && config.modbus_address > 127 ) {
//		fprintf(DEBUG,"# write_default_param_file() starting ...");
		write_default_param_file();
//		fprintf(DEBUG," complete\r\n");
	}

	/* start Modbus slave */
	setup_uart(TRUE);
	/* modbus_init turns on global interrupts */
//	fprintf(DEBUG,"# modbus_init() starting ...");
	modbus_init();
//	fprintf(DEBUG," complete\r\n");

	/* Prime ADC filter */
	for ( i=0 ; i<30 ; i++ ) {
		adc_update();
	}


	/* shut off RS-485 transmit once transmit buffer is empty */
//	while ( ! TRMT2 )
//		;
//	output_low(RS485_DE);
	/* done with RS-485 port startup message */


	for ( ; ; ) {
		restart_wdt();

		modbus_process();
	}
}
