#include "tsTSelector.h"

typedef struct {
	int8 modbus_address;
	int8 modbus_mode;

	int8 serial_prefix;
	int16 serial_number;

	int16 adc_sample_ticks;
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

void init() {
	setup_oscillator(OSC_4MHZ);	
//	setup_wdt(WDT_32); /* 32 second watchdog interval */

	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_20 );
	setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN10);

	setup_dac(DAC_OFF);
	setup_vref(VREF_OFF);
	setup_spi(SPI_DISABLED);


	set_tris_a(0b00101111);
	set_tris_b(0b10111110);
	set_tris_c(0b10000000);


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

void init_min_power(void) {
	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_20 );
	
	init();
}

void init_full_speed(void) {
	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_20 );

	init();
}


void main(void) {
	int8 i;

	current.restart_cause=restart_cause();
	current.rotary_switch_value=read_rotary_switch();


	/* off */
	output_low(BRIDGE_A);
	output_low(BRIDGE_B);
	delay_ms(500);
	restart_wdt();

	/* one direction */
	output_high(BRIDGE_A);
	output_low(BRIDGE_B);
	delay_ms(100);
	restart_wdt();

	/* off */
	output_high(BRIDGE_A);
	output_high(BRIDGE_B);
	delay_ms(500);
	restart_wdt();

	/* other direction */
	output_low(BRIDGE_A);
	output_high(BRIDGE_B);
	delay_ms(100);
	restart_wdt();

	/* off */
	output_low(BRIDGE_A);
	output_low(BRIDGE_B);
	delay_ms(500);
	restart_wdt();

	for ( ; ; ) {
//		current.rotary_switch_value=read_rotary_switch();

		output_bit(LED_A,input(ROTARY_SW_1));
		output_bit(LED_B,input(ROTARY_SW_2));
		output_bit(LED_C,input(ROTARY_SW_4));
		output_bit(LED_D,input(ROTARY_SW_8));

		restart_wdt();
	}

#if 0
	output_high(LED_A);
	delay_ms(500);
	restart_wdt();

	output_high(LED_B);
	delay_ms(500);
	restart_wdt();

	output_high(LED_C);
	delay_ms(500);
	restart_wdt();

	output_high(LED_D);
	delay_ms(500);
	restart_wdt();
#endif


	/* if rotary switch is set to 0, then we come up with RS-485 / Modbus and stay awake */
	if ( 0 == current.rotary_switch_value ) {
		init_full_speed();
	} else {
		init_min_power();
	}

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