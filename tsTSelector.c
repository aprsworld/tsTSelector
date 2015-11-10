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
	int16 t_adc[5];
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

void modbus_tristar_disable(int8 ch) {
	output_low(RS232_RX_NEN);
	output_high(RS232_TX_EN);
	delay_ms(10);

	/* disable output
	01,05,00,01,FF,00,DD,FA (slave address=1)
	02,05,00,01,FF,00,DD,C9 (slave address=2)
	*/
	if ( 2 == ch ) {
		fputc(0x02,STREAM_TRISTAR);
	} else { 
		fputc(0x01,STREAM_TRISTAR);
	}
	fputc(0x05,STREAM_TRISTAR);
	fputc(0x00,STREAM_TRISTAR);
	fputc(0x01,STREAM_TRISTAR);
	fputc(0xFF,STREAM_TRISTAR);
	fputc(0x00,STREAM_TRISTAR);

	/* CRC */
	fputc(0xDD,STREAM_TRISTAR);
	if ( 2 == ch ) {
		fputc(0xC9,STREAM_TRISTAR);
	} else {
		fputc(0xFA,STREAM_TRISTAR);
	}

	delay_ms(10);
	/* turn off RS-232 port */
	output_high(RS232_RX_NEN);
	output_low(RS232_TX_EN);
}

void modbus_tristar_enable(int8 ch) {
	output_low(RS232_RX_NEN);
	output_high(RS232_TX_EN);
	delay_ms(10);

	/* enable output
	01,05,00,01,00,00,9C,0A (slave address=1)
	02,05,00,01,00,00,9C,39 (slave address=2)
	*/

	if ( 2 == ch ) {
		fputc(0x02,STREAM_TRISTAR);
	} else {
		fputc(0x01,STREAM_TRISTAR);
	}

	fputc(0x05,STREAM_TRISTAR);
	fputc(0x00,STREAM_TRISTAR);
	fputc(0x01,STREAM_TRISTAR);
	fputc(0x00,STREAM_TRISTAR);
	fputc(0x00,STREAM_TRISTAR);
	
	/* CRC */
	fputc(0x9C,STREAM_TRISTAR);
	if ( 2 == ch ) {
		fputc(0x39,STREAM_TRISTAR);
	} else {
		fputc(0x0A,STREAM_TRISTAR);
	}

	delay_ms(10);
	/* turn off RS-232 port */
	output_high(RS232_RX_NEN);
	output_low(RS232_TX_EN);
}


void dump_to_internal(void) {
	output_high(LED_B); /* left / green LED */

	/* enable internal dump load tristar (modbus address=1) */
	modbus_tristar_enable(1);
	/* disable external dump load tristar (modbus address=2) */
	modbus_tristar_disable(2);

	output_low(LED_B);
	
//	fprintf(STREAM_TRISTAR,"# dumping to internal\r\n");
}

void dump_to_external(void) {
	output_high(LED_D); /* right / red LED */

	/* enable external dump load tristar (modbus address=2) */
	modbus_tristar_enable(2);
	/* disable internal dump load tristar (modbus address=1) */
	modbus_tristar_disable(1);

	output_low(LED_D);

//	fprintf(STREAM_TRISTAR,"# dumping to external\r\n");
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
	int16 adc;
	int16 tSet;

	/* record restart cause before it gets lost */
	current.restart_cause=restart_cause();

	/* setup hardware */
	init();

	/* turn on blue LED (D6) */
	output_high(LED_A);


	/* hard coded settings */
	set_config();


#if 0
	/* turn on RS-232 port */
	output_low(RS232_RX_NEN);
	output_high(RS232_TX_EN);
	delay_ms(10);
	fprintf(STREAM_TRISTAR,"# tsTSelector.c %s\r\n",__DATE__);
	fprintf(STREAM_TRISTAR,"# config.v_contactor_on_above=%lu\r\n",config.v_contactor_on_above);
	fprintf(STREAM_TRISTAR,"# config.v_contactor_off_below=%lu\r\n",config.v_contactor_off_below);
#endif

	/* read input voltage */
	adc=read_adc_average16(4);
//	fprintf(STREAM_TRISTAR,"# adc=%lu\r\n",adc);

	if ( adc > config.v_contactor_on_above ) {
		contactor_on();
	
		/* now check temperatures and enable / disable tristars */

		/* read rotary switch and lookup temperature set point */
		current.rotary_switch_value=read_rotary_switch();
		tSet=config.t_setpoints[current.rotary_switch_value];
//		fprintf(STREAM_TRISTAR,"# rotarySwitchValue=%u tSet=%lu\r\n",current.rotary_switch_value,tSet);

		/* read temperatures */
		current.t_adc[0]=read_adc_average16(3);
		current.t_adc[1]=read_adc_average16(2);
		current.t_adc[2]=read_adc_average16(1);
		current.t_adc[3]=read_adc_average16(0);
		current.t_adc[4]=read_adc_average16(10);

//		fprintf(STREAM_TRISTAR,"# [0]=%lu [1]=%lu [2]=%lu [3]=%lu [4]=%lu\r\n",current.t_adc[0],current.t_adc[1],current.t_adc[2],current.t_adc[3],current.t_adc[4]);

		if ( current.t_adc[0]<tSet || current.t_adc[1]<tSet || current.t_adc[2]<tSet || current.t_adc[3]<tSet || current.t_adc[4]<tSet ) {
			dump_to_external();
		} else {
			dump_to_internal();
		}

	}  else if ( adc < config.v_contactor_off_below ) {
		contactor_off();
	}

#if  0
	delay_ms(10);
	/* turn off RS-232 port */
	output_high(RS232_RX_NEN);
	output_low(RS232_TX_EN);
#endif

	output_low(LED_A);
	sleep();

}
