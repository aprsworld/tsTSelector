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
	int8 contactor_state; // 0=off, 1=on, all else undefined and will cause the contactor to be set on next pass
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

// #include "modbus_slave_tsTSelector.c"
// #include "modbus_handler_tsTSelector.c"

// #include "interrupt_tsTSelector.c"


 // #define RS232_DEBUG 1


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
#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR,"# contactor_off current.contactor_state=%u\r\n",current.contactor_state);
#endif

	/* check to see if we need to set */
	if ( 0 == current.contactor_state ) 
		return;

#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR,"---> contactor_off() energizing coil ...");
#endif
	/* energize coil to open contactor for 100 milliseconds */
	output_high(BRIDGE_A);
	output_low(BRIDGE_B);
	output_high(LED_C);
	delay_ms(100);

	/* de-energize coil */
	output_low(BRIDGE_A);
	output_low(BRIDGE_B);
	output_low(LED_C);
#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR," done\r\n");
#endif


	current.contactor_state=0;
}

void contactor_on(void) {
#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR,"# contactor_on current.contactor_state=%u\r\n",current.contactor_state);
#endif

	/* always set on. If we are turning on, then we are going to be dumping power,
	so we can spend a little making sure the dump loads are on */
	/* energize coil to close contactor for 100 milliseconds */

#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR,"---> contactor_on() energizing coil ...");
#endif
	output_low(BRIDGE_A);
	output_high(BRIDGE_B);
	output_high(LED_C);
	delay_ms(100);

	/* de-energize coil */
	output_low(BRIDGE_A);
	output_low(BRIDGE_B);
	output_low(LED_C);
#ifdef RS232_DEBUG
	fprintf(STREAM_TRISTAR," done\r\n");
#endif

	current.contactor_state=1;
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

#ifdef RS232_DEBUG	
	fprintf(STREAM_TRISTAR,"# dumping to internal\r\n");
#endif
}

void dump_to_external(void) {
	output_high(LED_D); /* right / red LED */

	/* enable external dump load tristar (modbus address=2) */
	modbus_tristar_enable(2);
	/* disable internal dump load tristar (modbus address=1) */
	modbus_tristar_disable(1);

	output_low(LED_D);
	
#ifdef RS232_DEBUG	
	fprintf(STREAM_TRISTAR,"# dumping to external\r\n");
#endif
}


void init() {
	setup_oscillator(OSC_4MHZ);	

	setup_adc(ADC_CLOCK_DIV_8); 
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
	current.contactor_state=2; /* cause it to be set next pass through */
}

void print_restart_cause(int8 val) {
	switch ( val ) {
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
}


void main(void) {
	int16 adc;
	int16 tSet;
	int8 bootRestartCause;

	/* record restart cause before it gets lost */
	bootRestartCause=restart_cause();

	restart_wdt();

	/* setup hardware */
	init();

	/* load  hard coded settings */
	set_config();


#ifdef RS232_DEBUG	
	/* turn on RS-232 port */
	output_low(RS232_RX_NEN);
	output_high(RS232_TX_EN);
	delay_ms(10);
	fprintf(STREAM_TRISTAR,"\r\n\r\n# tsTSelector.c cold boot %s\r\n",__DATE__);

	fprintf(STREAM_TRISTAR,"# boot restart_cause()=%u ",bootRestartCause);
	print_restart_cause(bootRestartCause);
#endif

	for ( ; ; ) {
		/* WDT should wake up from sleep and get here within a few instructions */
		current.restart_cause = restart_cause();

		/* turn on blue LED (D6) */
		output_high(LED_A);

		/* count cycles through */
		current.sequence_number++;


		/* every CYCLES_BEFORE_CONTACTOR_RESET we invalidate contactor state which causes the coil to be energized again */
		if ( CYCLES_BEFORE_CONTACTOR_RESET == current.sequence_number ) {
			current.contactor_state=3;
			current.sequence_number=0;
		}



#ifdef RS232_DEBUG	
		/* turn on RS-232 port */
		output_low(RS232_RX_NEN);
		output_high(RS232_TX_EN);
		delay_ms(10);
		fprintf(STREAM_TRISTAR,"\r\n\r\n# tsTSelector.c warm boot %s\r\n",__DATE__);

		fprintf(STREAM_TRISTAR,"# restart_cause()=%u ",current.restart_cause);
		print_restart_cause(current.restart_cause);

		fprintf(STREAM_TRISTAR,"# current.sequence_number=%lu\r\n",current.sequence_number);

		fprintf(STREAM_TRISTAR,"# config.v_contactor_on_above=%lu\r\n",config.v_contactor_on_above);
		fprintf(STREAM_TRISTAR,"# config.v_contactor_off_below=%lu\r\n",config.v_contactor_off_below);
#endif

		/* setup ADC and read input voltage */
		setup_adc(ADC_CLOCK_DIV_8); 
		setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN10, VSS_VDD);
		ADFM=1; /* right justify ADC results */
		adc=read_adc_average16(4);

#ifdef RS232_DEBUG	
		fprintf(STREAM_TRISTAR,"# input voltage adc=%lu\r\n",adc);
#endif

		if ( adc > config.v_contactor_on_above ) {
			contactor_on();
	
			/* now check temperatures and enable / disable tristars */

			/* read rotary switch and lookup temperature set point */
			current.rotary_switch_value=read_rotary_switch();
			tSet=config.t_setpoints[current.rotary_switch_value];

#ifdef RS232_DEBUG	
			fprintf(STREAM_TRISTAR,"# rotarySwitchValue=%u tSet=%lu\r\n",current.rotary_switch_value,tSet);
#endif

			/* read temperatures */
			current.t_adc[0]=read_adc_average16(3);
			current.t_adc[1]=read_adc_average16(2);
			current.t_adc[2]=read_adc_average16(1);
			current.t_adc[3]=read_adc_average16(0);
			current.t_adc[4]=read_adc_average16(10);

#ifdef RS232_DEBUG	
			fprintf(STREAM_TRISTAR,"# [0]=%lu [1]=%lu [2]=%lu [3]=%lu [4]=%lu\r\n",current.t_adc[0],current.t_adc[1],current.t_adc[2],current.t_adc[3],current.t_adc[4]);
#endif

			if ( current.t_adc[0]<tSet || current.t_adc[1]<tSet || current.t_adc[2]<tSet || current.t_adc[3]<tSet || current.t_adc[4]<tSet ) {
				dump_to_external();
			} else {
				dump_to_internal();
			}
		}  else if ( adc < config.v_contactor_off_below ) {
			contactor_off();
		}

#ifdef RS232_DEBUG	
		delay_ms(10);
		/* turn off RS-232 port */
		output_high(RS232_RX_NEN);
		output_low(RS232_TX_EN);
	#endif

		output_low(LED_A);

		/* shut off ADC before going to sleep to save power */
		setup_adc(ADC_OFF);

		/* sleep restarts the watchdog so we should get a relative consistent wakeup */
		sleep();
		/* instruction is pre-fetched prior to sleep ... so run a NOP as first new instruction */
		delay_cycles(1);
	}
}
