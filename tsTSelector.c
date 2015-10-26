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

int8 read_rot_sw(void) {
	int8 value=0;

	/* turn on port b pull up resistors on rotary switch pins */
	port_b_pullups(0b00111100);
	delay_ms(1);

	if ( 0 == input(ROT_SW_1) )
		value+=1;
	if ( 0 == input(ROT_SW_2) )
		value+=2;
	if ( 0 == input(ROT_SW_4) )
		value+=4;
	if ( 0 == input(ROT_SW_8) )
		value+=8;

	/* shut off all pullups to save power */.
	port_b_pullups(0b00000000);

}

void init() {
	int8 i;

	setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN10);
	setup_adc(ADC_CLOCK_DIV_16 | ADC_TAD_MUL_20 );

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


	/* one periodic interrupt @ 100uS. Generated from internal 16 MHz clock */
	/* prescale=16, match=24, postscale=1. Match is 24 because when match occurs, one cycle is lost */
	// setup_timer_2(T2_DIV_BY_16,24,1); 

	/* one periodic interrupt @ 100uS. Generated from system 12 MHz clock */
	/* prescale=4, match=74, postscale=1. Match is 74 because when match occurs, one cycle is lost */
	setup_timer_2(T2_DIV_BY_4,74,1);

	enable_interrupts(INT_TIMER2);
//	enable_interrupts(INT_RDA2); /* debug cable */
	/* RDA - PI is turned on in modbus_slave_pcwx's init */
}


void periodic_millisecond(void) {
	static int8 uptimeticks=0;
	static int16 adcTicks=0;
	static int16 ticks=0;
	/* button debouncing */
//	static int16 b0_state=0; /* bridge push button */
//	static int16 b1_state=0; /* reset line from PI */
	static int16 b2_state=0; /* watchdog line from PI */
	/* power control */
	static int16 adcValue; /* updates after each ADC sample run */

	timers.now_millisecond=0;

//	fputc('.',DEBUG);

#if 0
	/* button must be down for 12 milliseconds */
	b0_state=(b0_state<<1) | !bit_test(timers.port_b,BUTTON_BIT) | 0xe000;
	if ( b0_state==0xf000) {
		/* button pressed */
		current.bridged_uarts = !current.bridged_uarts;
		fprintf(DEBUG,"# bridged=%u\r\n",current.bridged_uarts);
	}

	/* if we are in bridged uarts ... only check for button press */
	if ( current.bridged_uarts ) {
		return;
	}
#endif

#if 0
	/* reset must be down for 12 milliseconds */
	b1_state=(b1_state<<1) | !bit_test(timers.port_c,PIC_BOOTLOAD_REQUEST_BIT) | 0xe000;
	if ( b1_state==0xf000) {
		/* reset line asserted */
		if ( config.allow_bootload_request ) {
			reset_cpu();
		}
		/* BUG - I think that bootload request should be high for x milliseconds, rather than low */
	}
#endif

	/* watchdog must be down for 12 milliseconds for hit to register */
	b2_state=(b2_state<<1) | !bit_test(timers.port_c,WATCHDOG_FROM_PI_BIT) | 0xe000;
	if ( b2_state==0xf000) {
		/* watchdog hit */
//		current.watchdog_seconds=0;
	}

	/* anemometers quit moving */
	if ( 0xffff == timers.pulse_period[0] )
				current.pulse_period[0]=0;
	if ( 0xffff == timers.pulse_period[1] )
				current.pulse_period[1]=0;
	if ( 0xffff == timers.pulse_period[2] )
				current.pulse_period[2]=0;


	/* read port_b and c pin states */
	timers.port_b=port_b;
	timers.port_c=port_c;

	/* green LED control */
	if ( current.bridged_uarts ) {
		/* always on when ports are bridged */
		output_high(LED_GREEN);
	} else {
		/* green LED in Modbus mode */
		if ( 0==timers.led_on_green ) {
			output_low(LED_GREEN);
		} else {
			output_high(LED_GREEN);
			timers.led_on_green--;
		}
	}




	/* some other random stuff that we don't need to do every cycle in main */
	if ( current.interval_milliseconds < 65535 ) {
		current.interval_milliseconds++;
	}

	/* seconds */
	ticks++;
	if ( 1000 == ticks ) {
		ticks=0;

		/* watchdog power control of pi */
		if ( current.watchdog_seconds != 65535 ) {
			current.watchdog_seconds++;
		}

		/* shut off when:
			a) watchdog_seconds_max != 0 AND watchdog_seconds is greater than watchdog_seconds_max AND it isn't already off 
		*/
		if ( 0 != config.watchdog_seconds_max && current.watchdog_seconds > config.watchdog_seconds_max && 0 == timers.load_off_seconds ) {
			timers.load_off_seconds=config.pi_offtime_seconds;
		}

		/* control power to the raspberrry pi load */
		if ( 0==timers.load_off_seconds ) {
			output_high(PI_POWER_EN);
		} else {
			output_low(PI_POWER_EN);
			timers.load_off_seconds--;

			if ( 0 == timers.load_off_seconds ) {
				/* reset watchdog seconds so we can turn back on */
				current.watchdog_seconds=0;
			}
		}

		
		/* uptime counter */
		uptimeTicks++;
		if ( 60 == uptimeTicks ) {
			uptimeTicks=0;
			if ( current.uptime_minutes < 65535 ) 
				current.uptime_minutes++;
		}
	}


	if ( 65535 == adcValue ) {
		/* signaled that a new ADC sample was taken and we need to run again */
		/* read current ADC value */	
		adcValue=adc_get(0);
	}

#if 0
	if ( adcValue > config.power_on_above_adc ) {
		if ( current.power_on_delay > 0 ) {
			current.power_on_delay--;
		} else {
			current.p_on=1;
		}
	} else {
		current.power_on_delay=config.power_on_above_delay;
	}
			

	if ( adcValue < config.power_off_below_adc ) {
		if ( current.power_off_delay > 0 ) {
			current.power_off_delay--;
		} else {
			current.p_on=0;
		}
	} else {
		current.power_off_delay=config.power_off_below_delay;
	}
#endif	

	/* ADC sample counter */
	if ( timers.now_adc_reset_count ) {
		timers.now_adc_reset_count=0;
		adcTicks=0;
	}

	/* ADC sampling trigger */
	adcTicks++;
	if ( adcTicks == config.adc_sample_ticks ) {
		adcTicks=0;
		timers.now_adc_sample=1;
		adcValue=65535; /* signal power control (above) on next pass to resample */
	}



}


void main(void) {
	int8 i;

	i=restart_cause();

	init();

	/* read rotary switch ... if pos = 0 then come up full speed, stay awake, and be modbus slave */

	/* turn on  and init */

	/* read input voltage and set latching contactor if different than EEPROM value */

	/* if contactor off, sleep */

	/* if contactor on, check temperatures to determine if any exceed set point */
	/* read set point for rotary switch as -20C + 5*ROTARY SWITCH VALUE */


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

	fprintf(DEBUG,"# read_param_file() starting ...");
	read_param_file();
	fprintf(DEBUG," complete\r\n");


	if ( config.modbus_address != 255 && config.modbus_address > 127 ) {
		fprintf(DEBUG,"# write_default_param_file() starting ...");
		write_default_param_file();
		fprintf(DEBUG," complete\r\n");
	}

	/* start Modbus slave */
	setup_uart(TRUE);
	/* modbus_init turns on global interrupts */
	fprintf(DEBUG,"# modbus_init() starting ...");
	modbus_init();
	fprintf(DEBUG," complete\r\n");

	fprintf(DEBUG,"# bridged_uarts=%u\r\n",current.bridged_uarts);

	/* Prime ADC filter */
	for ( i=0 ; i<30 ; i++ ) {
		adc_update();
	}

	/* set power switch to initial state */
	current.p_on=config.power_startup;


	/* shut off RS-485 transmit once transmit buffer is empty */
	while ( ! TRMT2 )
		;
	output_low(RS485_DE);
	output_low(RS485_NRE);
	/* done with RS-485 port startup message */


	for ( ; ; ) {
		restart_wdt();

#if 0
		if ( current.bridged_uarts ) {
			disable_interrupts(INT_TIMER2);
			if ( kbhit(DEBUG) ) {
				fputc(fgetc(DEBUG),MODBUS_SERIAL);
			}

			if ( !bit_test(timers.port_b,BUTTON_BIT) ) {
				current.bridged_uarts=0;
				enable_interrupts(INT_TIMER2);
			}

			continue;
		} 
#endif

		if ( timers.now_millisecond ) {
			periodic_millisecond();
		}


		if ( timers.now_adc_sample ) {
			timers.now_adc_sample=0;
			adc_update();
		}

//		if ( ! current.bridged_uarts ) {
			modbus_process();
//		}

	}
}

void init(void) {
	
	
}


void main(void) {
	
	
	
}