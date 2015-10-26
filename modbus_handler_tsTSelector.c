#define MAX_STATUS_REGISTER  51

#define MIN_CONFIG_REGISTER  1000
#define MAX_CONFIG_REGISTER  1011

#define MIN_EE_REGISTER      2000
#define MAX_EE_REGISTER      MIN_EE_REGISTER + 512


/* This function may come in handy for you since MODBUS uses MSB first. */
int8 swap_bits(int8 c) {
	return ((c&1)?128:0)|((c&2)?64:0)|((c&4)?32:0)|((c&8)?16:0)|((c&16)?8:0)|((c&32)?4:0)|((c&64)?2:0)|((c&128)?1:0);
}

void reset_modbus_stats(void) {
	current.modbus_our_packets=0;
	current.modbus_other_packets=0;
	current.modbus_last_error=0;
}

int16 map_modbus(int16 addr) {
	if ( addr >= MIN_EE_REGISTER && addr < MAX_EE_REGISTER ) {
		return (int16) read_eeprom(addr - MIN_EE_REGISTER);
	}

	switch ( addr ) {
		/* counters */

		/* analog channels */
		/* input voltage */
		case 18: return (int16) current.adc_buffer[0][current.adc_buffer_index];
		case 19: return (int16) adc_get(0);
		/* wind dir 0 */
		case 21: return (int16) current.adc_buffer[1][current.adc_buffer_index];
		case 22: return (int16) adc_get(1);
		/* wind dir 1 */
		case 24: return (int16) current.adc_buffer[2][current.adc_buffer_index];
		case 25: return (int16) adc_get(2);
		/* temperature */
		case 27: return (int16) current.adc_buffer[3][current.adc_buffer_index];
		case 28: return (int16) adc_get(3);
		/* user ADC 0 to 3 */
		case 30: return (int16) current.adc_buffer[4][current.adc_buffer_index];
		case 31: return (int16) adc_get(4);

		case 33: return (int16) current.adc_buffer[5][current.adc_buffer_index];
		case 34: return (int16) adc_get(5);


		/* status */
		case 42: return (int16) current.sequence_number++;

		/* modbus statistics */
		case 48: return (int16) current.modbus_our_packets;
		case 49: return (int16) current.modbus_other_packets;
		case 50: return (int16) current.modbus_last_error;
		/* triggers a modbus statistics reset */
		case 51: reset_modbus_stats(); return (int16) 0;

		/* configuration */
//		case 1000: return (int16) input(BUTTON);
//		case 1000: return (int16) modbus_rx.len;
		case 1000: return (int16) config.serial_prefix;
		case 1001: return (int16) config.serial_number;
		case 1002: return (int16) 'P';
		case 1003: return (int16) 'W';
		case 1004: return (int16) 'X';
		case 1005: return (int16) 1;
		case 1006: return (int16) config.modbus_address;
		case 1007: return (int16) config.adc_sample_ticks;

		/* we should have range checked, and never gotten here */
		default: return (int16) 65535;
	}

}


int8 modbus_valid_read_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;

	
	if ( start >= MIN_CONFIG_REGISTER && end <= MAX_CONFIG_REGISTER+1 )
		return 1;

	if ( start >= MIN_EE_REGISTER && end <= MAX_EE_REGISTER+1 )
		return 1;
	

	/* end is always start + at least one ... so no need to test for range starting at 0 */
	if ( end <= MAX_STATUS_REGISTER+1)
		return 1;

	return 0;
}

int8 modbus_valid_write_registers(int16 start, int16 end) {
	if ( 19999==start && 20000==end)
		return 1;

	if ( start >= MIN_EE_REGISTER && end <= MAX_EE_REGISTER+1 )
		return 1;

	if ( start >= MIN_CONFIG_REGISTER && end <= MAX_CONFIG_REGISTER+1 )
		return 1;
	
	/* end is always start + at least one ... so no need to test for range starting at 0 */
	if ( end <= MAX_STATUS_REGISTER+1)
		return 1;

	return 0;
}

void modbus_read_register_response(function func, int8 address, int16 start_address, int16 register_count ) {
	int16 i;
	int16 l;

	modbus_serial_send_start(address, func); // FUNC_READ_HOLDING_REGISTERS);
	modbus_serial_putc(register_count*2);


	for( i=0 ; i<register_count ; i++ ) {
		l=map_modbus(start_address+i);
		modbus_serial_putc(make8(l,1));
  		modbus_serial_putc(make8(l,0));
	}

	modbus_serial_send_stop();
}

/* 
try to write the specified register
if successful, return 0, otherwise return a modbus exception
*/
exception modbus_write_register(int16 address, int16 value) {

	if ( address >= MIN_EE_REGISTER && address < MAX_EE_REGISTER ) {
		if ( value > 256 ) return ILLEGAL_DATA_VALUE;
		write_eeprom(address,(int8) value);
		return 0;
	}

	/* if we have been unlocked, then we can modify serial number */
	if ( current.factory_unlocked ) {
		if ( 1000 == address ) {
			config.serial_prefix=value;
			return 0;
		} else if ( 1001 == address ) {
			config.serial_number=value;
			return 0;
		}
	}

	/* publicly writeable addresses */
	switch ( address ) {
			

		case 1006:
			/* Modbus address {0 to 127} */
			if ( value != 255 && value > 127 ) return ILLEGAL_DATA_VALUE;
			config.modbus_address=value;
			break;

		
		case 1998:
			/* write default config to EEPROM */
			if ( 1 != value ) return ILLEGAL_DATA_VALUE;
			write_default_param_file();
			break;
		case 1999:
			/* write config to EEPROM */
			if ( 1 != value ) return ILLEGAL_DATA_VALUE;
			write_param_file();
			break;
		case 19999:
			/* unlock factory programming registers when we get 1802 in passcode register */
			if ( 1802 != value ) {
				current.factory_unlocked=0;
				return ILLEGAL_DATA_VALUE;
			}
			current.factory_unlocked=1;
			/* green LED for 2 seconds */
			timers.led_on_c=200;
			timers.led_on_d=200;
			break;
		default:
			return ILLEGAL_DATA_ADDRESS;

	}

	/* must not have triggered an exception */
	return 0;
}


void modbus_process(void) {
	int16 start_addr;
	int16 num_registers;
	exception result;
	int8 i;


	/* check for message */
	if ( modbus_kbhit() ) {
//		output_high(TP_RED);

		if ( 255==config.modbus_address || modbus_rx.address==config.modbus_address ) {
			/* Modbus statistics */
			if ( current.modbus_our_packets < 65535 )
				current.modbus_our_packets++;
	
			/* green LED for 200 milliseconds */
			timers.led_on_c=20;

			switch(modbus_rx.func) {
				case FUNC_READ_HOLDING_REGISTERS: /* 3 */
				case FUNC_READ_INPUT_REGISTERS:   /* 4 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);
					num_registers=make16(modbus_rx.data[2],modbus_rx.data[3]);
	
					/* make sure our address is within range */
					if ( ! modbus_valid_read_registers(start_addr,start_addr+num_registers) ) {
					    modbus_exception_rsp(modbus_rx.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
						current.modbus_last_error=ILLEGAL_DATA_ADDRESS;

						/* red LED for 1 second */
						timers.led_on_d=100;
					} else {
						modbus_read_register_response(modbus_rx.func,modbus_rx.address,start_addr,num_registers);
					}
					break;
				case FUNC_WRITE_SINGLE_REGISTER: /* 6 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);

					/* try the write */
					result=modbus_write_register(start_addr,make16(modbus_rx.data[2],modbus_rx.data[3]));

					if ( result ) {
						/* exception */
						modbus_exception_rsp(modbus_rx.address,modbus_rx.func,result);
						current.modbus_last_error=result;

						/* red LED for 1 second */
						timers.led_on_d=100;
					}  else {
						/* no exception, send ack */
						modbus_write_single_register_rsp(modbus_rx.address,
							start_addr,
							make16(modbus_rx.data[2],modbus_rx.data[3])
						);
					}
					break;
				case FUNC_WRITE_MULTIPLE_REGISTERS: /* 16 */
					start_addr=make16(modbus_rx.data[0],modbus_rx.data[1]);
					num_registers=make16(modbus_rx.data[2],modbus_rx.data[3]);

					/* attempt to write each register. Stop if exception */
					for ( i=0 ; i<num_registers ; i++ ) {
						result=modbus_write_register(start_addr+i,make16(modbus_rx.data[5+i*2],modbus_rx.data[6+i*2]));

						if ( result ) {
							/* exception */
							modbus_exception_rsp(modbus_rx.address,modbus_rx.func,result);
							current.modbus_last_error=result;
	
							/* red LED for 1 second */
							timers.led_on_d=100;
			
							break;
						}
					}
		
					/* we could have gotten here with an exception already send, so only send if no exception */
					if ( 0 == result ) {
						/* no exception, send ack */
						modbus_write_multiple_registers_rsp(modbus_rx.address,start_addr,num_registers);
					}

					break;  
				default:
					/* we don't support most operations, so return ILLEGAL_FUNCTION exception */
					modbus_exception_rsp(modbus_rx.address,modbus_rx.func,ILLEGAL_FUNCTION);
					current.modbus_last_error=ILLEGAL_FUNCTION;

					/* red led for 1 second */
					timers.led_on_d=100;
			}
		} else {
			/* MODBUS packet for somebody else */
			if ( current.modbus_other_packets < 65535 )
				current.modbus_other_packets++;

			/* yellow LED 200 milliseconds */
			timers.led_on_c=20;
			timers.led_on_d=20;
		}

	}
//	output_low(TP_RED);
}