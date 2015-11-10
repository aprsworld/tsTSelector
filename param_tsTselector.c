#inline
char xor_crc(char oldcrc, char data) {
	return oldcrc ^ data;
}

char EEPROMDataRead( int16 address, int8 *data, int16 count ) {
	char crc=0;

	while ( count-- != 0 ) {
		*data = read_eeprom( address++ );
		crc = xor_crc(crc,*data);
		data++;
	}
	return crc;
}

char EEPROMDataWrite( int16 address, int8 *data, int16 count ) {
	char crc=0;

	while ( count-- != 0 ) {
		/* restart_wdt() */
		crc = xor_crc(crc,*data);
		write_eeprom( address++, *data++ );
	}

	return crc;
}

void write_param_file() {
	int8 crc;

	/* write the config structure */
	crc = EEPROMDataWrite(PARAM_ADDRESS,(void *)&config,sizeof(config));
	/* write the CRC was calculated on the structure */
	write_eeprom(PARAM_CRC_ADDRESS,crc);
}

void set_config(void) {
	config.modbus_address=41;


	config.serial_prefix='A';
	config.serial_number=9876;

	config.adc_sample_ticks=20;


	config.t_setpoints[0]=905;  // -15C
	config.t_setpoints[1]=871;  // -10C
	config.t_setpoints[2]=832;  // -5C
	config.t_setpoints[3]=786;  // 0C
	config.t_setpoints[4]=736;  // 5C
	config.t_setpoints[5]=683;  // 10C
	config.t_setpoints[6]=626;  // 15C
	config.t_setpoints[7]=569;  // 20C
	config.t_setpoints[8]=512;  // 25C
	config.t_setpoints[9]=457;  // 30C
	config.t_setpoints[10]=405; // 35C
	config.t_setpoints[11]=357; // 40C
	config.t_setpoints[12]=313; // 45C
	config.t_setpoints[13]=273; // 50C
	config.t_setpoints[14]=238; // 55C
	config.t_setpoints[15]=512; // user defined - default to 25C

	config.v_contactor_on_above=395;  // 13.50 volts
	config.v_contactor_off_below=380; // 13.00 volts

}

void write_default_param_file() {
	/* all LEDs for 1.5 seconds */
#if 0
	timers.led_on_a=150;
	timers.led_on_b=150;
	timers.led_on_c=150;
	timers.led_on_d=150;
#else
	output_high(LED_A);
	output_high(LED_B);
	output_high(LED_C);
	output_high(LED_D);
	delay_ms(1500);
	output_low(LED_A);
	output_low(LED_B);
	output_low(LED_C);
	output_low(LED_D);
#endif

	set_config();

	/* write them so next time we use from EEPROM */
	write_param_file();

}


void read_param_file() {
	int8 crc;

	crc = EEPROMDataRead(PARAM_ADDRESS, (void *)&config, sizeof(config)); 
		
	if ( crc != read_eeprom(PARAM_CRC_ADDRESS) ) {
		write_default_param_file();
	}
}


