// const int8 adcChannelMap[8]={AN_IN_VOLTS, AN_TEMPERATURE, AN_WIND_DIR_0, AN_WIND_DIR_1, AN_USER_USER_0, AN_USER_USER_1, AN_USER_USER_2, AN_USER_USER_3};

int16 adc_get(int8 ch) {
	int16 sum;
	int8 i;

	/* pre-compute address of channel adc buffer. Saves computing it 16 times in the loop below */
	int16 *p;
	p = current.adc_buffer[ch];

	// Calculate the mean.  This is done by summing up the
	// values and dividing by the number of elements.
	sum = 0;
	for( i = 0; i < 16 ; i++ ) {
//		sum += current.adc_buffer[ch][i];
		sum += p[i];
	}

	/* divide sum by our 16 samples and round by adding 8 */
	return ( (sum+8) >> 4 );
}


void adc_update(void) {
	int8 i;

	/* wrap buffer around */
	current.adc_buffer_index++;
	if ( current.adc_buffer_index >= 16 )
		current.adc_buffer_index=0;


	for ( i=0 ; i<6 ; i++ ) {
		set_adc_channel(i);
		/* delay needed? */
		current.adc_buffer[i][current.adc_buffer_index] = read_adc();
	}
}