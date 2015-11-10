int16 read_adc_average16(int8 ch) {
	int16 sum;
	int8 i;


	// Calculate the mean.  This is done by summing up the
	// values and dividing by the number of elements.
	sum = 0;

	set_adc_channel(ch);
	delay_us(10);

	for( i = 0; i < 16 ; i++ ) {
		sum += read_adc();
	}

	/* divide sum by our 16 samples and round by adding 8 */
	return ( (sum+8) >> 4 );
}

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
		if ( i <= 4 ) {
			set_adc_channel(i);
		}	else {
			set_adc_channel(10);
		}
		delay_us(10);
		
		current.adc_buffer[i][current.adc_buffer_index] = read_adc();
	}
}