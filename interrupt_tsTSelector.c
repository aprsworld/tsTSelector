/*  network connected serial port*/
#int_rda2
void isr_rda() {
	int8 c;

	c=fgetc(MODBUS_SERIAL);


	/* Modbus */
	if (!modbus_serial_new) {
		if(modbus_serial_state == MODBUS_GETADDY) {
			modbus_serial_crc.d = 0xFFFF;
			modbus_rx.address = c;
			modbus_serial_state++;
			modbus_rx.len = 0;
			modbus_rx.error=0;
		} else if(modbus_serial_state == MODBUS_GETFUNC) {
			modbus_rx.func = c;
			modbus_serial_state++;
		} else if(modbus_serial_state == MODBUS_GETDATA) {
			if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE) {
				modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;
			}
			modbus_rx.data[modbus_rx.len]=c;
			modbus_rx.len++;
		}

		modbus_calc_crc(c);
		modbus_enable_timeout(TRUE);
	}
}