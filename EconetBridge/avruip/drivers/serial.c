

#include <avr/io.h>
#include <avr/sleep.h>
#include "serial.h"

void serial_short(unsigned short val)
{

	unsigned short tmp;

	tmp = ((val & 0xFF00) >>8);		// calc high byte
	serial_tx_hex((unsigned char)tmp);	// output high byte in hex

	serial_tx(0x20);				// space

	tmp = (val & 0x00FF);			// calc lo byte
	serial_tx_hex((unsigned char)tmp);	// output high byte in hex

	return;

}

void serial_shortLH(unsigned short val)
{

	unsigned short tmp;

	tmp = (val & 0x00FF);			// calc lo byte
	serial_tx_hex((unsigned char)tmp);	// output high byte in hex

	serial_tx(0x20);				// space

	tmp = ((val & 0xFF00) >>8);		// calc high byte
	serial_tx_hex((unsigned char)tmp);	// output high byte in hex

	return;

}

void serial_ok(unsigned char okVal)
{
	serial_tx_str("ok ");
	serial_tx_hex(okVal);	// value
	serial_crlf(); 		// cr lf

	return;
}

void serial_rx(void)
{
	serial_tx_str("rx ");
}

void serial_txx(void)
{
	serial_tx_str("tx ");
}

void serial_eth(void)
{
	serial_tx_str("eth ");
}

void serial_eco(void)
{
	serial_tx_str("eco ");
}

void serial_error(unsigned char eVal)
{
	serial_tx_str("e ");
	serial_tx_hex(eVal);	// error value
	serial_crlf(); 		// cr lf
}

void serial_crlf(void)
{
	serial_tx_str("\r\n");
}

void serial_tx_str(char *msg)
{
	while (*msg)
		serial_tx (*(msg++));
}

void serial_tx_ip(uint8_t *buf)
{
  serial_tx_hex(buf[3]);
  serial_tx_hex(buf[2]);
  serial_tx_hex(buf[1]);
  serial_tx_hex(buf[0]);
}

void serial_packet(unsigned short pktbuff, unsigned short pktlen)
{
// print out packet content
	unsigned short c;
	unsigned short *x;
	x=pktbuff;

  for(c=0; c < pktlen/2; c++) {	
	serial_shortLH(*x++);
	serial_tx(0x20);
	}
	serial_crlf();

	return;
}

