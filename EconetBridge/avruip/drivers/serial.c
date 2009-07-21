

#include <avr/io.h>
#include <avr/sleep.h>
#include "serial.h"

extern void serial_tx(uint8_t mask);
extern void serial_tx_hex(uint8_t mask);

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

	serial_tx(0x6f);		// o
	serial_tx(0x6b);		// k
	serial_tx(0x20);		// space
	serial_tx_hex(okVal);	// value
	serial_crlf(); 		// cr lf

	return;
}

void serial_rx(void)
{

	serial_tx(0x72);		// r
	serial_tx(0x78);		// x
	serial_tx(0x20);		// space
	return;
}

void serial_txx(void)
{

	serial_tx(0x74);		// t
	serial_tx(0x78);		// x
	serial_tx(0x20);		// space
	return;
}

void serial_eth(void)
{

	serial_tx(0x65);		// e
	serial_tx(0x74);		// t
	serial_tx(0x68);		// h
	serial_tx(0x20);		// space
	return;
}

void serial_eco(void)
{

	serial_tx(0x65);		// e
	serial_tx('c');			// c
	serial_tx(0x6f);		// o
	serial_tx(0x20);		// space
	return;
}

void serial_error(unsigned char eVal)
{

	serial_tx(0x65);		// e
	serial_tx(0x20);		// space
	serial_tx_hex(eVal);	// error value
	serial_crlf(); 		// cr lf

	return;
}

void serial_crlf(void)
{

	serial_tx(13);		// cr
	serial_tx(10);		// lf

	return;
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

