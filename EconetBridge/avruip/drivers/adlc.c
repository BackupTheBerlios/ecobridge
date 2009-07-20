
#include "adlc.h"

volatile unsigned char ECONET_RX_BUF[2000];

volatile extern unsigned short adlc_rx_ptr asm("adlc_rx_ptr") = 0;

register unsigned char adlc_state	asm("r15");


void	adlc_poller(void)
{

	if (adlc_state == FRAME_COMPLETE)
	{
		// print out the packet on the serial
		serial_packet(ECONET_RX_BUF,0x12);
		adlc_ready_to_receive();
	}

	return;
}
