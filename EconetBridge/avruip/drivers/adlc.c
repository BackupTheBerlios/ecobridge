
#include "adlc.h"

unsigned char ECONET_RX_BUF[2000];


//unsigned char *ECONET_RX_BUF;		/* The Econet Rx Buf pointer points to received data. */

unsigned short	adlc_rx_ptr;
unsigned short	adlc_r24_ptr;
unsigned char 	adlc_state;

void adlc_init2(void)
{

//	*ECONET_RX_BUF  = &buf;
//	adlc_init();
	return;

}


void	adlc_poller(void)
{

	if (adlc_state == FRAME_COMPLETE)
	{
		// print out the packet on the serial
		serial_packet(&ECONET_RX_BUF,0x12);
	}

	return;
}
