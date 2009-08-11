/*****************************************************************************
*  Module Name:       NIC Driver Interface for uIP-AVR Port
*
*  Created By:        Louis Beaudoin (www.embedded-creations.com)
*
*  Original Release:  November 16, 2003
*
*  Module Description:
*  Provides three functions to interface with a NIC driver
*  These functions can be called directly from the main uIP control loop
*  to send packets from uip_buf and uip_appbuf, and store incoming packets to
*  uip_buf
*
*
*****************************************************************************/

#include "nic.h"
#include "mbuf.h"

#define IP_TCP_HEADER_LENGTH 40
#define TOTAL_HEADER_LENGTH (IP_TCP_HEADER_LENGTH + ETHERNET_HEADER_LENGTH)


void nic_init(void)
{
	NICInit();
}

struct mbuf *uip_to_mbufs(void)
{
  struct mbuf *mb;
  if (uip_len <= TOTAL_HEADER_LENGTH) {
    mb = copy_to_mbufs (uip_buf, uip_len);
  } else {
    mb = copy_to_mbufs (uip_buf, TOTAL_HEADER_LENGTH);
    struct mbuf *mb2 = copy_to_mbufs (uip_appdata, uip_len - TOTAL_HEADER_LENGTH);
    struct mbuf *mbp = mb;
    while (mbp->next) {
      mbp = mbp->next;
    }
    mbp->next = mb2;
    mb2->prev = mbp;
  }
  return mb;
}

void nic_send(struct mbuf *mb)
{
        uint8_t do_free = 0;

        if (mb == NULL) {
		mb = uip_to_mbufs();
                do_free = 1;
        }

        uint16_t length = 0;
        struct mbuf *mbp = mb;
        while (mbp) {
                length += mbp->length;
                mbp = mbp->next;
        }

	NICBeginPacketSend(length);
        mbp = mb;
        while (mbp) {
                NICSendPacketData(mbp->data, mbp->length);
                mbp = mbp->next;
        }

	NICEndPacketSend();

        if (do_free)
                mbuf_free_chain (mb);
}



#if UIP_BUFSIZE > 255
unsigned short nic_poll(void)
#else
unsigned char nic_poll(void)
#endif /* UIP_BUFSIZE > 255 */
{
	unsigned int packetLength;

	packetLength = NICBeginPacketRetreive();

	// if there's no packet or an error - exit without ending the operation
	if( !packetLength )
		return 0;

	// drop anything too big for the buffer
	if( packetLength > UIP_BUFSIZE )
	{
		NICEndPacketRetreive();
		return 0;
	}

	// copy the packet data into the uIP packet buffer
	NICRetreivePacketData( uip_buf, packetLength );

//serial_packet(uip_buf, packetLength);
//serial_eth();
//serial_rx();
//serial_crlf();


	NICEndPacketRetreive();

#if UIP_BUFSIZE > 255
	return packetLength;
#else
	return (unsigned char)packetLength;
#endif /* UIP_BUFSIZE > 255 */

}
