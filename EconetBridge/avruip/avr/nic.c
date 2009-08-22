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
#include <string.h>

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

static void send_frag(struct mbuf *mb, uint16_t length)
{
#ifdef DEBUG
  serial_tx_str ("xmit ");
  serial_tx_hex (length >> 8);
  serial_tx_hex (length);
  serial_crlf ();
  memset (&mb->data[0], 0xff, 6);
#endif
	NICBeginPacketSend(length);
        while (length) {
		uint16_t this_length = mb->length;
		if (this_length > length)
			this_length = length;
                NICSendPacketData(mb->data, this_length);
		length -= this_length;
                mb = mb->next;
        }
	NICEndPacketSend();
}

static int mtu = 1518;

struct frag_mbuf
{
  struct mbuf *next, *prev;
  uint8_t length;
  uint8_t pad;
  uint8_t data[UIP_IPH_LEN + UIP_LLH_LEN + 8];
};

static struct frag_mbuf frag_mbuf;

#define IP_MF   0x20

void nic_send(struct mbuf *mb)
{
        if (mb == NULL) {
		mb = uip_to_mbufs();
        }

	memcpy (&frag_mbuf.data[0], mb->data, UIP_IPH_LEN + UIP_LLH_LEN);

        uint16_t length = 0, offset = 0;
        struct mbuf *mbp = mb, *this_mb = mb;
        while (mbp) {
		if ((length + mbp->length) >= mtu) {
			/* packet needs fragmenting */
			struct uip_tcpip_hdr *BUF = &this_mb->data[UIP_LLH_LEN];
			uint16_t payload_length = length - (UIP_IPH_LEN + UIP_LLH_LEN);
			uint8_t overdone = payload_length & 7;
			length -= overdone;
			send_frag (this_mb, length);
			memcpy (&frag_mbuf.data[UIP_IPH_LEN + UIP_LLH_LEN], mb->prev->data + mb->prev->length - overdone, overdone);
			length = UIP_IPH_LEN + UIP_LLH_LEN + overdone;
			frag_mbuf.length = length;
			frag_mbuf.next = mbp;
			this_mb = (struct mbuf *)&frag_mbuf;
			offset += (payload_length - overdone);
			BUF->ipoffset[0] = ((offset / 8) >> 8) | IP_MF;
			BUF->ipoffset[1] = (offset / 8);
		}
                length += mbp->length;
                mbp = mbp->next;
        }

	struct uip_tcpip_hdr *BUF = &this_mb->data[UIP_LLH_LEN];
	BUF->ipoffset[0] &= ~IP_MF;
	send_frag (this_mb, length);

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
