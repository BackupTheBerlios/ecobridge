#include "internet.h"
#include "adlc.h"
#include "globals.h"
#include "uip_arp.h"
#include <string.h>

static uint8_t find_server_rxcb;

#define FIND_SERVER_PORT	0xb0
#define FIND_SERVER_REPLY_PORT	0xb1
#define IP_PORT			0xd2

#define EcCb_ARP		0xa1
#define EcCb_ARPreply		0xa2
#define EcCb_Frame		0x81

#define MY_SERVER_TYPE "INTERNET"
#define MY_SERVER_NAME "TCP/IP Gateway"
#define WILDCARD_SERVER_TYPE "        "

static uip_ipaddr_t econet_subnet, econet_netmask;

struct ethip_hdr {
  struct uip_eth_hdr ethhdr;
  /* IP header. */
  u8_t vhl,
    tos,
    len[2],
    ipid[2],
    ipoffset[2],
    ttl,
    proto;
  u16_t ipchksum;
  u16_t srcipaddr[2],
    destipaddr[2];
};

struct ec_arp {
  u16_t dstipaddr[2],
    srcipaddr[2];
};

void internet_init(void)
{
  uip_ipaddr(econet_subnet, eeGlobals.EconetIP[0], eeGlobals.EconetIP[1], eeGlobals.EconetIP[2], eeGlobals.EconetIP[3]);
  uip_ipaddr(econet_netmask, eeGlobals.EconetMask[0], eeGlobals.EconetMask[1], eeGlobals.EconetMask[2], eeGlobals.EconetMask[3]);
}

void do_send_mbuf(struct mbuf *mb)
{
  mb->data[2] = eeGlobals.Station;
  mb->data[3] = 0;
  mb->data[5] = 0xd2;
  enqueue_tx (mb);
}

void handle_ip_packet(uint8_t cb, uint16_t length)
{
  struct ec_arp *arpbuf = (struct ec_arp *)ECONET_RX_BUF + 6;
  switch (cb) {
  case EcCb_Frame:
    length -= 4;
    if (length <= (UIP_BUFSIZE - UIP_LLH_LEN))
    {
      memcpy (uip_buf + UIP_LLH_LEN, ECONET_RX_BUF + 4, length);
      uip_len = length;
      uip_arp_out ();
      nic_send (NULL);
    }
    break;
  case EcCb_ARP:
    if (uip_ipaddr_cmp(econet_subnet, arpbuf->dstipaddr)) {
      serial_tx_str ("arp me\r\n");
      struct mbuf *mb = mbuf_alloc ();
      struct ec_arp *arpbuf2 = &mb->data[6];
      uip_ipaddr_copy (arpbuf2->dstipaddr, arpbuf->srcipaddr);
      uip_ipaddr_copy (arpbuf2->srcipaddr, arpbuf->dstipaddr);
      mb->data[0] = ECONET_RX_BUF[2];
      mb->data[1] = ECONET_RX_BUF[3];
      mb->data[4] = EcCb_ARPreply;
      do_send_mbuf (mb);
    }
    break;
  case EcCb_ARPreply:
    break;
  }
}

#define IPBUF ((struct ethip_hdr *)&uip_buf[0])

uint8_t forward_to_econet (void)
{
  /* Check if the destination address is on the local network. */
  if (eeGlobals.EconetMask[0] && uip_ipaddr_maskcmp(IPBUF->destipaddr, econet_subnet, econet_netmask)) {
    struct arp_entry *arp = find_arp_entry (IPBUF->destipaddr);
    if (arp) {
      struct mbuf *mb = copy_to_mbufs (uip_buf + UIP_LLH_LEN - 6, uip_len - UIP_LLH_LEN + 6);
      mb->data[0] = arp->ethaddr.addr[0];
      mb->data[1] = arp->ethaddr.addr[1];
      mb->data[4] = EcCb_ARP;
      do_send_mbuf (mb);
    } else {
      struct mbuf *mb = mbuf_alloc();
      uint16_t *p = (uint16_t *)(mb->data + 6);
      uip_ipaddr_copy (p + 0, econet_subnet);
      uip_ipaddr_copy (p + 2, IPBUF->destipaddr);
      mb->data[0] = mb->data[1] = 0xff;
      mb->data[4] = EcCb_ARP;
      mb->length = 14;
      do_send_mbuf (mb);
    }
    return 1;
  }

  return 0;
}

void handle_port_b0(void)
{
  unsigned char *bcast_buf = ECONET_RX_BUF + 6;

  if (memcmp (bcast_buf, MY_SERVER_TYPE, 8) == 0
      || memcmp (bcast_buf, WILDCARD_SERVER_TYPE, 8) == 0)
    {
      struct mbuf *mb = mbuf_alloc();
      unsigned char *response_buffer = &mb->data[0];
      response_buffer[0] = ECONET_RX_BUF[2];
      response_buffer[1] = ECONET_RX_BUF[3];
      response_buffer[2] = eeGlobals.Station;
      response_buffer[3] = 0;
      response_buffer[4] = 0x80;
      response_buffer[5] = FIND_SERVER_REPLY_PORT;
      response_buffer[6] = 0;
      response_buffer[7] = IP_PORT;
      response_buffer[8] = 1;
      strcpy ((char *)response_buffer + 9, MY_SERVER_TYPE);
      response_buffer[17] = strlen(MY_SERVER_NAME);
      strcpy ((char *)response_buffer + 18, MY_SERVER_NAME);

      mb->length = 18 + strlen(MY_SERVER_NAME);
      enqueue_tx (mb);
    }
}
