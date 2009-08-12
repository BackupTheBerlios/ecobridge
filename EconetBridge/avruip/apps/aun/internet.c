#include "internet.h"
#include "adlc.h"
#include "globals.h"
#include <string.h>

static uint8_t find_server_rxcb;

static unsigned char bcast_buf[8];

#define FIND_SERVER_PORT	0xb0
#define FIND_SERVER_REPLY_PORT	0xb1
#define IP_PORT			0xd2

#define EcCb_ARP		0x21
#define EcCb_ARPreply		0x22
#define EcCb_Frame		0x01

static const char *MY_SERVER_TYPE = "INTERNET";
static const char *MY_SERVER_NAME = "TCP/IP Gateway";
static const char *WILDCARD_SERVER_TYPE = "        ";

static void setup_find_server_rxcb(void)
{
  find_server_rxcb = setup_rx(FIND_SERVER_PORT, 0, 0, bcast_buf, 8);
}

void internet_init(void)
{
  setup_find_server_rxcb();
}

void internet_poller(void)
{
  struct rx_control rxc;
  if (poll_rx (find_server_rxcb, &rxc) == RXCB_RECEIVED)
  {
    if (memcmp (bcast_buf, MY_SERVER_TYPE, 8) == 0
	|| memcmp (bcast_buf, WILDCARD_SERVER_TYPE, 8) == 0)
    {
      struct mbuf *mb = mbuf_alloc();
      unsigned char *response_buffer = &mb->data[0];
      response_buffer[0] = rxc.stn;
      response_buffer[1] = rxc.net;
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

    setup_find_server_rxcb ();
  }
}
