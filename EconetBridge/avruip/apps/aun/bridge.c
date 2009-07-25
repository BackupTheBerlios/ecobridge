#include "internet.h"
#include "adlc.h"
#include "serial.h"
#include <string.h>

extern uint8_t econet_net_nr;

static uint8_t bridge_rxcb;

static unsigned char bcast_buf[8];
static unsigned char response_buf[8];

#define BRIDGE_PORT	0x9c

static void setup_bridge_rxcb(void)
{
  bridge_rxcb = setup_rx(BRIDGE_PORT, 0, 0, bcast_buf, 8);
}

void bridge_init(void)
{
  setup_bridge_rxcb();
}

static void do_bridge_reply(uint8_t stn, uint8_t reply_port)
{
  response_buf[0] = stn;
  response_buf[1] = 0;
  response_buf[2] = 0;
  response_buf[3] = 0;
  response_buf[4] = 0x80;
  response_buf[5] = reply_port;
  response_buf[6] = econet_net_nr;
  response_buf[7] = 0x01;
  enqueue_tx (response_buf, 8);
}

static int is_bridge(void)
{
  static const char *bridge = "BRIDGE";
  int i;
  for (i = 0; i < 6; i++)
  {
    if ((bcast_buf[i] & 0xdf) != bridge[i])
    {
      serial_tx_str ("not bridge: ");
      serial_tx (bcast_buf[0]);
      serial_tx (bcast_buf[1]);
      serial_tx (bcast_buf[2]);
      serial_tx (bcast_buf[3]);
      serial_tx (bcast_buf[4]);
      serial_tx (bcast_buf[5]);
      serial_crlf ();
      return 0;
    }
  }
  return 1;
}

void bridge_poller(void)
{
  struct rx_control rxc;

  if (poll_rx (bridge_rxcb, &rxc) == RXCB_RECEIVED)
  {
    serial_tx_str ("BRIDGE");
    serial_tx_hex (rxc.cb);
    serial_crlf();

    switch (rxc.cb) {
    case 0x80: /* reset */
    case 0x81: /* advertise */
      break;
      
    case 0x82: /* what net */
    case 0x83: /* is net */
      if (!is_bridge()) {
	break;
      }
      uint8_t reply_port = bcast_buf[6];
      if (rxc.cb == 0x82) {
	do_bridge_reply (rxc.stn, reply_port);
      }
      break;
    }
    
    close_rx (bridge_rxcb);
    setup_bridge_rxcb ();
  }
}
