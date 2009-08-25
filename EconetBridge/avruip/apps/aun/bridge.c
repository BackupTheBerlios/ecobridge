#include "internet.h"
#include "adlc.h"
#include "aun.h"
#include "serial.h"
#include <string.h>

extern uint8_t econet_net_nr;

static uint8_t bridge_rxcb;

static unsigned char bcast_buf[8];

static void do_bridge_reply(uint8_t stn, uint8_t reply_port)
{
  struct mbuf *mb = mbuf_alloc();
  unsigned char *response_buf = &mb->data[0];
  response_buf[0] = stn;
  response_buf[1] = 0;
  response_buf[2] = 0;
  response_buf[3] = eeGlobals.Econet_Network;
  response_buf[4] = 0x80;
  response_buf[5] = reply_port;
  response_buf[6] = eeGlobals.Econet_Network;
  response_buf[7] = 0x01;
  mb->length = 8;
  enqueue_tx (mb);
}

static inline uint8_t is_bridge(void)
{
  uint8_t *bcast_buf = (ECONET_RX_BUF + 6);
  static const char *bridge = "BRIDGE";
  uint8_t i;
  for (i = 0; i < 6; i++)
  {
    if ((bcast_buf[i] & 0xdf) != bridge[i])
      return 0;
  }
  return 1;
}

void handle_port_9c(uint16_t length)
{
  uint8_t *bcast_buf = (ECONET_RX_BUF + 6);

  switch (ECONET_RX_BUF[4]) {
    case 0x80: /* reset */
    case 0x81: /* advertise */
      break;
      
    case 0x82: /* what net */
    case 0x83: /* is net */
      if (!is_bridge()) {
	break;
      }
      uint8_t reply_port = bcast_buf[6];
      if (ECONET_RX_BUF[4] == 0x83 && !rTableEthType[bcast_buf[7]])
	break;
      do_bridge_reply (ECONET_RX_BUF[2], reply_port);
      break;
  }
}
