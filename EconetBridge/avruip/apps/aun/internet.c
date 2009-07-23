#include "internet.h"
#include "adlc.h"
#include <string.h>

extern uint16_t my_station;

static uint8_t find_server_rxcb;

static unsigned char bcast_buf[8];
static unsigned char response_buffer[32];

#define FIND_SERVER_PORT	0xb0
#define FIND_SERVER_REPLY_PORT	0xb1

#define MY_SERVER_TYPE		"INTERNET"
#define MY_SERVER_NAME		"TCP/IP Gateway"

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
  uint8_t stn, net;
  if (poll_rx (find_server_rxcb, &stn, &net) == RXCB_RECEIVED)
  {
    serial_tx_str ("find server ");
    int i;
    for (i = 0; i < 8; i++)
      serial_tx (bcast_buf[i]);
    serial_tx_hex (net);
    serial_tx_hex (stn);
    serial_crlf();
    response_buffer[0] = stn;
    response_buffer[1] = net;
    response_buffer[2] = my_station;
    response_buffer[3] = 0;
    response_buffer[4] = 0x80;
    response_buffer[5] = FIND_SERVER_REPLY_PORT;
    response_buffer[6] = 0;
    response_buffer[7] = 0xd2;
    response_buffer[8] = 1;
    strcpy (response_buffer + 9, MY_SERVER_TYPE);
    response_buffer[17] = strlen(MY_SERVER_NAME);
    strcpy (response_buffer + 18, MY_SERVER_NAME);
    enqueue_tx (response_buffer, 18 + strlen(MY_SERVER_NAME));
    close_rx (find_server_rxcb);
    setup_find_server_rxcb ();
  }
}
