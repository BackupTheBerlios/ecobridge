#include <stdlib.h>
#include <stdint.h>

#include "adlc.h"
#include "serial.h"

#define MAX_TX	8
#define TX_RETRY_COUNT	16
#define TX_RETRY_DELAY	128

volatile unsigned char ECONET_RX_BUF[2000];
volatile unsigned short adlc_rx_ptr;
register unsigned char adlc_state	asm("r15");

struct tx_record
{
  unsigned char *buf;
  int len;
  unsigned char retry_count;
  unsigned char retry_timer;
  uint32_t requestor_ip;
  uint32_t requestor_handle;
};

struct tx_record tx_buf[MAX_TX];

#define TX_OK		0
#define LINE_JAMMED	1
#define NOT_LISTENING	2
#define NET_ERROR	3

#define NORMAL_PACKET	0
#define BROADCAST	1
#define IMMEDIATE	2

struct tx_record *get_tx_buf(void)
{
  int i;
  for (i = 0; i < MAX_TX; i++)
  {
    if (tx_buf[i].buf == NULL)
      return &tx_buf[i];
  }
  return NULL;
}

int do_tx_packet(struct tx_record *tx)
{
  unsigned char *buf = tx->buf;
  unsigned char type = NORMAL_PACKET;
  if (buf[0] == 0xff)
    type = BROADCAST;
  else if (buf[5] == 0)
    type = IMMEDIATE;

  serial_eco();
  serial_tx('t');
  serial_tx('x');
  serial_tx_hex (type);

  serial_crlf();

  if (adlc_await_idle())
    return LINE_JAMMED;

  adlc_tx_frame (tx->buf, tx->buf + tx->len, 1);
 
  return NOT_LISTENING;
}

int enqueue_tx(unsigned char *buf, int length)
{
  if (length < 6)
    return -2;		// can't enqueue runt packets

  struct tx_record *tx = get_tx_buf();
  if (tx)
  {
    tx->retry_count = TX_RETRY_COUNT;
    tx->retry_timer = 0;
    tx->len = length;
    tx->buf = buf;
    return 0;
  }
  return -1;
}

void adlc_poller(void)
{
  if (adlc_state == FRAME_COMPLETE)
  {
    // print out the packet on the serial
    serial_packet(ECONET_RX_BUF,0x12);
    adlc_ready_to_receive();
  }
  else if (adlc_state == RX_IDLE)
  {
    int i;
    for (i = 0; i < MAX_TX; i++)
    {
      struct tx_record *tx = &tx_buf[i];
      if (tx->buf)
      {
	if (tx->retry_timer == 0)
	{
	  switch (do_tx_packet (tx)) {
	  default:
	    if (tx->retry_count--) {
	      tx->retry_timer = TX_RETRY_DELAY;
	      break;
	    }
	  case TX_OK:
	  case LINE_JAMMED:
	    tx->buf = NULL;
	    break;
	  }
	}
	else
	  tx->retry_timer--;
      }
    }
  }
}

void test_bcast(void)
{
  static unsigned char buf[16];
  unsigned char *p = buf;
  *(p++) = 0xff;
  *(p++) = 0xff;
  *(p++) = 0xff;
  *(p++) = 0xff;
  *(p++) = 0x80;
  *(p++) = 0x9c;
  *(p++) = 0x3;

  serial_tx_hex (enqueue_tx(buf, p - buf));
}
