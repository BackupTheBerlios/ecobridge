#include <stdlib.h>
#include <stdint.h>

#include "adlc.h"
#include "serial.h"
#include "uip.h"

#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define MAX_TX	8
#define ADLC_TX_RETRY_COUNT	16
#define ADLC_TX_RETRY_DELAY	128

volatile unsigned char ECONET_RX_BUF[2000];
volatile unsigned short adlc_rx_ptr;
volatile register unsigned char adlc_state	asm("r15");

struct tx_record
{
  unsigned char *buf;
  int len;
  unsigned char retry_count;
  unsigned char retry_timer;
  unsigned short requestor_ip[2];
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

extern int get_adlc_state(void);
extern int adlc_await_idle(void);
extern void adlc_tx_frame(unsigned char *buf, unsigned char *end, unsigned char type);

static uint32_t ip_target;

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

  if (type == BROADCAST) {
    adlc_tx_frame (tx->buf, tx->buf + tx->len, 1);
    adlc_ready_to_receive ();
    return TX_OK;
  }

  if (type == IMMEDIATE)
    adlc_tx_frame (tx->buf, tx->buf + tx->len, 1);
  else
    adlc_tx_frame (tx->buf, tx->buf + 6, 1);

  unsigned char state;
  do {
    state = get_adlc_state();
  } while (state != RX_IDLE && state != (RX_SCOUT_ACK | FRAME_COMPLETE));

  if (state == RX_IDLE)
    return NOT_LISTENING;

  if (type == NORMAL_PACKET) {
    adlc_tx_frame (tx->buf, tx->buf + tx->len, 0);

    do {
      state = get_adlc_state();
    } while (state != RX_IDLE && state != (RX_SCOUT_ACK | FRAME_COMPLETE));
  }
    
  adlc_ready_to_receive ();

  return ((state & 0xf) == FRAME_COMPLETE) ? TX_OK : NET_ERROR;
}

int enqueue_tx(unsigned char *buf, int length)
{
  if (length < 6)
    return -2;		// can't enqueue runt packets

    struct mns_msg *m; 
    m = (struct mns_msg *)uip_appdata;

  struct tx_record *tx = get_tx_buf();
  if (tx)
  {
    tx->retry_count = ADLC_TX_RETRY_COUNT;
    tx->retry_timer = 0;
    tx->len = length;
    tx->buf = buf;
    tx->requestor_ip[0] = BUF->srcipaddr[0];
    tx->requestor_ip[1] = BUF->srcipaddr[1];
    tx->requestor_handle = m->mns_handle;
    return 0;
  }
  return -1;
}

int should_bridge(uint16_t dest, uint32_t *ip_target)
{
  /* ... */
  return 0;
}

void adlc_poller(void)
{
  if (adlc_state == RX_IDLE)
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
	      tx->retry_timer = ADLC_TX_RETRY_DELAY;
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
  else if (adlc_state == (RX_SCOUT | FRAME_COMPLETE))
  {
    uint16_t dst = *((uint16_t *)ECONET_RX_BUF);
    if (should_bridge (dst, &ip_target))
    {
      static unsigned char scout_buf[6];
      scout_buf[0] = ECONET_RX_BUF[2];
      scout_buf[1] = ECONET_RX_BUF[3];
      scout_buf[2] = ECONET_RX_BUF[0];
      scout_buf[3] = ECONET_RX_BUF[1];
      scout_buf[4] = ECONET_RX_BUF[4];
      scout_buf[5] = ECONET_RX_BUF[5];
      adlc_tx_frame (scout_buf, scout_buf + 6, 1);
    }
    adlc_ready_to_receive();
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

  enqueue_tx(buf, p - buf);
}

void test_immediate(void)
{
  static unsigned char buf[16];
  unsigned char *p = buf;
  *(p++) = 0xfe;
  *(p++) = 0x00;
  *(p++) = 0x51;
  *(p++) = 0x00;
  *(p++) = 0x88;
  *(p++) = 0x00;
  *(p++) = 0x00;
  *(p++) = 0x00;
  *(p++) = 0x00;

  enqueue_tx(buf, p - buf);
}

void test_4way(void)
{
  static unsigned char buf[16];
  unsigned char *p = buf;
  *(p++) = 0xfe;
  *(p++) = 0x00;
  *(p++) = 0x51;
  *(p++) = 0x00;
  *(p++) = 0x80;
  *(p++) = 0x99;
  *(p++) = 0x90;
  *(p++) = 25;
  *(p++) = 0x00;
  *(p++) = 0x00;
  *(p++) = 0x00;
  *(p++) = 0x00;
  *(p++) = 0x00;

  enqueue_tx(buf, p - buf);
}

unsigned char send_packet(unsigned char* buffer, unsigned short length)
{

//	serial_packet((buffer),length);

  	enqueue_tx(buffer, length);


    return 1;
}


