#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "adlc.h"
#include "aun.h"
#include "serial.h"
#include "uip.h"

#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define MAX_TX	8
#define MAX_RX	4
#define ADLC_TX_RETRY_COUNT	16
#define ADLC_TX_RETRY_DELAY	128

volatile unsigned char ECONET_RX_BUF[2000];
volatile unsigned short adlc_rx_ptr;
volatile register unsigned char adlc_state	asm("r15");

#define MACHINE_VENDOR          0x50
#define MACHINE_TYPE            0x50
#define MACHINE_VER_LOW         0x01
#define MACHINE_VER_HIGH        0x00

struct tx_record
{
  unsigned char *buf;
  int len;
  unsigned char retry_count;
  unsigned char retry_timer;
  unsigned short requestor_ip[2];
  uint32_t requestor_handle;
};

struct rx_record
{
  uint8_t state, port, cb, stn, net;
  unsigned char *buf;
  int len;
};

struct tx_record tx_buf[MAX_TX];
struct rx_record rx_buf[MAX_RX];

struct rx_record *current_rx;



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

uint8_t setup_rx(uint8_t port, uint8_t stn, uint8_t net, unsigned char *ptr, unsigned int length)
{
  int i;
  for (i = 0; i < MAX_RX; i++)
  {
    struct rx_record *rx = &rx_buf[i];
    if (rx->state == RXCB_INVALID)
    {
      rx->port = port;
      rx->stn = stn;
      rx->net = net;
      rx->buf = ptr;
      rx->len = length;
      rx->state = RXCB_READY;
      return i + 1;
    }
  }
  return 0;
}

uint8_t poll_rx(uint8_t i, uint8_t *stn, uint8_t *net)
{
  if (i == 0 || i > MAX_RX)
    return RXCB_INVALID;
  struct rx_record *rx = &rx_buf[i-1];
  *stn = rx->stn;
  *net = rx->net;
  return rx->state;
}

void close_rx(uint8_t i)
{
  rx_buf[i].state = RXCB_INVALID;
}

extern int get_adlc_state(void);
extern int adlc_await_idle(void);
extern void adlc_tx_frame(unsigned char *buf, unsigned char *end, unsigned char type);

static uint32_t ip_target;
uint16_t my_station;

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

  if (adlc_await_idle())
    return LINE_JAMMED;

  if (type == BROADCAST) {
    adlc_tx_frame (tx->buf, tx->buf + tx->len, 1);
    adlc_ready_to_receive (RX_SCOUT);
    serial_crlf();
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

  if (state == RX_IDLE) {
    serial_tx ('N');
    serial_crlf();
    return NOT_LISTENING;
  }

  serial_tx ('S');

  if (type == NORMAL_PACKET) {
    adlc_tx_frame (tx->buf, tx->buf + tx->len, 0);

    do {
      state = get_adlc_state();
    } while (state != RX_IDLE && state != (RX_DATA_ACK | FRAME_COMPLETE));
  }

  serial_tx ('V');
    
  adlc_ready_to_receive (RX_SCOUT);

  serial_crlf();

  return ((state & 0xf) == FRAME_COMPLETE) ? TX_OK : NET_ERROR;
}

struct rx_record *find_local_rxcb(uint8_t port, uint8_t station, uint8_t net)
{
  int i;
  for (i = 0; i < MAX_RX; i++) 
  {
    struct rx_record *rx = &rx_buf[i];
    if (rx->state == RXCB_READY 
	&& (rx->port == port || rx->port == 0)
	&& ((rx->stn == 0 && rx->net == 0)
	    || (rx->stn == station && rx->net == net)))
      return rx;
  }
  return NULL;
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

  /* if destination is reachable fill ip_target address */

  if (rTableEth[(dest-127)] != 0)  
  {
    *ip_target = rTableEth[dest];
    return 1;	
  }
  else {
    return 0;
  }

}

static unsigned char scout_buf[16];

static void make_scout(void)
{
  scout_buf[0] = ECONET_RX_BUF[2];
  scout_buf[1] = ECONET_RX_BUF[3];
  scout_buf[2] = ECONET_RX_BUF[0];
  scout_buf[3] = ECONET_RX_BUF[1];
}

static void do_local_immediate (uint8_t cb)
{
  switch (cb & 0x7f)
  {
  case Econet_MachinePeek:
    make_scout ();
    scout_buf[4] = MACHINE_TYPE;
    scout_buf[5] = MACHINE_VENDOR;
    scout_buf[6] = MACHINE_VER_LOW;
    scout_buf[7] = MACHINE_VER_HIGH;
    adlc_tx_frame (scout_buf, scout_buf + 8, 1);
    break;
  }
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
  else if ((adlc_state & 0x0f) == FRAME_COMPLETE)
  {
    if (adlc_state == (RX_SCOUT | FRAME_COMPLETE))
    {
      int frame_length = adlc_rx_ptr - (int)ECONET_RX_BUF;
      if (frame_length < 6) {
	adlc_ready_to_receive (RX_SCOUT);
	return;
      }
      uint16_t dst = *((uint16_t *)ECONET_RX_BUF);
      uint8_t cb = ECONET_RX_BUF[4];
      uint8_t port = ECONET_RX_BUF[5];
      if ((dst & 0xff) == 0xff)
      {
	struct rx_record *rx = find_local_rxcb (port, ECONET_RX_BUF[2], ECONET_RX_BUF[3]);
	if (rx)
	{
	  if (rx->len >= (frame_length - 6))
	  {
	    memcpy (rx->buf, ECONET_RX_BUF + 6, frame_length - 6);
	    rx->stn = ECONET_RX_BUF[2];
	    rx->net = ECONET_RX_BUF[3];
	    rx->cb = cb;
	    rx->port = port;
	    rx->state = RXCB_RECEIVED;
	  }
	}	
      } 
      if (should_bridge (dst, &ip_target))
      {
	serial_tx ('B');
	make_scout ();
	adlc_tx_frame (scout_buf, scout_buf + 4, 1);
	adlc_ready_to_receive (RX_DATA);
	return;
      }
      else if ((my_station & 0xff) && (dst == my_station))
      {
	if (port == 0)
	{
	  do_local_immediate (cb);
	}
	else
	{
	  current_rx = find_local_rxcb (port, ECONET_RX_BUF[2], ECONET_RX_BUF[3]);
	  if (current_rx)
	  {
	    current_rx->state = RXCB_RECEIVING;
	    make_scout ();
	    adlc_tx_frame (scout_buf, scout_buf + 4, 1);
	    adlc_ready_to_receive (RX_DATA);
	    return;
	  }
	}
      }
      adlc_ready_to_receive (RX_SCOUT);
    }
    else if (adlc_state == (RX_DATA | FRAME_COMPLETE))
    {
      adlc_ready_to_receive (RX_SCOUT);
    }
    else if ((adlc_state & 15) == FRAME_COMPLETE)
    {
      serial_tx_hex (adlc_state);
      serial_crlf ();
      adlc_ready_to_receive (RX_SCOUT);
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

  enqueue_tx(buf, p - buf);
}

void test_immediate(void)
{
  static unsigned char buf[16];
  unsigned char *p = buf;
  *(p++) = 0xfe;
  *(p++) = 0x00;
  *(p++) = my_station;
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
  *(p++) = my_station;
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


