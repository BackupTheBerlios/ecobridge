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

unsigned char ECONET_RX_BUF[2000];
volatile register unsigned char adlc_state	asm("r15");

#define MACHINE_VENDOR          0x50
#define MACHINE_TYPE            0x50
#define MACHINE_VER_LOW         0x01
#define MACHINE_VER_HIGH        0x00

struct stats
{
  int frames_in;
  int short_scouts;
  int rx_bcast;
  int tx_attempts;
  int tx_collision;
  int tx_not_listening;
  int tx_net_error;
  int tx_line_jammed;
};

static struct stats stats;

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
  uint8_t state, port, cb, stn, net, rx_port;
  unsigned char *buf;
  int len;
};

struct tx_record tx_buf[MAX_TX];
struct rx_record rx_buf[MAX_RX];

struct rx_record *current_rx;

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

uint8_t poll_rx(uint8_t i, struct rx_control *rxc)
{
  if (i == 0 || i > MAX_RX)
    return RXCB_INVALID;
  struct rx_record *rx = &rx_buf[i-1];
  rxc->stn = rx->stn;
  rxc->net = rx->net;
  rxc->cb = rx->cb;
  rxc->port = rx->rx_port;
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

  stats.tx_attempts++;

  if (adlc_await_idle()) {
    stats.tx_line_jammed++;
    return LINE_JAMMED;
  }

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
  tx->buf = NULL;

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

static void make_scout(uint8_t stn, uint8_t net)
{
  scout_buf[0] = stn;
  scout_buf[1] = net;
  scout_buf[2] = my_station & 0xff;
  scout_buf[3] = my_station >> 8;
}

static void do_local_immediate (uint8_t cb, uint8_t stn, uint8_t net)
{
  switch (cb & 0x7f)
  {
  case Econet_MachinePeek:
    make_scout (stn, net);
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
    uint16_t frame_length = adlc_rx_ptr - (int)ECONET_RX_BUF;
    stats.frames_in++;
    if (adlc_state == (RX_SCOUT | FRAME_COMPLETE))
    {
      if (frame_length < 6) {
        stats.short_scouts++;
	adlc_ready_to_receive (RX_SCOUT);
	return;
      }
      uint16_t dst = *((uint16_t *)ECONET_RX_BUF);
      uint8_t cb = ECONET_RX_BUF[4];
      uint8_t port = ECONET_RX_BUF[5];
      uint8_t src_stn = ECONET_RX_BUF[2];
      uint8_t src_net = ECONET_RX_BUF[3];
      if ((dst & 0xff) == 0xff)
      {
        stats.rx_bcast++;
	struct rx_record *rx = find_local_rxcb (port, src_stn, src_net);
	if (rx)
	{
	  if (rx->len >= (frame_length - 6))
	  {
	    memcpy (rx->buf, ECONET_RX_BUF + 6, frame_length - 6);
	    rx->stn = src_stn;
	    rx->net = src_net;
	    rx->cb = cb;
	    rx->rx_port = port;
	    rx->state = RXCB_RECEIVED;
	  }
	}
      }
      if (port != 0x9c && should_bridge (dst, &ip_target))
      {
	serial_tx ('B');
	make_scout (src_stn, src_net);
	adlc_tx_frame (scout_buf, scout_buf + 4, 1);
	adlc_ready_to_receive (RX_DATA);
	return;
      }
      else if ((my_station & 0xff) && (dst == my_station))
      {
	if (port == 0)
	{
	  do_local_immediate (cb, src_stn, src_net);
	}
	else
	{
	  current_rx = find_local_rxcb (port, src_stn, src_net);
	  if (current_rx)
	  {
	    current_rx->state = RXCB_RECEIVING;
	    current_rx->cb = cb;
	    current_rx->rx_port = port;
	    make_scout (src_stn, src_net);
	    adlc_tx_frame (scout_buf, scout_buf + 4, 1);
	    adlc_ready_to_receive (RX_DATA);
	    return;
	  }
	}
      }
    }
    else if (adlc_state == (RX_DATA | FRAME_COMPLETE))
    {
      memcpy (uip_appdata + 6, ECONET_RX_BUF + 4, frame_length - 4);
      aun_send_packet (ip_target, frame_length - 4);
    }
    else
    {
      serial_tx_hex (adlc_state);
      serial_crlf ();
    }
    adlc_ready_to_receive (RX_SCOUT);
  }
}

void adlc_forwarding_complete(uint8_t result)
{
  if (result == TX_OK)
  {
    make_scout (ECONET_RX_BUF[2], ECONET_RX_BUF[3]);
    adlc_tx_frame (scout_buf, scout_buf + 4, 1);
  }

  adlc_ready_to_receive (RX_SCOUT);
}
