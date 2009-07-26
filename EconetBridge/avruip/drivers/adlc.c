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
  unsigned char is_aun;
  unsigned short requestor_ip[2];
  uint32_t requestor_handle;
};

struct rx_record
{
  uint8_t state, port, cb, stn, net, rx_port;
  int len;
  union {
    unsigned char *buf;
    void (*callback)(int);
  } d;
};

struct tx_record tx_buf[MAX_TX];
struct rx_record rx_buf[MAX_RX];

struct rx_record *current_rx;

#define NORMAL_PACKET	0
#define BROADCAST	1
#define IMMEDIATE	2

static uint8_t get_tx_buf(void)
{
  uint8_t i;
  for (i = 0; i < MAX_TX; i++)
  {
    if (tx_buf[i].buf == NULL)
      return i;
  }
  return 0xff;
}

uint8_t setup_rx(uint8_t port, uint8_t stn, uint8_t net, unsigned char *ptr, unsigned int length)
{
  uint8_t i;
  for (i = 0; i < MAX_RX; i++)
  {
    struct rx_record *rx = &rx_buf[i];
    if (rx->state == RXCB_INVALID)
    {
      rx->port = port;
      rx->stn = stn;
      rx->net = net;
      rx->d.buf = ptr;
      rx->len = length;
      rx->state = RXCB_READY;
      return i + 1;
    }
  }
  return 0;
}

uint8_t setup_sync_rx(uint8_t port, uint8_t stn, uint8_t net, void (*callback)(int))
{
  uint8_t i;
  for (i = 0; i < MAX_RX; i++)
  {
    struct rx_record *rx = &rx_buf[i];
    if (rx->state == RXCB_INVALID)
    {
      rx->port = port;
      rx->stn = stn;
      rx->net = net;
      rx->d.callback = callback;
      rx->len = 0;
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
  uint8_t state = rx->state;
  if (state == RXCB_RECEIVED) {
    rxc->stn = rx->stn;
    rxc->net = rx->net;
    rxc->cb = rx->cb;
    rxc->port = rx->rx_port;
    rx->state = RXCB_INVALID;
  }
  return state;
}

void close_rx(uint8_t i)
{
  rx_buf[i].state = RXCB_INVALID;
}

extern uint8_t get_adlc_state(void);
extern uint8_t adlc_await_idle(void);
extern void adlc_tx_frame(unsigned char *buf, unsigned char *end, unsigned char type);

static uint32_t ip_target;
uint16_t my_station;
static uint8_t aun_cb, aun_port;

static int8_t do_tx_packet(struct tx_record *tx)
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
  uint8_t i;
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

int16_t enqueue_tx(unsigned char *buf, int length, unsigned char is_aun)
{
  if (length < 6)
    return -2;		// can't enqueue runt packets

  struct mns_msg *m;
  m = (struct mns_msg *)uip_appdata;

  uint8_t i = get_tx_buf();
  if (i != 0xff)
  {
    struct tx_record *tx = &tx_buf[i];
    tx->retry_count = ADLC_TX_RETRY_COUNT;
    tx->retry_timer = 0;
    tx->len = length;
    tx->buf = buf;
    if (is_aun) {
      tx->requestor_ip[0] = BUF->srcipaddr[0];
      tx->requestor_ip[1] = BUF->srcipaddr[1];
      tx->requestor_handle = m->mns_handle;
    }
    tx->is_aun = is_aun;
    return i + 1;
  }
  return -1;
}

uint8_t should_bridge(uint16_t dest, uint32_t *ip_target)
{

  /* if destination is reachable fill ip_target address */

  if (rTableEth[dest >> 8] != 0)
  {
    *ip_target = rTableEth[dest >> 8] | ((dest & 0xff) << 24);
    return 1;
  }
  else {
    return 0;
  }

}

static unsigned char scout_buf[16];

static void make_scout_acknowledge() __attribute__ ((noinline));

static void make_scout_acknowledge()
{
  scout_buf[0] = ECONET_RX_BUF[2];
  scout_buf[1] = ECONET_RX_BUF[3];
  scout_buf[2] = ECONET_RX_BUF[0];
  scout_buf[3] = ECONET_RX_BUF[1];
}

static void do_local_immediate (uint8_t cb, uint8_t stn, uint8_t net)
{
  switch (cb & 0x7f)
  {
  case Econet_MachinePeek:
    make_scout_acknowledge ();
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
    uint8_t i;
    for (i = 0; i < MAX_TX; i++)
    {
      struct tx_record *tx = &tx_buf[i];
      if (tx->buf)
      {
	if (tx->retry_timer == 0)
	{
	  int8_t state;
	  switch (state = do_tx_packet (tx)) {
	  default:
	    if (tx->retry_count--) {
	      tx->retry_timer = ADLC_TX_RETRY_DELAY;
	      break;
	    }
	  case TX_OK:
	  case LINE_JAMMED:
	    if (tx->is_aun)
	    {
	      aun_tx_complete (state, tx->requestor_ip[0], tx->requestor_ip[1], tx->requestor_handle);
	    }
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
	    memcpy (rx->d.buf, ECONET_RX_BUF + 6, frame_length - 6);
	    rx->stn = src_stn;
	    rx->net = src_net;
	    rx->cb = cb;
	    rx->rx_port = port;
	    rx->state = RXCB_RECEIVED;
	  }
	}
	if (port != 0x9c)
	{
	  memcpy (uip_appdata + 6, ECONET_RX_BUF + 4, frame_length - 4);
	  aun_send_broadcast (cb, port, frame_length - 4);
	}
      }
      else if (should_bridge (dst, &ip_target))
      {
	if (port == 0)
	{
	  if ((cb & 0x7f) == Econet_MachinePeek)
	    aun_send_immediate (cb, ECONET_RX_BUF + 4, frame_length - 4);
	  return;
	}
	aun_cb = cb;
	aun_port = port;
	scout_buf[0] = src_stn;
	scout_buf[1] = src_net;
	scout_buf[2] = dst & 0xff;
	scout_buf[3] = dst >> 8;
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
	    make_scout_acknowledge ();
	    adlc_tx_frame (scout_buf, scout_buf + 4, 1);
	    adlc_ready_to_receive (RX_DATA);
	    return;
	  }
	}
      }
    }
    else if (adlc_state == (RX_DATA | FRAME_COMPLETE))
    {
      memcpy (uip_appdata + 8, ECONET_RX_BUF + 4, frame_length - 4);
      aun_send_packet (aun_cb, aun_port, ip_target, frame_length - 4);
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
    make_scout_acknowledge ();
    adlc_tx_frame (scout_buf, scout_buf + 4, 1);
  }

  adlc_ready_to_receive (RX_SCOUT);
}

void adlc_immediate_complete(uint8_t result, uint8_t *buffer, uint16_t length)
{
  if (result == TX_OK)
  {
    make_scout_acknowledge ();
    if (length > 12)
      length = 12;
    memcpy (scout_buf + 4, buffer, length);
    adlc_tx_frame (scout_buf, scout_buf + 4 + length, 1);
  }

  adlc_ready_to_receive (RX_SCOUT);
}
