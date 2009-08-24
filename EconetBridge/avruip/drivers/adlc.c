#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "adlc.h"
#include "aun.h"
#include "serial.h"
#include "uip.h"
#include "mbuf.h"
#include "internet.h"

#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define MAX_TX	8
#define ADLC_TX_RETRY_COUNT	16
#define ADLC_TX_RETRY_DELAY	128

unsigned char ECONET_RX_BUF[ECONET_RX_BUF_SIZE];
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

#define NORMAL_PACKET	0
#define BROADCAST	1
#define IMMEDIATE	2

extern uint16_t get_adlc_rx_ptr(void);

static uint8_t get_tx_buf(void)
{
  uint8_t i;
  for (i = 0; i < MAX_TX; i++)
  {
    if (tx_buf[i].mb == NULL)
      return i;
  }
  return 0xff;
}

extern uint8_t get_adlc_state(void);
extern uint8_t adlc_await_idle(void);
extern void adlc_tx_frame(struct mbuf *mb, unsigned char type);

static uip_ipaddr_t ip_target;
static uint8_t aun_cb, aun_port;

struct scout_mbuf
{
  struct mbuf *next, *prev;
  uint8_t length;
  uint8_t pad;
  uint8_t data[16];
};

static struct scout_mbuf scout_mbuf;

static uint8_t do_tx_packet(struct tx_record *tx)
{
  unsigned char *buf = tx->mb->data;
  unsigned char type = NORMAL_PACKET;
  if (buf[0] == 0xff)
    type = BROADCAST;
  else if (buf[5] == 0)
    type = IMMEDIATE;

  stats.tx_attempts++;

  if (adlc_await_idle()) {
    stats.tx_line_jammed++;
    return LINE_JAMMED;
  }

  if (type == BROADCAST) {
    adlc_tx_frame (tx->mb, 1);
    adlc_ready_to_receive_scout();
    return TX_OK;
  }

  if (type == IMMEDIATE)
    adlc_tx_frame (tx->mb, 1);
  else {
    memcpy (scout_mbuf.data, tx->mb->data, 6);
    scout_mbuf.length = 6;
    adlc_tx_frame ((struct mbuf *)&scout_mbuf, 1);
  }

  unsigned char state;
  do {
    state = get_adlc_state();
  } while (state != RX_IDLE && state != (RX_SCOUT_ACK | FRAME_COMPLETE));

  if (state == RX_IDLE) {
    adlc_ready_to_receive_scout();
    return NOT_LISTENING;
  }

  if (type == NORMAL_PACKET) {
    adlc_tx_frame (tx->mb, 0);

    do {
      state = get_adlc_state();
    } while (state != RX_IDLE && state != (RX_DATA_ACK | FRAME_COMPLETE));
  }

  adlc_ready_to_receive_scout();

  return ((state & 0xf) == FRAME_COMPLETE) ? TX_OK : NET_ERROR;
}

uint8_t enqueue_tx(struct mbuf *mb)
{
  uint8_t i = get_tx_buf();
  if (i != 0xff)
  {
    struct tx_record *tx = &tx_buf[i];
    tx->retry_count = ADLC_TX_RETRY_COUNT;
    tx->retry_timer = 0;
    tx->mb = mb;
    tx->is_aun = 0;
    return i + 1;
  }
  serial_tx_str ("no bufs!\n");
  return 0;
}

uint8_t enqueue_aun_tx(struct mbuf *mb, struct uip_tcpip_hdr *hdr, uint32_t handle)
{
  uint8_t i = enqueue_tx(mb);
  if (i) {
    struct tx_record *tx = &tx_buf[i-1];
    tx->is_aun = 1;
    tx->requestor_handle = handle;
    uip_ipaddr_copy (tx->requestor_ip, hdr->srcipaddr);
    uip_ipaddr_copy (tx->target_ip, hdr->destipaddr);
    tx->retry_count = 0;
  }
  return i;
}

static uint8_t should_bridge(struct scout_packet *s, uint32_t *ip_targetp)
{

  /* if destination is reachable fill ip_target address */

  if (rTableEthType[s->DNet] != 0)
  {
    *ip_targetp = rTableEthIP[s->DNet] | ((unsigned long)(s->DStn) << 24);
    return 1;
  }
  else {
    return 0;
  }

}

static void make_scout_acknowledge(void) __attribute__ ((noinline));

static void make_scout_acknowledge(void)
{
  scout_mbuf.data[0] = ECONET_RX_BUF[2];
  scout_mbuf.data[1] = ECONET_RX_BUF[3];
  scout_mbuf.data[2] = ECONET_RX_BUF[0];
  scout_mbuf.data[3] = ECONET_RX_BUF[1];
}

static void make_and_send_scout(void)
{
  make_scout_acknowledge ();
  scout_mbuf.length = 4;
  adlc_tx_frame ((struct mbuf *)&scout_mbuf, 1);
}

static void do_local_immediate (uint8_t cb)
{
  switch (cb & 0x7f)
  {
  case Econet_MachinePeek:
    make_scout_acknowledge ();
    scout_mbuf.data[4] = MACHINE_TYPE;
    scout_mbuf.data[5] = MACHINE_VENDOR;
    scout_mbuf.data[6] = MACHINE_VER_LOW;
    scout_mbuf.data[7] = MACHINE_VER_HIGH;
    scout_mbuf.length = 8;
    adlc_tx_frame ((struct mbuf *)&scout_mbuf, 1);
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
      if (tx->mb)
      {
	if (tx->retry_timer == 0)
	{
	  uint8_t state = do_tx_packet (tx);
	  switch (state) {
	  default:
	    if (tx->retry_count--) {
	      tx->retry_timer = ADLC_TX_RETRY_DELAY;
	      break;
	    }
	  case TX_OK:
	  case LINE_JAMMED:
	    if (tx->is_aun)
	    {
	      if (tx->mb->data[0] != 0xff)
		aun_tx_complete (state, tx);
	    }
	    mbuf_free_chain(tx->mb);
	    tx->mb = NULL;
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
    uint16_t frame_length = get_adlc_rx_ptr() - (int)ECONET_RX_BUF;
    stats.frames_in++;
    if (adlc_state == (RX_SCOUT | FRAME_COMPLETE))
    {
      if (frame_length < 6) {
        stats.short_scouts++;
	goto out;
      }
      struct scout_packet *s = (struct scout_packet *)ECONET_RX_BUF;
      if (s->DStn == 0xff)
      {
        stats.rx_bcast++;
	if (s->Port == 0xb0)
	  handle_port_b0 ();
	if (s->Port == 0x9c)
	  handle_port_9c ();
	else
	{
	  memcpy (uip_appdata + 8, ECONET_RX_BUF + 6, frame_length - 6);
	  aun_send_broadcast (s, frame_length - 6);
	}
      }
      else if (should_bridge (s, &ip_target[0]))
      {
	if (s->Port == 0)
	{
	  aun_send_immediate (s, ECONET_RX_BUF + 6, frame_length - 6);
	  return;
	}
      do_rx_data:
	aun_cb = s->ControlByte;
	aun_port = s->Port;
	make_and_send_scout ();
	adlc_ready_to_receive (RX_DATA);
	return;
      }
      else if (s->DStn == eeGlobals.Station && s->DNet == 0)
      {
	if (s->Port == 0)
	{
	  do_local_immediate (s->ControlByte);
	}
	else if (s->Port == 0xd2)
	{
	  goto do_rx_data;
	}
      }
    }
    else if (adlc_state == (RX_DATA | FRAME_COMPLETE))
    {
      if (aun_port == 0xd2)
      {
	handle_ip_packet (aun_cb, frame_length);
	make_and_send_scout ();
	goto out;
      }
      if ((frame_length - 4) > (UIP_BUFSIZE - ((char *)uip_appdata - (char *)uip_buf)))
      {
	serial_tx_str ("too big!");
	serial_crlf();
      }
      else
      {
	memcpy (uip_appdata + 8, ECONET_RX_BUF + 4, frame_length - 4);
	aun_send_packet (aun_cb, aun_port, *((uint16_t *)(ECONET_RX_BUF + 2)), ip_target, frame_length - 4);
	adlc_state = BUSY_FORWARDING;
	return;
      }
    }
  out:
    adlc_ready_to_receive_scout();
  }
}

void adlc_forwarding_complete(uint8_t result, uint8_t *buffer, uint8_t length)
{
  if (result == TX_OK)
  {
    make_scout_acknowledge ();
    if (length > 12)
      length = 12;
    memcpy (&scout_mbuf.data[4], buffer, length);
    scout_mbuf.length = 4 + length;
    adlc_tx_frame ((struct mbuf *)&scout_mbuf, 1);
  }

  adlc_ready_to_receive_scout();
}
