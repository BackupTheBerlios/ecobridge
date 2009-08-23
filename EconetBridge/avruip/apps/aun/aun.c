/**
 * \addtogroup AUN
 * @{
 */

/**
 * \file
 *         An example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Mark Usher
 */

/*
 * This is a short example of how to write uIP applications using
 * protosockets.
 */

/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "aun.h"
#include "uip.h"
#include "serial.h"
#include "adlc.h"
#include "econet.h"
#include "nic.h"

#include <string.h>

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

static struct aun_state s;

#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3

#define ECONET_INTERFACE_NET  1
#define ETHNET_INTERFACE_NET  130


// boolean flags for the routing table
#define NOT_ROUTABLE	0x0
#define ROUTABLE 	0xFF

#define LOCAL_NETWORK 	0

struct EconetRouting
{
u_char	pad[UIP_LLH_LEN];
u_char	Network;
u_char	Station;
};


struct Econet_Header {
	unsigned char DSTN;
	unsigned char DNET;
	unsigned char SSTN;
	unsigned char SNET;
	unsigned char CB;
	unsigned char PORT;
	unsigned char DATA1;
	unsigned char DATA2;
	unsigned char DATA3;
	unsigned char DATA4;
	unsigned char DATA5;
	unsigned char DATA6;
	unsigned char DATA7;
	unsigned char DATA8;
};


struct aunhdr
{
	unsigned char code;		/* AUN magic protocol byte */
	unsigned char port;
	unsigned char cb;
	unsigned char status;
	unsigned long handle;
};

uint8_t rTableEco[256];
uint32_t rTableEthIP[256];
uint8_t rTableEthType[256];

unsigned char machine_type =  MACHINE_TYPE_ARC;
uint8_t econet_net_nr = ECONET_INTERFACE_NET;

static void newdata(void);

static char forwarding;
static uint16_t fwd_timeout;

static void do_uip_send(void)
{
  uip_process(UIP_UDP_SEND_CONN);
  uip_arp_out();
  nic_send(NULL);
  uip_len = 0;
}

/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
aun_init(void)
{
	unsigned char i;

	rTableEco[LOCAL_NETWORK] = ROUTABLE;
	rTableEco[ANY_NETWORK] = ROUTABLE;
	rTableEco[ECONET_INTERFACE_NET] = ROUTABLE;

	// set the ethernet routing table
	rTableEthIP[ETHNET_INTERFACE_NET] = (uint32_t)(0x0201 | (uint32_t)((uint32_t)ETHNET_INTERFACE_NET << 16));
	rTableEthType[ETHNET_INTERFACE_NET] = ETH_TYPE_LOCAL;

	// remove any open connections
	if (s.conn != NULL) {
		uip_udp_remove(s.conn);
	}

	// listen on the AUN port from all IP addresses

	s.conn = uip_udp_new(NULL, HTONS(MNSDATAPORT));


	if (s.conn != NULL) {
		s.conn->lport = HTONS(MNSDATAPORT);
	}



}

void aun_poller(void)
{
  if (forwarding) {
    if (--fwd_timeout == 0) {
      serial_tx('T');
      adlc_forwarding_complete (NOT_LISTENING);
      forwarding = 0;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*
 * In AUN.h we have defined the UIP_APPCALL macro to
 * AUN_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void
aun_appcall(void)
{

// Handle TELNET packets

  if(uip_conn->lport == HTONS(23)) {
	telnetd_appcall();
	return;
  }

// Handle AUN Packets

  if(uip_udp_conn->rport == HTONS(MNSDATAPORT)) {

   if(uip_aborted())
   {}
   if(uip_timedout())
   {}
   if(uip_closed())
   {}
   if(uip_connected())
   {}
   if(uip_acked())
   {}
   if(uip_poll())
   {}
   if(uip_newdata())
   {
      newdata();
    }
   if (uip_rexmit() ||
            uip_acked() ||
            uip_connected() ||
            uip_poll())
    {}

  }

// Handle AUN ATP requests

  if(uip_udp_conn->rport == HTONS(MNSATPPORT)) {

   if(uip_aborted())
   {}
   if(uip_timedout())
   {}
   if(uip_closed())
   {}
   if(uip_connected())
   {}
   if(uip_acked())
   {}
   if(uip_poll())
   {}
   if(uip_newdata())
   {
      newdata();
    }
   if (uip_rexmit() ||
            uip_acked() ||
            uip_connected() ||
            uip_poll())
    {}

  }
	return;
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/** \internal
 * Called when new UDP data arrives.
 */
/*---------------------------------------------------------------------------*/
static void
newdata(void)
{

	struct mns_msg *m;

	m = (struct mns_msg *)uip_appdata;

	switch (m->mns_opcode)
	{
	case BROADCAST_DATA_FRAME: 	//1
		//
		break;
	case DATA_FRAME:		//2
		foward_packet();
		//
		break;
	case DATA_FRAME_ACK:		//3
	  serial_tx('A');
	  if (forwarding) {
		adlc_forwarding_complete (TX_OK);
		forwarding = 0;
	  }
		//
		break;
	case DATA_FRAME_REJ:		//4
	  serial_tx('N');
	  if (forwarding) {
		adlc_forwarding_complete (NOT_LISTENING);
		forwarding = 0;
	  }
		//
		break;
	case IMMEDIATE_OP:		//5
         if (m->mns_control == Econet_MachinePeek){
            do_immediate();
		}
		break;
	case IMMEDIATE_OP_REPLY:	//6
		//
		break;
	default:
		// unknown mns opcode in frame
		break;
	}

}


/******************************************************************************/

/******************************************************************************/

void do_immediate(void)
/*
 * Deal with a machine type peek received
 *
 * Parameters:
 *    handle : Tx CB handle
 *    from   : sockaddr_in for machine from whence received
 *
 * Returns:
 *    None
 */
{

	struct mns_msg *m;
	m = (struct mns_msg *)uip_appdata;

	unsigned char SNet, SStn;
	unsigned char DNet, DStn;

	/* traditionally the network and station would be x.x.NET.STN of
	   the IP address. This may not be correct, and the sender should
	   be queried as to its AUN MAP and what exactly this IP address
	   maps to.
	*/

	SNet = (unsigned char) *(uip_buf+28);	// octet 3 of source IP
	SStn = (unsigned char) *(uip_buf+29);	// octet 4 of source IP
	DNet = (unsigned char) *(uip_buf+32);	// octet 3 of destin IP
	DStn = (unsigned char) *(uip_buf+33);	// octet 4 of destin IP

	DNet = ECONET_INTERFACE_NET; // for testing

	/* if the destination econet network is the same as the network
	   definition on this network interface, change it to 0, the
	   local network */

	if (DNet == ECONET_INTERFACE_NET){
		DNet = LOCAL_NETWORK;
	}

	// is the station available?
	if (Econet_Peek(DNet,DStn)==0) {
		return;
	}

	// if station found, send a reply back to the orginator
	if ( DNet != LOCAL_NETWORK ) {
		rTableEco[DNet] = ROUTABLE;
	}

	static uip_ipaddr_t tmpip;
	uip_ipaddr_copy(tmpip, BUF->srcipaddr);
	uip_ipaddr_copy(BUF->srcipaddr, BUF->destipaddr);
	uip_ipaddr_copy(BUF->destipaddr, tmpip);

	m->mns_opcode = IMMEDIATE_OP_REPLY;
	// all the codes inbetween are set from the incoming packet
	m->mns_machine = machine_type;
	m->mns_pad = 0;
	m->mns_release = ((RELEASE_NBR % 10) | ((RELEASE_NBR / 10) << 4));
	m->mns_version	= (char) VERSION_NBR;

	uip_send(uip_appdata,12);


   return;

} /* do_immediate() */



/******************************************************************************/

/******************************************************************************/

void do_atp(unsigned char *Net, unsigned char *Stn)

{

	struct atp_block *atp;


	atp = ((struct atp_block *)&uip_buf[UIP_LLH_LEN]);

	atp->atpb_net = *Net;
	atp->atpb_station = *Stn;


}

/******************************************************************************/

/******************************************************************************/

void foward_packet(void)

{
	// Get net and station

	unsigned char DNet, DStn;
	unsigned char SNet, SStn;
	unsigned short buf_len;

	buf_len = uip_len;

	struct aunhdr *ah;
	struct Econet_Header *eh;

	SNet = (unsigned char) *(uip_buf+28);	// octet 3 of source IP
	SStn = (unsigned char) *(uip_buf+29);	// octet 4 of source IP
	DNet = (unsigned char) *(uip_buf+32);	// octet 3 of destin IP
	DStn = (unsigned char) *(uip_buf+33);	// octet 4 of destin IP

	/* if the destination econet network is the same as the network
	   definition on this network interface, change it to 0, the
	   local network */

	if (DNet == ECONET_INTERFACE_NET) {
		DNet = LOCAL_NETWORK;
	}

//	DStn = 254;			// and fix the destination as Station 0x4
	SNet = ETHNET_INTERFACE_NET;


	// use Econet header structure instead of scout packet structure
	// so the Port byte can easily be duplicated.
	ah = (struct aunhdr *)(uip_appdata);
	eh = (struct Econet_Header *)(uip_appdata+2);

	uint32_t handle = ah->handle;
	uint8_t port = ah->port;

	eh->DSTN = DStn;
	eh->DNET = DNet;
	eh->SSTN = SStn;
	eh->SNET = SNet;
	eh->CB = 0x80;
	eh->PORT = port;

	int x;
	struct mbuf *mb = copy_to_mbufs (eh, buf_len - 2);
	x = enqueue_aun_tx(mb, BUF, handle);

	return;
}

/*
aun_send_immediate() works in the same way as aun_send_packet() but, obviously, 
for immediate ops. You need to call back to adlc_immediate_complete(), again 
with either TX_OK or NOT_LISTENING.  If the result was TX_OK, and there was data 
returned (i.e. from machine peek) then you need to supply a buffer and length as well. 
*/
void aun_send_immediate (struct scout_packet *s, uint32_t dest_ip, uint16_t data_length)
{
  adlc_immediate_complete (NOT_LISTENING, NULL, 0);
}


void aun_send_packet (uint8_t cb, uint8_t port, uint16_t src_stn_net, uip_ipaddr_t dest_ip, uint16_t data_length)
{
  struct aunhdr *ah;
  ah = (struct aunhdr *)(uip_appdata);

  ah->code = DATA_FRAME;
  ah->port = port;
  ah->cb = cb;
  ah->status = 0;
  ah->handle = ++s.handle;

  uint8_t src_stn = src_stn_net & 0xff;
  uint8_t src_net = src_stn_net >> 8;
  if (src_net == 0)
    src_net = ECONET_INTERFACE_NET;

  BUF->srcipaddr[0] = uip_hostaddr[0];
  BUF->srcipaddr[1] = (src_stn << 8) | src_net;
  uip_ipaddr_copy(BUF->destipaddr, dest_ip);
  BUF->destport = BUF->srcport = HTONS(MNSDATAPORT);

  uip_udp_send((int)uip_appdata + 8 + data_length - (int)uip_buf);
  do_uip_send();

  forwarding = 1;
  fwd_timeout = 20000;
}


/* 
   aun_send_broadcast() is for broadcast operations.
   There's no acknowledgement for these, you just need to send them off.
*/

void aun_send_broadcast (struct scout_packet *s, uint16_t data_length)
{
}


void aun_tx_complete (int8_t status, struct tx_record *tx)
{

  struct aunhdr *ah;
  uip_appdata = &uip_buf[UIP_IPUDPH_LEN + UIP_LLH_LEN];
  ah = (struct aunhdr *)(uip_appdata);

  uip_ipaddr_copy (BUF->destipaddr, tx->requestor_ip);
  uip_ipaddr_copy (BUF->srcipaddr, tx->target_ip);

  memset (ah, 0, sizeof (*ah));
  ah->code = (status == TX_OK) ? DATA_FRAME_ACK : DATA_FRAME_REJ;
  ah->handle = tx->requestor_handle;

  uip_udp_send((int)uip_appdata + 8 - (int)uip_buf);
  do_uip_send();
}

uint8_t aun_want_proxy_arp(uint16_t *ipaddr)
{
  if (ipaddr[0] == 0x0201
      && rTableEco[ipaddr[1] & 0xff])
    return 1;
  return 0;
}

/*
static void
check_entries(void)
{
	char i;
	i=1;
}

*/
