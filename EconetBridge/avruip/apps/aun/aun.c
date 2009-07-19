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
#include <string.h>

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

static struct aun_state s;


#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3

#define ECO_NET 1
#define ETH_NET 130

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
	unsigned char pad;
	unsigned long handle;
};

unsigned char machine_type =  MACHINE_TYPE_RISC_PC;


/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */

static void newdata(void);
//static void check_entries(void);


//static struct uip_udp_conn *resolv_conn = NULL;



/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
aun_init(void)
{
  
  s.state = LISTENING;

	// remove any open connections
	if (s.conn != NULL) {
		uip_udp_remove(s.conn);
	}


	// listen on the AUN port from all IP addresses

	s.conn = uip_udp_new(NULL, HTONS(AUN_PORT));

	if (s.conn != NULL) {
		s.conn->lport = HTONS(AUN_PORT);
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
	
	serial_ok(0x7e);

  if(uip_udp_conn->rport == HTONS(AUN_PORT)) {

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

	serial_ok(m->mns_opcode);

	switch (m->mns_opcode)
	{
	case BROADCAST_DATA_FRAME:
		//
		break;
	case DATA_FRAME:
		//
		break;
	case DATA_FRAME_ACK:
		//
		break;
	case DATA_FRAME_REJ:
		//
		break;
	case IMMEDIATE_OP:
         if (m->mns_control == Econet_MachinePeek){
            do_immediate();
		}
		break;
	case IMMEDIATE_OP_REPLY:
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
//   mns.mns_rximmcnt++;

struct EcoDest 
{
u_char	pad[UIP_LLH_LEN];
u_char	Network;
u_char	Station;
};

//#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])
#define BUF ((struct EcoDest *)&uip_buf[UIP_LLH_LEN])

	struct mns_msg *m; 

	/* traditionally the network and station would be x.x.NET.STN of 
	   the IP address. This may not be correct, and the sender should
	   be queried as to its AUN MAP and what exactly this IP address 
	   maps to.
	*/

	// Get net and station

	// If in our routing table, machine peek the target

	// if available, send the original answer back to the orginator
/*
	serial_tx_hex(BUF->Network);
	serial_tx(0x20);
	serial_tx_hex(BUF->Station);

*/

	m = (struct mns_msg *)uip_appdata;

	m->mns_opcode	= IMMEDIATE_OP_REPLY;
	m->mns_machine	= machine_type;
	m->mns_pad		= 0;
	m->mns_release	= ((RELEASE_NBR % 10) | ((RELEASE_NBR / 10) << 4));
	m->mns_version	= (char) VERSION_NBR;

	uip_udp_send(sizeof(struct mns_msg));
   
   return;

} /* do_immediate() */

/*
static void 
check_entries(void)
{
	char i;
	i=1;
}

*/
