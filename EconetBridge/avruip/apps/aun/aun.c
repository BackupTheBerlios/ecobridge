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
#include <string.h>

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

static struct aun_state s;

	#define BUF ((struct EcoDest *)&uip_buf[UIP_LLH_LEN])



#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3

#define ECONET_INTERFACE_NET	1
#define ETHNET_INTERFACE_NET  7


// boolean flags for the routing table
#define NOT_ROUTABLE	0
#define ROUTABLE 	1

#define LOCAL_NETWORK 	0

struct EcoDest 
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
	unsigned char pad;
	unsigned long handle;
};


struct RoutingTable
{
	unsigned char flag;
};


volatile static struct RoutingTable rTableEco[255];


unsigned char machine_type =  MACHINE_TYPE_ARC;


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

	// initialise econet routing table
	for (i=1; i<255; i++) {
		rTableEco[i].flag = 0;		// clear the table
	}

	rTableEco[LOCAL_NETWORK].flag = ROUTABLE;
	rTableEco[ANY_NETWORK].flag = ROUTABLE;
	rTableEco[ECONET_INTERFACE_NET].flag = ROUTABLE;
	

	s.state = LISTENING;

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
	
  if(uip_udp_conn->rport == HTONS(MNSDATAPORT)) {

serial_eth();
serial_rx();
serial_packet(uip_buf+42, 12);
//serial_packet(uip_appdata, 12);


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
		//
		break;
	case DATA_FRAME_REJ:		//4
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

	unsigned char Net, Stn;

	/* traditionally the network and station would be x.x.NET.STN of 
	   the IP address. This may not be correct, and the sender should
	   be queried as to its AUN MAP and what exactly this IP address 
	   maps to.
	*/

	// Get net and station

//	Net = (BUF->Network);
	Net = ECONET_INTERFACE_NET;	// temporarily set the network to our network
					// for testing
	Stn = (BUF->Station);
	

	// If in our routing table, machine peek the target
	if (rTableEco[Net].flag == NOT_ROUTABLE) {
		return;
	}

	if (Econet_Peek(&Net,&Stn)==0) {
		return;
	};

	// if available, send the answer back to the orginator

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
	struct scout_packet *m; 
	unsigned short buf_len;

	#define BUF ((struct EcoDest *)&uip_buf[UIP_LLH_LEN])
	
	buf_len = uip_len - UIP_LLH_LEN;

//	Net = (BUF->Network);
	DNet = ECONET_INTERFACE_NET;	// temporarily set the network to our network
					// for testing
	DStn = 4;			// and fix the destination as Station 0x4
	SStn = (BUF->Station);
	SNet = ETHNET_INTERFACE_NET;

	m = (struct scout_packet *)(uip_appdata-4);
	
	m->DStn = DStn;
	m->DNet = 0;
	m->SStn = SStn;
	m->SNet = SNet;
	m->ControlByte = 0x80;




	unsigned char x;
	x = send_packet(m, buf_len+6);

	return;
}

/*
static void 
check_entries(void)
{
	char i;
	i=1;
}

*/
