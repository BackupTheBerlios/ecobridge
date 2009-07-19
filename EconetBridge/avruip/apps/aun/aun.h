/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup helloworld Hello, world
 * @{
 *
 * A small example showing how to write applications with
 * \ref psock "protosockets".
 */

/**
 * \file
 *         Header file for AUN networking
 *         with protosockets.
 * \author
 *         Mark Usher
 */

#ifndef __AUN_H__
#define __AUN_H__

#define AUN_PORT 32768


/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */

struct aun_state {
  struct uip_udp_conn *conn;
  char state;
};

#define NOT_LISTEING	0
#define LISTENING		1

//dummy as not using tcp at the moment
typedef int uip_tcp_appstate_t;

typedef struct aun_state uip_udp_appstate_t;


/* Finally we define the application function to be called by uIP. */
void aun_appcall(void);

#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL aun_appcall
#endif /* UIP_APPCALL */

//dummy for tcp
#ifndef UIP_APPCALL
#define UIP_APPCALL aun_appcall
#endif /* UIP_APPCALL */


#include "uipopt.h"

void aun_init(void);


typedef unsigned char u_char;

typedef unsigned int u_int;

typedef unsigned short u_short;
typedef unsigned long u_long;


/* module.h
 *
 * Author: Keith Ruttle (Acorn)
 *
 * Description
 * ===========
 * Main Net module header file
 */


/* machine types returned by Econet_PeekMachine SWI */
#define MACHINE_TYPE_ARC      7
#define MACHINE_TYPE_RISC_PC 15
#define MACHINE_TYPE_BRIDGE  16

extern unsigned char machine_type;

#define VERSION_NBR       6
#define RELEASE_NBR       18     /* NB: Match with module version number in
                                  * CMHG file - curr 6.18 */

/* The minimum version we can cope with. We need a later version (403) for
 * certain features, but not all use
 */

#define MINIMUM_DCI_VERSION 401

#define STARTUP_TIMEOUT     10

#define DFT_SEGSIZE        8192
#define MAX_SEGSIZE       12288

#define MNS_HANDLE_BASE   2
#define MNS_HANDLE_INCR   4

#define RESERVED_NET    128
#define ITEM_NAMELEN     16
#define BRIDGE_PORT    0x9c

#define MAX_ALLOCATE   0x90

#define ALL_ONES         0xffffffff

#define Err_BadPort       1   /* Bad port number */
#define Err_TooBig        2   /* Bad buffer size */
#define Err_PtNtAlc       3   /* Port not already allocated */
#define Err_PortAlc       4   /* Port not unallocated */
#define Err_NoPorts       5   /* All ports allocated */
#define Err_NoMem         6   /* no store */
#define Err_BadSWI        7   /* SWI number out of range */
#define Err_BadMask       8   /* Bad mask */
#define Err_NoHware       9   /* No Econet hardware */
#define Err_Fatal        10   /* Fatal error */
#define Err_BadStn       11   /* Bad station number */
#define Err_BadNet       12   /* Bad net number */
#define Err_Channel      13   /* Bad handle */
#define Err_NotConf      14   /* AUN is not configured */

struct eblk {
    int   err_nbr;
    char *err_token;
};

#define Str_TxOk     0
#define Str_NotAcc   1
#define Str_FullAd   2
#define Str_Noclck   3
#define Str_IfType   4
#define Str_Down     5
#define Str_StaNum   6
#define Str_BrdCst   7
#define Str_GlbBct   8
#define Str_AccNet   9
#define Str_ModSts  10
#define Str_TxStat  11
#define Str_Data    12
#define Str_Immedt  13
#define Str_ImmRep  14
#define Str_Retry   15
#define Str_Error   16
#define Str_DtaAck  17
#define Str_DtaRej  18
#define Str_Local   19
#define Str_Global  20
#define Str_Dscard  21
#define Str_RxStat  22
#define Str_InvRep  23
#define Str_MNSBan  24
#define Str_Gteway  25
#define Str_Gtewys  26
#define Str_Warn    27
#define Str_NoMap   28
#define Str_NoRout  29
#define Str_GwConf  30
#define Str_BadSta  31
#define Str_BadDev  32
#define Str_BadInet 33
#define Str_NtvEco  34
#define Str_BadGwy  35
#define Str_GtwSta  36

/************************************************************************88****/

extern void *module_wsp;
extern int restarting;

/* MessageTrans file descriptors */
extern char msg_fd_eco[16];
extern char msg_fd_mns[16];

struct s_ebuf
{
   int  e_nbr;
   char e_string[36];
};

extern struct s_ebuf ebuf;

extern char textbuf[64];

/******************************************************************************/


struct mns
{
   int             mns_flags;
#define MNS_SOCKET             01
#define MNS_SEEKADDR           02
#define MNS_WANTMAP            04
#define MNS_WANTROUTE         010
#define MNS_DSINUSE           020
#define MNS_GATEWAY           040
#define MNS_MAPISSET         0100
#define MNS_SETADDR          0200
#define MNS_ALLSET           0400
   int              mns_atpsock;
   int              mns_routedsock;
   int              mns_rxdsock;
   int              mns_txdsock;
   int              mns_stationnumber;
   int              mns_randcount;
   int              mns_retrydelay;
   int              mns_retrycount;
#define RETRY_DELAY     100
#define RETRY_COUNT     2
   int              mns_nextport;
   u_char           mns_netnumber;
   u_char           mns_econetnumber;
   u_long           mns_econetipadr;
   u_long           mns_serverip;
   int              mns_ifcnt;
   char             mns_device[6];
   char             mns_module[24];
   u_long           mns_ifaddrs[4];
   int              mns_timer;
   u_int            mns_txhandle;
   u_int            mns_rxhandle;
   int              mns_segsize;
   struct address_q *mns_mapq;
   struct rxcb      *mns_rxlist;
   struct rxcb      *mns_rxwild;
   struct rxcb      *mns_rxfree;
   struct txcb      *mns_txlist;
   struct txcb      *mns_txfree;
   char             *mns_route;
   u_char   mns_states[256];
#define PORTALLOCATED        01
#define PORTCLAIMED          02
#define PORTRESERVED         04
#define ECONET_IS_CONNECTED 010
#define MNS_IS_CONNECTED    020
   /* stats counters */
   int              mns_txcnt;
   int              mns_txerrs;
   int              mns_txrej;
   int              mns_txacks;
   int              mns_tximmcnt;
   int              mns_tximmrcnt;
   int              mns_txretry;
   int              mns_txbccnt;
   int              mns_txlbc;
   int              mns_txgbc;
   int              mns_rxcnt;
   int              mns_rxacks;
   int              mns_rxrej;
   int              mns_rxdiscard;
   int              mns_rximmcnt;
   int              mns_rximreply;
   int              mns_rxackdiscard;
   int              mns_rxerrs;
   int              mns_rxbc;
   int              mns_rxretries;
};

extern struct mns mns;

/*
 *  MNS message format
 */

struct mns_msg
{
   u_char  mns_opcode;
   u_char  mns_port;
   u_char  mns_control;
#define Econet_MachinePeek        8
   u_char  mns_status;
#define MSG_IS_RETRY    01
#define MSG_IS_DATAGRAM 02
   u_int   mns_handle;
   u_char	mns_machine;
   u_char	mns_pad;
   u_char	mns_release;
   u_char   mns_version;
   u_short	mns_padend;
};

#define UNHDRSIZE  8

/*
 * Data Transform Protocol format
 *    ______________
 *   |opcode| count |
 *   |--------------|
 *   |  net |station|
 *   |--------------|
 *   |IP address blk|
 *   |______________|
 */

struct atp_block
{
   u_char   atpb_net;
   u_char   atpb_station;
   u_long   atpb_ipadr;
   char     atpb_sitename[ITEM_NAMELEN];
   char     atpb_netname[ITEM_NAMELEN];
};

struct atp_msg
{
   u_char    atp_opcode;
#define MNS_TRANSFORM_REQUEST       1
#define MNS_TRANSFORM_REPLY         2
   u_char   atp_count;
   u_char   atp_address;
};

struct address_q
{
   struct address_q  *q_next;
   u_char             q_net;
   u_char             q_station;
   u_long             q_ip;
   u_long             q_bcast;
   char               q_sitename[ITEM_NAMELEN];
   char               q_netname[ITEM_NAMELEN];
};


#define EventV               16
#define TickerV              28
#define Event_Econet_Rx      14
#define Event_Econet_Tx      15

#define Internet_Event       19
#define SocketIO              1
#define RarpReply             4
#define Event_Enable         14
#define Event_Disable        13

#define ANY_PORT    255
#define ANY_STATION 255
#define ANY_NETWORK 255

#define MNSDATAPORT    0x8000
#define MNSATPPORT     0x8001
#define ROUTEDPORT     520

#define BROADCAST_DATA_FRAME       1
#define DATA_FRAME                 2
#define DATA_FRAME_ACK             3
#define DATA_FRAME_REJ             4
#define IMMEDIATE_OP               5
#define IMMEDIATE_OP_REPLY         6

/*
 * cb template for list processing
 */
struct cbh
{
   struct cbh   *cb_next;
   u_int         cb_handle;
};

/*
 * Control blocks
 */

struct rxcb
{
   struct rxcb   *rx_next;
   u_int         rx_handle;
   u_char        rx_port;
   int           rx_station;
   int           rx_network;
   char          *rx_buf;
   int           rx_bufsize;
   int           rx_status;
#define Status_Free        -1
#define Status_RxReady      7
#define Status_Receiving    8
#define Status_Received     9
#define Status_NoReply     10
#define Status_Escape      11
#define Status_EcoReceived 12
#define Status_Dead        13
#define Status_Init        14
#define Status_EventPend   15
   int           rx_delay;
   int           rx_flag;
   int           rx_count;
   int           rx_ecotype;
   int           rx_wait;
};


struct txcb
{
   struct txcb   *tx_next;
   u_int         tx_handle;
   int           tx_status;
#define Status_Transmitted  0
#define Status_NetError     2
#define Status_NotListening 3
#define Status_TxReady      5
#define Status_Transmitting 6
   int           tx_control;
   int           tx_port;
   int           tx_network;
   int           tx_station;
   u_long        tx_dst;
   char          *tx_buf;
   int           tx_bufsize;
   int           tx_type;
   char          tx_ecotype;
   char          tx_callb;
   int           tx_rtcount;
   int           tx_rtmax;
   int           tx_rtdelay;
#define TX_RETRY_DELAY   500
   int           tx_rjcount;
   int           tx_rjmax;
   int           tx_rjdelay;
   int           tx_rjdelayval;
   int           tx_flag;
#define TX_WAIT            01
};





#endif /* __AUN_H__ */
/** @} */
/** @} */
