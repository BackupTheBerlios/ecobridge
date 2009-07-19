/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup ADLC ADLC_ wrapper
 * @{
 *
 */

/**
 * \file
 *         Header file for ADLC driver
 *         
 * \author
 *         Mark Usher
 */

#ifndef __ADLC_H__
#define __ADLC_H__

#include <stdint.h>

//extern unsigned char *ECONET_RX_BUF;		/* The Econet Rx Buf pointer points to received data. */

extern unsigned char ECONET_RX_BUF[2000];		/* The Econet Rx Buf pointer points to received data. */


extern unsigned short	adlc_rx_ptr;
extern unsigned short	adlc_r24_ptr;
extern unsigned char	adlc_state;

void	adlc_poller(void);
void	adlc_init2(void);


#define	RX_IDLE		0
#define	RX_CHECK_NET1	1
#define	RX_CHECK_NET2	2
#define	RX_DATA		3
#define	RX_SCOUT_ACK1	16
#define	RX_SCOUT_ACK2	17
#define	FRAME_COMPLETE	32
#define	PACKET_Rx		1


#endif /* __ADLC_H__ */
/** @} */
/** @} */
