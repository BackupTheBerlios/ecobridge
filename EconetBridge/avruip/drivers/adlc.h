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


struct scout_packet
{
  unsigned char DStn;
  unsigned char DNet;
  unsigned char SStn;
  unsigned char SNet;
  unsigned char ControlByte;
  unsigned char Port;
};

void	adlc_poller(void);

#define	ADLC_RS1		0			// PD0 Register Select 1			Econet Pin6
#define	ADLC_RnW		1			// PD1 Read/Write Control			Econet Pin2
#define	ADLC_RS0		4			// PD4 Register Select 0			Econet Pin5
#define	ADLC_nCE		0			// PE0						Econet Pin3
#define	ADLC_D0		2			// PE2						Econet Pin7

#define	RX_IDLE		0
#define	RX_CHECK_NET1	1
#define	RX_CHECK_NET2	2
#define	RX_DATA		3
#define	RX_SCOUT_ACK1	16
#define	RX_SCOUT_ACK2	17
#define	FRAME_COMPLETE	32
#define	PACKET_Rx		1

extern void adlc_ready_to_receive(void);
extern unsigned char send_packet(unsigned char*, unsigned short length);

#endif /* __ADLC_H__ */
/** @} */
/** @} */
