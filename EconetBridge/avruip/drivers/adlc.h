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

#ifndef __ASSEMBLER__

struct scout_packet
{
  unsigned char DStn;
  unsigned char DNet;
  unsigned char SStn;
  unsigned char SNet;
  unsigned char ControlByte;
  unsigned char Port;
};

extern void adlc_poller(void);
extern void adlc_ready_to_receive(void);
extern unsigned char send_packet(unsigned char*, unsigned short length);

#endif

#define	RX_IDLE		0
#define	RX_CHECK_NET1	1
#define	RX_CHECK_NET2	2
#define	RX_PAYLOAD	3
#define FRAME_COMPLETE  15

#define RX_SCOUT        (0 << 4)
#define RX_SCOUT_ACK    (1 << 4)
#define RX_DATA         (2 << 4)
#define RX_DATA_ACK     (3 << 4)

#define DISCONTINUED    64

// pin definitions
#define	ADLC_RS1		0			// PD0 Register Select 1			Econet Pin6
#define	ADLC_RnW		1			// PD1 Read/Write Control			Econet Pin2
#define	ADLC_RS0		4			// PD4 Register Select 0			Econet Pin5
#define	ADLC_nCE		0			// PE0						Econet Pin3
#define	ADLC_D0		2			// PE2						Econet Pin7

#define	EGPIO_ADLC_RESET		0x3			// ADLC Reset
#define	EGPIO_SET			0x80			// flag to set

#endif /* __ADLC_H__ */
/** @} */
/** @} */
