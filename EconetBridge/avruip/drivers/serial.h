/**
 * \addtogroup drivers
 * @{
 */

/**
 * \defgroup Serial
 * @{
 *
 */

/**
 * \file
 *         Header file for Serial routines
 *         
 * \author
 *         Mark Usher
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>

extern void serial_short(unsigned short);
extern void serial_shortLH(unsigned short);
extern void serial_ok(unsigned char);
extern void serial_rx(void);
extern void serial_txx(void);
extern void serial_eth(void);
extern void serial_eco(void);
extern void serial_error(unsigned char);
extern void serial_crlf(void);
extern void serial_packet(unsigned short, unsigned short);
extern void serial_tx_str(char *msg);

extern void serial_tx(uint8_t mask);
extern void serial_tx_hex(uint8_t mask);

#endif /* __ADLC_H__ */
/** @} */
/** @} */
