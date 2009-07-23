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


void serial_short(unsigned short);
void serial_shortLH(unsigned short);
void serial_ok(unsigned char);
void serial_rx(void);
void serial_txx(void);
void serial_eth(void);
void serial_eco(void);
void serial_error(unsigned char);
void serial_crlf(void);
void serial_packet(unsigned short, unsigned short);


#endif /* __ADLC_H__ */
/** @} */
/** @} */
