    /*******************************************************************\
    *                                                                   *
    *   Library         : lib_crc                                       *
    *   File            : lib_crc.h                                     *
    *   Author          : Lammert Bies  1999-2008                       *
    *   E-mail          : info@lammertbies.nl                           *
    *   Language        : ANSI C                                        *
    *                                                                   *
    *                                                                   *
    *   Description                                                     *
    *   ===========                                                     *
    *                                                                   *
    *   The file lib_crc.h contains public definitions  and  proto-     *
    *   types for the CRC functions present in lib_crc.c.               *
    *                                                                   *
    *                                                                   *
    *   Dependencies                                                    *
    *   ============                                                    *
    *                                                                   *
    *   none                                                            *
    *                                                                   *
    *                                                                   *
    *   Modification history                                            *
    *   ====================                                            *
    *                                                                   *
    *   Date        Version Comment                                     *
    *                                                                   *
    *   2010-10-20  1.17    Added several routines (JP)                 *
    *   2008-04-20  1.16    Added CRC-CCITT routine for Kermit          *
    *                                                                   *
    *   2007-04-01  1.15    Added CRC16 calculation for Modbus          *
    *                                                                   *
    *   2007-03-28  1.14    Added CRC16 routine for Sick devices        *
    *                                                                   *
    *   2005-12-17  1.13    Added CRC-CCITT with initial 0x1D0F         *
    *                                                                   *
    *   2005-02-14  1.12    Added CRC-CCITT with initial 0x0000         *
    *                                                                   *
    *   2005-02-05  1.11    Fixed bug in CRC-DNP routine                *
    *                                                                   *
    *   2005-02-04  1.10    Added CRC-DNP routines                      *
    *                                                                   *
    *   2005-01-07  1.02    Changes in tst_crc.c                        *
    *                                                                   *
    *   1999-02-21  1.01    Added FALSE and TRUE mnemonics              *
    *                                                                   *
    *   1999-01-22  1.00    Initial source                              *
    *                                                                   *
    \*******************************************************************/
#ifndef _LIB_CRC_H_
#define _LIB_CRC_H_

#include <stdint.h>

#define CRC_VERSION     "2.0"

uint16_t update_crc16_normal( uint16_t *table, uint16_t crc, char c );
uint16_t update_crc16_reflected( uint16_t *table, uint16_t crc, char c );
uint32_t update_crc24_normal( uint32_t *table, uint32_t crc, char c );
// update_crc24_reflected  same as update_crc32_reflected
uint32_t update_crc32_normal( uint32_t *table, uint32_t crc, char c );
uint32_t update_crc32_reflected( uint32_t *table, uint32_t crc, char c );

uint64_t update_crc40_normal( uint64_t *table, uint64_t crc, char c );
// update_crc40_reflected  same as update_crc64_reflected
uint64_t update_crc64_normal( uint64_t *table, uint64_t crc, char c );
uint64_t update_crc64_reflected( uint64_t *table, uint64_t crc, char c );

/* Does the CRC calculation 1 byte at a time */
uint8_t update_crc8_atm(uint8_t crc, uint8_t c);
uint8_t update_crc8_ccitt(uint8_t crc, uint8_t c);
uint8_t update_crc8_maxim(uint8_t crc, uint8_t c);
uint8_t update_crc8(uint8_t crc, uint8_t c);
uint8_t update_crc8_j1850(uint8_t crc, uint8_t c);
uint8_t update_crc8_wcdma(uint8_t crc, uint8_t c);
uint8_t update_crc8_rohc(uint8_t crc, uint8_t c);
uint8_t update_crc8_darc(uint8_t crc, uint8_t c);
uint16_t update_crc16_8005( uint16_t crc, char c );
uint16_t update_crc16_A001( uint16_t crc, char c );
uint16_t update_crc16_1021( uint16_t crc, char c );
uint16_t update_crc16_8408( uint16_t crc, char c );
uint16_t update_crc16_3D65( uint16_t crc, char c );
uint16_t update_crc16_dnp( uint16_t crc, char c );
uint16_t update_crc16_t10_dif( uint16_t crc, char c );
uint16_t update_crc16_0589( uint16_t crc, char c );
uint16_t update_crc16_teledisk( uint16_t crc, char c );
uint32_t update_crc24( uint32_t crc, char c );
uint32_t update_crc24_r64( uint32_t crc, char c );
uint32_t update_crc32_refl( uint32_t crc, char c );
uint32_t update_crc32_norm( uint32_t crc, char c );
uint32_t update_crc32_xfer( uint32_t crc, char c );
uint32_t update_crc32_c( uint32_t crc, char c );
uint32_t update_crc32_d( uint32_t crc, char c );
uint32_t update_crc32_k( uint32_t crc, char c );
uint32_t update_crc32_q( uint32_t crc, char c );
uint16_t update_crc_sick( uint16_t crc, char c, char prev_byte );

uint64_t update_crc40_gsm(uint64_t crc, char c);
uint64_t update_crc64(uint64_t crc, char c);
uint64_t update_crc64_1B(uint64_t crc, char c);
uint64_t update_crc64_jones(uint64_t crc, char c);

/* Does the CRC calculation over a string specified by length (allows 00 inside string) */
uint8_t calculate_crc8_itu(char *p, unsigned int length);
uint8_t calculate_crc8_atm(char *p, unsigned int length);
uint8_t calculate_crc8_ccitt(char *p, unsigned int length);
uint8_t calculate_crc8_maxim(char *p, unsigned int length);
uint8_t calculate_crc8(char *p, unsigned int length);
uint8_t calculate_crc8_icode(char *p, unsigned int length);
uint8_t calculate_crc8_j1850(char *p, unsigned int length);
uint8_t calculate_crc8_wcdma(char *p, unsigned int length);
uint8_t calculate_crc8_rohc(char *p, unsigned int length);
uint8_t calculate_crc8_darc(char *p, unsigned int length);

uint16_t calculate_crc16_Buypass(char *p, unsigned int length);
uint16_t calculate_crc16_DDS_110(char *p, unsigned int length);
uint16_t calculate_crc16_EN_13757(char *p, unsigned int length);
uint16_t calculate_crc16_Teledisk(char *p, unsigned int length);
uint16_t calculate_crc16(char *p, unsigned int length);
uint16_t calculate_crc16_Modbus(char *p, unsigned int length);
uint16_t calculate_crc16_Maxim(char *p, unsigned int length);
uint16_t calculate_crc16_USB(char *p, unsigned int length);
uint16_t calculate_crc16_T10_DIF(char *p, unsigned int length);
uint16_t calculate_crc16_Dect_X(char *p, unsigned int length);
uint16_t calculate_crc16_Dect_R(char *p, unsigned int length);
uint16_t calculate_crc16_sick(char *p, unsigned int length);
uint16_t calculate_crc16_DNP(char *p, unsigned int length);
uint16_t calculate_crc16_Ccitt_Xmodem(char *p, unsigned int length);
uint16_t calculate_crc16_Ccitt_FFFF(char *p, unsigned int length);
uint16_t calculate_crc16_Ccitt_1D0F(char *p, unsigned int length);
uint16_t calculate_crc16_Genibus(char *p, unsigned int length);
uint16_t calculate_crc16_Kermit(char *p, unsigned int length);
uint16_t calculate_crc16_X25(char *p, unsigned int length);
uint16_t calculate_crc16_MCRF4XX(char *p, unsigned int length);
uint16_t calculate_crc16_Riello(char *p, unsigned int length);
uint16_t calculate_chk16_Fletcher(char *p, unsigned int length);

uint32_t calculate_crc24_flexray_a(char *p, unsigned int length);
uint32_t calculate_crc24_flexray_b(char *p, unsigned int length);
uint32_t calculate_crc24_r64(char *p, unsigned int length);
uint32_t calculate_crc32(char *p, unsigned int length);
uint32_t calculate_crc32_jamcrc(char *p, unsigned int length);
uint32_t calculate_crc32_c(char *p, unsigned int length);
uint32_t calculate_crc32_d(char *p, unsigned int length);
uint32_t calculate_crc32_bzip2(char *p, unsigned int length);
uint32_t calculate_crc32_mpeg2(char *p, unsigned int length);
uint32_t calculate_crc32_posix(char *p, unsigned int length);
uint32_t calculate_crc32_k(char *p, unsigned int length);
uint32_t calculate_crc32_q(char *p, unsigned int length);
uint32_t calculate_crc32_xfer(char *p, unsigned int length);
uint64_t calculate_crc40_gsm(char *p, unsigned int length);
uint64_t calculate_crc64(char *p, unsigned int length);
uint64_t calculate_crc64_1b(char *p, unsigned int length);
uint64_t calculate_crc64_we(char *p, unsigned int length);
uint64_t calculate_crc64_jones(char *p, unsigned int length);

#endif /* _LIB_CRC_H_ */
