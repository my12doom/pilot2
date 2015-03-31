/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: parser.h 4 2007-08-27 13:11:03Z xtimor $
 *
 */

#ifndef __NMEA_PARSER_H__
#define __NMEA_PARSER_H__

#include "info.h"
#include "sentence.h"

#ifdef  __cplusplus
extern "C" {
#endif

/*
 * high level
 */

typedef struct _nmeaPARSER
{
    nmeaGPGGA GPGGA;
    nmeaGPGSA GPGSA;
    nmeaGPGSV GPGSV;
    nmeaGPRMC GPRMC;
    nmeaGPVTG GPVTG;
	nmeaGPZDA GPZDA;
} nmeaPARSER;

int     nmea_parser_init(nmeaPARSER *parser);
void    nmea_parser_destroy(nmeaPARSER *parser);

int     nmea_parse(
        nmeaPARSER *parser,
        const char *buff, int buff_sz,
        nmeaINFO *info
        );

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_PARSER_H__ */
