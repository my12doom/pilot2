/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: parser.c 17 2008-03-11 11:56:11Z xtimor $
 *
 */

/**
 * \file parser.h
 */

#include "tok.h"
#include "parse.h"
#include "parser.h"
#include "context.h"

#include <string.h>
#include <stdlib.h>

typedef struct _nmeaParserNODE
{
    int packType;
    void *pack;
    struct _nmeaParserNODE *next_node;

} nmeaParserNODE;

/*
 * high level
 */

/**
 * \brief Initialization of parser object
 * @return true (1) - success or false (0) - fail
 */
int nmea_parser_init(nmeaPARSER *parser)
{
    int resv = 0;
    int buff_size = nmea_property()->parse_buff_size;

    NMEA_ASSERT(parser);
    return resv;
}

/**
 * \brief Destroy parser object
 */
void nmea_parser_destroy(nmeaPARSER *parser)
{
    NMEA_ASSERT(parser);
    memset(parser, 0, sizeof(nmeaPARSER));
}

/**
 * \brief Analysis of buffer and put results to information structure
 * @return Number of packets wos parsed
 */
int nmea_parse(nmeaPARSER *parser, const char *buff, int buff_sz, nmeaINFO *info)
{
    int nparsed = 0, crc, sen_sz, ptype, buff_use = buff_sz;
    int nsentence = 0;

    NMEA_ASSERT(parser && parser->buffer);

    /* parse */
    for(;;)
    {
        sen_sz = nmea_find_tail(
            buff + nparsed,
            buff_use - nparsed, &crc);

        if(!sen_sz)
            break;
                
        else if(crc >= 0)
        {
            ptype = nmea_pack_type(
                (const char *)buff + nparsed + 1,
                buff_use - nparsed - 1);

            switch(ptype)
            {
            case GPGGA:
                if(nmea_parse_GPGGA(
                    buff + nparsed,
                    sen_sz, &parser->GPGGA))
                {
                    nmea_GPGGA2info(&parser->GPGGA, info);
                }
                break;
            case GPGSA:
                if(nmea_parse_GPGSA(
                    buff + nparsed,
                    sen_sz, &parser->GPGSA))
                {
                    nmea_GPGSA2info(&parser->GPGSA, info);
                }
                break;
            case GPGSV:
                if(nmea_parse_GPGSV(
                    buff + nparsed,
                    sen_sz, &parser->GPGSV))
                {
                    nmea_GPGSV2info(&parser->GPGSV, info);
                }
                break;
            case GPRMC:
                if(nmea_parse_GPRMC(
                    buff + nparsed,
                    sen_sz, &parser->GPRMC))
                {
                    nmea_GPRMC2info(&parser->GPRMC, info);
                }
                break;
            case GPVTG:
                if(nmea_parse_GPVTG(
                    buff + nparsed,
                    sen_sz, &parser->GPVTG))
                {
                    nmea_GPVTG2info(&parser->GPVTG, info);
                }
                break;
			case GPZDA:
				if(nmea_parse_GPZDA(
					buff + nparsed,
					sen_sz, &parser->GPZDA))
				{
					nmea_GPZDA2info(&parser->GPZDA, info);
				}
				break;
            default:
                break;
            };
        }

        nparsed += sen_sz;
        nsentence ++ ;
    }

    return nsentence;
}

