/* Copyright (C) 2014-2018 Enrico Rossi

 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301  USA
 */

/*! \file sim9.h
 * \brief SIM900 functions
 *
 * \important Command sent to the modem MUST be terminated with
 * '\r' char and not '\n'.
 */

#ifndef _SIM9_H_
#define _SIM9_H_

#include <avr/pgmspace.h>
#include "usart.h"

/* Fix these settings to match your circuit */

#define SIM9_PIN_ON PA6 //! Pout ON.
#define SIM9_STATUS PA5 //! Pin status
#define SIM9_RI PA4 //! Pin ring indicator
#define SIM9_NET_ST PD6 //! Pin NET status
#define SIM9_DTR PA7 //! Pin DTR

/*! USART port where the modem is connected.
 *
 * On the microcontroller which has more than 1 serial port.
 */
#define SIM9_SERIAL_PORT 0

/*! Debug serial port.
 * The port must be already initialized.
 *
 * Define it in the Makefile if needed.
 * #define SIM9_DEBUG_PORT 1
 */
#define SIM9_DEBUG_TXBUF usart1->tx

/*! should the modem works with ECHO enabled? */
#define SIM9_ECHO_ENA

/*! string search type of */
#define EQUAL 0
#define RELAX 1
#define STRICT 2
#define EEQUAL 3
#define ERELAX 4
#define ESTRICT 5

/*! connection statuses char
 * \note thiese numbers are modem dependant, do not change them.
 */
#define CONNECTING '0'
#define CONNECTED '1'
#define CLOSING '2'
#define CLOSED '3'

/*! search type for the sim9_send_at() */
#define SENDAT_TYPE_NONE 0
#define SENDAT_TYPE_OK 1
#define SENDAT_TYPE_MSGOK 2
#define SENDAT_TYPE_MSG 3

/*! flag type
 *
 */
#define STATUS_SET 0
#define STATUS_CLEAR 1
#define STATUS_CHECK 2
#define ALARM_SET 3
#define ALARM_CLEAR 4
#define ALARM_CHECK 5

#define IMEI_SIZE 18 //! IMEI size

//! APN 3d provider setup
#define SIM9_APN_SITE internet
#define SIM9_APN_USER
#define SIM9_APN_PASSWORD

/*! status flags */
#define SIM9_ST_RDY 0 //! Ready (pin ok, network registered)
#define SIM9_ST_GPRS 1 //! GPRS registered
#define SIM9_ST_CID 2 //! CID enabled
#define SIM9_ST_SAPBR 3 //! http stack enabled
#define SIM9_ST_HTTP 4 //! http stack enabled

#define GPS_LAT_SIZE 12
#define GPS_LON_SIZE 12

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

struct sim9_t {
	/*! status flags */
	union {
		/* c11 only */
		struct {

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			/* lsb bit 0 */
			uint16_t ready:1;
			uint16_t gprs:1; // GPRS connected
			uint16_t cid:1;
			uint16_t sapbr:1;
			uint16_t http:1;
			uint16_t provider:2; // 1 Internet, 2 VODAFONE, 3 TIM
			uint16_t tsmode:1; // tcpip transparent mode
			uint16_t tcpip:4; // AT+CIPSTATUS 0-9
			uint16_t echo:1; // command echo
			uint16_t connected:1; // On/Off line
			uint16_t unused:2;
#else
			/* msb */
			uint16_t unused:2;
			uint16_t connected:1;
			uint16_t echo:1;
			uint16_t tcpip:4;
			uint16_t tsmode:1;
			uint16_t provider:2;
			uint16_t http:1;
			uint16_t sapbr:1;
			uint16_t cid:1;
			uint16_t gprs:1;
			uint16_t ready:1;
#endif

		};

		uint16_t all;
	} status;

	/*! error flags */
	union {
		struct {

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			/* lsb bit 0 */
			uint16_t init:1;
			uint16_t off:1;
			uint16_t pin:1;
			uint16_t imei:1;
			uint16_t apn:1;
			uint16_t tcpip:1;
			uint16_t netreg:1;
			uint16_t discon:1;
			uint16_t gprs:1;
			uint16_t esc:1; // esc +++ sequence failed
			uint16_t connected:1; // ATO command failed
			uint16_t gps:1; // No Fix
			uint16_t unused:4;
#else
			/* msb */
			uint16_t unused:4;
			uint16_t gps:1;
			uint16_t connected:1;
			uint16_t esc:1;
			uint16_t gprs:1;
			uint16_t discon:1;
			uint16_t netreg:1;
			uint16_t tcpip:1;
			uint16_t apn:1;
			uint16_t imei:1;
			uint16_t pin:1;
			uint16_t off:1;
			uint16_t init:1;
#endif

		};

		uint16_t all;
	} errors;

	// Generic flags
	union {
		uint8_t flags;
		struct {

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			uint8_t gps_enable:1; // Enable GPS
			uint8_t unused:7;
#else
			uint8_t unused:7;
			uint8_t gps_ena:1;
#endif

		};
	};

	char *imei;
	char *gps_lat;
	char *gps_lon;
	char *tx_buf;
	volatile struct usart_t *usart;
};

/*! Global */
struct sim9_t *sim9;

void sim9_clear_rx_buff(void);
void sim9_send(const char *s);
void sim9_send_P(PGM_P s);
void sim9_suspend(void);
void sim9_resume(void);
struct sim9_t* sim9_init(void);
void sim9_on(void);
void sim9_off(void);
uint8_t sim9_msg(char *s, const uint8_t size, const uint8_t timeout);
uint8_t sim9_searchfor(const char *s, uint8_t timeout,
		char *extbuff, const uint8_t size, const uint8_t type);
uint8_t sim9_searchfor_P(PGM_P s, uint8_t timeout,
		char *extbuff, const uint8_t size, const uint8_t type);
uint8_t sim9_send_at(const char * cmd, char * msg,
		const uint8_t size, const uint8_t type);
uint8_t sim9_send_at_P(PGM_P cmd, char* msg,
		const uint8_t msgsize, const uint8_t type);
uint8_t sim9_connect(void);
void sim9_disconnect(void);
uint8_t sim9_check_connection(const char status);
void sim9_tcpip_on(void);
uint8_t sim9_wait4char(const char s, uint8_t timeout);
void sim9_escape(void);

#endif
