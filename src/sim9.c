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

/*! file sim9.c
 * \brief the cellular phone.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "sim9.h"

#ifdef SIM9_DEBUG_PORT
/*! send a string to the debug port.
 *
 * \param s the string to be sent, if NULL then the SIM9_DEBUG_TXBUF
 * will be sent instead.
 */
void sim9_debug(const char *s)
{
	usart_printstr(SIM9_DEBUG_PORT, s);
}

/*! send a string to the debug port.
 *
 * \param *s the string to send.
 */
void sim9_debug_P(PGM_P s)
{
	strcpy_P(SIM9_DEBUG_TXBUF, s);
	usart_printstr(SIM9_DEBUG_PORT, NULL);
}

/*! dump the whole usart buffer
 *
 * \note Print '*' on non-printable chars.
 */
void debug_usart_buffer(void)
{
	uint8_t i, c;

	sim9_debug_P(PSTR("B["));

	for (i=0; i<sim9->usart->rx->size; i++) {
		c = *(sim9->usart->rx->buffer + i);

		if (c>31 && c<127)
			usart_putchar(SIM9_DEBUG_PORT, c);
		else
			usart_putchar(SIM9_DEBUG_PORT, '*');
	}

	sim9_debug_P(PSTR("]\n"));
}
#endif

/*! send a string to the modem.
 *
 * Commands terminate with CR.
 *
 * \param s the string to be sent, if NULL then the TX_BUF
 * will be sent instead.
 */
void sim9_send(const char *s)
{
	usart_printstr(SIM9_SERIAL_PORT, s);

#ifdef SIM9_DEBUG_PORT
	sim9_debug_P(PSTR("-> "));

	if (s)
		sim9_debug(s);
	else
		sim9_debug(sim9->tx_buf);

	sim9_debug_P(PSTR("\n"));
#endif
}

/*! send a flash string to the modem.
 *
 * Commands terminate with CR.
 *
 * \param *s the string to send (PSTR() to store it in flash space).
 */
void sim9_send_P(PGM_P s)
{
	strncpy_P(sim9->tx_buf, s, sim9->usart->tx_size);
	sim9_send(NULL);
}

/*! \brief Clear RX buffer.
 *
 * Clear the serial buffer, used usually to start a new
 * conversation with the modem in order to remove garbage leftover.
 */
void sim9_clear_rx_buff(void)
{
	usart_clear_rx_buffer(SIM9_SERIAL_PORT);
}

/*! Get char from the modem
 *
 * Loop for <time> until the requested char is found.
 *
 * \param c the char to be searched.
 * \param timeout in seconds.
 * \return TRUE if found.
 * \note The char cannot be \0
 */
uint8_t sim9_wait4char(const char s, uint8_t timeout)
{
	char c;

	while (timeout--) {
		if (usart_get(SIM9_SERIAL_PORT, (uint8_t *)&c, 1) && c == s)
			return(TRUE);

		_delay_ms(1000);
	}

	return(FALSE);
}

/*! Check and get a message from the modem.
 *
 * Check the usart RX flag and, in case there is a message, get it.
 * You have a timeout in seconds to complete the operation.
 *
 * The message from the sim9xx is in the form of:
 * [CR][LF]<message>[CR][LF]
 *
 * which generates 2 message in the queue if [LF] is the End Of Message.
 *
 * \note the use of milliseconds instead of second is to increment
 * the number of checks the function will perform.
 *
 * \warning if the pre-allocated space 's' < sizeof(rx-buffer), a
 * truncated non-terminated message can be returned.
 * \warning if sizeof(msg) < 2 the msg is ignored.
 * \warning loop <= 0xffff
 * \warning msg[size - 2] will be = 0 to terminate the string by
 * eliminating the [cr][lf].
 *
 * \param s pre-allocated string space.
 * \param size size_of(s)
 * \param timeout timeout in seconds (max. 0xff).
 * \return the lenght of the message.
 */
uint8_t sim9_msg(char *s, const uint8_t size, const uint8_t timeout)
{
	uint8_t len;
	uint16_t loop;

	len = 0;

	/* loop and the _delay_ms must be equal to 1 second
	 * in order to keep the timeout valid.
	 */
	loop = timeout * 100;

	do {
		_delay_ms(10);

		if (sim9->usart->flags.eol) {
			len = usart_getmsg(SIM9_SERIAL_PORT,
					(uint8_t *)s, size);

			/* Ignore message compose only by CR LF */
			if (len < 3)
				len = 0;
		}
	} while (!len && loop--);

	/* if a valid message, terminate the string over the CR.
	 * len is 0 or > 2
	 */
	if (len) {
		s[len - 2] = 0;

#ifdef SIM9_DEBUG_PORT
		sim9_debug_P(PSTR("<- "));
		sim9_debug(s);
		sim9_debug_P(PSTR("\n"));
#endif
	}

	return (len);
}

/*! Search for string from the modem.
 *
 * For example used after sending an AT command to
 * the modem and wait for the OK string.
 *
 * \note A positive match will be returned also if the string is a
 * substring of what returned from the modem.
 *
 * \note If count == 0; then count = 1 by default.
 *
 * \note count require 1sec max timeout, ex. count = 5 and no message
 * incoming, it will wait 5 sec. before error.
 *
 * \note If the message is not complete then it will be 2 chars shorter
 * than expected, because it was suppose to be terminated by CRLF.
 *
 * \param s the string to look for.
 *
 * \param count max number of VALID msgs (\see sim9_msg())
 *  to analyze before error.
 *
 * \param extbuff is the space reserved for the complete matching
 *  string found. If NULL is passed, then a buffer of
 *  size = sim9->usart->rx->size will be allocated.
 *
 * \param size the size of extbuff.
 *
 * \param type the type of search can be:
 *  EQUAL the string must be equal to the message.
 *  EEQUAL as EQUAL or ERROR.
 *  RELAX the string can be any substring of the message.
 *  ERELAX as RELAX or ERROR.
 *  STRICT the string is a substring that match from the beginning
 *   of the message.
 *  ESTRICT as STRICT or ERROR.
 *
 * \return TRUE string found, FALSE not found.
 *
 * \warning count <= 0xff
 * \warning size <= 0xff
 * \bug the string s should be checked not to be larger than the
 * allocated RX buffer size or this function will always fail.
 */
uint8_t sim9_searchfor(const char *s, uint8_t count,
		char *extbuff, const uint8_t extsize, const uint8_t type)
{
	uint8_t ok, check_error, size;
	char *buffer;

	ok = FALSE;

	/* in case of "ERROR" string */
	check_error = FALSE;

	/* check for the external of allocated buffer */
	if (extbuff) {
		size = extsize;
		buffer = extbuff;
	} else {
		size = sim9->usart->rx->size;
		buffer = malloc(size);
	}

#ifdef SIM9_DEBUG_PORT
	sim9_debug_P(PSTR("?: "));
	sim9_debug(s);
	sim9_debug_P(PSTR(" ["));
	buffer = itoa(count, buffer, 10);
	sim9_debug(buffer);
	sim9_debug_P(PSTR("/"));
	buffer = itoa(sim9->usart->flags.eol, buffer, 10);
	sim9_debug(buffer);
	sim9_debug_P(PSTR("]\n"));
#endif

	/* Clear the buffer */
	*(buffer) = 0;

	do {
		/* this will take 1 second top if no msg is present */
		if (sim9_msg(buffer, size, 1)) {
			switch(type) {
				case ERELAX:
					check_error = TRUE;
				case RELAX:
					if (strstr(buffer, s) != NULL)
						ok = TRUE;
					break;
				case EEQUAL:
					check_error = TRUE;
				case EQUAL:
				default:
					if (strncmp(s, buffer, strlen(s)) == 0)
						ok = TRUE;
					break;
			}

			/* if I found ERROR as a message and
			 * it is not what I was looking for
			 * then exit.
			 */
			if (!ok && check_error &&
					!strcmp_P(buffer, PSTR("ERROR")))
				count = 0;
		}
	} while (!ok && count--);

#ifdef SIM9_DEBUG_PORT
	/* string is terminated by CRLF, but
	 * the serial port strip the LF, not the CR.
	 */
	if (ok) {
		sim9_debug_P(PSTR(" -[*]-\n"));
	} else {
		sim9_debug_P(PSTR(" -[NOTFOUND!]-\n"));
		debug_usart_buffer();
	}
#endif

	/* free the local buffer used. */
	if (!extbuff)
		free(buffer);

	return(ok);
}

/*! like the searchfor(), but it does not need to get the
 * matching string back and the search is a PROGMEM
 * string.
 *
 * \see sim9_searchfor()
 *
 * \warning This function allocate the current length of the
 *  string_P passed + the allocation of the searchfor()
 *  function.
 */
uint8_t sim9_searchfor_P(PGM_P s, uint8_t count,
		char *extbuff, const uint8_t extsize, const uint8_t type)
{
	char *buffer;
	uint8_t ok, size;

	/* allocate the minor of + \0 char */
	size = strnlen_P(s, sim9->usart->rx->size) + 1;
	buffer = malloc(size);
	/* copy the PROGMEM to ram */
	strncpy_P(buffer, s, size);
	/* termiante the string */
	buffer[size - 1] = 0;
	/* call the searchfor() */
	ok = sim9_searchfor(buffer, count, extbuff, extsize, type);
	/* deallocate the buffer */
	free(buffer);
	return (ok);
}

/* Send an AT command to the device
 *
 * This is needed to handle the different situation
 * where the echo is enabled or not. If echo is enabled,
 * then sending <command + [CR]> will echo back the same,
 * but [CR] is not considered EOL ([LF] is) therefore there
 * will be no message present in the buffer, until an answer
 * is triggered.
 *
 * Answers can be:
 *
 * SENDAT_TYPE_NONE:
 *    No Answer.
 *
 * SENDAT_TYPE_OK:
 *    [CR][LF]OK[CR][LF]
 *    2 messages in the buffer, check for OK is performed..
 *
 * SENDAT_TYPE_MSGOK:
 *    [CR][LF]<something>[CR][LF]
 *    [CR][LF]OK[CR][LF]
 *    4 messages in the buffer.
 *
 * SENDAT_TYPE_MSG:
 *    [CR][LF]<something>[CR][LF]
 *    2 messages in the buffer, the <something> must be
 *    searched after this func().
 *
 * UNIMPLEMENTED:
 *    [CR][LF]OK[CR][LF]
 *    [CR][LF]<something>[CR][LF]
 *    4 messages in the buffer.
 *
 * \note if cmd == NULL, then send AT alone.
 *
 * \param cmd the command with AT.
 * \param msg the <something> needed back.
 * \param type the type of answer, see above.
 */
uint8_t sim9_send_at(const char * cmd, char * msg,
		const uint8_t size, const uint8_t type)
{
	uint8_t ok = TRUE;

	sim9_send(cmd);
	sim9_send_P(PSTR("\r"));

	/* add the [LF] to trigger the EOM in the buffer
	 * in case of echo.
	 *
	 * FIXME: In theory it is possible to start receiving
	 * the answer to the command before the next \n is sent.
	 * Echo should not be used at all.
	 */
	if (sim9->status.echo) {
		sim9_send_P(PSTR("\n"));
		/* wait for the echo back */
		_delay_ms(100);
		/* get the echo back from the buffer. */
		ok = sim9_searchfor(cmd, sim9->usart->flags.eol + 1,
				NULL, 0, EEQUAL);
	}

	/* wait for processing serial data */
	_delay_ms(100);

	switch (type) {
		case SENDAT_TYPE_MSGOK:
			ok = ok && sim9_msg(msg, size,
					sim9->usart->flags.eol + 1);
		case SENDAT_TYPE_OK:
			/* search OK */
			ok = ok && sim9_searchfor_P(PSTR("OK"),
					sim9->usart->flags.eol + 1,
					NULL, 0, EEQUAL);
			break;
		case SENDAT_TYPE_MSG:
			ok = ok && sim9_msg(msg, size,
					sim9->usart->flags.eol + 1);
			break;
		default:
			break;
	}

	return (ok);
}

/*! PROGMEM version of the send_at()
 *
 * \see sim9_send_at
 * \warning allocate strlen(cmd)
 */
uint8_t sim9_send_at_P(PGM_P cmd, char* msg,
		const uint8_t msgsize, const uint8_t type)
{
	char *buffer;
	uint8_t ok, size;

	/* allocate the minor of cmd and rx->size + \0 */
	size = strnlen_P(cmd, sim9->usart->rx->size) + 1;
	buffer = malloc(size);
	/* copy the PROGMEM to ram */
	strncpy_P(buffer, cmd, size);
	/* termiante the string */
	buffer[size - 1] = 0;
	/* call the non _P() */
	ok = sim9_send_at(buffer, msg, msgsize, type);
	/* deallocate the buffer */
	free(buffer);
	return (ok);
}

/*! send the escape sequence to the modem.
 *
 * there should be 1000ms idle period before this sequence, 500ms idle
 * period after this sequence and no more then 500ms between each +.
 */
void sim9_escape(void)
{
	uint8_t retry=3;

	while (retry--) {
		if (sim9->status.connected) {
			_delay_ms(1000);
			sim9_send_P(PSTR("+++"));
			_delay_ms(500);

			/* send only to buffer the +++ with EOL */
			if (sim9->status.echo) {
				sim9_send_P(PSTR("\r\n"));
				sim9_searchfor_P(PSTR("+++"), 30,
						NULL, 0, EQUAL);
			} else {
				/* Long delays may happen */
				_delay_ms(1000);
			}
		}

		/* get the result */
		if (sim9_send_at_P(PSTR("AT"), NULL, 0,
					SENDAT_TYPE_OK)) {
			sim9->status.connected = FALSE;
			break;
		} else {
			sim9->status.connected = TRUE;
		}
	}

	if (sim9->status.connected)
		sim9->errors.esc = TRUE;
	else
		sim9->errors.esc = FALSE;
}

/*! Get the IMEI.
 *
 * The IMEI code should be between 15 and 17 chars.
 *
 * \note the response is <CR><LF>imei<CR><LF>
 * you need to skip the 1st message.
 */
void imei(void)
{
	uint8_t retry=10;

	*(sim9->imei) = 0;
	sim9->errors.imei = TRUE;
	sim9_clear_rx_buff();

	while (sim9->errors.imei && retry--)
		if (sim9_send_at_P(PSTR("AT+CGSN"),
					sim9->imei, IMEI_SIZE,
					SENDAT_TYPE_MSGOK) &&
				(strlen(sim9->imei) > 14))
			sim9->errors.imei = FALSE;
}

/* check for the SIM pin */
void pin_check(void)
{
#define SOB2 20 /* temp size of buffer */

	char *buffer;

	buffer = malloc(SOB2);

	if (sim9_send_at_P(PSTR("AT+CPIN?"),
				buffer, SOB2,
				SENDAT_TYPE_MSGOK) &&
			!memcmp_P(buffer, PSTR("+CPIN: READY"), 12))
		sim9->errors.pin = FALSE;
	else
		sim9->errors.pin = TRUE;

	free(buffer);
}

/* Check if we are registered on the network
 *
 * \note 0,1 means registered on the home network.
 */
void network_registered(void)
{
#define SOB626 20 /* temp buffer */

	char *buffer;
	uint8_t retry=5;

	buffer = malloc(SOB626);
	sim9->errors.netreg = TRUE;

	while (sim9->errors.netreg && retry--) {
		/* Wait some time to get registered
		 * in the network.
		 */
		_delay_ms(2000);

		if (sim9_send_at_P(PSTR("AT+CGREG?"),
					buffer, SOB626,
					SENDAT_TYPE_MSGOK) &&
				!memcmp_P(buffer,
					PSTR("+CGREG: 0,1"), 11))
			sim9->errors.netreg = FALSE;
	}

	free(buffer);
}

void sim9_suspend(void)
{
	usart_suspend(SIM9_SERIAL_PORT);
}

void sim9_resume(void)
{
	usart_resume(SIM9_SERIAL_PORT);
}

/*! \brief Initialize the serial port if requested.
 *
 * Also allocate the RXTX struct buffer.
 *
 * \note if the IRQ is used, then it must be already enabled.
 * \warning flags will be cleared on every sim9_on()
 */
struct sim9_t* sim9_init(void)
{
	if (!sim9) {
		sim9 = malloc(sizeof(struct sim9_t));
		/* clear flags */
		sim9->status.all = 0;
		sim9->errors.all = 0;
		sim9->flags = 0;
		/* allocate the IMEI string */
		sim9->imei = malloc(IMEI_SIZE);
		*(sim9->imei) = 0;
		/* allocate the GSP strings */
		sim9->gps_lat = malloc(GPS_LAT_SIZE);
		*(sim9->gps_lat) = 0;
		sim9->gps_lon = malloc(GPS_LON_SIZE);
		*(sim9->gps_lon) = 0;
		/* initialize the usart port */
		sim9->usart = usart_init(SIM9_SERIAL_PORT);
		/* convenient link to the TX buffer */
		sim9->tx_buf = sim9->usart->tx;
	}

	return(sim9);
}

/*! remove all the allocated struct
 */
void sim9_shut(void)
{
	usart_shut(SIM9_SERIAL_PORT);
	sim9->tx_buf = NULL;
	sim9->usart = NULL;
	free(sim9->gps_lon);
	free(sim9->gps_lat);
	free(sim9->imei);
	free(sim9);
}

/*! \brief power up the modem.
 *
 * \note turning on the modem will take from 11sec to 16 seconds
 */
void sim9_on(void)
{
	/* clear all flags */
	sim9->status.all = 0;
	sim9->errors.all = 0;
	/* start the serial port */
	usart_resume(SIM9_SERIAL_PORT);
	/* setup input signal pin */
	DDRA &= ~(_BV(SIM9_STATUS) | _BV(SIM9_RI) | _BV(SIM9_DTR));
	DDRD &= ~_BV(SIM9_NET_ST);

	/* Output the power on pin */
	DDRA |= _BV(SIM9_PIN_ON);
	/* Start the modem with 1 sec pulse __|--|__ */
	PORTA &= ~_BV(SIM9_PIN_ON);
	_delay_ms(1000);
	PORTA |= _BV(SIM9_PIN_ON);
	_delay_ms(1000);
	PORTA &= ~_BV(SIM9_PIN_ON);
	/* The modem may require 3 sec to start */
	_delay_ms(4000);
	/* clear the RX buffer from garbage */
	usart_clear_rx_buffer(SIM9_SERIAL_PORT);

	/* NOTE: all AT must be uppercase. */
	sim9_send_at_P(PSTR("AT"), NULL, 0, SENDAT_TYPE_OK);

	/* speed 9600 */
	sim9_send_at_P(PSTR("AT+IPR=9600"), NULL, 0, SENDAT_TYPE_OK);
	/* Enable URC presentation */
	sim9_send_at_P(PSTR("AT+CIURC=1"), NULL, 0, SENDAT_TYPE_OK);
	/* Wait for the Ready */
	sim9_searchfor_P(PSTR("Call Ready"), 60, NULL, 0, EQUAL);

	sim9_clear_rx_buff();

	/* Factory default */
	if (!sim9_send_at_P(PSTR("AT&F&C0&D0"), NULL, 0, SENDAT_TYPE_OK))
		sim9->errors.init = TRUE;

	/* set the echo */

#ifdef SIM9_ECHO_ENA
	sim9->status.echo = 1;
#endif

	if (sim9->status.echo)
		sim9_send_at_P(PSTR("ATE1"), NULL, 0, SENDAT_TYPE_OK);
	else
		sim9_send_at_P(PSTR("ATE0"), NULL, 0, SENDAT_TYPE_OK);

	/* set net light behaviour */
	sim9_send_at_P(PSTR("AT+SLEDS=1,53,790"),
			NULL, 0, SENDAT_TYPE_OK);
	sim9_send_at_P(PSTR("AT+SLEDS=2,53,2990"),
			NULL, 0, SENDAT_TYPE_OK);
	sim9_send_at_P(PSTR("AT+SLEDS=3,53,287"),
			NULL, 0, SENDAT_TYPE_OK);
	sim9_send_at_P(PSTR("AT+CNETLIGHT=1"),
			NULL, 0, SENDAT_TYPE_OK);

	/* check for the SIM pin */
	if (!sim9->errors.all)
		pin_check();

	if (!sim9->errors.all)
		imei();

	if (!sim9->errors.all) {
		/* delay sometime to register on the network */
		_delay_ms(5000);
		network_registered();
	}
}

/*! Power off the modem.
 */
void sim9_off(void)
{
	sim9_send_P(PSTR("AT+CPOWD=1\r"));

	if (sim9_searchfor_P(PSTR("NORMAL POWER DOWN"),
				5, NULL, 0, RELAX))
		sim9->status.ready = FALSE;
	else
		sim9->errors.off = TRUE;
}

void check_cgatt(void)
{
#define SIZEOFBUFFER 15

	char *buffer;

	buffer = malloc(SIZEOFBUFFER);

	/* Query the status of the connection */
	if (sim9_send_at_P(PSTR("AT+CGATT?"), buffer, SIZEOFBUFFER,
				SENDAT_TYPE_MSGOK)) {
		if (memcmp_P(buffer, PSTR("+CGATT: 1"), 9))
			sim9->status.gprs = FALSE;
		else
			sim9->status.gprs = TRUE;
	} else {
		sim9->errors.gprs = TRUE;
	}

	free(buffer);
}

/*! attach GPRS network
 *
 * AT+CGATT
 */
void gprs_connect(void)
{
	uint8_t retry;

	sim9->errors.gprs = FALSE;

	if (sim9_send_at_P(PSTR("AT+CGATT=1"),
				NULL, 0, SENDAT_TYPE_OK)) {
		retry = 5;

		do
			check_cgatt();
		while (!sim9->status.gprs && retry--);
	} else {
		sim9->errors.gprs = TRUE;
	}
}

/*! detach GPRS network
*/
void gprs_disconnect(void)
{
	uint8_t retry;

	sim9->errors.gprs = FALSE;

	if (sim9_send_at_P(PSTR("AT+CGATT=0"), NULL, 0, SENDAT_TYPE_OK)) {
		retry = 5;

		do
			check_cgatt();
		while (sim9->status.gprs && retry--);
	} else {
		sim9->errors.gprs = TRUE;
	}
}

/*! APN setup
 *
 * Try to automatically setup the APN.
 * Switch to Internet if neither VODAFONE or TIM is found.
 *
 * +COPS: 0,0,"I TIM"
 *
 * \note Internet should become the default.
 */
void apn_setup(void)
{
#define SIZEOF_S 30

	char *s;

	s = malloc(SIZEOF_S);
	sim9_send_at_P(PSTR("AT+COPS?"), NULL, 0, SENDAT_TYPE_NONE);

	if (sim9_searchfor("+COPS:", 5, s, SIZEOF_S, RELAX)) {
		/* memcmp false when match */
		if (!memcmp(s + 12, "I TIM", 5)) // TIM
			sim9->status.provider = 3;
		else if (!memcmp(s + 13, "odafo", 5)) // [V,v]odafone
			sim9->status.provider = 2;
		else
			sim9->status.provider = 1; // others

		sim9_searchfor_P(PSTR("OK"), 5, NULL, 0, RELAX);
	} else {
		sim9->status.provider = 0;
		sim9->errors.apn = TRUE;
	}

	free(s);
}

void gprs_wireless_connection(void)
{
	if (!sim9_send_at_P(PSTR("AT+CIICR"), NULL, 0,
			SENDAT_TYPE_OK))
		sim9->errors.tcpip = TRUE;
}

/*! TCPIP activate
 *
 * \param transparent enable/disable transparent mode.
 */
void sim9_tcpip_on(void)
{
	char *s;

	sim9->errors.tcpip = FALSE;

	/* show the TCP config */
	sim9_send_at_P(PSTR("AT+CIPCCFG?"), NULL, 0,
			SENDAT_TYPE_OK);

	/* set the transparent mode */
	if (sim9->status.tsmode)
		sim9_send_at_P(PSTR("AT+CIPMODE=1"),
				NULL, 0, SENDAT_TYPE_OK);
	else
		sim9_send_at_P(PSTR("AT+CIPMODE=0"),
				NULL, 0, SENDAT_TYPE_OK);

	/* attach GPRS network */
	gprs_connect();

	if (sim9->status.gprs)
		apn_setup();

	/* start task */
	switch(sim9->status.provider) {
		case 1: // others
			sim9_send_at_P(PSTR("AT+CSTT=\"SIM9_APN_SITE\",\"SIM9_APN_USER\",\"SIM9_APN_PASSWORD\""),
					NULL, 0, SENDAT_TYPE_OK);
			break;
		case 2: // Vodafone
			sim9_send_at_P(PSTR("AT+CSTT=\"web.omnitel.it\""),
					NULL, 0, SENDAT_TYPE_OK);
			break;
		case 3: // TIM
			sim9_send_at_P(PSTR("AT+CSTT=\"ibox.tim.it\""),
					NULL, 0, SENDAT_TYPE_OK);
			break;
		default: // Error
			sim9->errors.apn = TRUE;
	}

	if (!sim9->errors.all)
		gprs_wireless_connection();

	/* GET the assigned IP address */
	if (!sim9->errors.all) {
		s = malloc(30);
		sim9_send_at_P(PSTR("AT+CIFSR"), s, 30,
				SENDAT_TYPE_MSG);
		free(s);
	}
}
