#include <stdio.h>
#include "uart.h"

#include <HardwareSerial.h>

#define FULL_TERM_SUPPORT

FILE uartstream;

#define RX_BUFSIZE 80

int uart_putchar(char c, FILE *stream)
{
	if (c == '\n')
		Serial.write('\r');
	Serial.write(c);
	return 0;
}


#ifdef FULL_TERM_SUPPORT
/*
 * Receive a character from the UART Rx.
 *
 * This features a simple line-editor that allows to delete and
 * re-edit the characters entered, until either CR or NL is entered.
 * Printable characters entered will be echoed using uart_putchar().
 *
 * Editing characters:
 *
 * . \b (BS) or \177 (DEL) delete the previous character
 * . ^u kills the entire input buffer
 * . ^w deletes the previous word
 * . ^r sends a CR, and then reprints the buffer
 * . \t will be replaced by a single space
 *
 * All other control characters will be ignored.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).  If the buffer
 * is full (i. e., at RX_BUFSIZE-1 characters in order to keep space for
 * the trailing \n), any further input attempts will send a \a to
 * uart_putchar() (BEL character), although line editing is still
 * allowed.
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int
uart_getchar(FILE *stream)
{
	uint8_t c;
	char *cp, *cp2;
	static char b[RX_BUFSIZE];
	static char *rxp;

	if (rxp == 0)
		for (cp = b;;)
		{
			while (Serial.available() <= 0) {};

			c = Serial.read();
			/* behaviour similar to Unix stty ICRNL */
			if (c == '\r')
				c = '\n';
			if (c == '\n')
			{
				*cp = c;
				uart_putchar(c, stream);
				rxp = b;
				break;
			}
			else if (c == '\t')
				c = ' ';

			if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') ||
					c >= (uint8_t)'\xa0')
			{
				if (cp == b + RX_BUFSIZE - 1)
					uart_putchar('\a', stream);
				else
				{
					*cp++ = c;
					uart_putchar(c, stream);
				}
				continue;
			}

			switch (c)
			{
				case 'c' & 0x1f:
					return -1;

				case '\b':
				case '\x7f':
					if (cp > b)
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;

				case 'r' & 0x1f:
					uart_putchar('\r', stream);
					for (cp2 = b; cp2 < cp; cp2++)
						uart_putchar(*cp2, stream);
					break;

				case 'u' & 0x1f:
					while (cp > b)
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;

				case 'w' & 0x1f:
					while (cp > b && cp[-1] != ' ')
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;
			}
		}

	c = *rxp++;
	if (c == '\n')
		rxp = 0;

	return c;
}

#else
int
uart_getchar(FILE *stream)
{
	uint8_t c;
	while (Serial.available() <= 0) {};

	c = Serial.read();
	return c;
}
#endif

void setup_uart() {
	fdev_setup_stream(&uartstream, uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = &uartstream;
	stdin = &uartstream;
}

