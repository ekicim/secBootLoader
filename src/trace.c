#include <bsp.h>
#include <uart.h>
#include <string.h>
#include <trace.h>


void Trace( char* msg)
{

	UARTSend(PORT_TRACE, (unsigned char*)msg, strlen(msg));

}

void TraceNL( char* msg)
{
	UARTSend(PORT_TRACE, (unsigned char*)msg, strlen(msg));
	UARTSend(PORT_TRACE, (unsigned char*)"\r\n", 2);
}

void TracePutc( char c )
{
	UARTSend(PORT_TRACE, (unsigned char*)&c, 1L);
}

void TracePutcHex( char c )
{
	unsigned char  buff[10];
    int count;
	count = sprintf( buff, "%02x", c);
	UARTSend( PORT_TRACE, buff, count );

}

void TraceDumpHex( char* pMsg, int len )
{
#if defined (BOOTROM_DEBUG)
	int i, count;
	unsigned char buffer[300];
	unsigned char buff[17];
	unsigned char *pc = (unsigned char*) pMsg;

	UARTSend( PORT_TRACE, "\r\n", 2);
	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0) {
				count = sprintf(buffer, "  %s\r\n", buff);
				UARTSend( PORT_TRACE, buffer, count);
			}
		}
		count = sprintf(buffer, " %02x", pc[i]);
		UARTSend( PORT_TRACE, buffer, count );

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
    	UARTSend( PORT_TRACE, "   ", 3);
        i++;
    }
	count = sprintf(buffer, "  %s\r\n", buff);
	UARTSend( PORT_TRACE, buffer, count);
#endif
}
