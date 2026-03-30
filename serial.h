#ifndef _serial_h
#define _serial_h

void uartInit(void);
void uartSendChar(char data);
void uartSend(char *str);
void uartSend_P(const char *str);
void uartSendUint16_t(uint16_t num);

#endif