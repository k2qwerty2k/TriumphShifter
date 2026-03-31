#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#include "serial.h"

#ifndef TX_BUFFER_SIZE
#define TX_BUFFER_SIZE 128 /**< TX Buffer Size in Bytes (Power of 2) */
#endif // TX_BUFFER_SIZE

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0; // Индекс для добавления данных
volatile uint8_t tx_tail = 0; // Индекс для отправки данных
volatile uint8_t tx_running = 0; // Статус передачи

ISR(USART_UDRE_vect) {
    if(!tx_running) {
        return;
    }
    // Проверяем, есть ли ещё данные в буфере для отправки
    if (tx_head != tx_tail) {
        // Загружаем следующий символ в регистр данных для передачи
        uint8_t c = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        UDR0 = c;
        
    } else {
        // Буфер опустел, останавливаем передачу
        tx_running = 0;
        PORTD &= ~(1<<6);
    }
}

void uartInit(void) {
    // Установка скорости 57600 (даташит)
#if (F_CPU == 8000000)
    UBRR0H = 0;
    UBRR0L = 16;
    UCSR0A = (1 << U2X0); // Включить удвоенную скорость
    // Ошибка 2.1%
#elif (F_CPU == 16000000)
    UBRR0H = 0;
    UBRR0L = 34;
    UCSR0A = (1 << U2X0); // Отключить удвоенную скорость
    // Ошибка -0.8%
#endif

    // Включение передатчика
    UCSR0B = (1 << TXEN0);
    
    // Формат кадра: 8 бит данных, 1 стоп-бит, без паритета
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    tx_running = 0;
    PORTD &= ~(1<<6);
    tx_head = tx_tail = 0;
    UCSR0B |= (1 << UDRIE0);
}

// Возвращает 1 при успешном добавлении, 0 при ошибке (буфер полон)
void uartSendChar(char data) {
    uint8_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    // Проверка, есть ли место в буфере
    if (next_head == tx_tail) {
        // Буфер полон
        return;
    }

    tx_buffer[tx_head] = data;
    tx_head = next_head;

    // Если передача не запущена, запускаем её
    if (!tx_running) {
        tx_running = 1;
        PORTD |= (1<<6);

        // Загружаем первый символ в регистр данных
        next_head = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        UDR0 = next_head;
    }
}

void uartSend(char *str) {
    char c;
    while('\0' != (c = *str++)) {
        uartSendChar(c);
    }
}

void uartSend_P(const char *str) {
    char c;
    while('\0' != (c = pgm_read_byte(str++))) {
        uartSendChar(c);
    }
}

void uartSendUint16_t(uint16_t num) {
    uint8_t buf[5] = { 0, 0, 0, 0, 0 };
    uint8_t n = 0;
    if(0 == num) {
        n = 1;
    } else {
        while(num > 0) {
            buf[n++] = num % 10;
            num /= 10;
        }
    }

    for (uint8_t i = n; i > 0; i--) {
        uartSendChar(buf[i-1] + '0');
    }
}
