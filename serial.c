#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#include "serial.h"

#ifndef TX_BUFFER_SIZE
#define TX_BUFFER_SIZE 64 /**< TX Buffer Size in Bytes (Power of 2) */
#endif // TX_BUFFER_SIZE

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0; // Индекс для добавления данных
volatile uint8_t tx_tail = 0; // Индекс для отправки данных
volatile uint8_t tx_running = 0; // Статус передачи

ISR(USART_TX_vect) {
    // Проверяем, есть ли ещё данные в буфере для отправки
    if (tx_head != tx_tail) {
        // Загружаем следующий символ в регистр данных для передачи
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    } else {
        // Буфер опустел, останавливаем передачу
        tx_running = 0;

        UCSR0B &= ~(1 << UDRIE0);
    }
}

void uartInit(void) {
    uint16_t ubrr_value = (F_CPU / (16UL * 115200UL)) - 1;

    // Установка скоростных параметров
    UBRR0H = (ubrr_value >> 8) & 0xFF;
    UBRR0L = ubrr_value & 0xFF;

    // Включение передатчика
    UCSR0B = (1 << TXEN0);
    
    // Формат кадра: 8 бит данных, 1 стоп-бит, без паритета
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Возвращает 1 при успешном добавлении, 0 при ошибке (буфер полон)
void uartSendChar(char data) {
    uint8_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    // // Проверка, есть ли место в буфере
    // if (next_head == tx_tail) {
    //     // Буфер полон
    //     return 0;
    // }
    // Вырезано, пусть будет перезапись

    tx_buffer[tx_head] = data;
    tx_head = next_head;

    // Если передача не запущена, запускаем её
    if (!tx_running) {
        tx_running = 1;
        // Загружаем первый символ в регистр данных
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        // Включаем прерывание по завершению передачи
        UCSR0B |= (1 << UDRIE0);
    }
}

void uartSend(char *str) {
    char c;
    while(0 != (c = *str++)) {
        uartSendChar(c);
    }
}

void uartSend_P(const char *str) {
    char c;
    while(0 != (c = pgm_read_byte(str++))) {
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
