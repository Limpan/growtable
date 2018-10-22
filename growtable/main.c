/*
 * GrowTable.cpp
 *
 * Created: 2018-04-09 20:34:28
 * Author : Linus Törngren
 */

#define F_CPU 20000000L  // 20 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include "buffer.h"

#define USART_BAUDRATE 250000
#define USART_BAUD_PRESCALE ((F_CPU / (USART_BAUDRATE * 16UL))) - 1

buffer_t volatile buffer_tx;
buffer_t volatile buffer_rx;

enum STATE_RX {WAITING, DECODE, ERROR};
	
uint8_t volatile fan_speed_a = 50;
uint8_t volatile fan_speed_b = 10;

ISR(USART_RX_vect) {
	/*
		Receive data from USART.
	*/
	uint8_t data;
	data = UDR0;

	buffer_write(&buffer_rx, data);

	if (data == 0x00) {
		uint8_t msg[] = {0x50, 0x49, 0x4E, 0x47};
		transmit_message(&msg, 4);
	}
	
	if (data == 0x01) {
		uint8_t msg[] = {0x10, fan_speed_a, fan_speed_b};
		transmit_message(&msg, 3);
		
		PORTB ^= (1<<PB0);
	}

	/*
	uint8_t temp;
	temp = UDR0;
	if (temp == 'a') {
		UDR0 = 'x';
		PORTB |= (1<<PB0);
	}
	else if (temp == 'A') {
		UDR0 = 'X';
		PORTB &= ~(1<<PB0);
	}
	else if (temp == 'b') {
		UDR0 = 'y';
		PORTB |= (1<<PB1);
	}
	else if (temp == 'B') {
		UDR0 = 'Y';
		PORTB &= ~(1<<PB1);
	}
	else if (temp == 'c') {
		UDR0 = 'z';
		PORTB |= (1<<PB2);
	}
	else if (temp == 'C') {
		UDR0 = 'Z';
		PORTB &= ~(1<<PB2);
	}
	else if (temp == 'd') {
		UDR0 = 'w';
		PORTC = (1<<PC0);
	} 
	*/
}

ISR(USART_UDRE_vect) {
	/* 
		Transmit data with USART.
	*/
	uint8_t data;
		
	if (buffer_getCount(&buffer_tx) > 0) {
		data = buffer_read(&buffer_tx);
		UDR0 = data;
	} else {
		// Clear UDRIE (disable UDRE)
		UCSR0B &= ~(1 << UDRIE0);
	}
}

void init_USART(void) {
	buffer_initBuffer(&buffer_tx);
	buffer_initBuffer(&buffer_rx);

	UBRR0H = (uint8_t) USART_BAUD_PRESCALE >> 8;
	UBRR0L = (uint8_t) USART_BAUD_PRESCALE;

	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
}

void init_PORTS(void) {
	// PB7:	XTAL2
	// PB6: XTAL1
	// PB5: SCK
	// PB4: MISO
	// PB3: MOSI
	// PB2: Light1
	// PB1: Heat2
	// PB0: Heat1
	DDRB  = 0b00000111;
	PORTB = 0b11111000;

	// PC7:	N/A
	// PC6: Reset
	// PC5: RTC (SCL)
	// PC4: RTC (SDA)
	// PC3: USB sleep
	// PC2: Dallas 1-wire input
	// PC1: Dallas 1-wire output
	// PC0: Fan A power
	DDRC  = 0b00001011;
	PORTC = 0b00110000;

	// PD7: Fan B power
	// PD6: Fan A PWM
	// PD5: Fan B PWM
	// PD4: Fan A tach
	// PD3: Fan B tach
	// PD2: RTC interrupt
	// PD1: USB Tx
	// PD0: USB Rx
	DDRD  = 0b11100000;
	PORTD = 0b00011111;
}


void init_TIMER0(void) {
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);
	
	OCR0A = fan_speed_a;
	OCR0B = fan_speed_b;
	PORTC |= (1 << 0);
}


void transmit_message(uint8_t *msg, uint8_t length) {
	/*
		Transmit message.
	*/
	uint8_t *read_ptr = msg;
	uint8_t *scan_ptr = msg;
	uint8_t *scan_end_ptr = msg + length;
	uint8_t cobs_code = 1;
	uint8_t byte;
	
	while (scan_ptr <= scan_end_ptr) {
		byte = *scan_ptr++;
		
		if (byte == 0x00 || !(scan_ptr <= scan_end_ptr)) {
			buffer_write(&buffer_tx, cobs_code);
			while ((read_ptr + 1) < scan_ptr) {
				buffer_write(&buffer_tx, *read_ptr++);
			}
			read_ptr++;
			cobs_code = 1;
		}
		else {
			cobs_code++;
		}
	}
	
	buffer_write(&buffer_tx, 0x00);

	// Set UDRIE (enable UDRE)
	UCSR0B |= (1 << UDRIE0);
}


void receive_message(uint8_t *data, uint8_t length) {
	uint8_t *scan_ptr = data;
	uint8_t *scan_end_ptr = data + length;
	uint8_t byte = 0;
	uint8_t count = *scan_ptr++;
	
	while (scan_ptr < scan_end_ptr) {
		byte = *scan_ptr++;

		if (byte == 0) {
			return;
		}

		if (count-- == 1) {
			count = byte;
			printf("0x%02x (Received)\n", 0x00);
		}
		else {
			printf("0x%02x (Received)\n", byte);
		}
	}
}


int main(void) {
	uint8_t temp;

	init_PORTS();
	init_USART();
	init_TIMER0();
	sei();
	
	uint8_t hello[] = {0x47, 0x52, 0x4F, 0x57, 0x2D, 0x32, 0x30, 0x31, 0x38, 0x31, 0x30, 0x32, 0x32, 0x2D, 0x4C, 0x69, 0x6E, 0x75, 0x73, 0x20, 0x54, 0x6F, 0x72, 0x6E, 0x67, 0x72, 0x65, 0x6E};
	transmit_message(&hello, 28);

	while (1) {}
}
