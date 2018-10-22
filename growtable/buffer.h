/*
 * buffer.h
 *
 * Created: 2018-04-16 21:11:46
 *  Author: linus
 *
 * Inspired by:
 * http://www.fourwalledcubicle.com/files/LightweightRingBuff.h
 */ 

#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 128

typedef struct {
	uint8_t buffer[BUFFER_SIZE];
	uint8_t* write;
	uint8_t* read;
	uint8_t count;
} buffer_t;

static inline void buffer_initBuffer(buffer_t* const buffer) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		buffer->write = buffer->buffer;
		buffer->read = buffer->buffer;
		buffer->count = 0;
	}
}

static inline uint8_t buffer_getCount(buffer_t* const buffer) {
	uint8_t count;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		count = buffer->count;
	}

	return count;
}

static inline bool buffer_isFull(buffer_t* const buffer) {
	return (buffer_getCount(buffer) == BUFFER_SIZE);
}

static inline bool buffer_isEmpty(buffer_t* const buffer) {
	return (buffer_getCount(buffer) == 0);
}

static inline void buffer_write(buffer_t* const buffer, const uint8_t data) {
	*buffer->write = data;
	
	if (++buffer->write == &buffer->buffer[BUFFER_SIZE])
		buffer->write = buffer->buffer;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		buffer->count++;
	}
}

static inline uint8_t buffer_read(buffer_t* const buffer) {
	uint8_t data = *buffer->read;
	
	if (++buffer->read == &buffer->buffer[BUFFER_SIZE])
		buffer->read = buffer->buffer;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		buffer->count--;
	}

	return data;
}

static inline uint8_t buffer_peek(buffer_t* const buffer) {
	uint8_t data = *buffer->read;
	
	return data;
}