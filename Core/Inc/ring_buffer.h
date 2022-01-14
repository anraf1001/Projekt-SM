/*
 * ring_buffer.h
 *
 *  Created on: Jun 7, 2021
 *      Author: anraf1001
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#define RING_BUFFER_SIZE 16

typedef enum {
	RB_OK = 0,
	RB_ERROR = 1
} RB_Status;

typedef struct {
	uint16_t Head;
	uint16_t Tail;
	uint8_t Buffer[RING_BUFFER_SIZE];
} RingBuffer_t;

RB_Status RB_Read(RingBuffer_t* Buf, uint8_t* Value);
RB_Status RB_Write(RingBuffer_t* Buf, uint8_t Value);
void RB_Flush(RingBuffer_t* Buf);
void RB_TakeLine(RingBuffer_t* Buf, uint8_t* Destination);

#endif /* INC_RING_BUFFER_H_ */
