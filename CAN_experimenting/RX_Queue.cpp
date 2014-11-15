/*
 * RX_Queue.cpp
 * Contains implementation of the RX_Queue class.
 */

#include "RX_Queue.h"

uint16_t RX_Queue::size() {
	uint16_t diff = new_index - old_index;
	if (diff <= 0) {
		diff = diff + RX_QUEUE_SIZE;
	}
}