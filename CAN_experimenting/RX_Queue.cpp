/*
 * RX_Queue.cpp
 * Contains implementation of the RX_Queue class.
 */

#include "RX_Queue.h"
#include <stdint.h>

//---------------------------------
void RX_Deque::enqueue_head(const Frame& f) {
	if (!is_full()) {
		buf[head++] = f;

		// Wrap Head
		if (head >= RX_DEQUE_SIZE) {
			head = 0;
		}

		//Check whether full
		if (head == tail) {
			isFull = true;
		}
	}
	else { // If full, we overwrite the last added message.
		buf[(head + RX_DEQUE_SIZE - 1) % RX_DEQUE_SIZE] = f;
	}
}

Frame RX_Deque::dequeue_tail() {
	if (!is_empty()) {
		uint8_t readloc = tail++;

		//Wrap tail
		if (tail >= RX_DEQUE_SIZE) {
			tail = 0;
		}

		//Check whether queue has been emptied
		if (tail == head) {
			isFull = false;
		}

		return buf[readloc];
	}
	return Frame();
}

void RX_Deque::enqueue_tail(const Frame& f) {
	if (!is_full()) {
		if (tail == 0) {
			tail = RX_DEQUE_SIZE;
		}

		buf[--tail] = f;

		//Check whether full
		if (head == tail) {
			isFull = true;
		}
	}
	else { // If full, we overwrite the last added message.
		buf[tail] = f;
	}
}

Frame RX_Deque::dequeue_head() {
	if (!is_empty()) {
		//Wrap head
		if (head == 0) {
			head = RX_DEQUE_SIZE;
		}

		uint8_t readloc = --head;

		//Check whether queue has been emptied
		if (tail == head) {
			isFull = false;
		}

		return buf[readloc];
	}
	return Frame();
}
