/*
 * RX_Queue.h
 * Contains definition for the RX_Queue class.
 */

#ifndef RX_Queue_h
#define RX_Queue_h

#include <stdint.h>
#include "MCP2515_defs.h"

/*
 * Static receiving deque for CAN_IO class. Holds frames that
 * have come in over the CAN bus.
 */
template<int Tsize>
class RX_Queue {
	/*Usage:
		- void	enqueue(Frame)	 -- Adds Frame to the head of the queue
		- Frame dequeue()		 -- Returns a Frame from the tail of the queue
		- bool	is_full()		 -- Returns true if the queue is full
		- bool	is_empty()		 -- Returns true if there are no elements in the queue
		- int	size()			 -- Returns the number of elements in the queue
		*/
public:
	static const int RX_QUEUE_SIZE = Tsize;

	/*
	 * Constructor. Initializes the queue.
	 */
	RX_Queue() : head(0), tail(0) {}

	/*
	 * Returns true if the queue is full.
	 */
	bool is_full() {
		return isFull && head == tail;
	}

	/*
	 * Returns true if the queue is empty.
	 */
	bool is_empty() {
		return (!isFull) && head == tail;
	}


	int size() {
		if (is_full())
			return RX_QUEUE_SIZE;
		else
			return (head - tail + RX_QUEUE_SIZE) % RX_QUEUE_SIZE;
	}
	/*
	 * Adds a frame to the front of the queue.
	 */
	void enqueue(const Frame& f) {
		if (!is_full()) {
			//Add frame to head
			buf[head++] = f;

			// Wrap Head
			if (head >= RX_QUEUE_SIZE) {
				head = 0;
			}

			//Check whether full
			if (head == tail) {
				isFull = true;
			}
		}
		else { // If full, we overwrite the last added message.
			buf[(head + RX_QUEUE_SIZE - 1) % RX_QUEUE_SIZE] = f;
		}
	}

	/*
	 * Returns a frame from the back of the queue.
	 */
	Frame dequeue() {
		if (!is_empty()) {
			//Update tail location
			uint8_t readloc = tail++;

			//Wrap tail
			if (tail >= RX_QUEUE_SIZE) {
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

private:
	Frame buf[RX_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;
	bool	isFull;
};

/* Frame Deque */
template <int Tsize>
class RX_Deque {
public:
	static const int RX_DEQUE_SIZE = Tsize;

	/*
	 * Constructor. Initializes the deque.
	 */
	RX_Deque() : head(0), tail(0) {}

	/*
	 * Returns true if the deque is full.
	 */
	bool is_full() {
		return isFull && head == tail;
	}

	/*
	 * Returns true if the deque is empty.
	 */
	bool is_empty() {
		return (!isFull) && head == tail;
	}

	/*
	 * Returns the size of the deque
	 */
	int size() {
		if (is_full())
			return RX_DEQUE_SIZE;
		else
			return (head - tail + RX_DEQUE_SIZE) % RX_DEQUE_SIZE;
	}

	/*
	 * Adds a frame at the head of the deque.
	 */
	void enqueue_head(const Frame& f) {
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


	/*
	 * Returns a frame from the head of the deque
	 */
	Frame dequeue_head() {
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

	/*
	 * Adds a frame at the tail of the deque.
	 */
	void enqueue_tail(const Frame& f) {
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

	/*
	 * Returns a frame from the tail of the deque
	 */
	Frame dequeue_tail() {
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

private:
	Frame buf[RX_DEQUE_SIZE];
	uint8_t head;
	uint8_t tail;
	bool	isFull;
};
#endif
