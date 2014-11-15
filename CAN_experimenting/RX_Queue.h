/*
 * RX_Queue.h
 * Contains definition for the RX_Queue class.
 */

#ifndef RX_Queue_h
#define RX_Queue_h

#include <stdint.h>
#include "MCP2515_defs.h"

/*
 * Static receiving queue for CAN_IO class. Holds frames that
 * have come in over the CAN bus.
 */
class RX_Queue {
public:
	static const int RX_QUEUE_SIZE = 8;

	/*
	 * Constructor. Initializes the queue.
	 */
	RX_Queue() : old_index(0), new_index(0) {}

	/*
	 * Returns true if the queue is full.
	 */
	bool is_full();

	/*
	 * Returns true if the queue is empty.
	 */
	bool is_empty();

	/*
	 * Adds a frame to the front of the queue.
	 */
	void enqueue(Frame& frame);

	/*
	 * Returns a frame from the back of the queue.
	 */
	Frame dequeue();
private:
	Frame buffer[RX_QUEUE_SIZE];
	uint16_t old_index;
	uint16_t new_index;
};

#endif