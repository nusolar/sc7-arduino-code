/*
 * RX_Queue.h
 * Contains definition for the RX_Queue class.
 */

#ifndef RX_Queue_h
#define RX_Queue_h

#include <stdint.h>

/*
 * Static receiving queue for CAN_IO class. Holds frames that
 * have come in over the CAN bus.
 */
class RX_Queue {
public:
	const int RX_QUEUE_SIZE = 8;

	/*
	 * Constructor. Initializes the queue.
	 */
	RX_Queue() : old_index(0), new_index(0) {}

	/*
	 * Returns the number of elements currently in the queue.
	 */
	uint16_t size();

	/*
	 * Adds a frame to the front of the queue.
	 */
	void push(Frame& frame);

	/*
	 * Returns a frame from the back of the queue.
	 */
	Frame pop();
private:
	Frame buffer[RX_QUEUE_SIZE];
	uint16_t old_index;
	uint16_t new_index;
};

#endif