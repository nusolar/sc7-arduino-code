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
	RX_Queue() : head(0), tail(0) {}

	/*
	 * Returns true if the queue is full.
	 */
	bool is_full()
	{	return isFull && head == tail;    	}

	/*
	 * Returns true if the queue is empty.
	 */
	bool is_empty()
	{	return (!isFull) && head == tail;	}


        int size()
	{
    	  if (is_full())
    	    return RX_QUEUE_SIZE;
          else
    	    return (head - tail + RX_QUEUE_SIZE) % RX_QUEUE_SIZE; 
        }
	/*
	 * Adds a frame to the front of the queue.
	 */
	void enqueue(const Frame& frame);

	/*
	 * Returns a frame from the back of the queue.
	 */
	Frame dequeue();

private:
	Frame buf[RX_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;
	bool	isFull;
};

class RX_Deque{
public:
	static const int RX_DEQUE_SIZE = 8;

	/*
	 * Constructor. Initializes the deque.
	 */
	RX_Deque() : head(0), tail(0) {}

	/*
	 * Returns true if the deque is full.
	 */
	bool is_full()
	{	return isFull && head == tail;    	}

	/*
	 * Returns true if the deque is empty.
	 */
	bool is_empty()
	{	return (!isFull) && head == tail;	}

	/*
	 * Returns the size of the deque
	 */
	int size()
	{
		if (is_full())
			return RX_DEQUE_SIZE;
		else
			return (head - tail + RX_DEQUE_SIZE) % RX_DEQUE_SIZE; 
	}

	/*
	 * Adds a frame at the head of the deque.
	 */
	void enqueue_head(const Frame& frame);

	/*
	 * Returns a frame from the head of the deque
	 */
	Frame dequeue_head();

	/*
	 * Adds a frame at the tail of the deque.
	 */
	void enqueue_tail(const Frame& frame);

	/*
	 * Returns a frame from the tail of the deque
	 */
	Frame dequeue_tail();
private:
	Frame buf[RX_DEQUE_SIZE];
	uint8_t head;
	uint8_t tail;
	bool	isFull;
};
#endif
