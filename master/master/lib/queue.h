//*************************************************************************************
/** \file queue.h
 *    This file implements a simple queue (or circular buffer). Data which is placed in 
 *    the queue is saved for future reading. The queue class is implemented as a 
 *    template; this means that a queue can be configured to store data of almost any
 *    type. 
 *
 *  Usage:
 *    The template specifies the type of data being stored as qType, the type of
 *    integer used for the index as qIndexType, and the number of data items in the
 *    queue as qSize. First choose the type of data being stored; this may be simple
 *    data types such as characters or integers, but it may also be something more
 *    complex such as objects of a class (if there's enough memory to hold them all).
 *    Then look at the number of items in the queue. If there are less than 255, use
 *    a character as qIndexType; otherwise use a regular 16-bit integer. For example,
 *    a queue holding 100 long integers (which would use 400 bytes for the data plus
 *    a few more bytes for the indices) could be declared as 
 *    "queue<long, char, 100> my_queue". 
 *
 *  Revisions:
 *    \li 08-28-03  JRR  Reference: www.yureu.com/logo_plotter/queue_code.htm
 *    \li 08-28-03  JRR  Ported to C++ from C
 *    \li 12-14-07  JRR  Doxygen comments added
 *    \li 02-17-08  JRR  Changed index type from define to template parameter
 *    \li 08-12-08  JRR  Added overloaded array subscript operator
 *
 *  License:
 *    This file copyright 2007-2008 by JR Ridgely. It is released under the Lesser GNU
 *    public license, version 2. It is intended for educational use only, but the
 *    user is free to use it for any purpose permissible under the LGPL. The author
 *    has no control over the use of this file and cannot take any responsibility
 *    for any consequences of this use. 
 */
//*************************************************************************************

/// These defines prevent this file from being included more than once in a *.cc file
#ifndef _QUEUE_H_
#define _QUEUE_H_
	// s

//-------------------------------------------------------------------------------------
/** This class implements a simple queue. As a template class, it can be used to 
 *  create queues which hold any type of object whose data is copied in with an
 *  assignment operator. The size and type of object are determined at compile time
 *  for efficiency. 
 */

template <class qType, class qIndexType, qIndexType qSize> 
class queue
{
	protected:
		qType buffer[qSize];			///< This memory buffer holds the contents
		qIndexType i_put;			   ///< Index where newest data will be written
		qIndexType i_get;			   ///< Index where oldest data was written
		qIndexType how_full;			///< How many elements are full at this time

	public:

		queue (void);				// Constructor
		bool put (qType);			// Adds one item into queue
		bool jam (qType);			// Force entry even if queue full
		qType get (void);			// Gets an item from the queue
		bool is_empty (void);		// Is the queue empty or not?
		void flush (void);			// Empty out the whole buffer
		void delete_one (void);		// Delete the last entry

		// This operator returns an item at the given index in the queue
		qType operator[] (qIndexType);

		/** This method returns the number of items in the queue */
		qIndexType num_items (void) { return (how_full); }
		
		/** This method returns a pointer to the buffer which holds the data in the 
		 *  queue. Normally it's not used, but in some cases (particularly queues of
		 *  characters) one may need to access the data buffer directly.
		 *  @return A pointer to the buffer holding the data in the queue
		 */
		qType* get_p_buffer (void) { return buffer; }
};


//-------------------------------------------------------------------------------------
/** This constructor creates a queue item.  Note that it doesn't have to allocate 
 *  memory because that was already done statically by the template mechanism at 
 *  compile time. If no parameters are supplied, a 16-element queue for characters 
 *  which uses 8-bit array indices (for a maximum of 255 elements) is created. 
 *  The template parameters are as follows: 
 *	\li qType: The type of data which will be stored in the queue (default char)
 *	\li qIndexType: The type of data used for array indices
 *	\li qSize: The number of items in the queue's memory buffer (default 16)
 */ 

template <class qType, class qIndexType, qIndexType qSize> 
queue<qType, qIndexType, qSize>::queue (void)
{
	flush ();
}


//-------------------------------------------------------------------------------------
/** This method empties the buffer. It doesn't actually erase everything; it just sets
 *  the indices and fill indicator all to zero as if the buffer contained nothing.
 */

template <class qType, class qIndexType, qIndexType qSize> 
void queue<qType, qIndexType, qSize>::flush (void)
{
	i_put = 0;
	i_get = 0;
	how_full = 0;
}


//-------------------------------------------------------------------------------------
/** This method adds an item into the queue. If the buffer is full then nothing is 
 *  written and true (meaning buffer is full) is returned. This allows the calling
 *  program to see that the data couldn't be written and try again if appropriate. 
 *  @param data The data to be written into the queue 
 *  @return True if the buffer was full and data was not written, false otherwise
 */

template <class qType, class qIndexType, qIndexType qSize> 
bool queue<qType, qIndexType, qSize>::put (qType data)
{
	// Check if the buffer is already full
	if (how_full >= qSize)
		return true;

	// OK, there's room in the buffer so add the data in
	buffer[i_put] = data;
	if (++i_put >= qSize) 
		i_put = 0;
	how_full++;

	return false;
}


//-------------------------------------------------------------------------------------
/** This method jams an item into the queue whether the queue is empty or not.  This 
 *  can overwrite existing data, so it must be used with caution.
 *  @param data The data to be jammed into the queue
 *  @return True if some data was lost, false otherwise
 */

template <class qType, class qIndexType, qIndexType qSize> 
bool queue<qType, qIndexType, qSize>::jam (qType data)
{
	// Write the data and move the write pointer to the next element
	buffer[i_put] = data;
	if (i_put >= qSize) 
		i_put = 0;

	// Check if the buffer is already full; if so, the read index has to be moved so
	// that it points to the oldest unread data, which isn't the data written now
	if (how_full >= qSize)
	{
		if (++i_get >= qSize)
			i_get = 0;
		return true;
	}
	else
	{
		how_full++;						 // If we get here, the buffer isn't full
		return false;
	}
}


//-------------------------------------------------------------------------------------
/** This method returns the oldest item in the queue.  If the queue was empty, this is
 *  already-retreived data.  Somebody should have checked if there was new data 
 *  available using the is_empty() method.
 *  @return The data which is pulled out from the queue at the current location
 */

template <class qType, class qIndexType, qIndexType qSize> 
qType queue<qType, qIndexType, qSize>::get (void)
{
	qType whatIgot;						 // Temporary storage for what was read

	whatIgot = buffer[i_get];			   // Read and hold the data

	if (how_full > 0)					   // If the buffer's not empty,
	{								   // moveflush the read pointer to the next full
		if (++i_get >= qSize)			   // element
			i_get = 0;
		how_full--;						 // There's now one less item in the buffer
	}

	return (whatIgot);
}


//-------------------------------------------------------------------------------------
/** This method returns true if the queue is empty.  Empty means that there isn't 
 *  any data which hasn't previously been read.
 *  @return True if the queue has unread data, false if it doesn't
 */

template <class qType, class qIndexType, qIndexType qSize> 
bool queue<qType, qIndexType, qSize>::is_empty (void)
{
	return (how_full == 0);
}


//-------------------------------------------------------------------------------------
/** This overloaded array subscript operator implements subscripts in the queue. The 
 *  index is defined relative to the location at which the oldest data in the queue
 *  was written. This operator returns the (index)-th item in the queue, or (-1) 
 *  typecast to the queue data type if there aren't enough items in the queue to get 
 *  to the given index. The retreived data is not removed from the queue. 
 *  @param index The array index into the queue at which to get a data item
 *  @return The data which is at the given index
 */

template <class qType, class qIndexType, qIndexType qSize> 
qType queue<qType, qIndexType, qSize>::operator[] (qIndexType index)
{
	// Check if there's data written at the given location
	if (index > how_full)
		return ((qType)(-1));

	// Find an index pointing to the correct location in the queue
	qIndexType getIndex = i_get + index;
	if (getIndex > qSize)
		getIndex -= qSize;

	// Get the data at that index location
	return (buffer[getIndex]);
}

//-------------------------------------------------------------------------------------
/** This method decrements the i_put and how_full variables to "delete" the latest character.
 */

template <class qType, class qIndexType, qIndexType qSize> 
void queue<qType, qIndexType, qSize>::delete_one (void)
{
	i_put--;
	how_full--;
}

#endif // _QUEUE_H_
