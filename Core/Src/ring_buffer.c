/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "ring_buffer.h"


bool RingBuffer_Init(RingBuffer *ringBuffer, char *dataBuffer, size_t dataBufferSize) 
{
	assert(ringBuffer);
	assert(dataBuffer);
	assert(dataBufferSize > 0);
	
	if ((ringBuffer) && (dataBuffer) && (dataBufferSize > 0)) {
	  //TODO
	  ringBuffer->buffer = dataBuffer;
	  ringBuffer->tail = 0;
	  ringBuffer->head = 0;
	  ringBuffer->max = dataBufferSize;
	  ringBuffer->length = 0;
	  
	  return true;
	}
	

	return false;
}

bool RingBuffer_Clear(RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
	    ringBuffer->tail = 0;
	    ringBuffer->head = 0;
	    ringBuffer->length = 0;
        
        
        return true;
	}
	return false;
}

bool RingBuffer_IsEmpty(const RingBuffer *ringBuffer)
{
  assert(ringBuffer);	
	return !ringBuffer->length;
}

size_t RingBuffer_GetLen(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return ringBuffer->length;
	}
	return 0;
	
}

size_t RingBuffer_GetCapacity(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return ringBuffer->max;
	}
	return 0;	
}


bool RingBuffer_PutChar(RingBuffer *ringBuffer, char c)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		
		if(ringBuffer->max - ringBuffer->length){ //jest miejsce
		    ringBuffer->buffer[ringBuffer->head] = c;
		    ringBuffer->length++;
		    
		   
		   ringBuffer->head = (ringBuffer->head + 1);
		   if(ringBuffer->head == (ringBuffer->max)){
		       ringBuffer->head = 0;
		   }
		   
		
		   return true; 
		}
		
		
	}
	return false;
}

bool RingBuffer_GetChar(RingBuffer *ringBuffer, char *c)
{
	assert(ringBuffer);
	assert(c);
	
  if ((ringBuffer) && (c)) {
		
		if(RingBuffer_IsEmpty(ringBuffer)){
		    return false;
		}
		
		*c = ringBuffer->buffer[ringBuffer->tail];
		
		ringBuffer-> tail = (ringBuffer->tail + 1);
		   if(ringBuffer-> tail == ringBuffer->max){
		       ringBuffer->tail = 0;
		   }
		ringBuffer->length--;
		return true;
	}
	return false;
}
