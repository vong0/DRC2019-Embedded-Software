/*
    Written by Matthew Vong
    - A ring buffer class to store data efficiently
    - A FIFO buffer that overwrites the first inserted element if
        we exceed the buffer size

*/
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "Arduino.h"
#include "Config.h"

//#define PRINT_FROM_ZERO  // changes how ring buffer is printed

// --------------Class definition--------------
template<class T>
class RingBuffer {
private:
    T buff[RING_BUFFER_SIZE];
    int head;

public:
    // Constructor
    RingBuffer();

    // Insert operation
    void insert(const T &obj);

    // Peek operation
    T peek(int i);

    // --------Debug--------
    void printBuffer();
    int positiveModulus(int x);     // helper function to wrap to valid index
};

// --------------Function definitions--------------
// Constructor
template<class T>
RingBuffer<T>::RingBuffer()
    : buff{}, head(0)  {
}

// Insert operation
template<class T>
void RingBuffer<T>::insert(const T &obj) {
    buff[head] = obj;
    head = positiveModulus(head+1);
}

// Peek operation - from head (0 = last element, 1 = element before last)
template<class T>
T RingBuffer<T>::peek(int i) {
    // Get index of member you want to see
    int ii = positiveModulus(head-i-1);
    return buff[ii];
}

// Prints the buffer
template<class T>
void RingBuffer<T>::printBuffer() {
    int end = positiveModulus(head-1);
    int i = positiveModulus(head);

    #ifndef PRINT_FROM_ZERO
    // PRINT METHOD 1: from latest element always last
    while(i != end) {
        //Serial.print(buff[i]);
        //Serial.print(" ");
        i = positiveModulus(i+1);
    }
    Serial.print(buff[end]);
    #endif

    #ifdef PRINT_FROM_ZERO
    // PRINT METHOD 2: from 0 to RING_BUFFER_SIZE
    for(int x = 0; x <RING_BUFFER_SIZE; x++) {
        //Serial.print(buff[x]);
        //Serial.print(" ");
    }
    #endif

    //Serial.println();
}

template<class T>
int RingBuffer<T>::positiveModulus(int x) {
    return (RING_BUFFER_SIZE + x) % RING_BUFFER_SIZE;
}

#endif