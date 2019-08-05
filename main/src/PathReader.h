#ifndef PATH_READER_H
#define PATH_READER_H

#include "Arduino.h"
#include "Pose.h"
#include "Config.h"

// State machine states
#define STATE_HEADER 0
#define STATE_BYTES 1
#define STATE_POSE 2
#define STATE_FLOATS 3
#define STATE_CHECKSUM 4

// Handshake
#define HANDSHAKE 0xF5

// Union for float data
typedef union {
  float data;
  byte bytes[4];
} binaryFloat;

// Union for integer data
typedef union {
  uint32_t data;
  byte bytes[4];
} binaryLongInt;

// Path reader
class PathReader {
private:
    // Byte header
    byte header;                // header
    uint8_t state;              // state
    uint8_t i;                  // current number of bytes
    uint8_t checkSum;           // checksum

    // Verification
    uint8_t noBytes;            // Number of bytes
    uint8_t verifyCheckSum;     // checksum to verify
    bool valid;                 // check if valid data
    bool flag;                  // flag to print
    binaryFloat my_float;       // to store float bytes

    // Interrupts
    bool readFlag = true;       // flag to tell if path has been read
    uint8_t rx;                 // received byte
    uint8_t floatCounter;       // counts 4 bytes
    
    // Path/pose data
    PathData path;
    Pose pose;

public:
    // Config serial ports
    void begin(uint32_t baudRate = BAUD_RATE);

    // Read path data
    bool readPathData(uint8_t c);
    bool readStatus();
    void setReadFlag();
    void clearReadFlag();
    void sendPose(const Pose& robot);

    // Verify if path data loaded correctly
    bool verifyPathData();

    // Return copy of path data
    PathData getPathData();
    PathData* getPathDataPtr();
    Pose getPose();

    // Helper functions - read path state machine
    void loadHeader();
    void loadBytes();
    void loadPose();
    void loadFloats();
    void loadCheckSum();

    // Helper functions
    void clearInputBuffer();

    // Debug
    void printPathData();
};

#endif