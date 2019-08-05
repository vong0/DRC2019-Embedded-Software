#include "PathReader.h"

void PathReader::begin(uint32_t baudRate) {
    nucSerial.begin(baudRate);
    debugSerial.begin(baudRate);
}

// Read path data
bool PathReader::readPathData(uint8_t c) {
    // Load byte into rx
    rx = c;
    // If we have already read the previous data
    if(readFlag) {
        // Debug serial
        //debugSerial.print(c,HEX); debugSerial.print(" ");

        // STATE 1: Check header
        if(state == STATE_HEADER) {
            //debugSerial.println("\nLoad header");
            loadHeader();
        }
        // STATE 2: Load bytes
        else if (state == STATE_BYTES) {
            //debugSerial.println("\nLoad bytes");
            loadBytes();
        }
        // STATE 2a: Load pose
        else if(state == STATE_POSE) {
            //debugSerial.println("\nLoad pose");
            loadPose();
        }
        // STATE 3: Load floats
        else if (state == STATE_FLOATS) {
            //debugSerial.println("\nLoad floats");
            loadFloats();
        }
        // STATE 4: Load checksum
        else if (state == STATE_CHECKSUM) {
            //debugSerial.println("\nLoad checksum");
            loadCheckSum();

             // Check checksum
            if (checkSum == verifyCheckSum) {
              readFlag = false;
            }
        }
    }
    return readFlag;
}

bool PathReader::readStatus() {
  return readFlag;
}

void PathReader::setReadFlag() {
  readFlag = true;
}

void PathReader::clearReadFlag() {
  readFlag = false;
  i = 0;
  floatCounter = 0;
  state = STATE_HEADER;
}

void PathReader::sendPose(const Pose& robot) {
    // Send header
    nucSerial.write(HANDSHAKE);
    nucSerial.write(sizeof(Pose) + 1);

    // Send bytes to serial
    unsigned char checksum = sizeof(Pose) + 1;
    unsigned char *bytePtr = (unsigned char*)&robot;
    for(int j = 0; j  < sizeof(Pose); j ++) {
        nucSerial.write( *(bytePtr+j ) );
        checksum += *(bytePtr + j );
    }
    nucSerial.write(checksum);
}

// Verify if path data loaded correctly
bool PathReader::verifyPathData() {
    if (checkSum != verifyCheckSum) {
        readFlag = true;  // Reset flag
        return false;
    }
    return true;
}

// Return copy of path data
PathData PathReader::getPathData() {
    return path;
}

// Return ptr of path data
PathData* PathReader::getPathDataPtr() {
    return &path;
}

// Return copy of pose data
Pose PathReader::getPose() {
    return pose;
}

// State machines
void PathReader::loadHeader() {
    header = rx;

    // Check if header is correct
    if (header == HANDSHAKE) {
        state = STATE_BYTES;
    }
    // If incorrect data - flush the buffer
    else {
        // debugSerial.println("Incorrect data");
    }
}

void PathReader::loadBytes() {
    // Update bytes
    noBytes = rx;

    // Update N
    path.N = noBytes/8;
    state = STATE_FLOATS;
    // state = STATE_POSE;
    verifyCheckSum =noBytes;
}

void PathReader::loadPose() {
    // Create byte ptr
    unsigned char *rxPtr = (unsigned char *)&pose;

    // Load byte
    rxPtr[i++] = rx;

    // Next stage
    if(i >= sizeof(Pose)) {
        state = STATE_FLOATS;
    }
}

void PathReader::loadFloats() {
    // Create byte ptr
    unsigned char *rxPtr;

    // Even is x
    if((i/4)%2 == 0) {
        rxPtr = (unsigned char*)path.cx;
    }
    // Odd is y
    else {
        rxPtr = (unsigned char*)path.cy;
    }

    // Load byte into array
    unsigned char numFloat = i/8;
    rxPtr[floatCounter+numFloat*4] = rx;

    // Update bytes
    floatCounter++; i++;

    if(floatCounter >= 4) floatCounter = 0;

    // Count checksum
    verifyCheckSum += rx;

    // Check bytes
    if (i >= noBytes-1) {
        state = STATE_CHECKSUM;
        i = 0;
    }
}

void PathReader::loadCheckSum() {
    checkSum = rx;
    state = STATE_HEADER;
}


// Print data
void PathReader::printPathData() {
    // Print header
    debugSerial.println(header);

    // Print bytes
    debugSerial.print("N: ");
    debugSerial.print(noBytes);
    debugSerial.println();

    // Print number
    debugSerial.print("no_entry: "); debugSerial.println(path.N);

    // Print float array
    debugSerial.print("cx: ");
    for (int x = 0; x < path.N; x++) {
        debugSerial.print(path.cx[x]);
        debugSerial.print(" ");
    }
    debugSerial.println();

    debugSerial.print("cy: ");
    for (int x = 0; x < path.N; x++) {
        debugSerial.print(path.cy[x]);
        debugSerial.print(" ");
    }
    debugSerial.println();

    // Print checkSum and verifyCheckSum
    debugSerial.print("checkSum: ");
    debugSerial.print(checkSum);

    debugSerial.print("\tverifyCheckSum: ");
    debugSerial.print(verifyCheckSum);

    if(checkSum != verifyCheckSum) {
        debugSerial.print("\nERROR!!!!!!!!!!!!!!!!!!!!!! CHECKSUM DOES NOT MATCH");
    }
    debugSerial.println();
}
