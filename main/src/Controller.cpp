#include "Controller.h"

// --------Config--------
// Contructor
Controller::Controller() {
    
}   

// Set gains
void Controller::setGains(float *e_in, uint8_t _N_e, float *m_in, uint8_t _N_m) {
    // Define number of elements
    N_e = _N_e;
    N_m = _N_m;

    // Copy coefficients
    int i;
    for(i = 0; i < N_e; i++) {
        e_gains[i] = e_in[i];
    }
    for(i = 0; i < N_m; i++) {
        m_gains[i] = m_in[i];
    }
}

// Set controller limits
void Controller::setRange(float min, float max) {
    m_min = min;
    m_max = max;
}

// --------Controller functions--------
float Controller::getControlEffort(float e) {
    float m = 0;
    uint8_t i;

    // Insert current error into e_history
    e_history.insert(e);

    // Add error components multiplied by their gains
    for(i = 0; i < N_e; i++) {
        float tmp = e_history.peek(i);
        m += e_gains[i] * tmp;
    }

    // Add control effort components multipled by their gains
    for(i = 0; i < N_m; i++) {
        float tmp = m_history.peek(i);
        m += m_gains[i] * tmp;
    }

    // Make sure control effort is within the limits
    m = saturateControlEffort(m);

    // Insert current control effort into m_history
    m_history.insert(m);

    return m;
}

float Controller::getControlEffort(float w_d, float w_a) {
    return getControlEffort( w_d - w_a );
}

// --------Debug--------
// Prints controller configurations
void Controller::printSettings() {
    uint8_t i;

    // Print e_gains
    //Serial.print("e_gains: ");
    //for(i = 0; i < N_e; i++) {
    //    Serial.print(e_gains[i]);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print m_gains
    //Serial.print("m_gains: ");
    //for(i = 0; i < N_m; i++) {
    //    Serial.print(m_gains[i]);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print m_limits
    //Serial.print("m_range: ");
    //Serial.print(m_min); Serial.print(" ");
    //Serial.print(m_max);

    //Serial.println();
}

// Prints ring buffers
void Controller::printHistory() {
    // Print e_gains
    //Serial.print("e_history: ");
    //e_history.printBuffer();

    // Print m_gains
    //Serial.print("m_history: ");
    //m_history.printBuffer();
}

// Prints error and control output
void Controller::printData() {
    //Serial.print("e: "); 
    //Serial.print(e_history.peek(0)); Serial.print(" ");
    //Serial.print(m_history.peek(0));
    //Serial.println();
}

// Helper function
float Controller::saturateControlEffort(float m) {
    if(m < m_min) {
        m = m_min;
    }
    else if(m > m_max) {
        m = m_max;
    }
    return m;
}