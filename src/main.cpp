/*******************************************************************************
  Dennis van Gils
  09-11-2018
 ******************************************************************************/

#include <Arduino.h>
#include <math.h>
#include "DvG_SerialCommand.h"
#include "ZeroTimer.h"

// Serial   : Programming USB port
// SerialUSB: Native USB port. Baudrate setting gets ignored and is always as
//            fast as possible.
#define Ser       Serial
#define Ser_debug SerialUSB

/*
serial port   baud    BUFFER_SIZE  ISR_CLOCK   sim   remark
--------------------------------------------------------------------------------
Serial        115200  200          800         sin   gaps in receive
Serial        1e6     200          500         sin   fine
Serial        1e6     200          300         sin   fails, retest
Serial        1e6     200          300         ramp  fine

Serial        1e6     200          200         ramp  fine
Serial        1e6     200          100         ramp  number of bytes to unpack error
SerialUSB     1e6     200          200         ramp  fine
SerialUSB     1e6     200          100         ramp  number of bytes to unpack error

Serial        2e6     1000         200         ramp  fine
Serial        2e6     1000         100         ramp  number of bytes to unpack error
Serial        2e6     1000         150         ramp  communication lock-up
Serial        1.5e6   1000         150         ramp  communication lock-up
Serial        1.5e6   1000         200         ramp  fine

Serial        1.5e6   256          200         ramp  fine
SerialUSB     1.5e6   256          150         ramp  number of bytes to unpack error
Serial        1.5e6   256          150         ramp  borderline, gaps in receive
Serial        1.5e6   256          100         ramp  number of bytes to unpack error

TOP PICK
Serial        1.5e6   256          200         ramp  fine, results in Fs = 5000 Hz, perfect dt = 200 us
*/

#define BUFFER_SIZE 100
#define ISR_CLOCK   200     // [usec]

// Buffers
const char som[] = {0x00, 0x00, 0x00, 0x00, 0xee}; // start of message
const char eom[] = {0x00, 0x00, 0x00, 0x00, 0xff}; // end of message
const uint16_t DOUBLE_BUFFER_SIZE = 2 * BUFFER_SIZE;
volatile uint32_t buffer_time[DOUBLE_BUFFER_SIZE] = {0};
volatile float    buffer_wave[DOUBLE_BUFFER_SIZE] = {0};
const uint16_t num_bytes_time = BUFFER_SIZE * sizeof((uint32_t) 0);
const uint16_t num_bytes_wave = BUFFER_SIZE * sizeof((float) 0.0);
const uint8_t  num_bytes_som  = sizeof(som);
const uint8_t  num_bytes_eom  = sizeof(eom);

volatile bool fSendBuffer1 = false;
volatile bool fSendBuffer2 = false;
volatile uint8_t current_buffer = 1;

/*------------------------------------------------------------------------------
    Interrupt service routine
------------------------------------------------------------------------------*/

#define WAVE_FREQ 0.3   // [Hz]

void my_ISR() {
  // Generate wave sample every clock cycle
  uint32_t now = micros();
  static uint16_t i_buffer = 0;  // Position in double buffer
  static float wave = 0.0;
  
  //wave = sin(2*PI*WAVE_FREQ*now/1e6);  // ~ 280 usec to execute
  wave += 1;
  if (wave > BUFFER_SIZE * 3) {wave = 0;}
  
  // Store in buffers
  buffer_time[i_buffer] = now;
  buffer_wave[i_buffer] = wave;
  i_buffer++;
  
  if (i_buffer == BUFFER_SIZE) {
      fSendBuffer1 = true;
      current_buffer = 2;
  } else if (i_buffer == DOUBLE_BUFFER_SIZE) {
      fSendBuffer2 = true;
      current_buffer = 1;
      i_buffer = 0;
  }
}

/*------------------------------------------------------------------------------
    Setup
------------------------------------------------------------------------------*/

void setup() {
  Ser.begin(1500000);
  //Ser_debug.begin(9600);

  TC.startTimer(ISR_CLOCK, my_ISR);
}

/*------------------------------------------------------------------------------
    Loop
------------------------------------------------------------------------------*/

void loop() {
  if (fSendBuffer1 || fSendBuffer2) {
    uint16_t bytes_written = 0;
    uint16_t offset;
    
    if (fSendBuffer1) {
      //Ser_debug.print("buffer 1: ");
      offset = 0;
    } else {
      //Ser_debug.print("buffer 2: ");
      offset = BUFFER_SIZE;
    }
    
    bytes_written += Ser.write((uint8_t *) &som                , num_bytes_som);
    bytes_written += Ser.write((uint8_t *) &buffer_time[offset], num_bytes_time);
    bytes_written += Ser.write((uint8_t *) &buffer_wave[offset], num_bytes_wave);
    bytes_written += Ser.write((uint8_t *) &eom                , num_bytes_eom);
    if (fSendBuffer1) {fSendBuffer1 = false;}
    if (fSendBuffer2) {fSendBuffer2 = false;}

    //Ser_debug.println(bytes_written);
  }
}
