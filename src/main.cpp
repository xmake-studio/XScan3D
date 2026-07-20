#include <Arduino.h>

#include "ahrs.h"
#include "scanner.h"

#define RED_LASER_PIN 29
#define ARGB_LED_PIN  16

// The work is split across both RP2040 cores:
//
//   core0  setup()/loop()   the Scanner -- motor, lidar UART, USB protocol
//   core1  setup1()/loop1() the AHRS -- I2C, IMU, Madgwick filter
//
// Core1 exists to hold the filter's 100 Hz tick to a real 100 Hz; on core0 it
// was scheduled behind the step train and the lidar drain, and Madgwick
// integrates with a fixed dt whether or not the sample arrived on time. See
// ahrs.h for the ownership rules -- the short version is that Serial belongs
// to core0 alone and I2C to core1 alone, and neither core waits on the other.
//
// Send 's' over the serial port to run a sweep; see protocol.h for the rest of
// the commands, and tools/scan3d.py for the host side.
Scanner scanner;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  pinMode(RED_LASER_PIN, OUTPUT);
  digitalWrite(RED_LASER_PIN, HIGH);

  // Ends with ahrs::begin(), which is what releases core1 below.
  scanner.begin();
}

void loop() {
  scanner.update();
}

// Defining setup1() is what makes the core rp2040 framework launch core1 at
// all. Both cores are running by the time core0 reaches setup(), so core1Main()
// blocks on its own start gate until core0 has finished claiming its pins.
void setup1() {}

void loop1() {
  ahrs::core1Main();  // never returns
}
