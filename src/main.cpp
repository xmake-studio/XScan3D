#include <Arduino.h>

#include "scanner.h"

#define RED_LASER_PIN 29

// Everything runs on core0: the motor, the lidar UART and the USB protocol.
// There used to be an AHRS on core1 fusing an IMU into a pose, because the
// lidar rode a tilting platform whose true attitude the step count only
// approximated. The lidar now sits directly on the stepper shaft, turning about
// the world vertical, so the step count is the pose exactly -- there is nothing
// left to measure and nothing left to fuse.
//
// Send 's' over the serial port to run a sweep; see protocol.h for the rest of
// the commands, and tools/scanner_ui.py for the host side.
Scanner scanner;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  pinMode(RED_LASER_PIN, OUTPUT);
  digitalWrite(RED_LASER_PIN, HIGH);

  scanner.begin();
}

void loop() {
  scanner.update();
}
