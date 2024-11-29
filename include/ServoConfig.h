// include/ServoConfig.h

#ifndef SERVOCONFIG_H
#define SERVOCONFIG_H

#define NUM_SERVOS 5

// Servo Names
const char* SERVO_NAMES[NUM_SERVOS] = {"base", "shoulder", "elbow", "wrist", "claw"};

// User-Defined Min and Max Angles for Each Servo (Degrees)
const int SERVO_MIN_ANGLES[NUM_SERVOS] = {0, 35, 30, 0, 40};
const int SERVO_MAX_ANGLES[NUM_SERVOS] = {180, 115, 165, 180, 135};

// Servo Inversion Mask: true = inverted, false = normal
const bool SERVO_INVERT_MASK[NUM_SERVOS] = {false, false, true, true, false};

// User-Specified Starting Angles for Each Servo (Degrees)
const int SERVO_START_ANGLES[NUM_SERVOS] = {90, 35, 160, 90, 120};

// Shared Servo Degree Limits for Mapping
const int SERVO_MIN_DEGREE = 0;
const int SERVO_MAX_DEGREE = 180;

// **New Definitions for Pulse Widths**
const int SERVO_MIN_PULSE_WIDTH = 184;  // Corresponds to 900 µsec
const int SERVO_MAX_PULSE_WIDTH = 430;  // Corresponds to 2100 µsec

// **I2C Address Definitions**
#define I2C_ADDRESS_NANO     0x08  // I2C address for Arduino Nano
#define I2C_ADDRESS_PWMDRV   0x40  // I2C address for PWM Servo Driver (default 0x40)

#endif // SERVOCONFIG_H