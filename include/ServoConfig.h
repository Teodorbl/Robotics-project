// include/ServoConfig.h

#ifndef SERVOCONFIG_H
#define SERVOCONFIG_H

const uint8_t NUM_SERVOS = 5;
const char* SERVO_NAMES[NUM_SERVOS] = {"base", "shoulder", "elbow", "wrist", "claw"};

// User-Defined Min and Max Angles for Each Servo (Degrees)
const uint8_t SERVO_MIN_ANGLES[NUM_SERVOS]     = {  0,  35,  30,   0,  40};
const uint8_t SERVO_DEFAULT_ANGLES[NUM_SERVOS] = { 90,  35, 160,  90, 120};
const uint8_t SERVO_MAX_ANGLES[NUM_SERVOS]     = {180, 115, 165, 180, 135};

// Servo Inversion Mask: true = inverted, false = normal
const bool SERVO_INVERT_MASK[NUM_SERVOS] = {false, false, true, true, false};

// User-Specified Starting Angles for Each Servo (Degrees)

// Shared Servo Degree Limits for Mapping
const uint8_t SERVO_MIN_DEGREE = 0;
const uint8_t SERVO_MAX_DEGREE = 180;

// **New Definitions for Pulse Widths**
const uint16_t SERVO_MIN_PULSE_WIDTH = 184;  // Corresponds to 900 µsec
const uint16_t SERVO_MAX_PULSE_WIDTH = 430;  // Corresponds to 2100 µsec

const uint8_t I2C_ADDRESS_PWMDRV = 0x40;   // I2C address for PWM Servo Driver (default 0x40)
const uint8_t I2C_ADDRESS_NANO = 0x08;     // I2C address for Arduino Nano

// Define baud rate constants
//const uint32_t BAUD_RATE_UNO = 115200;
const uint32_t BAUD_RATE_UNO = 57600;
const uint32_t BAUD_RATE_NANO = 57600;

const uint8_t PWM_FREQ = 50;

#endif // SERVOCONFIG_H