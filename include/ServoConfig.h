// include/ServoConfig.h

#ifndef SERVOCONFIG_H
#define SERVOCONFIG_H

#define NUM_SERVOS 5

// Servo Names
const char* SERVO_NAMES[NUM_SERVOS] = {"base", "shoulder", "elbow", "wrist", "claw"};

// User-Defined Min and Max Angles for Each Servo (Degrees)
const int SERVO_MIN_ANGLES[NUM_SERVOS] = {10, 20, 15, 30, 25};  // Example minimum angles
const int SERVO_MAX_ANGLES[NUM_SERVOS] = {170, 160, 165, 150, 155};  // Example maximum angles

// User-Specified Starting Angles for Each Servo (Degrees)
const int SERVO_START_ANGLES[NUM_SERVOS] = {90, 50, 45, 90, 120};  // Example starting angles

#endif // SERVOCONFIG_H