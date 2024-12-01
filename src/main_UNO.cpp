// src/main_UNO.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include "ServoConfig.h"  // Include the ServoConfig header

// Servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BUFFER_SIZE 50
#define COMMAND_PREFIX "moveServo"
#define COMMAND_PREFIX_MOVE_SERVO "moveServos"
#define COMMAND_PREFIX_USE_KNOBS "useKnobs"

bool useKnobsEnabled = false;

// Variables to store data from NANO
uint16_t knobValues[NUM_SERVOS];
uint16_t servo5Feedback = 0;

// Update the initial servo angles using ServoConfig.h
int servoAngles[NUM_SERVOS] = { 
    SERVO_START_ANGLES[0], 
    SERVO_START_ANGLES[1], 
    SERVO_START_ANGLES[2], 
    SERVO_START_ANGLES[3], 
    SERVO_START_ANGLES[4] 
};

// // Servo names stored in PROGMEM to save SRAM
// const char servoName0[] PROGMEM = "base";
// const char servoName1[] PROGMEM = "shoulder";
// const char servoName2[] PROGMEM = "elbow";
// const char servoName3[] PROGMEM = "wrist";
// const char servoName4[] PROGMEM = "claw";
// const char* const servoNames[NUM_SERVOS] PROGMEM = {
//     servoName0, servoName1, servoName2, servoName3, servoName4
// };

// Define baud rate constant
const uint32_t BAUD_RATE_UNO = 115200;

// Function prototypes
void TaskSerialCommand(void* pvParameters);
void TaskServoFeedback(void* pvParameters);
void TaskRequestData(void* pvParameters);
uint16_t degreeToPulseWidth(uint8_t degree, uint8_t servoIndex);
//int findServoIndex(const char* name);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    while (1);
}

void setup() {
    Serial.begin(BAUD_RATE_UNO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    Wire.begin();

    //Serial.println("Initializing PWM driver...");
    pwm.begin();
    pwm.setPWMFreq(50);  // Set to 50 Hz

    // Initialize servos to desired positions using ServoConfig.h
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint16_t pulseWidth = degreeToPulseWidth(servoAngles[i], i);  // Pass servo index
        pwm.setPWM(i, 0, pulseWidth);
        //Serial.print("Servo ");
        //Serial.print(i);
        //Serial.print(" initialized to ");
        //Serial.print(servoAngles[i]);
        //Serial.println(" degrees.");
    }

    // Create the Serial Control Task
    BaseType_t xReturned;
    xReturned = xTaskCreate(
        TaskSerialCommand,   // Task function
        "SerialCommand",     // Task name
        256,                 // Stack size in words (adjusted for input buffering)
        NULL,                // Task parameter
        1,                   // Task priority
        NULL                 // Task handle
    );

    if (xReturned != pdPASS) {
        //Serial.println("TaskSerialControl creation failed.");
        while (1);
    } else {
        //Serial.println("TaskSerialControl created successfully.");
    }

    // Create the Servo Feedback Task
    xReturned = xTaskCreate(
        TaskServoFeedback,   // Task function
        "ServoFB",           // Task name
        128,                 // Stack size in words (adjust as needed)
        NULL,                // Task parameter
        1,                   // Task priority
        NULL                 // Task handle
    );

    if (xReturned != pdPASS) {
        //Serial.println("TaskServoFeedback creation failed.");
        while (1);
    } else {
        //Serial.println("TaskServoFeedback created successfully.");
    }

    // Create the TaskRequestData
    xTaskCreate(
        TaskRequestData,      // Task function
        "RequestData",        // Task name
        128,                  // Stack size
        NULL,                 // Task parameter
        1,                    // Task priority
        NULL                  // Task handle
    );

    //Serial.println("Starting scheduler...");
    vTaskStartScheduler();

    //Serial.println("Scheduler failed to start.");
    while (1);
}

void loop() {
    // Empty. All work is done in tasks.
}

// Task for reading servo position commands
void TaskSerialCommand(void* pvParameters) {
    (void) pvParameters;

    char inputBuffer[BUFFER_SIZE];
    uint8_t index = 0;
    char c;
    char latestCommand[BUFFER_SIZE] = {0};  // Buffer to store the latest complete command

    //Serial.println("TaskSerialCommand started. Awaiting 'moveServos' commands...");

    // Define TickType_t for task timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = pdMS_TO_TICKS(100); // 100 ms interval

    for (;;) {
        // Read all available serial data
        while (Serial.available() > 0) {
            c = Serial.read();

            // Check for end of line
            if (c == '\n' || c == '\r') {
                if (index > 0) {
                    inputBuffer[index] = '\0';  // Null-terminate the string

                    // Store the latest complete command
                    strncpy(latestCommand, inputBuffer, BUFFER_SIZE - 1);
                    latestCommand[BUFFER_SIZE - 1] = '\0';  // Ensure null-termination

                    // Reset buffer index for next input
                    index = 0;
                }
            }
            else {
                // Overwrite buffer to keep only the latest data
                if (index < (BUFFER_SIZE - 1)) {
                    inputBuffer[index++] = c;
                }
                else {
                    // Buffer full, shift left to discard oldest character
                    memmove(inputBuffer, inputBuffer + 1, BUFFER_SIZE - 2);
                    inputBuffer[BUFFER_SIZE - 2] = c;
                    inputBuffer[BUFFER_SIZE - 1] = '\0';
                }
            }
        }

        // After reading all data, process only the latest complete command
        if (strlen(latestCommand) > 0) {
            // Check if the command starts with "moveServos"
            if (strncmp(latestCommand, COMMAND_PREFIX_MOVE_SERVO, strlen(COMMAND_PREFIX_MOVE_SERVO)) == 0) {
                int servoValues[NUM_SERVOS];
                // Parse the command with five degree values
                int parsed = sscanf(latestCommand, "moveServos %d %d %d %d %d",
                                    &servoValues[0],
                                    &servoValues[1],
                                    &servoValues[2],
                                    &servoValues[3],
                                    &servoValues[4]);
                if (parsed == NUM_SERVOS) {
                    bool valid = true;
                    // Validate each servo index and degree
                    for (int i = 0; i < NUM_SERVOS; i++) {
                        if (servoValues[i] < SERVO_MIN_ANGLES[i] || servoValues[i] > SERVO_MAX_ANGLES[i]) {
                            //Serial.print("Error: Degree out of range for servo ");
                            //Serial.println(i);
                            valid = false;
                            break;
                        }
                    }

                    if (valid) {
                        // Convert degrees to pulse widths and set PWM for each servo
                        for (int i = 0; i < NUM_SERVOS; i++) {
                            uint16_t pulseWidth = degreeToPulseWidth(servoValues[i], i);
                            pwm.setPWM(i, 0, pulseWidth);
                        }
                        //Serial.println("Moved all servos to specified degrees.");
                    }
                }
                else {
                    //Serial.println("Error: Invalid command format. Use 'moveServos [value0] [value1] [value2] [value3] [value4]'.");
                }
            } else if (strncmp(latestCommand, COMMAND_PREFIX_USE_KNOBS, strlen(COMMAND_PREFIX_USE_KNOBS)) == 0) {
                int knobValue;
                int parsed = sscanf(latestCommand, "useKnobs %d", &knobValue);
                if (parsed == 1) {
                    bool newState = (knobValue != 0);
                    if (newState != useKnobsEnabled) {
                        useKnobsEnabled = newState;
                        // Send detailed ACK with the new state
                        Serial.print("ACK: useKnobsEnabled=");
                        Serial.println(useKnobsEnabled ? "true" : "false");
                    }
                }
            } else {
                //Serial.println("Error: Unknown command. Use 'moveServos [value0] [value1] [value2] [value3] [value4]'.");
            }

            // Clear the latestCommand buffer after processing
            latestCommand[0] = '\0';
        }

        // Wait for the next tick to maintain consistent timing
        xTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}

// Task to read servo feedback and send data for plotting
void TaskServoFeedback(void* pvParameters) {
    (void) pvParameters;

    const uint8_t numFeedbackPins = 4;
    const uint8_t feedbackPins[numFeedbackPins] = {A0, A1, A2, A3};
    uint16_t analogValues[numFeedbackPins];
    uint16_t degrees[numFeedbackPins];
    // Remove local servo5Feedback
    // uint16_t servo5Feedback = 0;  // Variable to store fifth servo feedback

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = pdMS_TO_TICKS(100); // 100 ms interval

    for (;;) {
        // Read analog inputs for the first four servos
        for (uint8_t i = 0; i < numFeedbackPins; i++) {
            analogValues[i] = analogRead(feedbackPins[i]);
            degrees[i] = analogValues[i];  // Write as voltage
        }

        // Use servo5Feedback from TaskRequestData
        // Remove independent I2C request
        // Wire.requestFrom(I2C_ADDRESS_NANO, 2);  // Removed
        // if (Wire.available() >= 2) {
        //     uint8_t highByte = Wire.read();
        //     uint8_t lowByte = Wire.read();
        //     servo5Feedback = (highByte << 8) | lowByte;
        // } else {
        //     servo5Feedback = 0;  // Default or handle error
        // }

        // Use the shared servo5Feedback variable
        // Assuming servo5Feedback is updated by TaskRequestData
        // No need to declare a local variable

        // Create a comma-separated string including the fifth servo
        char dataString[40];
        snprintf(dataString, sizeof(dataString), ">%d,%d,%d,%d,%d",
                 degrees[0], degrees[1], degrees[2], degrees[3], servo5Feedback);

        // Send the data string over Serial
        Serial.println(dataString);

        // Delay for 100 ms before next reading
        vTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}

// Create a new task to request data from NANO
void TaskRequestData(void* pvParameters) {
    (void) pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = pdMS_TO_TICKS(100); // 100 ms interval

    for (;;) {
        // Request data from NANO via I2C
        Wire.requestFrom(I2C_ADDRESS_NANO, NUM_SERVOS * 2 + 2); // KnobValues + servo5Feedback

        // Read knob values
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (Wire.available() >= 2) {
                uint8_t highByte = Wire.read();
                uint8_t lowByte = Wire.read();
                knobValues[i] = (highByte << 8) | lowByte;
            }
        }

        // Read servo5Feedback
        if (Wire.available() >= 2) {
            uint8_t highByte = Wire.read();
            uint8_t lowByte = Wire.read();
            servo5Feedback = (highByte << 8) | lowByte;
        }

        // If useKnobs is enabled, move servos based on knobValues
        if (useKnobsEnabled) {
            for (uint8_t i = 0; i < NUM_SERVOS; i++) {
                // Map knob value to servo angle
                int degree = map(knobValues[i], 32, 1023, SERVO_MAX_ANGLES[i], SERVO_MIN_ANGLES[i]);
                servoAngles[i] = degree;

                // Move servo to the new angle
                uint16_t pulseWidth = degreeToPulseWidth(servoAngles[i], i);
                pwm.setPWM(i, 0, pulseWidth);
            }
        }

        // Delay for 100 ms
        vTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}

// Utility function to convert degrees to pulse width using a fixed mapping
uint16_t degreeToPulseWidth(uint8_t degree, uint8_t servoIndex) {
    degree = constrain(degree, SERVO_MIN_ANGLES[servoIndex], SERVO_MAX_ANGLES[servoIndex]);

    if (SERVO_INVERT_MASK[servoIndex]) {
        // Invert the degree mapping
        degree = SERVO_MIN_ANGLES[servoIndex] + SERVO_MAX_ANGLES[servoIndex] - degree;
    }
    
    return map(degree, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
}

// // Function to find servo index based on name
// int findServoIndex(const char* name) {
//     char storedName[20];
//     for (uint8_t i = 0; i < NUM_SERVOS; i++) {
//         strcpy_P(storedName, (char*)pgm_read_word(&(servoNames[i])));
//         if (strcasecmp(storedName, name) == 0) {
//             return i;
//         }
//     }
//     return -1;  // Not found
// }