// src/main_UNO.cpp

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <string.h>
#include <SoftwareWire.h>
#include "ServoConfig.h"
#include "ErrorCodes.h"

// Define new digital pins for I2C
#define SDA_PIN 2
#define SCL_PIN 3

SoftwareWire softWire(SDA_PIN, SCL_PIN);  // Create SoftwareWire instance

// Servo feedback pins
const uint8_t FEEDBACK_PINS[NUM_SERVOS] = {A0, A1, A2, A3, A4};

// Global variable to store analog feedback values
uint16_t feedbackValues[NUM_SERVOS] = {0};

// Array for commanded servo angles
uint8_t servoCommandAngles[NUM_SERVOS];

// Servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Declare a mutex for protecting servoCommandAngles and feedbackValues
SemaphoreHandle_t xFeedbackMutex;
SemaphoreHandle_t xServoCommandMutex;

// Task priority levels
#define PRIORITY_CONTROL_TASK 3
#define PRIORITY_SERIAL_TASK 2
#define PRIORITY_I2C_TASK 1

// Task interval and delays
#define CONTROL_LOOP_INTERVAL_MS 1000
#define SERIAL_DELAY_MS 1000
#define I2C_DELAY_MS 1000

// Define stack sizes for each task
#define CONTROL_LOOP_STACK_SIZE 256
#define SERIAL_TASK_STACK_SIZE 256
#define I2C_TASK_STACK_SIZE 128

#define ENABLE_ERROR 1  // Enables error prints
#define ENABLE_DEBUG 1  // Enables debug prints

// Modified logging macros to accept optional data arguments
#if ENABLE_ERROR
    #define LOG_ERROR(code, ...) \
        do { \
            Serial.print(F("E:")); \
            Serial.print(static_cast<uint8_t>(code)); \
            Serial.print(F(":")); \
            if (sizeof(#__VA_ARGS__) > 1) { \
                Serial.println(__VA_ARGS__); \
            } else { \
                Serial.println(); \
            } \
        } while(0)
#else
    #define LOG_ERROR(code, ...)
#endif

#if ENABLE_DEBUG
    #define LOG_DEBUG(code, ...) \
        do { \
            Serial.print(F("D:")); \
            Serial.print(static_cast<uint8_t>(code)); \
            Serial.print(F(":")); \
            if (sizeof(#__VA_ARGS__) > 1) { \
                Serial.println(__VA_ARGS__); \
            } else { \
                Serial.println(); \
            } \
        } while(0)
#else
    #define LOG_DEBUG(code, ...)
#endif

// Function prototypes
void vControlLoopTask(void *pvParameters);
void vSerialTask(void *pvParameters);
void vI2CTask(void *pvParameters);
uint16_t degreeToPulseWidth(uint8_t degree, uint8_t servoIndex);
void ReadAnalogInputs();
void ComputeControl();
void UpdateServos();
void ProcessSerialCommands();
void SendDataToComputer();
void RequestI2CData();
void ProcessI2CData();

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    #ifdef LOG_ERROR
    LOG_ERROR(ERROR_STACK_OVERFLOW, pcTaskName);
    #endif

    #ifdef LOG_DEBUG
    // LOG_DEBUG(DEBUG_CONTROL_LOOP_START);
    #endif

    // Blink an LED to indicate stack overflow
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    while (1);
}

void setup() {
    Serial.begin(BAUD_RATE_UNO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    softWire.begin();  // Initialize SoftwareWire

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // Initialize servos to desired positions using ServoConfig.h
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoCommandAngles[i] = SERVO_DEFAULT_ANGLES[i];
        uint16_t pulseWidth = degreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);
    }

    xServoCommandMutex = xSemaphoreCreateMutex();
    if (xServoCommandMutex == NULL) {
        #ifdef LOG_ERROR
        LOG_ERROR(ERROR_MUTEX_CREATION_FAIL_SERVO);
        #endif
        while (1); // Halt if mutex creation fails
    }

    xFeedbackMutex = xSemaphoreCreateMutex();
    if (xFeedbackMutex == NULL) {
        #ifdef LOG_ERROR
        LOG_ERROR(ERROR_MUTEX_CREATION_FAIL_FEEDBACK);
        #endif
        while (1); // Halt if mutex creation fails
    }

    xTaskCreate(
        vControlLoopTask,                // Task function
        "ControlLoop",                   // Task name
        CONTROL_LOOP_STACK_SIZE,         // Stack size
        NULL,                            // Task parameters
        PRIORITY_CONTROL_TASK,           // Priority
        NULL                             // Task handle
    );

    xTaskCreate(
        vSerialTask,
        "SerialTask",
        SERIAL_TASK_STACK_SIZE,
        NULL,
        PRIORITY_SERIAL_TASK,
        NULL
    );

    xTaskCreate(
        vI2CTask,
        "I2CTask",
        I2C_TASK_STACK_SIZE,
        NULL,
        PRIORITY_I2C_TASK,
        NULL
    );

    vTaskStartScheduler();

    #ifdef LOG_ERROR
    LOG_ERROR(ERROR_SCHEDULER_START_FAIL);
    #endif
    
    while (1);
}

void vControlLoopTask(void *pvParameters) {
    const TickType_t xTimeIncrementControl = pdMS_TO_TICKS(CONTROL_LOOP_INTERVAL_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_CONTROL_LOOP_START);
    #endif

    for (;;) {
        // Wait for the next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementControl);

        // Read analog inputs (feedback)
        ReadAnalogInputs();

        // Perform computations (e.g., Kalman filter)
        ComputeControl();

        // Send commands to servos
        UpdateServos();

        #ifdef LOG_DEBUG
        LOG_DEBUG(DEBUG_CONTROL_LOOP_COMPLETE);
        #endif

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            #ifdef LOG_ERROR
            LOG_ERROR(ERROR_DELAYED_CONTROL_LOOP);
            #endif
        }
    }
}

void vSerialTask(void *pvParameters) {
    const TickType_t xTimeIncrementSerial = pdMS_TO_TICKS(SERIAL_DELAY_MS);

    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_TASK_START_SERIAL);
    #endif

    for (;;) {
        // Check for serial data
        if (Serial.available() > 0) {

            #ifdef LOG_DEBUG
            LOG_DEBUG(DEBUG_SERIAL_DATA_AVAILABLE);
            #endif

            // Read and process serial commands
            ProcessSerialCommands();
        }

        // Send data back to the computer
        SendDataToComputer();

        #ifdef LOG_DEBUG
        LOG_DEBUG(DEBUG_SERIAL_CYCLE_COMPLETE);
        #endif

        // Delay before the next read
        vTaskDelay(xTimeIncrementSerial);
    }
}

void vI2CTask(void *pvParameters) {
    const TickType_t xTimeIncrementI2C = pdMS_TO_TICKS(I2C_DELAY_MS);

    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_TASK_START_I2C);
    #endif

    for (;;) {
        // Request data from peripheral sensors
        RequestI2CData();

        // Process received data

        #ifdef LOG_DEBUG
        LOG_DEBUG(DEBUG_I2C_CYCLE_COMPLETE);
        #endif

        // Delay before the next request
        vTaskDelay(xTimeIncrementI2C);
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

void ReadAnalogInputs() {
    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_READING_ANALOG);
    #endif
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        feedbackValues[i] = analogRead(FEEDBACK_PINS[i]);

        #ifdef LOG_DEBUG
        // Replace String concatenation with separate Serial.print calls
        Serial.print(F("D:"));
        Serial.print(DEBUG_READING_ANALOG);
        Serial.print(F(":"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(feedbackValues[i]);
        #endif
    }
}

void ComputeControl() {
    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_COMPUTING_CONTROL);
    #endif
    // TODO: Implement control algorithms (e.g., Kalman filter)

    // Example error condition after implementing control algorithms
    // if (controlAlgorithmFailed) {
    //     #ifdef LOG_ERROR
    //     LOG_ERROR(F("Control algorithm failed."));
    //     #endif
    // }
}

void UpdateServos() {
    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_UPDATING_SERVOS);
    #endif
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint16_t pulseWidth = degreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);

        #ifdef LOG_DEBUG
        // Replace String concatenation with separate Serial.print calls
        Serial.print(F("D:"));
        Serial.print(DEBUG_UPDATING_SERVOS);
        Serial.print(F(":"));
        Serial.print(i);
        Serial.print(F(" updated to pulse width "));
        Serial.println(pulseWidth);
        #endif

        // Example error condition after setting PWM
        // if (pwmUpdateFailed) {
        //     #ifdef LOG_ERROR
        //     LOG_ERROR(F("Failed to update servo "), String(i));
        //     #endif
        // }
    }
}

// Define a designated start character for command lines
#define COMMAND_START_CHAR '>'
#define COMMAND_END_CHAR '\n'

void ProcessSerialCommands() {
    static char inputBuffer[100];
    static uint8_t bufferIndex = 0;
    bool newLineReceived = false;

    // Discard any characters until the start character is found
    while (Serial.available()) {
        char inChar = Serial.read();
        if (inChar == COMMAND_START_CHAR) {
            bufferIndex = 0; // Reset buffer index for a new command
            inputBuffer[bufferIndex++] = inChar;

            #ifdef LOG_DEBUG
            LOG_DEBUG(DEBUG_COMMAND_START);
            #endif
        } else if (bufferIndex > 0) {
            if (inChar == COMMAND_END_CHAR) {
                inputBuffer[bufferIndex] = '\0';
                newLineReceived = true;

                #ifdef LOG_DEBUG
                LOG_DEBUG(DEBUG_COMMAND_END);
                #endif
                break;
            } else if (bufferIndex < sizeof(inputBuffer) - 1) {
                inputBuffer[bufferIndex++] = inChar;
            }
        }
    }

    if (newLineReceived) {
        // Parse the input line for five integers after the start character
        uint8_t tempAngles[NUM_SERVOS];
        int parsed = sscanf(inputBuffer, "%*c %hhu %hhu %hhu %hhu %hhu",
                            &tempAngles[0], &tempAngles[1],
                            &tempAngles[2], &tempAngles[3],
                            &tempAngles[4]);
        if (parsed == NUM_SERVOS) {
            #ifdef LOG_DEBUG
            LOG_DEBUG(DEBUG_COMMAND_END);
            #endif
            // Acquire mutex before updating servoCommandAngles
            if (xSemaphoreTake(xServoCommandMutex, portMAX_DELAY)) {
                memcpy(servoCommandAngles, tempAngles, sizeof(tempAngles));
                xSemaphoreGive(xServoCommandMutex);
            }
        } else {
            #ifdef LOG_ERROR
            LOG_ERROR(ERROR_MALFORMED_INPUT_BUFFER, inputBuffer);
            #endif
        }

        // Clear the buffer after processing
        bufferIndex = 0;
    }

    // Discard any remaining characters in the buffer to keep only the latest line
    while (Serial.available()) {
        char inChar = Serial.read();
        if (inChar == COMMAND_START_CHAR) {
            bufferIndex = 0;
            inputBuffer[bufferIndex++] = inChar;

            #ifdef LOG_DEBUG
            LOG_DEBUG(DEBUG_COMMAND_START);
            #endif
        }
    }
}

void SendDataToComputer() {
    if (xSemaphoreTake(xFeedbackMutex, portMAX_DELAY)) {

        Serial.print(F("D:"));
        Serial.print(DEBUG_FEEDBACK_SENT);
        Serial.print(F(":"));

        // Send feedback values as a space-separated string with code
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            Serial.print(feedbackValues[i]);
            if (i < NUM_SERVOS - 1) {
                Serial.print(" ");
            }
        }
        Serial.println();

        #ifdef LOG_DEBUG
        LOG_DEBUG(DEBUG_FEEDBACK_SENT);
        #endif

        xSemaphoreGive(xFeedbackMutex);
    } else {
        #ifdef LOG_ERROR
        LOG_ERROR(ERROR_TAKE_FEEDBACK_MUTEX_SENDDATA);
        #endif
    }
}

void RequestI2CData() {
    softWire.requestFrom(I2C_ADDRESS_NANO, (uint8_t)2);  // Use SoftwareWire request

    #ifdef LOG_DEBUG
    LOG_DEBUG(DEBUG_I2C_REQUEST);
    #endif
}

void ProcessI2CData() {
    if (softWire.available() >= 2) {
        uint16_t receivedValue = softWire.read() | (softWire.read() << 8);

        #ifdef LOG_DEBUG
        Serial.print(F("D:"));
        Serial.print(DEBUG_I2C_DATA_RECEIVED);
        Serial.print(F(":"));
        Serial.println(receivedValue);
        #endif

        if (xSemaphoreTake(xFeedbackMutex, portMAX_DELAY)) {
            feedbackValues[4] = receivedValue; // Store in the fifth servo's feedback

            #ifdef LOG_DEBUG
            Serial.print(F("D:"));
            Serial.print(DEBUG_FEEDBACK_UPDATED);
            Serial.print(F(":4,"));
            Serial.println(receivedValue);
            #endif

            xSemaphoreGive(xFeedbackMutex);
        } else {
            #ifdef LOG_ERROR
            LOG_ERROR(ERROR_TAKE_FEEDBACK_MUTEX_PROCESSI2C);
            #endif
        }
    }
    #ifdef LOG_DEBUG
    else {
        Serial.print(F("D:"));
        Serial.print(DEBUG_I2C_NO_DATA);
        Serial.print(F(":"));
        Serial.println("");
    }
    #endif
}

void loop() {
    // Empty. All work is done in tasks.
}