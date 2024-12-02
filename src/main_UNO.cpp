// src/main_UNO.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "ServoConfig.h"  // Include the ServoConfig header
#include <string.h>  // For memcpy and string operations

// Servo feedback pins
const uint8_t FEEDBACK_PINS[4] = {A0, A1, A2, A3};

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
    // Blink an LED to indicate stack overflow
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }

    // Send a message over the serial port
    Serial.print("! ERROR: Stack overflow in task: ");Serial.println(pcTaskName);
    while (1);
}

void setup() {
    Serial.begin(BAUD_RATE_UNO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    Wire.begin();

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // Initialize servos to desired positions using ServoConfig.h
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoCommandAngles[i] = SERVO_DEFAULT_ANGLES[i];
        uint16_t pulseWidth = degreeToPulseWidth(servoCommandAngles[i], i);  // Pass servo index
        pwm.setPWM(i, 0, pulseWidth);
    }

    xServoCommandMutex = xSemaphoreCreateMutex();
    if (xServoCommandMutex == NULL) {
        Serial.println("Failed to create servo command mutex");
        while (1); // Halt if mutex creation fails
    }

    xFeedbackMutex = xSemaphoreCreateMutex();
    if (xFeedbackMutex == NULL) {
        Serial.println("Failed to create feedback mutex");
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

    //Serial.println("Scheduler failed to start.");
    while (1);
}

void vControlLoopTask(void *pvParameters) {
    const TickType_t xTimeIncrementControl = pdMS_TO_TICKS(CONTROL_LOOP_INTERVAL_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Wait for the next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementControl);

        // Read analog inputs (feedback)
        ReadAnalogInputs();

        // Perform computations (e.g., Kalman filter)
        ComputeControl();

        // Send commands to servos
        UpdateServos(); // Removed argument

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            // Handle the case where the task missed its deadline
            Serial.println("! ERROR: vControlLoopTask was delayed");
        }
    }
}

void vSerialTask(void *pvParameters) {
    const TickType_t xTimeIncrementSerial = pdMS_TO_TICKS(SERIAL_DELAY_MS);

    for (;;) {
        // Check for serial data
        if (Serial.available() > 0) {
            // Read and process serial commands
            ProcessSerialCommands();
        }

        // Send data back to the computer
        SendDataToComputer();

        // Delay before the next read
        vTaskDelay(xTimeIncrementSerial);
    }
}

void vI2CTask(void *pvParameters) {
    const TickType_t xTimeIncrementI2C = pdMS_TO_TICKS(I2C_DELAY_MS);

    for (;;) {
        // Request data from peripheral sensors
        RequestI2CData();

        // Process received data
        ProcessI2CData();

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
    for (uint8_t i = 0; i < 4; i++) {
        feedbackValues[i] = analogRead(FEEDBACK_PINS[i]);
    }
}

void ComputeControl() {
    // TODO: Implement control algorithms (e.g., Kalman filter)
}

void UpdateServos() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        pwm.setPWM(i, 0, degreeToPulseWidth(servoCommandAngles[i], i));
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
        } else if (bufferIndex > 0) {
            if (inChar == COMMAND_END_CHAR) {
                inputBuffer[bufferIndex] = '\0';
                newLineReceived = true;
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
            // Acquire mutex before updating servoCommandAngles
            if (xSemaphoreTake(xServoCommandMutex, portMAX_DELAY)) {
                memcpy(servoCommandAngles, tempAngles, sizeof(tempAngles));
                xSemaphoreGive(xServoCommandMutex);
            }
        } else {
            // Handle malformed input
            Serial.print("! ERROR: Malformed input buffer: ");Serial.println(inputBuffer);
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
        }
    }
}

void SendDataToComputer() {
    if (xSemaphoreTake(xFeedbackMutex, portMAX_DELAY)) {
        // Send feedback values as a space-separated string
        Serial.print("Feedback: ");
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            Serial.print(feedbackValues[i]);
            if (i < NUM_SERVOS - 1) {
                Serial.print(" ");
            }
        }
        Serial.println();
        xSemaphoreGive(xFeedbackMutex);
    }
}

void RequestI2CData() {
    Wire.requestFrom(I2C_ADDRESS_NANO, (uint8_t)2); // Request 2 bytes for uint16_t
}

void ProcessI2CData() {
    if (Wire.available() >= 2) {
        uint16_t receivedValue = Wire.read() | (Wire.read() << 8);
        if (xSemaphoreTake(xFeedbackMutex, portMAX_DELAY)) {
            feedbackValues[4] = receivedValue; // Store in the fifth servo's feedback
            xSemaphoreGive(xFeedbackMutex);
        }
    }
}

void loop() {
    // Empty. All work is done in tasks.
}