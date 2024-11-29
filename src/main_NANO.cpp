// src/main_NANO.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include "ServoConfig.h"  // Include the ServoConfig header

// Update analog pins for potentiometers to include A7 for the fifth servo
const uint8_t POT_PINS[5] = {A0, A1, A2, A3, A7};

// Define the fifth user input potentiometer pin
const uint8_t USER_POT_PIN5 = A6;

// Define baud rate constant
const uint32_t BAUD_RATE_NANO = 57600;

// Function prototypes
void TaskReadPotentiometers(void* pvParameters);

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    while (1);
}

volatile bool dataRequested = false;

// Buffer to hold potentiometer data
uint16_t potValues[NUM_SERVOS];

#define I2C_ADDRESS_NANO     0x08  // Ensure this matches ServoConfig.h

volatile uint16_t servo5Feedback = 0;

// Handler for receiving data from Master (if needed)
void receiveEvent(int howMany) {
    // Optional: Handle incoming data if the Master sends commands
    while (Wire.available()) {
        Wire.read();  // Read and discard the incoming byte
        // Process command if necessary
    }
}

// Handler for sending data to Master upon request
void requestEvent() {
    Wire.write((servo5Feedback >> 8) & 0xFF);  // Send high byte
    Wire.write(servo5Feedback & 0xFF);         // Send low byte
}

void setup() {
    Serial.begin(BAUD_RATE_NANO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    Wire.begin(I2C_ADDRESS_NANO);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // Create the Read Potentiometers Task
    BaseType_t xReturned;
    xReturned = xTaskCreate(
        TaskReadPotentiometers,    // Task function
        "ReadPots",                // Task name
        128,                       // Stack size in words
        NULL,                      // Task parameter
        1,                         // Task priority
        NULL                       // Task handle
    );

    if (xReturned != pdPASS) {
        // Handle task creation failure
        while (1);
    }

    // Start the scheduler
    vTaskStartScheduler();

    while (1);
}

void loop() {
    // Empty. All work is done in tasks.
}

// Modify the TaskReadPotentiometers to read user input potentiometers and servo feedback
void TaskReadPotentiometers(void* pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Read user input potentiometers (first four)
        for (uint8_t i = 0; i < 4; i++) {
            uint16_t rawValue = analogRead(POT_PINS[i]);
            if (rawValue < 32) {
                rawValue = 32;
            }
            potValues[i] = rawValue;  // Store user input potentiometer values
        }

        // Read the fifth user input potentiometer from A6
        uint16_t rawUserInput5 = analogRead(USER_POT_PIN5);
        if (rawUserInput5 < 32) {
            rawUserInput5 = 32;
        }
        potValues[4] = rawUserInput5;  // Store the fifth user input potentiometer value

        // Read fifth servo feedback from A7
        uint16_t rawServo5 = analogRead(A7);
        if (rawServo5 < 32) {
            rawServo5 = 32;
        }
        servo5Feedback = rawServo5;

        // Create a comma-separated string of all five user input potentiometers
        char dataString[40];
        snprintf(dataString, sizeof(dataString), ">%d,%d,%d,%d,%d",
                 potValues[0], potValues[1], potValues[2], potValues[3], potValues[4]);

        // Send the user input potentiometers data string over Serial
        Serial.println(dataString);

        // Delay for consistent intervals (e.g., 100 ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}