// src/main_NANO.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include "ServoConfig.h"  // Include the ServoConfig header

// Analog pins for potentiometer knobs
const uint8_t KNOB_PINS[5] = {A0, A1, A2, A3, A6};

// Buffer to hold knob values
uint16_t knobValues[NUM_SERVOS];

// Function prototypes
void TaskReadKnobs(void* pvParameters);

// Handler for sending data to Master upon request
void requestEvent() {
    // Create a comma-separated string of all five user input potentiometers
    char dataString[40];
    snprintf(dataString, sizeof(dataString), "knobs: %d,%d,%d,%d,%d",
             knobValues[0], knobValues[1], knobValues[2], knobValues[3], knobValues[4]);

    Wire.write(dataString);  // Send data to Master
}

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
    Serial.begin(BAUD_RATE_NANO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    Wire.begin(I2C_ADDRESS_NANO);  // Initialize hardware I2C in Slave mode
    Wire.onRequest(requestEvent);

    // Create the Read Potentiometers Task
    xTaskCreate(
        TaskReadKnobs,             // Task function
        "ReadKnobs",               // Task name
        128,                       // Stack size in words
        NULL,                      // Task parameter
        1,                         // Task priority
        NULL                       // Task handle
    );

    // Start the scheduler
    vTaskStartScheduler();

    while (1);
}

void loop() {
    // Empty. All work is done in tasks.
}

void TaskReadKnobs(void* pvParameters) {
    (void) pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = pdMS_TO_TICKS(100); // 100 ms interval

    for (;;) {
        // Read user input knobs
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            uint16_t rawValue = analogRead(KNOB_PINS[i]);
            if (rawValue < 32) {
                rawValue = 32;
            }
            knobValues[i] = rawValue;  // Store user input potentiometer values
        }

        // Send the user input potentiometers data string over Serial
        char dataString[40];
        snprintf(dataString, sizeof(dataString), "knobs: %d,%d,%d,%d,%d",
                 knobValues[0], knobValues[1], knobValues[2], knobValues[3], knobValues[4]);
        Serial.println(dataString);

        // Delay for consistent intervals (e.g., 100 ms)
        xTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}

void RequestI2CData() {
    // softWire.requestFrom(...);  // Use SoftwareWire request if needed
}