// src/main_NANO.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include "ServoConfig.h"  // Include the ServoConfig header

// Update analog pins for potentiometers to include A6 for the fifth knob
const uint8_t KNOB_PINS[5] = {A0, A1, A2, A3, A6};
const uint8_t SERVO5_PIN = A7;

volatile bool dataRequested = false;

// Buffer to hold knob values and servo5Feedback
uint16_t knobValues[NUM_SERVOS];
uint16_t servo5Feedback = 0;

// Function prototypes
void TaskReadKnobs(void* pvParameters);

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
    // Send only servo5Feedback
    uint16_t value = servo5Feedback;
    Wire.write((value >> 8) & 0xFF); // High byte
    Wire.write(value & 0xFF);        // Low byte
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

    Wire.begin(I2C_ADDRESS_NANO);
    Wire.onReceive(receiveEvent);
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
        for (uint8_t i = 0; i < 5; i++) {
            uint16_t rawValue = analogRead(KNOB_PINS[i]);
            if (rawValue < 32) {
                rawValue = 32;
            }
            knobValues[i] = rawValue;  // Store user input potentiometer values
        }

        // Read fifth servo feedback from A7
        servo5Feedback = analogRead(A7);

        // Create a comma-separated string of all five user input potentiometers
        char dataString[40];
        snprintf(dataString, sizeof(dataString), ">%d,%d,%d,%d,%d",
                 knobValues[0], knobValues[1], knobValues[2], knobValues[3], knobValues[4]);

        // Send the user input potentiometers data string over Serial
        Serial.println(dataString);

        // Delay for consistent intervals (e.g., 100 ms)
        xTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}