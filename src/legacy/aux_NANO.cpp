// src/aux_NANO.cpp

#include <Arduino.h>
#include <SPI.h>  // Include SPI library

#define DEBUG_PRINTS

#ifdef DEBUG_PRINTS
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define BAUD_RATE_NANO 57600

#define SS_PIN 10  // Define Slave Select pin

// Shared variable to be sent to master
volatile uint8_t auxValue = 0;

// Buffer to hold data to send to master (single byte)
volatile uint8_t txBuffer[1] = {0};
volatile uint8_t txIndex = 0;  // Index for the next byte to send

// Flag to indicate SPI transfer occurred (for debugging outside ISR)
volatile bool spiTransferOccurred = false;

// Function prototype
void LoadSPIBuffer();

void setup() {
    Serial.begin(BAUD_RATE_NANO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    // Set MISO as OUTPUT
    pinMode(MISO, OUTPUT);
    pinMode(SS_PIN, INPUT_PULLUP);  // Initialize SS pin as input with pull-up

    // Initialize SPI as Slave
    SPI.begin(); 
    SPCR |= _BV(SPE) | _BV(SPIE); // Enable SPI, Enable SPI Interrupt

    // Initialize auxValue (set to a known state or sensor reading)
    auxValue = 0; // Example initialization

    // Load initial value into txBuffer
    LoadSPIBuffer();

    // Preload SPDR with the first byte to send
    txIndex = 0;
    SPDR = txBuffer[txIndex++];  // Start SPI transmission

    DEBUG_PRINTLN(F("Setup completed"));
}

void loop() {
    // Example: Update auxValue periodically (e.g., read a sensor)
    // Here, we'll simulate it by incrementing auxValue every second
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastUpdateTime >= 1000) {  // Update every 1000 ms
        lastUpdateTime = currentTime;
        auxValue++;

        // Ensure auxValue stays within 0-255
        if (auxValue > 255) {
            auxValue = 0;
        }

        // Update the SPI buffer with the latest auxValue
        LoadSPIBuffer();
    }

    // Handle debug prints outside ISR
    if (spiTransferOccurred) {
        DEBUG_PRINT(F("SPI Transfer Occurred, Sent byte: "));
        DEBUG_PRINTLN(txBuffer[txIndex == 0 ? 0 : txIndex - 1]);
        spiTransferOccurred = false;
    }
    // Other tasks can be added here
}

void LoadSPIBuffer() {
    noInterrupts();  // Ensure atomic access to txBuffer
    txBuffer[0] = auxValue;  // Assign auxValue to txBuffer
    interrupts();  // Re-enable interrupts

    DEBUG_PRINT(F("Loaded txBuffer[0]: "));
    DEBUG_PRINTLN(txBuffer[0]);
}

// SPI Interrupt Service Routine
ISR(SPI_STC_vect) {
    // Load the next byte to send into SPDR
    SPDR = txBuffer[txIndex++];

    // Set flag to indicate SPI transfer occurred
    spiTransferOccurred = true;

    // Reset txIndex if end of buffer is reached
    if (txIndex >= sizeof(txBuffer)) {
        txIndex = 0;
    }
}