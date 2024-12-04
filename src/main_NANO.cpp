// src/main_NANO.cpp

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <SPI.h>  // Add SPI library

// #define DEBUG_PRINTS

#ifdef DEBUG_PRINTS
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define NUM_SERVOS 5

// I2C address for the Nano
#define I2C_ADDRESS_NANO 0x08
#define BAUD_RATE_NANO 57600

// Task priority levels
#define PRIORITY_MAIN_TASK 1

// Define stack sizes for each task
#define MAIN_STACK_SIZE 258

// Task interval and delays
#define MAIN_INTERVAL_MS 100

// Analog pins for current readings
const uint8_t CURRENT_SENSOR_PINS[5] = {A0, A1, A2, A3, A4};
const uint8_t FIFTH_FEEDBACK_PIN = A7;

// Max apms allowed
#define MAX_AMPS 2

// Shared variables to be sent to master
volatile uint16_t fifthFeedbackValue = 0;
volatile uint16_t currentLevels[NUM_SERVOS] = {0};
volatile bool stopFlag = false;          // Signal to stop because of overcurrent

// volatile bool spiDataReady = false;
// volatile uint8_t receivedData[13];
// volatile uint8_t dataIndex = 0;

volatile uint8_t txBuffer[13];      // Buffer to hold data to send to master
volatile uint8_t txIndex = 0;       // Index for the next byte to send

const char* mainTaskName = "main";
const TickType_t xTimeIncrementMain = pdMS_TO_TICKS(MAIN_INTERVAL_MS);

// Modify task creation to store task handles
TaskHandle_t xMainTaskHandle = NULL;

// Function prototypes
void vMainTask(void* pvParameters);
void ReadAnalogInputs();
void CheckOvercurrent();
void LoadSPIBuffer();
//void RequestEvent();

#ifdef DEBUG_PRINTS
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle);
#endif

// Function to print stack high water mark
#ifdef DEBUG_PRINTS
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle) {
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle);
    DEBUG_PRINT(taskName);  // taskName is a pointer to a string in Flash memory
    DEBUG_PRINT(F(" Stack High Water Mark: "));  // Use F() macro to store string in Flash memory
    DEBUG_PRINTLN(uxHighWaterMark);
}
#endif

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
}

void setup() {
    Serial.begin(BAUD_RATE_NANO);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    pinMode(MISO, OUTPUT); // Set MISO as output
    SPCR |= _BV(SPE); // Enable SPI as Slave
    SPI.attachInterrupt(); // Enable SPI interrupt

    // Create the Main Task
    xTaskCreate(
        vMainTask,                  // Task function
        mainTaskName,               // Task name
        MAIN_STACK_SIZE,            // Stack size in words
        NULL,                       // Task parameter
        PRIORITY_MAIN_TASK,         // Task priority
        &xMainTaskHandle            // Task handle
    );

    DEBUG_PRINTLN(F("Setup completed"));

    // Start the scheduler
    vTaskStartScheduler();

    while (1);
}

ISR(SPI_STC_vect) {
    // Load the next byte to send into SPDR
    SPDR = txBuffer[txIndex++];
    if (txIndex >= sizeof(txBuffer)) {
        txIndex = 0;
    }
}

void vMainTask(void* pvParameters) {
    DEBUG_PRINTLN(F("Main task: Init"));
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Wait for the next cycle
        DEBUG_PRINTLN(F("Main task: Waiting for cycle"));
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementMain);

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            DEBUG_PRINTLN(F("Main task: Overtime"));
        }

        // Read analog inputs
        DEBUG_PRINTLN(F("Main task: Reading analog"));
        ReadAnalogInputs();

        // Perform computations
        DEBUG_PRINTLN(F("Main task: Checking current"));
        CheckOvercurrent();

        // Prepare to send data
        DEBUG_PRINTLN(F("Main task: Loading SPI buffer"));
        LoadSPIBuffer();

        // Monitor stack usage periodically
        #ifdef DEBUG_PRINTS
        PrintStackUsage(mainTaskName, xMainTaskHandle);
        #endif
    }
}

void ReadAnalogInputs() {
    // Read feedback of fifth servo
    fifthFeedbackValue = analogRead(FIFTH_FEEDBACK_PIN);

    // Read current level of all servos
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        currentLevels[i] = analogRead(CURRENT_SENSOR_PINS[i]);
    }

    // Send to serial output
    Serial.print("A:");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (i != 0) {
            Serial.print(",");
        }
        Serial.print(currentLevels[i]);
    }
    Serial.println();
}

void CheckOvercurrent() {
    // TODO: Implement overcurrent detection logic
    // Example:
    // for(uint8_t i = 0; i < NUM_SERVOS; i++) {
    //     if(currentLevels[i] > OVERCURRENT_THRESHOLD) {
    //         stopFlag = true;
    //     }
    // }
}

void LoadSPIBuffer() {
        // Disable interrupts to safely copy data
        noInterrupts();

        txBuffer[0] = stopFlag ? 1 : 0;
        txBuffer[1] = lowByte(fifthFeedbackValue);
        txBuffer[2] = highByte(fifthFeedbackValue);

        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            txBuffer[3 + i * 2] = lowByte(currentLevels[i]);
            txBuffer[4 + i * 2] = highByte(currentLevels[i]);
        }

        // Enable interrupts
        interrupts();
}



void loop() {
    // Empty. All work is done in tasks.
}

