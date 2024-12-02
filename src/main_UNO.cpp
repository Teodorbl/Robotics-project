// src/main_UNO.cpp
// PRUNED VERSION

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <avr/pgmspace.h>
#include <Wire.h>


#define I2C_ADDRESS_PWMDRV 0x40
#define I2C_ADDRESS_NANO 0x08     // I2C address for Arduino Nano

#define BAUD_RATE_UNO 57600
#define BAUD_RATE_NANO 57600

#define PWM_FREQ 50

// Task priority levels
#define PRIORITY_CONTROL_TASK 2
#define PRIORITY_COMS_TASK 1

// Define stack sizes for each task
#define CONTROL_STACK_SIZE 100
#define COMS_TASK_STACK_SIZE 100

// Task interval and delays
#define CONTROL_INTERVAL_MS 500
#define COMS_DELAY_MS 1000

const uint8_t NUM_SERVOS = 5;

const uint8_t SERVO_MIN_ANGLES[NUM_SERVOS]     = {  0,  35,  30,   0,  40};
const uint8_t SERVO_DEFAULT_ANGLES[NUM_SERVOS] PROGMEM = { 90,  35, 160,  90, 120};
const uint8_t SERVO_MAX_ANGLES[NUM_SERVOS]     = {180, 115, 165, 180, 135};
const bool SERVO_INVERT_MASK[NUM_SERVOS] = {false, false, true, true, false};

const uint8_t FEEDBACK_PINS[NUM_SERVOS] = {A0, A1, A2, A3, A4};

const uint16_t SERVO_MIN_PULSE_WIDTH = 184;  // Corresponds to 900 µsec
const uint16_t SERVO_MAX_PULSE_WIDTH = 430;  // Corresponds to 2100 µsec

// Convert the tick value to ticks and store it in PROGMEM
const TickType_t xTimeIncrementControl = pdMS_TO_TICKS(CONTROL_INTERVAL_MS);
const TickType_t xTimeIncrementComs PROGMEM = pdMS_TO_TICKS(COMS_DELAY_MS);

//PUT IN PROGMEM?
const char* controlTaskName = "ctrl";
const char* comsTaskName = "coms";

// Global variable to store analog feedback values
uint16_t feedbackValues[NUM_SERVOS] = {0};

// Array for commanded servo angles
uint8_t servoCommandAngles[NUM_SERVOS];

// Servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Declare a mutex for protecting servoCommandAngles and feedbackValues
SemaphoreHandle_t xGlobalMutex;

// Function prototypes
void vControlTask(void *pvParameters);
void vComsTask(void *pvParameters);
uint16_t degreeToPulseWidth(uint8_t degree, uint8_t servoIndex);
void ReadAnalogInputs();
void ComputeControl();
void UpdateServos();
void ProcessSerialCommands();
void SendDataToComputer();
void RequestI2CData();
void ProcessI2CData();
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle);

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

// Function to print stack high water mark
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle) {
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle);
    Serial.print(taskName);  // taskName is a pointer to a string in Flash memory
    Serial.print(F(" Stack High Water Mark: "));  // Use F() macro to store string in Flash memory
    Serial.println(uxHighWaterMark);
}

// Modify task creation to store task handles
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xComsTaskHandle = NULL;

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
        servoCommandAngles[i] = pgm_read_byte(&SERVO_DEFAULT_ANGLES[i]);
        uint16_t pulseWidth = degreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);
    }

    xGlobalMutex = xSemaphoreCreateMutex();
    if (xGlobalMutex == NULL) {
        while (1); // Halt if mutex creation fails
    }

    xTaskCreate(
        vControlTask,               // Task function
        controlTaskName,              // Task name
        CONTROL_STACK_SIZE,         // Stack size
        NULL,                       // Task parameters
        PRIORITY_CONTROL_TASK,      // Priority
        &xControlTaskHandle         // Task handle
    );
    
    Serial.println(F("Control task created"));

    xTaskCreate(
        vComsTask,
        comsTaskName,
        COMS_TASK_STACK_SIZE,
        NULL,
        PRIORITY_COMS_TASK,
        &xComsTaskHandle
    );

    Serial.println(F("Setup done"));
    vTaskStartScheduler();

    while (1);
}

void vControlTask(void *pvParameters) {
    Serial.println(F("Control task init"));
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Wait for the next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementControl);

        // Read analog inputs (feedback)
        ReadAnalogInputs();

        // Perform computations (e.g., Kalman filter)
        ComputeControl();

        // Send commands to servos
        UpdateServos();

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            Serial.println(F("Control task delayed"));
        }

        // Monitor stack usage periodically
        PrintStackUsage(controlTaskName, xControlTaskHandle);
    }
}

void vComsTask(void *pvParameters) {
    Serial.println(F("Coms task init"));

    for (;;) {
        // Check for serial data
        if (Serial.available() > 0) {

            // Read and process serial commands
            ProcessSerialCommands();
        }

        // Request and store data from over I2C
        RequestI2CData();
        ProcessI2CData();

        // Send data back to the computer over serial
        SendDataToComputer();

        // Monitor stack usage periodically
        PrintStackUsage(comsTaskName, xComsTaskHandle);

        // Delay before the next read
        vTaskDelay(pgm_read_byte(&xTimeIncrementComs));

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
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        feedbackValues[i] = analogRead(FEEDBACK_PINS[i]);
    }
}

void ComputeControl() {
    // TODO: Implement control algorithms (e.g., Kalman filter)

    // Example error condition after implementing control algorithms
    // if (controlAlgorithmFailed) {
    //     #ifdef LOG_ERROR
    //     LOG_ERROR(F("Control algorithm failed."));
    //     #endif
    // }
}

void UpdateServos() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint16_t pulseWidth = degreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);

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
    static char inputBuffer[61];
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
        if (parsed == NUM_SERVOS)

            // Acquire mutex before updating servoCommandAngles
            if (xSemaphoreTake(xGlobalMutex, portMAX_DELAY)) {
                memcpy(servoCommandAngles, tempAngles, sizeof(tempAngles));
                xSemaphoreGive(xGlobalMutex);
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
    if (xSemaphoreTake(xGlobalMutex, portMAX_DELAY)) {

        // Send feedback values as a comma-separated string with code
        Serial.print(F("FEEDBACK:"));
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (i != 0) {
                Serial.print(",");
            }
            Serial.print(feedbackValues[i]);
        }
        Serial.println();

        xSemaphoreGive(xGlobalMutex);
    }
}

void RequestI2CData() {
    Wire.requestFrom(I2C_ADDRESS_NANO, 2);  // TODO: Use SoftwareWire request
}

void ProcessI2CData() {
    if (Wire.available() >= 2) {
        uint16_t receivedValue = Wire.read() | (Wire.read() << 8);
        Serial.println(receivedValue);

        if (xSemaphoreTake(xGlobalMutex, portMAX_DELAY)) {
            //TODO

            xSemaphoreGive(xGlobalMutex);
        }
    }

}

void loop() {
    // Empty. All work is done in tasks.
    ;
}