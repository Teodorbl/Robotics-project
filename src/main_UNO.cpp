// src/main_UNO.cpp
// PRUNED VERSION

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>



// #define DEBUG_PRINTS

#ifdef DEBUG_PRINTS
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


#define I2C_ADDRESS_PWMDRV 0x40
#define I2C_ADDRESS_NANO 0x08     // I2C address for Arduino Nano

#define BAUD_RATE_UNO 57600

#define PWM_FREQ 50

// Task priority levels
#define PRIORITY_CONTROL_TASK 2
#define PRIORITY_COMS_TASK 1

// Define stack sizes for each task
#define CONTROL_STACK_SIZE 128
#define COMS_TASK_STACK_SIZE 128

// Task interval and delays
#define CONTROL_INTERVAL_MS 100
#define COMS_DELAY_MS 100

const uint8_t NUM_SERVOS = 5;

const uint8_t SERVO_MIN_ANGLES[NUM_SERVOS]     = {  0,  35,  30,   0,  40};
const uint8_t SERVO_DEFAULT_ANGLES[NUM_SERVOS] = { 90,  35, 160,  90, 120};
const uint8_t SERVO_MAX_ANGLES[NUM_SERVOS]     = {180, 115, 165, 180, 135};
const bool SERVO_INVERT_MASK[NUM_SERVOS] = {false, false, true, true, false};

const uint8_t FEEDBACK_PINS[NUM_SERVOS] = {A0, A1, A2, A3};

const uint16_t SERVO_MIN_PULSE_WIDTH = 184;  // Corresponds to 900 µsec
const uint16_t SERVO_MAX_PULSE_WIDTH = 430;  // Corresponds to 2100 µsec

const TickType_t xTimeIncrementControl = pdMS_TO_TICKS(CONTROL_INTERVAL_MS);
const TickType_t xTimeIncrementComs = pdMS_TO_TICKS(COMS_DELAY_MS);

const char* controlTaskName = "ctrl";
const char* comsTaskName = "coms";

// Global variable to store analog feedback values
uint16_t feedbackValues[NUM_SERVOS] = {0};
uint16_t currentLevels[NUM_SERVOS] = {0};

// Signal to stop because of overcurrent
bool stopFlag = false;

// Array for commanded servo angles
uint8_t servoCommandAngles[NUM_SERVOS];

// Servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2C_ADDRESS_PWMDRV);

// Define separate mutexes for feedbackValues
SemaphoreHandle_t xFeedbackValuesMutex;

// Function prototypes
void vControlTask(void *pvParameters);
void vComsTask(void *pvParameters);
uint16_t DegreeToPulseWidth(uint8_t degree, uint8_t servoIndex);
void ReadAnalogInputs();
void ComputeControl();
void UpdateServos();
void ProcessSerialCommands();
void SendDataToComputer();
void RequestI2CData();
void ProcessI2CData();

#ifdef DEBUG_PRINTS
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle);
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

// Function to print stack high water mark
#ifdef DEBUG_PRINTS
void PrintStackUsage(const char* taskName, TaskHandle_t xHandle) {
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(xHandle);
    DEBUG_PRINT(taskName);  // taskName is a pointer to a string in Flash memory
    DEBUG_PRINT(F(" Stack High Water Mark: "));  // Use F() macro to store string in Flash memory
    DEBUG_PRINTLN(uxHighWaterMark);
}
#endif

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
        servoCommandAngles[i] = SERVO_DEFAULT_ANGLES[i];
        uint16_t pulseWidth = DegreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);
    }

    // Create separate mutexes
    xFeedbackValuesMutex = xSemaphoreCreateMutex();
    if (xFeedbackValuesMutex == NULL) {
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
    
    DEBUG_PRINTLN(F("Control task created"));

    xTaskCreate(
        vComsTask,
        comsTaskName,
        COMS_TASK_STACK_SIZE,
        NULL,
        PRIORITY_COMS_TASK,
        &xComsTaskHandle
    );

    DEBUG_PRINTLN(F("Setup done"));
    vTaskStartScheduler();

    while (1);
}

void vControlTask(void *pvParameters) {
    DEBUG_PRINTLN(F("Control task: Init"));
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Wait for the next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementControl);

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            DEBUG_PRINTLN(F("Control task: Overtime"));
        }

        // Check for serial data
        if (Serial.available() > 0) {
            // Read and process serial commands
            ProcessSerialCommands();
        }

        // Read analog inputs (feedback)
        ReadAnalogInputs();

        // Perform computations (e.g., Kalman filter)
        ComputeControl();

        // Send commands to servos
        UpdateServos();

        // Monitor stack usage periodically
        #ifdef DEBUG_PRINTS
        PrintStackUsage(controlTaskName, xControlTaskHandle);
        #endif
    }
}




void vComsTask(void *pvParameters) {
    DEBUG_PRINTLN(F("Coms task: Init"));
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Wait for the next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xTimeIncrementComs);

        // Check if the task was delayed
        if (xWasDelayed == pdFALSE) {
            DEBUG_PRINTLN(F("Coms task: Overtime"));
        }

        // Request and store data over I2C
        RequestI2CData();
        ProcessI2CData();

        // Send data back to the computer over serial
        SendDataToComputer();

        // Monitor stack usage periodically
        #ifdef DEBUG_PRINTS
        PrintStackUsage(comsTaskName, xComsTaskHandle);
        #endif


    }
}

// Utility function to convert degrees to pulse width using a fixed mapping
uint16_t DegreeToPulseWidth(uint8_t degree, uint8_t servoIndex) {
    degree = constrain(degree, SERVO_MIN_ANGLES[servoIndex], SERVO_MAX_ANGLES[servoIndex]);

    if (SERVO_INVERT_MASK[servoIndex]) {
        // Invert the degree mapping
        degree = SERVO_MIN_ANGLES[servoIndex] + SERVO_MAX_ANGLES[servoIndex] - degree;
    }
    
    return map(degree, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
}

void ReadAnalogInputs() {
    // Acquire feedback values mutex with timeout
    if (xSemaphoreTake(xFeedbackValuesMutex, pdMS_TO_TICKS(100))) {
        for (uint8_t i = 0; i < NUM_SERVOS-1; i++) {
            feedbackValues[i] = analogRead(FEEDBACK_PINS[i]);
        }
        xSemaphoreGive(xFeedbackValuesMutex);
    } else {
        DEBUG_PRINTLN(F("Failed to acquire feedback values mutex"));
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
    // Directly update servos
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint16_t pulseWidth = DegreeToPulseWidth(servoCommandAngles[i], i);
        pwm.setPWM(i, 0, pulseWidth);
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
        sscanf(inputBuffer, "%*c %hhu %hhu %hhu %hhu %hhu",
                &servoCommandAngles[0], &servoCommandAngles[1],
                &servoCommandAngles[2], &servoCommandAngles[3],
                &servoCommandAngles[4]);

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
    if (xSemaphoreTake(xFeedbackValuesMutex, portMAX_DELAY)) {

        // Send feedback values as a comma-separated string with code
        Serial.print("FB:");
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (i != 0) {
                Serial.print(",");
            }
            Serial.print(feedbackValues[i]);
        }
        Serial.println();

        xSemaphoreGive(xFeedbackValuesMutex);
    }
}

void RequestI2CData() {
    // Request 13 bytes: 2 (servo feedback) + 10 (5 current levels) + 1 (stop flag)
    Wire.requestFrom(I2C_ADDRESS_NANO, 13);
}

void ProcessI2CData() {
    if (Wire.available() >= 13) {
        // Acquire feedback values mutex with timeout
        if (xSemaphoreTake(xFeedbackValuesMutex, pdMS_TO_TICKS(100))) {
            DEBUG_PRINTLN(F("Received I2C data"));

            // Read stop flag
            stopFlag = Wire.read() != 0;

            // Read servo analog feedback (the fifth one is read by the NANO)
            int byte1 = Wire.read();
            int byte2 = Wire.read();
            if (byte1 != -1 && byte2 != -1) {
                feedbackValues[4] = static_cast<uint16_t>(byte1) | (static_cast<uint16_t>(byte2) << 8);
            } else {
                DEBUG_PRINTLN(F("Error reading servo analog feedback"));
            }

            // Read 5 current level readings
            for (uint8_t i = 0; i < 5; i++) {
                int cl_byte1 = Wire.read();
                int cl_byte2 = Wire.read();
                if (cl_byte1 != -1 && cl_byte2 != -1) {
                    currentLevels[i] = static_cast<uint16_t>(cl_byte1) | (static_cast<uint16_t>(cl_byte2) << 8);
                } else {
                    DEBUG_PRINTLN(F("Error reading current level"));
                }
            }

            xSemaphoreGive(xFeedbackValuesMutex);
        } else {
            DEBUG_PRINTLN(F("Failed to acquire feedback values mutex"));
        }
    }
}

void loop() {
    // Empty. All work is done in tasks.
    ;
}