// src/main.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>

// Servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constants for servo control
#define NUM_SERVOS 5
#define SERVO_MIN_PULSE_WIDTH 184  // Corresponds to 900 µsec
#define SERVO_MAX_PULSE_WIDTH 430  // Corresponds to 2100 µsec
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

// Initial servo angles
uint8_t servoAngles[NUM_SERVOS] = {90, 50, 45, 90, 120};

// Servo names stored in PROGMEM to save SRAM
const char servoName0[] PROGMEM = "base";
const char servoName1[] PROGMEM = "shoulder";
const char servoName2[] PROGMEM = "elbow";
const char servoName3[] PROGMEM = "wrist";
const char servoName4[] PROGMEM = "claw";
const char* const servoNames[NUM_SERVOS] PROGMEM = {
    servoName0, servoName1, servoName2, servoName3, servoName4
};

// Function prototypes
void TaskSerialControl(void* pvParameters);
void TaskServoFeedback(void* pvParameters);
uint16_t degreeToPulseWidth(uint8_t degree);
int findServoIndex(const char* name);
uint16_t analogToDegree(uint16_t analogValue);

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    //Serial.print("Stack overflow in task: ");
    //Serial.println(pcTaskName);
    // Halt execution
    while (1);
}

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;  // Wait for serial port to connect (needed for native USB)
    }

    Wire.begin();

    //Serial.println("Initializing PWM driver...");
    pwm.begin();
    pwm.setPWMFreq(50);  // Set to 50 Hz

    // Initialize servos to desired positions
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        uint16_t pulseWidth = degreeToPulseWidth(servoAngles[i]);
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
        TaskSerialControl,   // Task function
        "SerialCtrl",        // Task name
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

    //Serial.println("Starting scheduler...");
    vTaskStartScheduler();

    //Serial.println("Scheduler failed to start.");
    while (1);
}

void loop() {
    // Empty. All work is done in tasks.
}

// Task to handle serial input and control servos
void TaskSerialControl(void* pvParameters) {
    (void) pvParameters;

    const uint8_t bufferSize = 50;
    char inputBuffer[bufferSize];
    uint8_t index = 0;
    char c;

    //Serial.println("TaskSerialControl started. Awaiting commands...");

    for (;;) {
        if (Serial.available() > 0) {
            // Read characters until end of line
            while (Serial.available() > 0) {
                c = Serial.read();

                // Check for end of line
                if (c == '\n' || c == '\r') {
                    if (index == 0) {
                        // Empty input
                        continue;
                    }
                    inputBuffer[index] = '\0';  // Null-terminate the string
                    index = 0;  // Reset index for next input

                    //Serial.print("Received: ");
                    //Serial.println(inputBuffer);

                    // Parse the command
                    char servoName[20];
                    int degree;
                    if (sscanf(inputBuffer, "%19s %d", servoName, &degree) == 2) {
                        // Find servo index
                        int servoIndex = findServoIndex(servoName);
                        if (servoIndex == -1) {
                            //Serial.println("Error: Invalid servo name.");
                            continue;
                        }

                        // Validate degree
                        if (degree < SERVO_MIN_DEGREE || degree > SERVO_MAX_DEGREE) {
                            //Serial.println("Error: Degree out of range (0-180).");
                            continue;
                        }

                        // Update servo angle
                        servoAngles[servoIndex] = (uint8_t)degree;
                        uint16_t pulseWidth = degreeToPulseWidth(servoAngles[servoIndex]);
                        pwm.setPWM(servoIndex, 0, pulseWidth);
                        //Serial.print("Moved servo ");
                        //Serial.print(servoName);
                        //Serial.print(" to ");
                        //Serial.print(degree);
                        //Serial.println(" degrees.");
                    } else {
                        //Serial.println("Error: Invalid command format. Use '[servoName] [degrees]'.");
                    }
                } else {
                    if (index < (bufferSize - 1)) {
                        inputBuffer[index++] = c;
                    } else {
                        // Buffer overflow attempt
                        //Serial.println("Error: Input buffer overflow.");
                        index = 0;  // Reset index
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Yield to other tasks
    }
}

// Task to read servo feedback and send data for plotting
void TaskServoFeedback(void* pvParameters) {
    (void) pvParameters;

    const uint8_t numFeedbackPins = 5;
    const uint8_t feedbackPins[numFeedbackPins] = {A0, A1, A2, A3, A4};
    uint16_t analogValues[numFeedbackPins];
    uint16_t degrees[numFeedbackPins];

    for (;;) {
        // Read analog inputs
        for (uint8_t i = 0; i < numFeedbackPins; i++) {
            analogValues[i] = analogRead(feedbackPins[i]);
            //degrees[i] = analogToDegree(analogValues[i]);
            degrees[i] = analogValues[i];                       // CHANGE: Write as voltage
        }

        // Create a comma-separated string: servo0, servo1, servo2, servo3, servo4
        // Example: 75,30,90,45,120
        char dataString[50];
        snprintf(dataString, sizeof(dataString), ">%d,%d,%d,%d,%d",
                 degrees[0], degrees[1], degrees[2], degrees[3], degrees[4]);

        // Send the data string over Serial
        Serial.println(dataString);

        // Delay for 100 ms before next reading
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Utility function to convert degrees to pulse width
uint16_t degreeToPulseWidth(uint8_t degree) {
    degree = constrain(degree, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
    return map(degree, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE,
               SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
}

// Function to convert analog value to degree
uint16_t analogToDegree(uint16_t analogValue) {
    // Assuming analogValue ranges from 0 to 1023 corresponding to 0 to 180 degrees
    return map(analogValue, 0, 1023, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
}

// Function to find servo index based on name
int findServoIndex(const char* name) {
    char storedName[20];
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        strcpy_P(storedName, (char*)pgm_read_word(&(servoNames[i])));
        if (strcasecmp(storedName, name) == 0) {
            return i;
        }
    }
    return -1;  // Not found
}