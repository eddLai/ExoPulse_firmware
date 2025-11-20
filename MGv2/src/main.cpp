#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Include modularized headers
#include "config.h"
#include "motor_protocol.h"
#include "motor_operations.h"
#include "calibration.h"
#include "serial_commands.h"

/*
 * LK-TECH Dual Motor Status Reader - Modularized Version
 * Purpose: READ TWO motors (ID 1 and ID 2) status data with high-frequency updates
 *
 * Motor: LK-TECH M Series (MS/MF/MG/MH)
 * CAN Baud Rate: 1 Mbps
 * Motor IDs: 1 and 2
 */

// Global objects
MCP_CAN CAN(CAN_CS);

// FreeRTOS objects
QueueHandle_t motorDataQueue;
SemaphoreHandle_t canMutex;
TaskHandle_t canReadTaskHandle;
TaskHandle_t serialOutputTaskHandle;

// Software angle offset (safe, no ROM write)
int64_t motor1_angle_offset = 0;
int64_t motor2_angle_offset = 0;

// Latest motor status (for calibration commands)
volatile int64_t motor1_latest_angle = 0;
volatile int64_t motor2_latest_angle = 0;
volatile bool motor1_data_valid = false;
volatile bool motor2_data_valid = false;

// CAN Reading Task (High Priority) - Runs on Core 1
void canReadTask(void *parameter) {
    MotorStatus status1 = {0};
    MotorStatus status2 = {0};

    while (true) {
        // Read Motor 1
        if (readMotorComplete(MOTOR_ID_1, CAN_ID_1, status1)) {
            xQueueSend(motorDataQueue, &status1, 0);
            // Update latest angle for calibration
            motor1_latest_angle = status1.motorAngle;
            motor1_data_valid = true;
            digitalWrite(LED_PIN, HIGH);
        }

        // Small delay between motors
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read Motor 2
        if (readMotorComplete(MOTOR_ID_2, CAN_ID_2, status2)) {
            xQueueSend(motorDataQueue, &status2, 0);
            // Update latest angle for calibration
            motor2_latest_angle = status2.motorAngle;
            motor2_data_valid = true;
            digitalWrite(LED_PIN, LOW);
        }

        // Wait before next read cycle (10Hz = 100ms total)
        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

// Serial Output Task (Lower Priority) - Runs on Core 0
void serialOutputTask(void *parameter) {
    MotorStatus status;
    uint32_t outputCount1 = 0;
    uint32_t outputCount2 = 0;

    while (true) {
        // Check for serial commands (non-blocking)
        if (Serial.available() > 0) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            processSerialCommand(cmd);
        }

        // Wait for data from queue (blocking with timeout)
        if (xQueueReceive(motorDataQueue, &status, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Get appropriate offset
            int64_t offset = (status.motorID == MOTOR_ID_1) ? motor1_angle_offset : motor2_angle_offset;

            // Output in compact format for GUI parsing
            printMotorData(status, offset);

            // Track output counts per motor
            if (status.motorID == MOTOR_ID_1) {
                outputCount1++;
            } else if (status.motorID == MOTOR_ID_2) {
                outputCount2++;
            }

            // Print detailed status every 10 reads per motor (1 second at 10Hz)
            if ((status.motorID == MOTOR_ID_1 && outputCount1 % 10 == 0) ||
                (status.motorID == MOTOR_ID_2 && outputCount2 % 10 == 0)) {
                printDetailedStatus(status, offset);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("   LK-TECH Dual Motor Status Reader");
    Serial.println("   FreeRTOS Optimized - 10Hz");
    Serial.println("   Modularized Version");
    Serial.println("========================================\n");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize SPI
    Serial.println("[1] Initializing SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("    SPI initialized");

    // Initialize MCP2515 at 1Mbps
    Serial.println("\n[2] Initializing MCP2515...");
    Serial.println("    Trying 1MBPS @ 8MHz...");

    byte result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

    if (result != CAN_OK) {
        Serial.println("    Failed! Trying 1MBPS @ 16MHz...");
        result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
    }

    if (result != CAN_OK) {
        Serial.println("\n[ERROR] MCP2515 initialization FAILED!");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }

    Serial.println("    MCP2515 initialized successfully!");

    // Set to NORMAL mode
    Serial.println("\n[3] Setting NORMAL mode...");
    CAN.setMode(MCP_NORMAL);
    Serial.println("    Mode set to NORMAL");

    // Create FreeRTOS objects
    Serial.println("\n[4] Creating FreeRTOS objects...");

    // Create queue for motor data (capacity: 20 items for both motors)
    motorDataQueue = xQueueCreate(20, sizeof(MotorStatus));
    if (motorDataQueue == NULL) {
        Serial.println("    [ERROR] Failed to create queue!");
        while(1);
    }

    // Create mutex for CAN bus access
    canMutex = xSemaphoreCreateMutex();
    if (canMutex == NULL) {
        Serial.println("    [ERROR] Failed to create mutex!");
        while(1);
    }
    Serial.println("    FreeRTOS objects created!");

    // NOTE: Auto-reset DISABLED because 0x95 command is not implemented in motor firmware
    Serial.println("\n[5] Skipping auto-reset (use SET_ZERO commands if needed)...");

    // Create tasks
    Serial.println("\n[6] Creating FreeRTOS tasks...");

    // Create CAN reading task (high priority, core 1)
    xTaskCreatePinnedToCore(
        canReadTask,
        "CAN_Read",
        4096,
        NULL,
        3,
        &canReadTaskHandle,
        1
    );

    // Create Serial output task (lower priority, core 0)
    xTaskCreatePinnedToCore(
        serialOutputTask,
        "Serial_Output",
        4096,
        NULL,
        1,
        &serialOutputTaskHandle,
        0
    );

    Serial.println("    Tasks created successfully!");
    Serial.println("\n[OK] Dual Motor Status Reader ready!");
    Serial.println("========================================");
    Serial.print("Monitoring Motors: ID=");
    Serial.print(MOTOR_ID_1);
    Serial.print(" and ID=");
    Serial.println(MOTOR_ID_2);
    Serial.print("Update rate: ");
    Serial.print(1000 / UPDATE_INTERVAL_MS);
    Serial.println(" Hz per motor\n");
}

void loop() {
    // Main loop is now empty - everything handled by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
