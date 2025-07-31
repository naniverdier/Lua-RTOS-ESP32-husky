/**
 * @file    huskylens.c
 * @brief   Library for HuskyLens AI Camera Sensor
 * @author  Adapted from DFRobot HUSKYLENS Arduino Library
 *
 * @copyright	[DFRobot]( http://www.dfrobot.com ), 2016
 * @copyright	GNU Lesser General Public License
 *
 * This library interfaces the HuskyLens AI Camera to ESP32 over I2C.
 * Adapted from Arduino library to work with ESP32 FreeRTOS environment.
 */

#include "huskylens.h"

#define max(a,b) ((a) > (b) ? (a) : (b))

// Helper function to get current time in milliseconds
static unsigned long get_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000);
}

// Helper function to write protocol data
static void protocol_write(huskylens_t *husky, uint8_t *buffer, int length) {
    i2c_util_writeMulti(husky->i2cdevice, husky->address, 0, buffer, length);
}

// Helper function to read protocol data
static bool protocol_read(huskylens_t *husky, uint8_t *buffer, int length) {
    int result = i2c_util_readMulti(husky->i2cdevice, husky->address, 0, buffer, length);
    return (result >= 0);
}

// Helper function to start timeout timer
static void timer_begin(huskylens_t *husky) {
    husky->timeOutTimer = get_millis();
}

// Helper function to check if timeout has occurred
static bool timer_available(huskylens_t *husky) {
    return (get_millis() - husky->timeOutTimer > husky->timeOutDuration);
}

// Helper function to check if protocol data is available
static bool protocol_available(huskylens_t *husky) {
    uint8_t buffer[16];
    if (protocol_read(husky, buffer, 16)) {
        for (int i = 0; i < 16; i++) {
            if (husky_lens_protocol_receive(buffer[i])) {
                return true;
            }
        }
    }
    return false;
}

// Helper function to wait for specific command
static bool wait_for_command(huskylens_t *husky, uint8_t command) {
    timer_begin(husky);
    while (!timer_available(husky)) {
        if (protocol_available(husky)) {
            if (command) {
                if (husky_lens_protocol_read_begin(command)) {
                    return true;
                }
            } else {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent busy waiting
    }
    return false;
}

// Helper function to process return data
static bool process_return(huskylens_t *husky) {
    husky->currentIndex = 0;
    
    if (!wait_for_command(husky, COMMAND_RETURN_INFO)) {
        return false;
    }
    
    // Read return info
    husky->protocolSize = husky_lens_protocol_read_int16();
    husky->frameNum = husky_lens_protocol_read_int16();
    husky->knowledgeSize = husky_lens_protocol_read_int16();
    husky_lens_protocol_read_end();
    
    // Allocate memory for results
    if (husky->protocolPtr) {
        free(husky->protocolPtr);
    }
    husky->protocolPtr = (huskylens_result_t *)malloc(max((int)husky->protocolSize, 1) * sizeof(huskylens_result_t));
    
    if (!husky->protocolPtr) {
        return false;
    }
    
    // Read all results
    for (int i = 0; i < husky->protocolSize; i++) {
        if (!wait_for_command(husky, 0)) {
            return false;
        }
        
        if (husky_lens_protocol_read_begin(COMMAND_RETURN_BLOCK)) {
            husky->protocolPtr[i].command = COMMAND_RETURN_BLOCK;
            husky->protocolPtr[i].first = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].second = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].third = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].fourth = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].fifth = husky_lens_protocol_read_int16();
            husky_lens_protocol_read_end();
        } else if (husky_lens_protocol_read_begin(COMMAND_RETURN_ARROW)) {
            husky->protocolPtr[i].command = COMMAND_RETURN_ARROW;
            husky->protocolPtr[i].first = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].second = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].third = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].fourth = husky_lens_protocol_read_int16();
            husky->protocolPtr[i].fifth = husky_lens_protocol_read_int16();
            husky_lens_protocol_read_end();
        } else {
            return false;
        }
    }
    
    return true;
}

// Helper function to read knock (handshake)
static bool read_knock(huskylens_t *husky) {
    for (int i = 0; i < 5; i++) {
        // Send knock command
        uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_KNOCK);
        int length = husky_lens_protocol_write_end();
        protocol_write(husky, buffer, length);
        
        if (wait_for_command(husky, COMMAND_RETURN_OK)) {
            return true;
        }
    }
    return false;
}

// Initialize HuskyLens device
bool huskylens_init(huskylens_t *husky) {
    printf("Initializing HuskyLens...\n");
    
    // Initialize structure
    memset(husky, 0, sizeof(huskylens_t));
    husky->address = HUSKYLENS_I2C_ADDR;
    husky->timeOutDuration = 100; // Default timeout 100ms
    
    // Set default result values
    husky->resultDefault.command = -1;
    husky->resultDefault.first = -1;
    husky->resultDefault.second = -1;
    husky->resultDefault.third = -1;
    husky->resultDefault.fourth = -1;
    husky->resultDefault.fifth = -1;
    
    // Attach to I2C
    driver_error_t *error;
    uint8_t i2c = CONFIG_APDS9960_I2C_CHANNEL; // Using same config as example
    if ((error = i2c_attach(i2c, I2C_MASTER, CONFIG_APDS9960_SPEED, 0, 0, &husky->i2cdevice))) {
        printf("Failed to attach I2C: %s\n", error->msg ? error->msg : "Unknown error");
        free(error);
        return false;
    }
    
    printf("I2C attached successfully\n");
    
    // Perform handshake
    if (!read_knock(husky)) {
        printf("Failed to establish communication with HuskyLens\n");
        return false;
    }
    
    printf("HuskyLens initialized successfully\n");
    return true;
}

// Set timeout duration
void huskylens_set_timeout(huskylens_t *husky, unsigned long duration) {
    husky->timeOutDuration = duration;
}

// Request data from HuskyLens
bool huskylens_request(huskylens_t *husky) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return process_return(husky);
}

// Request blocks data from HuskyLens
bool huskylens_request_blocks(huskylens_t *husky) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_BLOCKS);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return process_return(husky);
}

// Request arrows data from HuskyLens
bool huskylens_request_arrows(huskylens_t *husky) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_ARROWS);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return process_return(husky);
}

// Get number of available results
int16_t huskylens_available(huskylens_t *husky) {
    int16_t result = huskylens_count(husky);
    husky->currentIndex = (husky->currentIndex < result) ? husky->currentIndex : result;
    return result - husky->currentIndex;
}

// Read next result
huskylens_result_t huskylens_read(huskylens_t *husky) {
    return huskylens_get(husky, husky->currentIndex++);
}

// Get result by index
huskylens_result_t huskylens_get(huskylens_t *husky, int16_t index) {
    if (index < huskylens_count(husky)) {
        return husky->protocolPtr[index];
    }
    return husky->resultDefault;
}

// Get frame number
int16_t huskylens_frame_number(huskylens_t *husky) {
    return husky->frameNum;
}

// Get count of learned IDs
int16_t huskylens_count_learned_ids(huskylens_t *husky) {
    return husky->knowledgeSize;
}

// Get total count of results
int16_t huskylens_count(huskylens_t *husky) {
    return husky->protocolSize;
}

// Set algorithm type
bool huskylens_write_algorithm(huskylens_t *husky, uint8_t algorithm) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_ALGORITHM);
    husky_lens_protocol_write_int16(algorithm);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return wait_for_command(husky, COMMAND_RETURN_OK);
}

// Learn object with ID
bool huskylens_write_learn(huskylens_t *husky, int id) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_LEARN);
    husky_lens_protocol_write_int16(id);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return wait_for_command(husky, COMMAND_RETURN_OK);
}

// Forget all learned objects
bool huskylens_write_forget(huskylens_t *husky) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_FORGET);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    return wait_for_command(husky, COMMAND_RETURN_OK);
}

// Test function implementation
void huskylens_test(void) {
    printf("=== HuskyLens Test Program ===\n");
    
    // Initialize HuskyLens
    huskylens_t husky;
    if (!huskylens_init(&husky)) {
        printf("Failed to initialize HuskyLens!\n");
        return;
    }
    
    printf("HuskyLens initialized successfully!\n");
    
    // Set algorithm to object tracking for testing
    if (huskylens_write_algorithm(&husky, ALGORITHM_OBJECT_TRACKING)) {
        printf("Algorithm set to OBJECT_TRACKING\n");
    } else {
        printf("Failed to set algorithm\n");
    }
    
    // Request data once
    printf("\n--- Requesting data from HuskyLens ---\n");
    
    // Request data
    if (huskylens_request(&husky)) {
        printf("Data received successfully!\n");
        printf("Frame number: %d\n", huskylens_frame_number(&husky));
        printf("Knowledge size: %d\n", huskylens_count_learned_ids(&husky));
        printf("Total results: %d\n", huskylens_count(&husky));
        
        // Read all available results
        int16_t available = huskylens_available(&husky);
        printf("Available results: %d\n", available);
        
        for (int i = 0; i < available; i++) {
            huskylens_result_t result = huskylens_read(&husky);
            printf("Result %d: command=%d, first=%d, second=%d, third=%d, fourth=%d, fifth=%d\n",
                   i, result.command, result.first, result.second, result.third, result.fourth, result.fifth);
        }
    } else {
        printf("Failed to receive data from HuskyLens\n");
    }
    
    printf("=== HuskyLens Test Completed ===\n");
} 