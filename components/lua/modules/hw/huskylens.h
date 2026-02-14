/**
 * @file    huskylens.h
 * @brief   Library for HuskyLens AI Camera Sensor
 * @author  Adapted from DFRobot HUSKYLENS Arduino Library
 *
 * @copyright	[DFRobot]( http://www.dfrobot.com ), 2016
 * @copyright	GNU Lesser General Public License
 *
 * This library interfaces the HuskyLens AI Camera to ESP32 over I2C.
 * Adapted from Arduino library to work with ESP32 FreeRTOS environment.
 */

 #ifndef HUSKYLENS_H
 #define HUSKYLENS_H
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdio.h>
 
 #include "sdkconfig.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_timer.h"
 
 #include "drivers/i2c_util.h"
 #include "HuskyLensProtocolCore.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 // HuskyLens I2C address
 #define HUSKYLENS_I2C_ADDR       0x32
 
 // Protocol commands
 #define COMMAND_REQUEST          0x20
 #define COMMAND_REQUEST_BLOCKS   0x21
 #define COMMAND_REQUEST_ARROWS   0x22
 #define COMMAND_REQUEST_LEARNED  0x23
 #define COMMAND_REQUEST_BLOCKS_LEARNED 0x24
 #define COMMAND_REQUEST_ARROWS_LEARNED 0x25
 #define COMMAND_REQUEST_BY_ID    0x26
 #define COMMAND_REQUEST_BLOCKS_BY_ID 0x27
 #define COMMAND_REQUEST_ARROWS_BY_ID 0x28
 
 #define COMMAND_RETURN_INFO      0x29
 #define COMMAND_RETURN_BLOCK     0x2A
 #define COMMAND_RETURN_ARROW     0x2B
 
 #define COMMAND_REQUEST_KNOCK    0x2C
 #define COMMAND_REQUEST_ALGORITHM 0x2D
 
 #define COMMAND_RETURN_OK        0x2E
 #define COMMAND_REQUEST_LEARN    0x2F
 #define COMMAND_REQUEST_FORGET   0x30
 
 // Additional commands
 #define COMMAND_REQUEST_CUSTOMNAMES 0x31
 #define COMMAND_REQUEST_PHOTO       0x32
 #define COMMAND_REQUEST_SEND_KNOWLEDGES 0x33
 #define COMMAND_REQUEST_RECEIVE_KNOWLEDGES 0x34
 #define COMMAND_REQUEST_CUSTOM_TEXT 0x35
 #define COMMAND_REQUEST_CLEAR_TEXT  0x36
 #define COMMAND_REQUEST_SAVE_SCREENSHOT 0x37
 #define COMMAND_REQUEST_IS_PRO      0x38
 #define COMMAND_REQUEST_SENSOR      0x39
 #define COMMAND_REQUEST_FIRMWARE_VERSION 0X3C
 
 // Algorithm types
 #define ALGORITHM_FACE_RECOGNITION    0
 #define ALGORITHM_OBJECT_TRACKING     1
 #define ALGORITHM_OBJECT_RECOGNITION  2
 #define ALGORITHM_LINE_TRACKING       3
 #define ALGORITHM_COLOR_RECOGNITION   4
 #define ALGORITHM_TAG_RECOGNITION     5
#define ALGORITHM_OBJECT_CLASSIFICATION 6

/** Default timeout (ms) to wait for first valid response with knowledgeSize > 0 after algorithm change. */
#define HUSKYLENS_ALGORITHM_READY_TIMEOUT_MS  300

// Result structure
 typedef struct {
     uint8_t command;
     int16_t first;
     int16_t second;
     int16_t third;
     int16_t fourth;
     int16_t fifth;
 } huskylens_result_t;
 
// Main HuskyLens structure
typedef struct {
    int i2cdevice;
    uint8_t address;
    unsigned long timeOutDuration;
    unsigned long timeOutTimer;
    /** Max ms to wait for first frame with knowledgeSize > 0 after algorithm change (0 = use default). */
    unsigned long algorithmReadyTimeoutMs;
    int16_t currentIndex;
     int16_t protocolSize;
     int16_t frameNum;
     int16_t knowledgeSize;
     huskylens_result_t *protocolPtr;
     huskylens_result_t resultDefault;
 } huskylens_t;
 
 // Function declarations
 
 /**
  * @brief Initialize HuskyLens device
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_init(huskylens_t *husky);
 
 /**
  * @brief Set timeout duration for operations
  * @param husky Pointer to huskylens_t structure
  * @param duration Timeout duration in milliseconds
  */
 void huskylens_set_timeout(huskylens_t *husky, unsigned long duration);
 
 /**
  * @brief Request data from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request(huskylens_t *husky);
 
 /**
  * @brief Request data by ID from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @param id ID to request
  * @return true if successful, false otherwise
  */
 bool huskylens_request_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Request blocks data from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request_blocks(huskylens_t *husky);
 
 /**
  * @brief Request blocks by ID from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @param id ID to request
  * @return true if successful, false otherwise
  */
 bool huskylens_request_blocks_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Request arrows data from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request_arrows(huskylens_t *husky);
 
 /**
  * @brief Request arrows by ID from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @param id ID to request
  * @return true if successful, false otherwise
  */
 bool huskylens_request_arrows_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Request learned data from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request_learned(huskylens_t *husky);
 
 /**
  * @brief Request learned blocks from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request_blocks_learned(huskylens_t *husky);
 
 /**
  * @brief Request learned arrows from HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_request_arrows_learned(huskylens_t *husky);
 
 /**
  * @brief Get number of available results
  * @param husky Pointer to huskylens_t structure
  * @return Number of available results
  */
 int16_t huskylens_available(huskylens_t *husky);
 
 /**
  * @brief Read next result
  * @param husky Pointer to huskylens_t structure
  * @return Result structure
  */
 huskylens_result_t huskylens_read(huskylens_t *husky);
 
 /**
  * @brief Get result by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of result to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get result by ID and index
  * @param husky Pointer to huskylens_t structure
  * @param id ID to get
  * @param index Index of result to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_by_id(huskylens_t *husky, int16_t id, int16_t index);
 
 /**
  * @brief Get block by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of block to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_block(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get block by ID and index
  * @param husky Pointer to huskylens_t structure
  * @param id ID to get
  * @param index Index of block to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_block_by_id(huskylens_t *husky, int16_t id, int16_t index);
 
 /**
  * @brief Get arrow by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of arrow to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_arrow(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get arrow by ID and index
  * @param husky Pointer to huskylens_t structure
  * @param id ID to get
  * @param index Index of arrow to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_arrow_by_id(huskylens_t *husky, int16_t id, int16_t index);
 
 /**
  * @brief Get learned object by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of learned object to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_learned(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get learned block by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of learned block to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_block_learned(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get learned arrow by index
  * @param husky Pointer to huskylens_t structure
  * @param index Index of learned arrow to get
  * @return Result structure
  */
 huskylens_result_t huskylens_get_arrow_learned(huskylens_t *husky, int16_t index);
 
 /**
  * @brief Get frame number
  * @param husky Pointer to huskylens_t structure
  * @return Frame number
  */
 int16_t huskylens_frame_number(huskylens_t *husky);
 
 /**
  * @brief Get count of learned IDs
  * @param husky Pointer to huskylens_t structure
  * @return Number of learned IDs
  */
 int16_t huskylens_count_learned_ids(huskylens_t *husky);
 
 /**
  * @brief Get total count of results
  * @param husky Pointer to huskylens_t structure
  * @return Total count of results
  */
 int16_t huskylens_count(huskylens_t *husky);
 
 /**
  * @brief Get count of results by ID
  * @param husky Pointer to huskylens_t structure
  * @param id ID to count
  * @return Count of results for the given ID
  */
 int16_t huskylens_count_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Get count of blocks
  * @param husky Pointer to huskylens_t structure
  * @return Count of blocks
  */
 int16_t huskylens_count_blocks(huskylens_t *husky);
 
 /**
  * @brief Get count of blocks by ID
  * @param husky Pointer to huskylens_t structure
  * @param id ID to count
  * @return Count of blocks for the given ID
  */
 int16_t huskylens_count_blocks_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Get count of arrows
  * @param husky Pointer to huskylens_t structure
  * @return Count of arrows
  */
 int16_t huskylens_count_arrows(huskylens_t *husky);
 
 /**
  * @brief Get count of arrows by ID
  * @param husky Pointer to huskylens_t structure
  * @param id ID to count
  * @return Count of arrows for the given ID
  */
 int16_t huskylens_count_arrows_by_id(huskylens_t *husky, int16_t id);
 
 /**
  * @brief Get count of learned objects
  * @param husky Pointer to huskylens_t structure
  * @return Count of learned objects
  */
 int16_t huskylens_count_learned(huskylens_t *husky);
 
 /**
  * @brief Get count of learned blocks
  * @param husky Pointer to huskylens_t structure
  * @return Count of learned blocks
  */
 int16_t huskylens_count_blocks_learned(huskylens_t *husky);
 
 /**
  * @brief Get count of learned arrows
  * @param husky Pointer to huskylens_t structure
  * @return Count of learned arrows
  */
int16_t huskylens_count_arrows_learned(huskylens_t *husky);

/**
 * @brief Set max time (ms) to wait for device to report trained after algorithm change. 0 = use default (300ms).
 * @param husky Pointer to huskylens_t structure
 * @param ms Timeout in milliseconds (0 for default)
 */
void huskylens_set_algorithm_ready_timeout_ms(huskylens_t *husky, unsigned long ms);

/**
 * @brief Set algorithm type; blocks until device sends a frame with knowledgeSize > 0 (trained) or timeout.
 * @param husky Pointer to huskylens_t structure
 * @param algorithm Algorithm type to set
 * @return true if successful (device reported trained), false on timeout or not trained for this algorithm
 */
bool huskylens_write_algorithm(huskylens_t *husky, uint8_t algorithm);
 
 /**
  * @brief Learn object with ID
  * @param husky Pointer to huskylens_t structure
  * @param id ID to learn
  * @return true if successful, false otherwise
  */
 bool huskylens_write_learn(huskylens_t *husky, int id);
 
 /**
  * @brief Forget all learned objects
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_write_forget(huskylens_t *husky);
 
 /**
  * @brief Check if there are learned objects
  * @param husky Pointer to huskylens_t structure
  * @return true if there are learned objects, false otherwise
  */
 bool huskylens_is_learned(huskylens_t *husky);
 
 /**
  * @brief Send sensor data to HuskyLens
  * @param husky Pointer to huskylens_t structure
  * @param sensor0 First sensor value
  * @param sensor1 Second sensor value
  * @param sensor2 Third sensor value
  * @return true if successful, false otherwise
  */
 bool huskylens_write_sensor(huskylens_t *husky, int sensor0, int sensor1, int sensor2);
 
 /**
  * @brief Set custom name for an ID
  * @param husky Pointer to huskylens_t structure
  * @param name Custom name string
  * @param id ID to set name for
  * @return true if successful, false otherwise
  */
 bool huskylens_set_custom_name(huskylens_t *husky, const char *name, uint8_t id);
 
 /**
  * @brief Save picture to SD card
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_save_picture_to_sd(huskylens_t *husky);
 
 /**
  * @brief Save model to SD card
  * @param husky Pointer to huskylens_t structure
  * @param fileNum File number to save to
  * @return true if successful, false otherwise
  */
 bool huskylens_save_model_to_sd(huskylens_t *husky, int fileNum);
 
 /**
  * @brief Load model from SD card
  * @param husky Pointer to huskylens_t structure
  * @param fileNum File number to load from
  * @return true if successful, false otherwise
  */
 bool huskylens_load_model_from_sd(huskylens_t *husky, int fileNum);
 
 /**
  * @brief Clear custom text
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_clear_custom_text(huskylens_t *husky);
 
 /**
  * @brief Display custom text
  * @param husky Pointer to huskylens_t structure
  * @param text Text to display
  * @param x X coordinate
  * @param y Y coordinate
  * @return true if successful, false otherwise
  */
 bool huskylens_custom_text(huskylens_t *husky, const char *text, uint16_t x, uint8_t y);
 
 /**
  * @brief Save screenshot to SD card
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_save_screenshot_to_sd(huskylens_t *husky);
 
 /**
  * @brief Check if HuskyLens is Pro version
  * @param husky Pointer to huskylens_t structure
  * @return true if Pro version, false otherwise
  */
 bool huskylens_is_pro(huskylens_t *husky);
 
 /**
  * @brief Check firmware version
  * @param husky Pointer to huskylens_t structure
  * @return true if successful, false otherwise
  */
 bool huskylens_check_firmware_version(huskylens_t *husky);
 
 /**
  * @brief Write firmware version
  * @param husky Pointer to huskylens_t structure
  * @param version Version string
  * @return true if successful, false otherwise
  */
 bool huskylens_write_firmware_version(huskylens_t *husky, const char *version);
 
 /**
  * @brief Run a complete test of HuskyLens functionality
  * This function initializes the device, sets up object tracking,
  * requests data once, and displays the results
  */
 void huskylens_test(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // HUSKYLENS_H 