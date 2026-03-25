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
 * I2C access uses raw write/read (no register byte) to match Arduino Wire behavior.
 */

 #include "huskylens.h"
 #include "drivers/i2c.h"
 
 // Helper macro for max function
 #define max(a,b) ((a) > (b) ? (a) : (b))
 
 #define HUSKYLENS_I2C_READ_CHUNK  16
 #define HUSKYLENS_LEFTOVER_MAX    (HUSKYLENS_I2C_READ_CHUNK - 1)
 
 // Bytes left in the last chunk after a complete frame; fed at start of next protocol_available
 static uint8_t s_leftover[HUSKYLENS_LEFTOVER_MAX];
 static int s_leftover_len;
 
 // HuskyLens uses frame-based protocol; no I2C register. Raw write: address + payload only.
 static bool huskylens_i2c_write_raw(int deviceid, uint8_t address, const uint8_t *data, int len) {
     driver_error_t *error;
     int transaction = I2C_TRANSACTION_INITIALIZER;
     if ((error = i2c_start(deviceid, &transaction))) {
         free(error);
         return false;
     }
     if ((error = i2c_write_address(deviceid, &transaction, (char)address, 0))) {
         free(error);
         return false;
     }
     if ((error = i2c_write(deviceid, &transaction, (char *)data, len))) {
         free(error);
         return false;
     }
     if ((error = i2c_stop(deviceid, &transaction))) {
         free(error);
         return false;
     }
     return true;
 }
 
 // Raw read: address + read only (no register byte before read), like Arduino requestFrom().
 static bool huskylens_i2c_read_raw(int deviceid, uint8_t address, uint8_t *buffer, int len) {
     driver_error_t *error;
     int transaction = I2C_TRANSACTION_INITIALIZER;
     if ((error = i2c_start(deviceid, &transaction))) {
         free(error);
         return false;
     }
     if ((error = i2c_write_address(deviceid, &transaction, (char)address, 1))) {
         free(error);
         return false;
     }
     if ((error = i2c_read(deviceid, &transaction, (char *)buffer, len))) {
         free(error);
         return false;
     }
     if ((error = i2c_stop(deviceid, &transaction))) {
         free(error);
         return false;
     }
     return true;
 }
 
 // Helper function to get current time in milliseconds
 static unsigned long get_millis() {
     return (unsigned long)(esp_timer_get_time() / 1000);
 }

static void huskylens_reset_results(huskylens_t *husky) {
    if (!husky) {
        return;
    }
    husky->currentIndex = 0;
    husky->protocolSize = 0;
    husky->frameNum = 0;
    husky->knowledgeSize = 0;
}
 
 // Write protocol frame (no register byte)
 static void protocol_write(huskylens_t *husky, uint8_t *buffer, int length) {
     (void)huskylens_i2c_write_raw(husky->i2cdevice, husky->address, buffer, length);
 }
 
 // Read up to length bytes into buffer (no register byte). Returns true if read succeeded.
 static bool protocol_read(huskylens_t *husky, uint8_t *buffer, int length) {
     return huskylens_i2c_read_raw(husky->i2cdevice, husky->address, buffer, length);
 }
  
  // Helper function to start timeout timer
  static void timer_begin(huskylens_t *husky) {
      husky->timeOutTimer = get_millis();
  }
  
  // Helper function to check if timeout has occurred
  static bool timer_available(huskylens_t *husky) {
      return (get_millis() - husky->timeOutTimer > husky->timeOutDuration);
  }
  
  // Like Arduino: request 16 bytes, drain into parser. When a frame completes we return
  // true and stash any remaining bytes in the chunk for the next call (receive_buffer
  // must not be overwritten before the caller uses it).
  static bool protocol_available(huskylens_t *husky) {
      uint8_t buffer[HUSKYLENS_I2C_READ_CHUNK];
      int j;
      /* Feed any leftover bytes from the previous chunk */
      for (j = 0; j < s_leftover_len; j++) {
          if (husky_lens_protocol_receive(s_leftover[j])) {
              j++;
              if (j < s_leftover_len) {
                  memmove(s_leftover, &s_leftover[j], (size_t)(s_leftover_len - j));
                  s_leftover_len -= j;
              } else {
                  s_leftover_len = 0;
              }
              return true;
          }
      }
      s_leftover_len = 0;
      /* Read and process chunks */
      while (protocol_read(husky, buffer, HUSKYLENS_I2C_READ_CHUNK)) {
          for (int i = 0; i < HUSKYLENS_I2C_READ_CHUNK; i++) {
              if (husky_lens_protocol_receive(buffer[i])) {
                  i++;
                  if (i < HUSKYLENS_I2C_READ_CHUNK) {
                      int tail = HUSKYLENS_I2C_READ_CHUNK - i;
                      memcpy(s_leftover, &buffer[i], (size_t)tail);
                      s_leftover_len = tail;
                  }
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
      
      /* FiveInt16 wire order matches Arduino HUSKYLENS protocolReadReturnInfo: first=protocolSize,
       * second=knowledgeSize, third=frameNum (not frameNum before knowledgeSize). */
      husky->protocolSize = husky_lens_protocol_read_int16();
      husky->knowledgeSize = husky_lens_protocol_read_int16();
      husky->frameNum = husky_lens_protocol_read_int16();
      (void)husky_lens_protocol_read_int16(); /* fourth */
      (void)husky_lens_protocol_read_int16(); /* fifth */
      if (!husky_lens_protocol_read_end()) {
          return false;
      }
      
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
              if (!husky_lens_protocol_read_end()) {
                  return false;
              }
          } else if (husky_lens_protocol_read_begin(COMMAND_RETURN_ARROW)) {
              husky->protocolPtr[i].command = COMMAND_RETURN_ARROW;
              husky->protocolPtr[i].first = husky_lens_protocol_read_int16();
              husky->protocolPtr[i].second = husky_lens_protocol_read_int16();
              husky->protocolPtr[i].third = husky_lens_protocol_read_int16();
              husky->protocolPtr[i].fourth = husky_lens_protocol_read_int16();
              husky->protocolPtr[i].fifth = husky_lens_protocol_read_int16();
              if (!husky_lens_protocol_read_end()) {
                  return false;
              }
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
     // Share the same I2C bus as APDS9960 (both devices have different addresses)
     // APDS9960: 0x39, HuskyLens: 0x32
     driver_error_t *error;
     uint8_t i2c = CONFIG_APDS9960_I2C_CHANNEL;
     
     printf("Attaching HuskyLens to I2C channel %d (shared with APDS9960)\n", i2c);
     
     // Try to attach with speed configuration
     error = i2c_attach(i2c, I2C_MASTER, CONFIG_HUSKYLENS_SPEED, 0, 0, &husky->i2cdevice);
     
     if (error) {
         // Bus might already be configured by APDS9960, try to attach without reconfiguring
         printf("I2C bus already in use, attaching to existing bus...\n");
         free(error);
         
         // Attach with speed=0 means: don't reconfigure the bus, just attach
         error = i2c_attach(i2c, I2C_MASTER, 0, 0, 0, &husky->i2cdevice);
         
         if (error) {
             printf("Failed to attach to I2C bus: %s\n", error->msg ? error->msg : "Unknown error");
             free(error);
             return false;
         }
         printf("Successfully attached to existing I2C bus\n");
     } else {
         printf("I2C bus initialized and attached successfully\n");
     }
      
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
  
  // Request data by ID from HuskyLens
  bool huskylens_request_by_id(huskylens_t *husky, int16_t id) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_BY_ID);
      husky_lens_protocol_write_int16(id);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return process_return(husky);
  }
  
  // Request blocks by ID from HuskyLens
  bool huskylens_request_blocks_by_id(huskylens_t *husky, int16_t id) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_BLOCKS_BY_ID);
      husky_lens_protocol_write_int16(id);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return process_return(husky);
  }
  
  // Request arrows by ID from HuskyLens
  bool huskylens_request_arrows_by_id(huskylens_t *husky, int16_t id) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_ARROWS_BY_ID);
      husky_lens_protocol_write_int16(id);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return process_return(husky);
  }
  
  // Request learned data from HuskyLens
  bool huskylens_request_learned(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_LEARNED);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return process_return(husky);
  }
  
  // Request learned blocks from HuskyLens
  bool huskylens_request_blocks_learned(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_BLOCKS_LEARNED);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return process_return(husky);
  }
  
  // Request learned arrows from HuskyLens
  bool huskylens_request_arrows_learned(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_ARROWS_LEARNED);
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
  
  // Get result by ID and index
  huskylens_result_t huskylens_get_by_id(huskylens_t *husky, int16_t id, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].fifth == id) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get block by index
  huskylens_result_t huskylens_get_block(huskylens_t *husky, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get block by ID and index
  huskylens_result_t huskylens_get_block_by_id(huskylens_t *husky, int16_t id, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK && husky->protocolPtr[i].fifth == id) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get arrow by index
  huskylens_result_t huskylens_get_arrow(huskylens_t *husky, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get arrow by ID and index
  huskylens_result_t huskylens_get_arrow_by_id(huskylens_t *husky, int16_t id, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW && husky->protocolPtr[i].fifth == id) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get learned object by index
  huskylens_result_t huskylens_get_learned(huskylens_t *husky, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].fifth > 0) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get learned block by index
  huskylens_result_t huskylens_get_block_learned(huskylens_t *husky, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK && husky->protocolPtr[i].fifth > 0) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
      }
      return husky->resultDefault;
  }
  
  // Get learned arrow by index
  huskylens_result_t huskylens_get_arrow_learned(huskylens_t *husky, int16_t index) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW && husky->protocolPtr[i].fifth > 0) {
              if (index == counter++) {
                  return husky->protocolPtr[i];
              }
          }
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
  
  // Get count of results by ID
  int16_t huskylens_count_by_id(huskylens_t *husky, int16_t id) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].fifth == id) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of blocks
  int16_t huskylens_count_blocks(huskylens_t *husky) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of blocks by ID
  int16_t huskylens_count_blocks_by_id(huskylens_t *husky, int16_t id) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK && husky->protocolPtr[i].fifth == id) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of arrows
  int16_t huskylens_count_arrows(huskylens_t *husky) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of arrows by ID
  int16_t huskylens_count_arrows_by_id(huskylens_t *husky, int16_t id) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW && husky->protocolPtr[i].fifth == id) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of learned objects
  int16_t huskylens_count_learned(huskylens_t *husky) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].fifth > 0) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of learned blocks
  int16_t huskylens_count_blocks_learned(huskylens_t *husky) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_BLOCK && husky->protocolPtr[i].fifth > 0) {
              counter++;
          }
      }
      return counter;
  }
  
  // Get count of learned arrows
  int16_t huskylens_count_arrows_learned(huskylens_t *husky) {
      int16_t counter = 0;
      for (int i = 0; i < husky->protocolSize; i++) {
          if (husky->protocolPtr[i].command == COMMAND_RETURN_ARROW && husky->protocolPtr[i].fifth > 0) {
              counter++;
          }
      }
      return counter;
  }
  
 void huskylens_set_algorithm_ready_timeout_ms(huskylens_t *husky, unsigned long ms) {
     husky->algorithmReadyTimeoutMs = ms;
 }
 
/* Set algorithm; after COMMAND_RETURN_OK, retry huskylens_request until process_return succeeds (0..N results OK) or timeout. */
bool huskylens_write_algorithm(huskylens_t *husky, uint8_t algorithm) {
    uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_ALGORITHM);
    husky_lens_protocol_write_int16(algorithm);
    int length = husky_lens_protocol_write_end();
    protocol_write(husky, buffer, length);
    if (!wait_for_command(husky, COMMAND_RETURN_OK)) {
        return false;
    }
    unsigned long ready_timeout = husky->algorithmReadyTimeoutMs > 0
        ? husky->algorithmReadyTimeoutMs
        : (unsigned long)HUSKYLENS_ALGORITHM_READY_TIMEOUT_MS;
    unsigned long saved_timeout = husky->timeOutDuration;
    unsigned long deadline = get_millis() + ready_timeout;
    while (get_millis() < deadline) {
        husky->timeOutDuration = 25;
        if (huskylens_request(husky)) {
            husky->timeOutDuration = saved_timeout;
            huskylens_reset_results(husky);
            return true;
        }
        husky->timeOutDuration = saved_timeout;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    husky->timeOutDuration = saved_timeout;
    return false;
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
  
  // Send sensor data to HuskyLens
  bool huskylens_write_sensor(huskylens_t *husky, int sensor0, int sensor1, int sensor2) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_SENSOR);
      husky_lens_protocol_write_int16(sensor0);
      husky_lens_protocol_write_int16(sensor1);
      husky_lens_protocol_write_int16(sensor2);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Set custom name for an ID
  bool huskylens_set_custom_name(huskylens_t *husky, const char *name, uint8_t id) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_CUSTOMNAMES);
      husky_lens_protocol_write_uint8(id);
      husky_lens_protocol_write_uint8(strlen(name));
      for (int i = 0; i < strlen(name) && i < 20; i++) {
          husky_lens_protocol_write_uint8(name[i]);
      }
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Save picture to SD card
  bool huskylens_save_picture_to_sd(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_PHOTO);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Save model to SD card
  bool huskylens_save_model_to_sd(huskylens_t *husky, int fileNum) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_SEND_KNOWLEDGES);
      husky_lens_protocol_write_int16(fileNum);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Load model from SD card
  bool huskylens_load_model_from_sd(huskylens_t *husky, int fileNum) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_RECEIVE_KNOWLEDGES);
      husky_lens_protocol_write_int16(fileNum);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Clear custom text
  bool huskylens_clear_custom_text(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_CLEAR_TEXT);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Display custom text
  bool huskylens_custom_text(huskylens_t *husky, const char *text, uint16_t x, uint8_t y) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_CUSTOM_TEXT);
      husky_lens_protocol_write_uint8(strlen(text));
      if (x >= 255) {
          husky_lens_protocol_write_uint8(0xFF);
      } else {
          husky_lens_protocol_write_uint8(0x00);
      }
      husky_lens_protocol_write_uint8(x & 0xFF);
      husky_lens_protocol_write_uint8(y);
      for (int i = 0; i < strlen(text) && i < 20; i++) {
          husky_lens_protocol_write_uint8(text[i]);
      }
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Save screenshot to SD card
  bool huskylens_save_screenshot_to_sd(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_SAVE_SCREENSHOT);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Check if HuskyLens is Pro version
  bool huskylens_is_pro(huskylens_t *husky) {
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_IS_PRO);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      
      if (wait_for_command(husky, COMMAND_RETURN_INFO)) {
          /* Arduino: protocolReadOneInt16 -> one int16 then read_end */
          int16_t first = husky_lens_protocol_read_int16();
          if (!husky_lens_protocol_read_end()) {
              return false;
          }
          return (first != 0);
      }
      return false;
  }
  
  // Check firmware version
  bool huskylens_check_firmware_version(huskylens_t *husky) {
      return huskylens_write_firmware_version(husky, "0.4.1");
  }
  
  // Write firmware version (Arduino: length byte then version string, no null)
  bool huskylens_write_firmware_version(huskylens_t *husky, const char *version) {
      size_t len = strlen(version);
      uint8_t *buffer = husky_lens_protocol_write_begin(COMMAND_REQUEST_FIRMWARE_VERSION);
      husky_lens_protocol_write_uint8((uint8_t)len);
      husky_lens_protocol_write_buffer_uint8((uint8_t *)version, (uint32_t)len);
      int length = husky_lens_protocol_write_end();
      protocol_write(husky, buffer, length);
      return wait_for_command(husky, COMMAND_RETURN_OK);
  }
  
  // Check if there are learned objects
  bool huskylens_is_learned(huskylens_t *husky) {
      return (husky->knowledgeSize > 0);
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