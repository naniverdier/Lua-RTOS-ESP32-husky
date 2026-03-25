#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_HUSKYLENS

#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "modules.h"

#ifdef __cplusplus
  #include "lua.hpp"
#else
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#endif

#include "huskylens.h"
#include "huskylens_mapper.h"
#include "sys.h"

static bool initialized = false;
static huskylens_t husky_instance;
static lua_callback_t *s_mapper_conn_cb = NULL;

static void mapper_conn_cb(bool connected) {
    if (!s_mapper_conn_cb) {
        return;
    }
    lua_pushboolean(luaS_callback_state(s_mapper_conn_cb), connected);
    luaS_callback_call(s_mapper_conn_cb, 1);
}

static int l_huskylens_init(lua_State *L) {
    if (!initialized) {
        bool ok = huskylens_init(&husky_instance);
        if (!ok) {
            lua_pushnil(L);
            lua_pushstring(L, "init failed");
            return 2;
        }
        initialized = true;
    }
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_test(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    // Call the HuskyLens test function
    huskylens_test();
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request(&husky_instance);
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_get_results(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count(&husky_instance);
    int16_t frame = huskylens_frame_number(&husky_instance);
    
    // Create result table
    lua_newtable(L);
    
    // Add frame number
    lua_pushstring(L, "frame");
    lua_pushinteger(L, frame);
    lua_settable(L, -3);
    
    // Add count
    lua_pushstring(L, "count");
    lua_pushinteger(L, count);
    lua_settable(L, -3);
    
    // Add results array
    lua_pushstring(L, "results");
    lua_newtable(L);
    
    for (int i = 0; i < count; i++) {
        huskylens_result_t result = huskylens_get(&husky_instance, i);
        
        lua_pushinteger(L, i + 1); // Lua arrays start at 1
        lua_newtable(L);
        
        lua_pushstring(L, "command");
        lua_pushinteger(L, result.command);
        lua_settable(L, -3);
        
        lua_pushstring(L, "x");
        lua_pushinteger(L, result.first);
        lua_settable(L, -3);
        
        lua_pushstring(L, "y");
        lua_pushinteger(L, result.second);
        lua_settable(L, -3);
        
        lua_pushstring(L, "width");
        lua_pushinteger(L, result.third);
        lua_settable(L, -3);
        
        lua_pushstring(L, "height");
        lua_pushinteger(L, result.fourth);
        lua_settable(L, -3);
        
        lua_pushstring(L, "id");
        lua_pushinteger(L, result.fifth);
        lua_settable(L, -3);
        
        lua_settable(L, -3); // Set result table in results array
    }
    
    lua_settable(L, -3); // Set results array in main table
    
    return 1;
}

static int l_huskylens_set_algorithm(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int algorithm = luaL_checkinteger(L, 1);
    bool ok = huskylens_write_algorithm(&husky_instance, algorithm);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "failed to set algorithm");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_learn(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int id = luaL_checkinteger(L, 1);
    bool ok = huskylens_write_learn(&husky_instance, id);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "failed to learn");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_forget(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_write_forget(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "failed to forget");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_is_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool learned = huskylens_is_learned(&husky_instance);
    lua_pushboolean(L, learned);
    return 1;
}

static int l_huskylens_available(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t available = huskylens_available(&husky_instance);
    lua_pushinteger(L, available);
    return 1;
}

// Request functions
static int l_huskylens_request_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    bool ok = huskylens_request_by_id(&husky_instance, id);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_by_id failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_blocks(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request_blocks(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_blocks failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_blocks_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    bool ok = huskylens_request_blocks_by_id(&husky_instance, id);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_blocks_by_id failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_arrows(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request_arrows(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_arrows failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_arrows_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    bool ok = huskylens_request_arrows_by_id(&husky_instance, id);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_arrows_by_id failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request_learned(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_learned failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_blocks_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request_blocks_learned(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_blocks_learned failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_request_arrows_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_request_arrows_learned(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "request_arrows_learned failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

// Count functions
static int l_huskylens_count(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t count = huskylens_count_by_id(&husky_instance, id);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_blocks(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_blocks(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_blocks_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t count = huskylens_count_blocks_by_id(&husky_instance, id);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_arrows(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_arrows(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_arrows_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t count = huskylens_count_arrows_by_id(&husky_instance, id);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_learned(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_blocks_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_blocks_learned(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_arrows_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_arrows_learned(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

static int l_huskylens_count_learned_ids(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t count = huskylens_count_learned_ids(&husky_instance);
    lua_pushinteger(L, count);
    return 1;
}

// Get functions
static int l_huskylens_get(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t index = luaL_checkinteger(L, 2);
    huskylens_result_t result = huskylens_get_by_id(&husky_instance, id, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_block(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get_block(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_block_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t index = luaL_checkinteger(L, 2);
    huskylens_result_t result = huskylens_get_block_by_id(&husky_instance, id, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_arrow(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get_arrow(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_arrow_by_id(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t id = luaL_checkinteger(L, 1);
    int16_t index = luaL_checkinteger(L, 2);
    huskylens_result_t result = huskylens_get_arrow_by_id(&husky_instance, id, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get_learned(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_block_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get_block_learned(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_get_arrow_learned(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t index = luaL_checkinteger(L, 1);
    huskylens_result_t result = huskylens_get_arrow_learned(&husky_instance, index);
    
    lua_newtable(L);
    lua_pushstring(L, "command");
    lua_pushinteger(L, result.command);
    lua_settable(L, -3);
    
    lua_pushstring(L, "x");
    lua_pushinteger(L, result.first);
    lua_settable(L, -3);
    
    lua_pushstring(L, "y");
    lua_pushinteger(L, result.second);
    lua_settable(L, -3);
    
    lua_pushstring(L, "width");
    lua_pushinteger(L, result.third);
    lua_settable(L, -3);
    
    lua_pushstring(L, "height");
    lua_pushinteger(L, result.fourth);
    lua_settable(L, -3);
    
    lua_pushstring(L, "id");
    lua_pushinteger(L, result.fifth);
    lua_settable(L, -3);
    
    return 1;
}

// Configuration functions
static int l_huskylens_frame_number(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int16_t frame = huskylens_frame_number(&husky_instance);
    lua_pushinteger(L, frame);
    return 1;
}

// Advanced functions
static int l_huskylens_write_sensor(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int sensor0 = luaL_checkinteger(L, 1);
    int sensor1 = luaL_checkinteger(L, 2);
    int sensor2 = luaL_checkinteger(L, 3);
    
    bool ok = huskylens_write_sensor(&husky_instance, sensor0, sensor1, sensor2);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "write_sensor failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_set_custom_name(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    const char *name = luaL_checkstring(L, 1);
    uint8_t id = luaL_checkinteger(L, 2);
    
    bool ok = huskylens_set_custom_name(&husky_instance, name, id);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "set_custom_name failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_save_picture_to_sd(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_save_picture_to_sd(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "save_picture_to_sd failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_save_model_to_sd(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int fileNum = luaL_checkinteger(L, 1);
    bool ok = huskylens_save_model_to_sd(&husky_instance, fileNum);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "save_model_to_sd failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_load_model_from_sd(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    int fileNum = luaL_checkinteger(L, 1);
    bool ok = huskylens_load_model_from_sd(&husky_instance, fileNum);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "load_model_from_sd failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_clear_custom_text(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_clear_custom_text(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "clear_custom_text failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_custom_text(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    const char *text = luaL_checkstring(L, 1);
    uint16_t x = luaL_checkinteger(L, 2);
    uint8_t y = luaL_checkinteger(L, 3);
    
    bool ok = huskylens_custom_text(&husky_instance, text, x, y);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "custom_text failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_save_screenshot_to_sd(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_save_screenshot_to_sd(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "save_screenshot_to_sd failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_is_pro(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool isPro = huskylens_is_pro(&husky_instance);
    lua_pushboolean(L, isPro);
    return 1;
}

static int l_huskylens_check_firmware_version(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    bool ok = huskylens_check_firmware_version(&husky_instance);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "check_firmware_version failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_write_firmware_version(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }
    
    const char *version = luaL_checkstring(L, 1);
    bool ok = huskylens_write_firmware_version(&husky_instance, version);
    
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "write_firmware_version failed");
        return 2;
    }
    
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_mapper_start(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }

    bool ok = huskylens_mapper_start(&husky_instance);
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper start failed");
        return 2;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_mapper_stop(lua_State *L) {
    (void)L;
    huskylens_mapper_stop();
    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_mapper_status(lua_State *L) {
    lua_newtable(L);
    
    lua_pushstring(L, "running");
    lua_pushboolean(L, huskylens_mapper_is_running());
    lua_settable(L, -3);
    
    lua_pushstring(L, "ble_initialized");
    lua_pushboolean(L, huskylens_mapper_is_ble_initialized());
    lua_settable(L, -3);
    
    lua_pushstring(L, "connected");
    lua_pushboolean(L, huskylens_mapper_is_connected());
    lua_settable(L, -3);
    
    return 1;
}

static int l_huskylens_mapper_get(lua_State *L) {
    uint8_t pairs[HUSKYLENS_MAP_MAX * 2];
    uint8_t count = huskylens_mapper_get_pairs(pairs, HUSKYLENS_MAP_MAX);

    lua_newtable(L);
    for (uint8_t i = 0; i < count; i++) {
        lua_pushinteger(L, i + 1);
        lua_newtable(L);

        lua_pushstring(L, "physical");
        lua_pushinteger(L, pairs[i * 2]);
        lua_settable(L, -3);

        lua_pushstring(L, "logical");
        lua_pushinteger(L, pairs[i * 2 + 1]);
        lua_settable(L, -3);

        lua_settable(L, -3);
    }

    return 1;
}

static int l_huskylens_mapper_set(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }

    luaL_checktype(L, 1, LUA_TTABLE);
    size_t len = lua_rawlen(L, 1);
    if (len > HUSKYLENS_MAP_MAX) {
        lua_pushnil(L);
        lua_pushstring(L, "too many mappings");
        return 2;
    }

    uint8_t pairs[HUSKYLENS_MAP_MAX * 2];
    uint8_t count = 0;

    for (size_t i = 1; i <= len; i++) {
        lua_rawgeti(L, 1, (lua_Integer)i);
        if (!lua_istable(L, -1)) {
            lua_pop(L, 1);
            lua_pushnil(L);
            lua_pushstring(L, "mapping entry must be a table");
            return 2;
        }

        lua_getfield(L, -1, "physical");
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            lua_rawgeti(L, -1, 1);
        }
        if (!lua_isnumber(L, -1)) {
            lua_pop(L, 2);
            lua_pushnil(L);
            lua_pushstring(L, "missing physical id");
            return 2;
        }
        int physical = lua_tointeger(L, -1);
        lua_pop(L, 1);

        lua_getfield(L, -1, "logical");
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            lua_rawgeti(L, -1, 2);
        }
        if (!lua_isnumber(L, -1)) {
            lua_pop(L, 2);
            lua_pushnil(L);
            lua_pushstring(L, "missing logical id");
            return 2;
        }
        int logical = lua_tointeger(L, -1);
        lua_pop(L, 1);

        lua_pop(L, 1);

        if (physical < 0 || physical > 255 || logical < 0 || logical > 255) {
            lua_pushnil(L);
            lua_pushstring(L, "id out of range (0-255)");
            return 2;
        }

        pairs[count * 2] = (uint8_t)physical;
        pairs[count * 2 + 1] = (uint8_t)logical;
        count++;
    }

    bool ok = huskylens_mapper_set_pairs(pairs, count);
    if (!ok) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper set failed");
        return 2;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int l_huskylens_mapper_sequence_get(lua_State *L) {
    uint8_t seq[HUSKYLENS_SEQ_MAX];
    uint8_t len = huskylens_mapper_get_sequence(seq, HUSKYLENS_SEQ_MAX);

    lua_newtable(L);
    for (uint8_t i = 0; i < len; i++) {
        lua_pushinteger(L, i + 1);
        lua_pushinteger(L, seq[i]);
        lua_settable(L, -3);
    }

    return 1;
}

static int l_huskylens_mapper_sequences_get(lua_State *L) {
    uint8_t lens[HUSKYLENS_SEQ_MAX_SEQS];
    uint8_t seqs[HUSKYLENS_SEQ_MAX_SEQS * HUSKYLENS_SEQ_MAX];
    uint8_t count = huskylens_mapper_get_sequences(seqs, lens, HUSKYLENS_SEQ_MAX_SEQS, HUSKYLENS_SEQ_MAX);

    lua_newtable(L);
    for (uint8_t i = 0; i < count; i++) {
        lua_pushinteger(L, i + 1);
        lua_newtable(L);
        for (uint8_t j = 0; j < lens[i]; j++) {
            lua_pushinteger(L, j + 1);
            lua_pushinteger(L, seqs[(i * HUSKYLENS_SEQ_MAX) + j]);
            lua_settable(L, -3);
        }
        lua_settable(L, -3);
    }

    return 1;
}

static int l_huskylens_mapper_on_connect(lua_State *L) {
    if (!initialized) {
        lua_pushnil(L);
        lua_pushstring(L, "not initialized");
        return 2;
    }

    if (lua_isnil(L, 1)) {
        if (s_mapper_conn_cb) {
            luaS_callback_destroy(s_mapper_conn_cb);
            s_mapper_conn_cb = NULL;
        }
        huskylens_mapper_set_conn_cb(NULL);
        lua_pushboolean(L, true);
        return 1;
    }

    luaL_checktype(L, 1, LUA_TFUNCTION);
    if (s_mapper_conn_cb) {
        luaS_callback_destroy(s_mapper_conn_cb);
        s_mapper_conn_cb = NULL;
    }

    s_mapper_conn_cb = luaS_callback_create(L, 1);
    if (!s_mapper_conn_cb) {
        lua_pushnil(L);
        lua_pushstring(L, "callback create failed");
        return 2;
    }

    huskylens_mapper_set_conn_cb(mapper_conn_cb);
    lua_pushboolean(L, true);
    return 1;
}

static const luaL_Reg huskylens[] = {
    // Basic functions
    {"init", l_huskylens_init},
    {"test", l_huskylens_test},
    {"available", l_huskylens_available},
    {"frame_number", l_huskylens_frame_number},
    
    // Request functions
    {"request", l_huskylens_request},
    {"request_by_id", l_huskylens_request_by_id},
    {"request_blocks", l_huskylens_request_blocks},
    {"request_blocks_by_id", l_huskylens_request_blocks_by_id},
    {"request_arrows", l_huskylens_request_arrows},
    {"request_arrows_by_id", l_huskylens_request_arrows_by_id},
    {"request_learned", l_huskylens_request_learned},
    {"request_blocks_learned", l_huskylens_request_blocks_learned},
    {"request_arrows_learned", l_huskylens_request_arrows_learned},
    
    // Count functions
    {"count", l_huskylens_count},
    {"count_by_id", l_huskylens_count_by_id},
    {"count_blocks", l_huskylens_count_blocks},
    {"count_blocks_by_id", l_huskylens_count_blocks_by_id},
    {"count_arrows", l_huskylens_count_arrows},
    {"count_arrows_by_id", l_huskylens_count_arrows_by_id},
    {"count_learned", l_huskylens_count_learned},
    {"count_blocks_learned", l_huskylens_count_blocks_learned},
    {"count_arrows_learned", l_huskylens_count_arrows_learned},
    {"count_learned_ids", l_huskylens_count_learned_ids},
    
    // Get functions
    {"get_results", l_huskylens_get_results},
    {"get", l_huskylens_get},
    {"get_by_id", l_huskylens_get_by_id},
    {"get_block", l_huskylens_get_block},
    {"get_block_by_id", l_huskylens_get_block_by_id},
    {"get_arrow", l_huskylens_get_arrow},
    {"get_arrow_by_id", l_huskylens_get_arrow_by_id},
    {"get_learned", l_huskylens_get_learned},
    {"get_block_learned", l_huskylens_get_block_learned},
    {"get_arrow_learned", l_huskylens_get_arrow_learned},
    
    // Configuration functions
    {"set_algorithm", l_huskylens_set_algorithm},
    {"learn", l_huskylens_learn},
    {"forget", l_huskylens_forget},
    {"is_learned", l_huskylens_is_learned},
    
    // Advanced functions
    {"write_sensor", l_huskylens_write_sensor},
    {"set_custom_name", l_huskylens_set_custom_name},
    {"save_picture_to_sd", l_huskylens_save_picture_to_sd},
    {"save_model_to_sd", l_huskylens_save_model_to_sd},
    {"load_model_from_sd", l_huskylens_load_model_from_sd},
    {"clear_custom_text", l_huskylens_clear_custom_text},
    {"custom_text", l_huskylens_custom_text},
    {"save_screenshot_to_sd", l_huskylens_save_screenshot_to_sd},
    {"is_pro", l_huskylens_is_pro},
    {"check_firmware_version", l_huskylens_check_firmware_version},
    {"write_firmware_version", l_huskylens_write_firmware_version},

    {"mapper_start", l_huskylens_mapper_start},
    {"mapper_stop", l_huskylens_mapper_stop},
    {"mapper_status", l_huskylens_mapper_status},
    {"mapper_get", l_huskylens_mapper_get},
    {"mapper_set", l_huskylens_mapper_set},
    {"mapper_on_connect", l_huskylens_mapper_on_connect},
    {"mapper_sequence_get", l_huskylens_mapper_sequence_get},
    {"mapper_sequences_get", l_huskylens_mapper_sequences_get},
    
    {NULL, NULL}
};

LUALIB_API int luaopen_huskylens( lua_State *L ) {
    luaL_newlib(L, huskylens);
    
    // Add algorithm constants
    lua_pushstring(L, "ALGORITHM_FACE_RECOGNITION");
    lua_pushinteger(L, ALGORITHM_FACE_RECOGNITION);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_OBJECT_TRACKING");
    lua_pushinteger(L, ALGORITHM_OBJECT_TRACKING);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_OBJECT_RECOGNITION");
    lua_pushinteger(L, ALGORITHM_OBJECT_RECOGNITION);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_LINE_TRACKING");
    lua_pushinteger(L, ALGORITHM_LINE_TRACKING);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_COLOR_RECOGNITION");
    lua_pushinteger(L, ALGORITHM_COLOR_RECOGNITION);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_TAG_RECOGNITION");
    lua_pushinteger(L, ALGORITHM_TAG_RECOGNITION);
    lua_settable(L, -3);
    
    lua_pushstring(L, "ALGORITHM_OBJECT_CLASSIFICATION");
    lua_pushinteger(L, ALGORITHM_OBJECT_CLASSIFICATION);
    lua_settable(L, -3);
    
    return 1;
}

MODULE_REGISTER_RAM(HUSKYLENS, huskylens, luaopen_huskylens, 1);

#ifdef __cplusplus
}
#endif

#endif 
