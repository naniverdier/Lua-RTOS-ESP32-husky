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

static bool initialized = false;
static huskylens_t husky_instance;

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

static const luaL_Reg huskylens[] = {
    {"init", l_huskylens_init},
    {"test", l_huskylens_test},
    {"request", l_huskylens_request},
    {"get_results", l_huskylens_get_results},
    {"set_algorithm", l_huskylens_set_algorithm},
    {"learn", l_huskylens_learn},
    {"forget", l_huskylens_forget},
    {"is_learned", l_huskylens_is_learned},
    {"available", l_huskylens_available},
    {NULL, NULL}
};

LUALIB_API int luaopen_huskylens( lua_State *L ) {
    luaL_newlib(L, huskylens);
    return 1;
}

MODULE_REGISTER_RAM(HUSKYLENS, huskylens, luaopen_huskylens, 1);

#ifdef __cplusplus
}
#endif

#endif 