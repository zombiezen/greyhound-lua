/*
 *  LuaRobot.cpp - C++ code to start Lua
 *  FIRSTLua
 *
 *  Copyright (c) 2010 Ross Light
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 *  and/or sell copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include "WPILib.h"

extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

#define FIRST_LUA_BOOT_FILE "lua/boot.lua"

extern "C" int luaopen_wpilib(lua_State *L);
extern "C" int luaopen_bit(lua_State *L);

class LuaRobot : public RobotBase
{
public:
    LuaRobot()
    {
        L = luaL_newstate();
        
        // Load libraries
        luaL_openlibs(L);
        lua_pushcfunction(L, luaopen_bit);
        lua_pcall(L, 0, 0, 0);
        lua_pushcfunction(L, luaopen_wpilib);
        lua_pcall(L, 0, 0, 0);
    }
    
    virtual ~LuaRobot()
    {
        lua_close(L);
    }
    
protected:
    lua_State *L;
    
    
	virtual void StartCompetition()
    {
        int retcode;
        const char *errstr;
        
        // Load Lua bootloader script
        // The "bootloader" is the first script we run because it allows us to
        // manage code reloads in a controlled environment.
        // The bootloader should rarely need to be changed (it would require
        // restarting the robot), and should be very error tolerant.
        if ((retcode = luaL_loadfile(L, FIRST_LUA_BOOT_FILE)) != 0)
        {
            switch (retcode)
            {
                case LUA_ERRSYNTAX:
                    fprintf(stderr, "Syntax error in Lua bootloader\n");
                    break;
                case LUA_ERRMEM:
                    fprintf(stderr, "Yikes! Lua ran out of memory!\n");
                    break;
                case LUA_ERRFILE:
                    fprintf(stderr, "I/O error while reading Lua bootloader\n");
                    break;
            }
            // Enter an infinite loop.  We won't be doing anything useful anyway.
            while (1);
        }
        
        // Now we actually run the bootloader.  If all goes well, this function
        // will never return.
        retcode = lua_pcall(L, 0, 0, 0);
        
        // If the code makes it here, then the bootloader failed its job.  We
        // will print an error to stderr, then infinite loop so that the
        // operator must restart.
        switch (retcode)
        {
            case LUA_ERRRUN:
                errstr = lua_tostring(L, 0);
                if (errstr != NULL)
                    fprintf(stderr, "Lua runtime error: %s\n", errstr);
                else
                    fprintf(stderr, "Lua runtime error: <NO MESSAGE>\n");
                break;
            case LUA_ERRMEM:
                fprintf(stderr, "Lua ran out of memory while running!\n");
                break;
        }
        
        // Enter an infinite loop.  We won't be doing anything useful anyway.
        while (1);
    }
};

START_ROBOT_CLASS(LuaRobot);
