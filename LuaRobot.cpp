/*
 *  LuaRobot.cpp - C++ code to start Lua
 *  Greyhound Lua
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

////////////////// LIBRARIES ////////////////// 

extern "C" int luaopen_wpilib(lua_State *L);
extern "C" int luaopen_bit(lua_State *L);

struct libentry
{
    char *name;
    lua_CFunction func;
    bool registerPackage;
};

struct libentry libList[] =
{
    {"bit", luaopen_bit, false},
    {"wpilib", luaopen_wpilib, true},
    {NULL, NULL, false}
};

////////////////// LIBRARIES ////////////////// 

class LuaRobot : public RobotBase
{
public:
    LuaRobot()
    {
        int i;
        
        L = luaL_newstate();
        
        // Load builtin libraries
        luaL_openlibs(L);
        
        // Get package.loaded table
        lua_getglobal(L, "package");
        lua_getfield(L, -1, "loaded");
        lua_remove(L, -2);
        
        // Load user libraries
        for (i = 0; libList[i].name != NULL; i++)
        {
            // Initialize the library
            lua_pushcfunction(L, libList[i].func);
            lua_pushstring(L, libList[i].name);
            lua_pcall(L, 1, 0, 0);
            // If necessary, put the module onto the package.loaded list.
            if (libList[i].registerPackage)
            {
                // Get module table
                lua_getglobal(L, libList[i].name);
                // Add module table to package.loaded.
                // For efficiency, we only retrieve package.loaded once, so it
                // should be the penultimate item on the stack right now.
                if (!lua_isnil(L, -1))
                    lua_setfield(L, -2, libList[i].name);
                else
                    lua_pop(L, 1);
            }
        }
        
        // Clean up state
        lua_settop(L, 0);
    }
    
    virtual ~LuaRobot()
    {
        closeLua();
    }
    
protected:
    lua_State *L;
    
    void closeLua()
    {
        if (L != NULL)
        {
            lua_close(L);
            L = NULL;
        }
    }
    
    void writeMessageFile(const char *fname, const char *fmt, va_list args)
    {
    	FILE *errfile;
    	
    	errfile = fopen(fname, "w");
    	vfprintf(errfile, fmt, args);
    	fputs("\n", errfile);
    	fclose(errfile);
    }
    
    void writeMessageFile(const char *fname, const char *fmt, ...)
    {
    	va_list args;
    	
    	va_start(args, fmt);
    	writeMessageFile(fname, fmt, args);
    	va_end(args);
    }
    
    void writeError(const char *errfmt, ...)
    {
    	va_list args;
    	
    	va_start(args, errfmt);
    	writeMessageFile("boot-error.txt", errfmt, args);
    	va_end(args);
    }
    
	virtual void StartCompetition()
    {
        int retcode;
        const char *errstr;
        
        // Ensure that the stack is clear.
        lua_settop(L, 0);
        
        // Retrieve the debug.traceback function. We'll use this for the Lua
        // error function.
        lua_getglobal(L, "debug");
        lua_getfield(L, 1, "traceback");
        lua_remove(L, 1);
        
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
                    writeError("Syntax error in Lua bootloader");
                    break;
                case LUA_ERRMEM:
                	writeError("Yikes! Lua ran out of memory!");
                    break;
                case LUA_ERRFILE:
                	writeError("I/O error while reading Lua bootloader");
                    break;
            }
            // Enter an infinite loop.  We won't be doing anything useful anyway.
            while (1)
                Wait(0.1);
        }
        
        // Now we actually run the bootloader.  If all goes well, this function
        // will never return.
        retcode = lua_pcall(L, 0, 0, 1);
        
        // Remove the traceback function from the stack.
        lua_remove(L, 1);
        
        // If the code makes it here, then the bootloader failed its job.  We
        // will print an error to stderr, then infinite loop so that the
        // operator must restart.
        switch (retcode)
        {
            case LUA_ERRRUN:
                errstr = lua_tostring(L, 1);
            	writeError("Lua runtime error: %s", errstr != NULL ? errstr : "<NO MESSAGE>");
                break;
            case LUA_ERRERR:
                writeError("Lua bootloader crashed and the debug.traceback function caused an error");
                break;
            case LUA_ERRMEM:
            	writeError("Lua ran out of memory while running!");
                break;
        }
        
        // Clean up and enter an infinite loop.  We won't be doing anything useful anyway.
        closeLua();
        while (1)
            Wait(0.1);
    }
};

START_ROBOT_CLASS(LuaRobot);
