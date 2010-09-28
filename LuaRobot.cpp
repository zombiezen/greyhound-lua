/*
 *  LuaRobot.cpp
 *  FIRSTLua
 *
 *  Created by Ross Light on 2010-09-27.
 */

#include <stdio.h>
#include <WPILib/WPILib.h>
#include "lua/lua.h"

#define FIRST_LUA_BOOT_FILE "lua/boot.lua"

extern int luaopen_wpilib(lua_State *L);

class LuaRobot : public RobotBase
{
public:
    LuaRobot()
    {
        L = luaL_newstate();
        
        // Load libraries
        luaL_openlibs(L);
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
                    break
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
                fprintf(stderr, "Lua ran out of memory while running!\n")
                break;
        }
        
        // Enter an infinite loop.  We won't be doing anything useful anyway.
        while (1);
    }
}

START_ROBOT_CLASS(LuaRobot);
