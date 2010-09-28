--[[
boot-sample.lua
Created by Ross Light on 2010-09-28.

This is a sample boot.lua file.  Feel free to use it in your own programs, or
write your own, if you wish.

This file is public domain.
--]]

function restartRobot()
    for name, _ in items(package.loaded) do
        package.loaded[name] = nil
        _G[name] = nil
    end
    startRobot()
end

local function startRobot()
    require "robot"
    robot.run()
end

startRobot()
