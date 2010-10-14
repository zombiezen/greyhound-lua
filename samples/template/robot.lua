-- samples/template/robot.lua - Boilerplate code to get robot running
-- This file is in the public domain.

module(..., package.seeall)

function run()
    -- Main loop
    while true do
        if wpilib.isDisabled() then
            disabled()
            repeat wpilib.Wait(0.01) until not wpilib.isDisabled()
        elseif wpilib.isAutonomous() then
            autonomous()
            repeat wpilib.Wait(0.01) until not wpilib.isAutonomous() or not wpilib.isEnabled()
        else
            teleop()
            repeat wpilib.Wait(0.01) until not wpilib.isOperatorControl() or not wpilib.isEnabled()
        end
    end
end

function teleop()
    while wpilib.isOperatorControl() and wpilib.isEnabled() do
        -- Run one cycle of teleoperated here...
        wpilib.Wait(0.01)
    end
end

function autonomous()
    -- Run autonomous code here...
end

function disabled()
    -- Do something in disabled mode here...
end
