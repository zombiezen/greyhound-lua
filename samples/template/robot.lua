-- samples/template/robot.lua - Boilerplate code to get robot running
-- This file is in the public domain.

module(..., package.seeall)

function run()
    -- Main loop
    while true do
        if wpilib.IsDisabled() then
            disabled()
            repeat wpilib.Wait(0.01) until not wpilib.IsDisabled()
        elseif wpilib.IsAutonomous() then
            autonomous()
            repeat wpilib.Wait(0.01) until not wpilib.IsAutonomous() or not wpilib.IsEnabled()
        else
            teleop()
            repeat wpilib.Wait(0.01) until not wpilib.IsOperatorControl() or not wpilib.IsEnabled()
        end
    end
end

function teleop()
    while wpilib.IsOperatorControl() and wpilib.IsEnabled() do
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
