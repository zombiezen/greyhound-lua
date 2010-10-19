-- samples/basic/robot.lua - Arcade drive for 4-motor drive robot
-- This file is in the public domain.

module(..., package.seeall)

stick1 = wpilib.Joystick(1)
stick2 = wpilib.Joystick(2)

-- Initialize motors.  Feel free to change channel numbers to match your needs.
leftMotor1 = wpilib.Victor(1)
leftMotor2 = wpilib.Victor(3)
rightMotor1 = wpilib.Victor(2)
rightMotor2 = wpilib.Victor(4)

-- Use WPILib's robot drive for simplicity.
drive = wpilib.RobotDrive(leftMotor1, leftMotor2, rightMotor1, rightMotor2)

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
    local dog = wpilib.GetWatchdog()
    dog:SetEnabled(true)
    dog:SetExpiration(0.25)
    
    while wpilib.IsOperatorControl() and wpilib.IsEnabled() do
        dog:Feed()
        drive:ArcadeDrive(-stick1:GetY(), -stick2:GetX())
        wpilib.Wait(0.01)
    end
end

function autonomous()
    wpilib.GetWatchdog():SetEnabled(false)
    -- Drive forward at half-speed for two seconds, then stop.
    drive:Drive(0.5, 0.0)
    wpilib.Wait(2.0)
    drive:Drive(0.0, 0.0)
end

function disabled()
    -- Do something in disabled mode here...
end
