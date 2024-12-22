package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RobotSpeedCommand extends InstantCommand {
    public RobotSpeedCommand(double speed) {
        super(() -> Robot.getInstance().setMaxPower(speed));
    }
 }
