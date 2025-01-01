package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.DriveConstants;

public class PathTimeoutChangeCommand extends InstantCommand {
    public PathTimeoutChangeCommand(int timeout) {
        super(() -> DriveConstants.pathEndTimeoutConstraint = timeout);
    }
 }
