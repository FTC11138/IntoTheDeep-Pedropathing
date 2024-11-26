package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class ArmPositionCommand extends InstantCommand {
    public ArmPositionCommand(double pos) {
        super(
                () -> Robot.getInstance().intakeSubsystem.setArmPosition(pos)
        );
    }
}
