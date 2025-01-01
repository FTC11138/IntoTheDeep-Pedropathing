package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ExtensionPositionSyncCommand extends CommandBase {

    private final int target;

    public ExtensionPositionSyncCommand(int target) {
        this.target = target;
    }

    @Override
    public void initialize() {
        Robot.getInstance().specimenSubsystem.setTargetSpecimenLiftPosition(target);
    }

    @Override
    public boolean isFinished() {
        return Robot.getInstance().intakeSubsystem.extensionBusy();
    }
}
