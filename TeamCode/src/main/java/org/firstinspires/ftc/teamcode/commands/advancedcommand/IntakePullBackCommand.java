package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionSyncCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakePullBackCommand extends SequentialCommandGroup {
    public IntakePullBackCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::stopIntaking),
                new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                new ExtensionPositionSyncCommand(Constants.extMin),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
        );
    }
}
