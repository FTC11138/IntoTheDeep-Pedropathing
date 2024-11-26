package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class SampleTransferCommand extends ConditionalCommand {

    public SampleTransferCommand() {
        super(
                new SequentialCommandGroup(
                        new ArmStateCommand(IntakeSubsystem.ArmState.TRANSFER),
                        new WaitCommand(200),
                        new InstantCommand(Robot.getInstance().data::setSampleLoaded),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                        new WaitCommand(500),
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
                ),
                new InstantCommand(),
                () -> (
                        !Robot.getInstance().data.intaking && !Robot.getInstance().data.scoring
                )
        );
    }
}
