package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftDownCommand extends SequentialCommandGroup {
    public LiftDownCommand() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new LiftPositionCommand(Constants.liftSlow, 1),
                                new WaitCommand(700)
                        ),
                        new InstantCommand(),
                        () -> Robot.getInstance().depositSubsystem.getLiftPosition() > Constants.liftSlow
                ),
                new LiftPositionCommand(Constants.liftMin1, Constants.liftSlowRatio),
                new InstantCommand(Robot.getInstance().data::stopScoring)
        );
    }
}
