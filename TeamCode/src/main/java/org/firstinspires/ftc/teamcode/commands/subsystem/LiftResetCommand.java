package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftResetCommand extends InstantCommand {
    public LiftResetCommand() {
        super(
                () -> Robot.getInstance().depositSubsystem.resetLift()
        );
    }
}
