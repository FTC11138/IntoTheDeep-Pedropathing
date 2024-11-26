package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class SampleEjectCommand extends SequentialCommandGroup {
    public SampleEjectCommand() {
        super(
                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                new WaitCommand(500),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
        );
    }
}
