package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;

public class SpecimenLiftStateCommand extends InstantCommand {
    public SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState state) {
        super(
                () -> {
                    Robot.getInstance().specimenSubsystem.updateSpecimenLiftState(state);
                    if (state == SpecimenSubsystem.SpecimenLiftState.GRAB) {
                        Robot.getInstance().specimenSubsystem.updateSpecimenClawState(SpecimenSubsystem.SpecimenClawState.OPEN);
                    }
                }
        );
    }
}
