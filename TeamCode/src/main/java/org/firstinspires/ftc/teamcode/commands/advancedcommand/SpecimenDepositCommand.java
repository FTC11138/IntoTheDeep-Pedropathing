package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenClawState.*;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenLiftState.*;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class SpecimenDepositCommand extends ConditionalCommand {
    public SpecimenDepositCommand() {
        super(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SpecimenLiftStateCommand(DEPOSITED_HIGH),
                                new SpecimenLiftStateCommand(DEPOSITED_LOW),
                                () -> Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == HIGH
                        ),
                        new WaitCommand(500),
                        new SpecimenClawStateCommand(OPEN)
                ),
                new SpecimenLiftStateCommand(GRAB),
                () -> Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == HIGH ||
                        Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == LOW
        );
    }
}
