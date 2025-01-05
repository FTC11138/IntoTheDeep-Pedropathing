package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;


public class SampleExtendPickupCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();

    private double dist;

    @Override
    public void initialize() {
        robot.intakeSubsystem.setExtensionPower(1);
        robot.intakeSubsystem.updateIntakeState(IntakeSubsystem.IntakeState.IN);
    }

    @Override
    public void execute() {
        dist = robot.intakeSubsystem.getDistance();
    }

    @Override
    public boolean isFinished() {
        return dist < Constants.samplePickupTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        robot.intakeSubsystem.setTargetExtensionPosition(Constants.extMin);
        robot.intakeSubsystem.updateArmState(IntakeSubsystem.ArmState.UP);
        robot.intakeSubsystem.updateIntakeState(IntakeSubsystem.IntakeState.STOP);
    }

}
