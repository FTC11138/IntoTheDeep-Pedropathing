package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Constants;


public class SampleAlignCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID;

    private double angle;

    @Override
    public void initialize() {
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        robot.startTeleopDrive();
        robot.startCamera();
    }

    @Override
    public void execute() {
        angle = robot.sampleAlignmentProcessor.getAngleToRotate();

        robot.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);

        robot.setTeleOpMovementVectors(0, 0, -anglePID.runPIDF(), true);
    }

    @Override
    public boolean isFinished() {
        return angle < Constants.sampleAlignAngleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        robot.setTeleOpMovementVectors(0, 0, 0, true);
        robot.stopCamera();
    }

}
