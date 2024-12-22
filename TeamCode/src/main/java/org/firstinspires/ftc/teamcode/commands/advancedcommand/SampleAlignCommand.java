package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Constants;


public class SampleAlignCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID, movePID;

    private double angle, move;

    public SampleAlignCommand() {

    }

    @Override
    public void initialize() {
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        movePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPMove, Constants.kIMove, Constants.kDMove, Constants.kFMove));
        robot.startTeleopDrive();
        robot.startCamera();
    }

    @Override
    public void execute() {
        angle = robot.sampleAlignmentProcessor.getAngleToRotate();
        move = robot.sampleAlignmentProcessor.getDistanceToMove();

        robot.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);

        movePID.updatePosition(0);
        movePID.setTargetPosition(move);

        robot.setTeleOpMovementVectors(-movePID.runPIDF(), 0, -anglePID.runPIDF(), true);
    }

    @Override
    public boolean isFinished() {
        return angle < Constants.sampleAlignAngleTolerance && move < Constants.sampleAlignDistTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        robot.setTeleOpMovementVectors(0, 0, 0, true);
        robot.stopCamera();
    }

}
