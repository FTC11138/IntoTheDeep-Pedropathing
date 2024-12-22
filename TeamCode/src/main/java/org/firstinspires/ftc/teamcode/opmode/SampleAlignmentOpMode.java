package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config
@TeleOp
public class SampleAlignmentOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID, movePID;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double kPMove = 0;
    public static double kIMove = 0;
    public static double kDMove = 0;
    public static double kFMove = 0;

    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        robot.setPose(new Pose(0, 0, 0));
        anglePID = new PIDFController(new CustomPIDFCoefficients(kP, kI, kD, kF));
        movePID = new PIDFController(new CustomPIDFCoefficients(kPMove, kIMove, kDMove, kFMove));
        robot.startTeleopDrive();
        robot.startCamera();
    }

    @Override
    public void loop() {
        double angle = robot.sampleAlignmentProcessor.getAngleToRotate();
        double move = robot.sampleAlignmentProcessor.getDistanceToMove();

        robot.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);
        anglePID.setCoefficients(new CustomPIDFCoefficients(kP, kI, kD, kF));

        movePID.updatePosition(0);
        movePID.setTargetPosition(move);
        movePID.setCoefficients(new CustomPIDFCoefficients(kPMove, kIMove, kDMove, kFMove));

        robot.setTeleOpMovementVectors(-movePID.runPIDF(), 0, -anglePID.runPIDF(), true);

        telemetry.addData("angle", angle);
        telemetry.addData("pid error", anglePID.getError());
        telemetry.addData("pid val", anglePID.runPIDF());

        telemetry.addData("move", move);
        telemetry.addData("pid error", movePID.getError());
        telemetry.addData("pid val", movePID.runPIDF());
        telemetry.update();
    }
}
