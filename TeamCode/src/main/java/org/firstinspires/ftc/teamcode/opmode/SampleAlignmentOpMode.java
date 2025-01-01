package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
@TeleOp
public class SampleAlignmentOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID, movePID;

    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        robot.setPose(new Pose(0, 0, 0));
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        robot.startTeleopDrive();
        robot.startCamera();
    }

    @Override
    public void loop() {
        double angle = robot.sampleAlignmentProcessor.getAngleToRotate();

        robot.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);
        anglePID.setCoefficients(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));


        robot.setTeleOpMovementVectors(0, 0, -anglePID.runPIDF(), true);

        telemetry.addData("angle", angle);
        telemetry.addData("pid error", anglePID.getError());
        telemetry.addData("pid val", anglePID.runPIDF());

        telemetry.update();
    }
}
