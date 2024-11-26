package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathChainCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "0+4", preselectTeleOp = "Solo")
public class Auto_0Plus4 extends LinearOpMode {

    public static double intakeX = 10;
    public static double intakeY = 41;
    public static double intakeDegrees = -90;

    public static double parkX = 16;
    public static double parkY = 16;
    public static double parkDegrees = 180;

    public static Path preload;
    public static Path sampleDragPath1, sampleDragPath2, sampleDragPath3;
    public static Path obsvToIntakePath;
    public static Path intakeToChamberPath, chamberToIntakePath;
    public static Path parkPath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.redObsv;
        Pose scorePose = PoseConstants.Score.redChamber;
        Pose intakePose = new Pose(intakeX, intakeY, Math.toRadians(intakeDegrees));
        Pose parkPose = new Pose(parkX, parkY, Math.toRadians(parkDegrees));

        preload = new Path(new BezierLine(
                new Point(startPose),
                new Point(scorePose)
        ));
        preload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        sampleDragPath1 = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(14.900, 16.650, Point.CARTESIAN),
                new Point(73.450, 50.300, Point.CARTESIAN),
                new Point(68.000, 24.000, Point.CARTESIAN)
        ));
        sampleDragPath1.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(-89));

        sampleDragPath2 = new Path(new BezierLine(
                new Point(68.000, 24.000, Point.CARTESIAN),
                new Point(20, 24.500, Point.CARTESIAN)
        ));
        sampleDragPath2.setConstantHeadingInterpolation(Math.toRadians(90));

        sampleDragPath3 = new Path(new BezierCurve(
                new Point(20, 24.500, Point.CARTESIAN),
                new Point(110.900, 24.5, Point.CARTESIAN),
                new Point(20, 16.000, Point.CARTESIAN)
        ));
        sampleDragPath3.setConstantHeadingInterpolation(Math.toRadians(90));






        obsvToIntakePath = new Path(new BezierCurve(
                new Point(20, 16.000, Point.CARTESIAN),
                new Point(19.900, 45.650, Point.CARTESIAN),
                new Point(22.6, 40.2, Point.CARTESIAN),
                new Point(intakePose)
        ));
        obsvToIntakePath.setLinearHeadingInterpolation(Math.toRadians(-90), intakePose.getHeading());

        intakeToChamberPath = new Path(new BezierCurve(
                new Point(intakePose),
                new Point(32.3, 40.3, Point.CARTESIAN),
                new Point(13.4, 69.2, Point.CARTESIAN),
                new Point(scorePose)
        ));
        intakeToChamberPath.setLinearHeadingInterpolation(intakePose.getHeading(), scorePose.getHeading());

        chamberToIntakePath = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(32.3, 40.3, Point.CARTESIAN),
                new Point(17.1, 59.7, Point.CARTESIAN),
                new Point(intakePose)
        ));
        chamberToIntakePath.setLinearHeadingInterpolation(scorePose.getHeading(), intakePose.getHeading());

        parkPath = new Path(new BezierLine(
                new Point(scorePose),
                new Point(parkPose)
        ));
        parkPath.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        switch (Globals.ALLIANCE) {
            case RED:
                robot.intakeSubsystem.leds.setPattern(Constants.redPattern);
                break;
            case BLUE:
                robot.intakeSubsystem.leds.setPattern(Constants.bluePattern);
                break;
        }

        buildPaths();

        while (!isStarted()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.setPose(PoseConstants.Start.redObsv);
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(preload, 0.7),

                        new WaitCommand(1000),
                        new SpecimenDepositCommand(),
                        new WaitCommand(500),

                        new PathChainCommand(0.8, sampleDragPath1, sampleDragPath2, sampleDragPath3)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(2000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),

                        new PathCommand(obsvToIntakePath, 0.5),
                        new WaitCommand(1000),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(1000),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToChamberPath),
                        new WaitCommand(1000),
                        new SpecimenDepositCommand(),
                        new WaitCommand(500),

                        new PathCommand(chamberToIntakePath)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new WaitCommand(1000),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(1000),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToChamberPath),
                        new WaitCommand(1000),
                        new SpecimenDepositCommand(),
                        new WaitCommand(500),

                        new PathCommand(chamberToIntakePath)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new WaitCommand(1000),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(1000),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToChamberPath),
                        new WaitCommand(1000),
                        new SpecimenDepositCommand(),
                        new WaitCommand(500),

                        new PathCommand(parkPath, 0.5)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(2000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.DOWN)
                                ))
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

    }

}
