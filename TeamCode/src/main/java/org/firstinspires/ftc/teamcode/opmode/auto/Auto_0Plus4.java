package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
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

    public static double intakeX = 9;
    public static double intakeY = 45;
    public static double intakeDegrees = -90;

    public static double score1x = 38;
    public static double score1y = 70;
    public static double score1angle = 90;

    public static double score2x = 39.5;
    public static double score2y = 67;
    public static double score2angle = 90;

    public static double score3x = 39.5;
    public static double score3y = 64;
    public static double score3angle = 90;

    public static double score4x = 39.5;
    public static double score4y = 61;
    public static double score4angle = 90;

    public static double parkX = 13;
    public static double parkY = 16;
    public static double parkDegrees = 180;

    public static Path preload;
    public static Path sampleDragPath1, sampleDragPath2, sampleDragPath3, sampleDragPath4;
    public static Path obsvToIntakePath;
    public static Path intakeToScore2Path, score2ToIntakePath;
    public static Path intakeToScore3Path, score3ToIntakePath;
    public static Path intakeToScore4Path, score4ToIntakePath;
    public static Path parkPath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.redObsv;
        Pose scorePose = PoseConstants.Score.redChamber;
        Pose intakePose = new Pose(intakeX, intakeY, Math.toRadians(intakeDegrees));
        Pose parkPose = new Pose(parkX, parkY, Math.toRadians(parkDegrees));

        Pose score1Pose = new Pose(score1x, score1y, Math.toRadians(score1angle));
        Pose score2Pose = new Pose(score2x, score2y, Math.toRadians(score2angle));
        Pose score3Pose = new Pose(score3x, score3y, Math.toRadians(score3angle));
        Pose score4Pose = new Pose(score4x, score4y, Math.toRadians(score4angle));

        preload = new Path(new BezierLine(
                new Point(startPose),
                new Point(score1Pose)
        ));
        preload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());

        sampleDragPath1 = new Path(new BezierCurve(
                preload.getLastControlPoint(),
                new Point(14.900, 16.650, Point.CARTESIAN),
                new Point(73.450, 50.300, Point.CARTESIAN),
                new Point(68.000, 24.000, Point.CARTESIAN)
        ));
        sampleDragPath1.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(-90));

        sampleDragPath2 = new Path(new BezierLine(
                sampleDragPath1.getLastControlPoint(),
                new Point(20, 24, Point.CARTESIAN)
        ));
        sampleDragPath2.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

        sampleDragPath3 = new Path(new BezierCurve(
                sampleDragPath2.getLastControlPoint(),
                new Point(69.3, 29.5, Point.CARTESIAN),
                new Point(61.4, 14, Point.CARTESIAN)
        ));
        sampleDragPath3.setConstantHeadingInterpolation(Math.toRadians(-90));

        sampleDragPath4 = new Path(new BezierCurve(
                sampleDragPath3.getLastControlPoint(),
                new Point(20.00, 14, Point.CARTESIAN)
        ));
        sampleDragPath4.setConstantHeadingInterpolation(Math.toRadians(-90));

        obsvToIntakePath = new Path(new BezierCurve(
                sampleDragPath4.getLastControlPoint(),
                new Point(19.900, 45.650, Point.CARTESIAN),
                new Point(22.6, 40.2, Point.CARTESIAN),
                new Point(intakePose)
        ));
        obsvToIntakePath.setLinearHeadingInterpolation(Math.toRadians(-90), intakePose.getHeading());



        // Separate paths for intake to score2, 3, 4, and back to intake
        intakeToScore2Path = new Path(new BezierCurve(
                new Point(intakePose),
                new Point(17.6, 66.5, Point.CARTESIAN),
                new Point(score2Pose)
        ));
        intakeToScore2Path.setLinearHeadingInterpolation(intakePose.getHeading(), score2Pose.getHeading());

        score2ToIntakePath = new Path(new BezierCurve(
                new Point(score2Pose),
                new Point(17.6, 66.5, Point.CARTESIAN),
                new Point(intakePose)
        ));
        score2ToIntakePath.setLinearHeadingInterpolation(score2Pose.getHeading(), intakePose.getHeading(), 0.5);

        intakeToScore3Path = new Path(new BezierCurve(
                new Point(intakePose),
                new Point(17.6, 66.5, Point.CARTESIAN),
                new Point(score3Pose)
        ));
        intakeToScore3Path.setLinearHeadingInterpolation(intakePose.getHeading(), score3Pose.getHeading());

        score3ToIntakePath = new Path(new BezierCurve(
                new Point(score3Pose),
                new Point(17.6, 66.5, Point.CARTESIAN),
                new Point(8, 45)
        ));
        score3ToIntakePath.setLinearHeadingInterpolation(score3Pose.getHeading(), intakePose.getHeading(), 0.5);

        intakeToScore4Path = new Path(new BezierCurve(
                new Point(intakePose),
                new Point(17.6, 66.5, Point.CARTESIAN),
                new Point(score4Pose)
        ));
        intakeToScore4Path.setLinearHeadingInterpolation(intakePose.getHeading(), score4Pose.getHeading());

        parkPath = new Path(new BezierLine(
                new Point(score4Pose),
                new Point(parkPose)
        ));
        parkPath.setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading());
    }


    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        ElapsedTime timer = new ElapsedTime();

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
                        new PathCommand(preload),

                        new SpecimenDepositCommand(),

                        new PathChainCommand(1, sampleDragPath1, sampleDragPath2, sampleDragPath3, sampleDragPath4)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),

                        new PathCommand(obsvToIntakePath, 1),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore2Path),
                        new SpecimenDepositCommand(),

                        new PathCommand(score2ToIntakePath, 1)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore3Path),
                        new SpecimenDepositCommand(),

                        new PathCommand(score3ToIntakePath, 1)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore4Path),
                        new SpecimenDepositCommand(),

                        new ConditionalCommand(
                                new PathCommand(parkPath, 1)
                                        .alongWith(new SequentialCommandGroup(
                                                new WaitCommand(2000),
                                                new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.DOWN)
                                        )),
                                new SequentialCommandGroup(
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.DOWN),
                                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.OPEN),
                                        new IntakePullBackCommand(),
                                        new LiftDownCommand()
                                ),
                                () -> timer.seconds() <= 29
                        )
                )
        );

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

    }

}
