package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenGrabCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathChainCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.RobotSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "0+4 SlowGrab", preselectTeleOp = "Solo")
public class Auto_0Plus4_SlowGrab extends LinearOpMode {

    public static double grab2X = 9;
    public static double grab2Y = 40;
    public static double grab2Degrees = -90;

    public static double grab3X = 9.1;
    public static double grab3Y = 40;
    public static double grab3Degrees = -90;

    public static double grab4X = 9.1;
    public static double grab4Y = 40;
    public static double grab4Degrees = -90;

    public static double preGrab2X = 16;
    public static double preGrab2Y = 40;
    public static double preGrab2Degrees = -90;

    public static double preGrab3X = 16;
    public static double preGrab3Y = 40;
    public static double preGrab3Degrees = -90;

    public static double preGrab4X = 16;
    public static double preGrab4Y = 42;
    public static double preGrab4Degrees = -90;


    public static double intake1X = 34;
    public static double intake1Y = 42;
    public static double intake1Degrees = -60;
    public static int intake1Ext = 400;

    public static double intake2X = 40.5;
    public static double intake2Y = 30;
    public static double intake2Degrees = -90;
    public static int intake2Ext = 0;

    public static double eject1X = 32;
    public static double eject1Y = 28.5;
    public static double eject1Degrees = -150;
    public static int eject1Ext = 800;

    public static double eject2X = 9.3;
    public static double eject2Y = 41;
    public static double eject2Degrees = -90;
    public static int eject2Ext = 700;


    public static double score1x = 37;
    public static double score1y = 73;
    public static double score1angle = 90;

    public static double score2x = 37;
    public static double score2y = 69;
    public static double score2angle = 90;

    public static double score3x = 37;
    public static double score3y = 65;
    public static double score3angle = 90;

    public static double score4x = 37;
    public static double score4y = 61;
    public static double score4angle = 90;

    public static double score5x = 37;
    public static double score5y = 57;
    public static double score5angle = 90;

    public static double parkX = 11;
    public static double parkY = 38;
    public static double parkDegrees = 180;


    public static int slowWait = 500;
    public static double slowSpeed = 0.6;


    public static Path preload;

    public static Path sampleDragPath1, sampleDragPath2, sampleDragPath3, sampleDragPath4;
    public static Path obsvToIntakePath;

    public static Path intakeSamplePath1, intakeSamplePath2;
    public static Path ejectSamplePath1, ejectSamplePath2;

    public static Path intakeToScore2Path, score2ToIntakePath, preGrabtoGrab2Path;
    public static Path intakeToScore3Path, score3ToIntakePath, preGrabtoGrab3Path;
    public static Path intakeToScore4Path, preGrabtoGrab4Path;
    public static Path parkPath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.redObsv;
        Pose scorePose = PoseConstants.Score.redChamber;

        Pose parkPose = new Pose(parkX, parkY, Math.toRadians(parkDegrees));

        Pose grab2Pose = new Pose(grab2X, grab2Y, Math.toRadians(grab2Degrees));
        Pose grab3Pose = new Pose(grab3X, grab3Y, Math.toRadians(grab3Degrees));
        Pose grab4Pose = new Pose(grab4X, grab4Y, Math.toRadians(grab4Degrees));

        Pose preGrab2Pose = new Pose(preGrab2X, preGrab2Y, Math.toRadians(preGrab2Degrees));
        Pose preGrab3Pose = new Pose(preGrab3X, preGrab3Y, Math.toRadians(preGrab3Degrees));
        Pose preGrab4Pose = new Pose(preGrab4X, preGrab4Y, Math.toRadians(preGrab4Degrees));

        Pose intake1Pose = new Pose(intake1X, intake1Y, Math.toRadians(intake1Degrees));
        Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Degrees));

        Pose eject1Pose = new Pose(eject1X, eject1Y, Math.toRadians(eject1Degrees));
        Pose eject2Pose = new Pose(eject2X, eject2Y, Math.toRadians(eject2Degrees));

        Pose score1Pose = new Pose(score1x, score1y, Math.toRadians(score1angle));
        Pose score2Pose = new Pose(score2x, score2y, Math.toRadians(score2angle));
        Pose score3Pose = new Pose(score3x, score3y, Math.toRadians(score3angle));
        Pose score4Pose = new Pose(score4x, score4y, Math.toRadians(score4angle));
        Pose score5Pose = new Pose(score5x, score5y, Math.toRadians(score5angle));


        preload = buildPath(startPose, score1Pose);






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
                new Point(9, 45)
        ));
        obsvToIntakePath.setLinearHeadingInterpolation(Math.toRadians(-90), grab2Pose.getHeading());



        intakeSamplePath1 = buildCurve(score1Pose, intake1Pose, new Point(0, 74.000));

        ejectSamplePath1 = buildPath(intake1Pose, eject1Pose);

        intakeSamplePath2 = buildPath(eject1Pose, intake2Pose);

        ejectSamplePath2 = buildPath(intake2Pose, preGrab2Pose);

        preGrabtoGrab2Path = buildPath(preGrab2Pose, grab2Pose);



        intakeToScore2Path = buildCurve(grab2Pose, score2Pose, new Point(17.6, 66.5), 0.5);

        score2ToIntakePath = buildCurve(score2Pose, preGrab3Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);

        preGrabtoGrab3Path = buildPath(preGrab3Pose, grab3Pose);

        intakeToScore3Path = buildCurve(preGrab3Pose, score3Pose, new Point(17.6, 66.5), 0.5);

        score3ToIntakePath = buildCurve(score3Pose, preGrab4Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);

        preGrabtoGrab4Path = buildPath(preGrab4Pose, grab4Pose);

        intakeToScore4Path = buildCurve(preGrab4Pose, score4Pose, new Point(17.6, 66.5), 0.5);


        parkPath = buildPath(score5Pose, parkPose);

    }


    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        ElapsedTime timer = new ElapsedTime();

        Globals.IS_AUTO = true;
        DriveConstants.pathEndTimeoutConstraint = 500;

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
        robot.resetIMU();
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(preload),
                        new SpecimenDepositCommand(),


                        new PathCommand(intakeSamplePath1)
                                .alongWith(new SequentialCommandGroup(
                                        new IntakePushOutCommand(intake1Ext),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
//                        new SampleAlignCommand(),
//                        new ExtensionJumpCommand(1),
//                        new WaitCommand(400),

                        new PathCommand(ejectSamplePath1)
                                .alongWith(new ExtensionPositionCommand(eject1Ext)),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                        new WaitCommand(400),

                        new PathCommand(intakeSamplePath2)
                                .alongWith(new SequentialCommandGroup(
                                        new IntakePushOutCommand(intake2Ext)
                                )),
//                        new SampleAlignCommand(),
                        new ExtensionJumpCommand(1),
                        new WaitCommand(400),


                        new PathCommand(ejectSamplePath2).alongWith(new IntakePullBackCommand()),

                        new PathCommand(preGrabtoGrab2Path, slowSpeed)
                                .alongWith(new IntakePushOutCommand(eject2Ext)),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),


                        new SpecimenGrabCommand(),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore2Path).alongWith(new IntakePullBackCommand()),
                        new SpecimenDepositCommand(),

                        new PathCommand(score2ToIntakePath, 1)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new PathCommand(preGrabtoGrab3Path, slowSpeed),
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
                        new PathCommand(preGrabtoGrab4Path, slowSpeed),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore4Path),
                        new SpecimenDepositCommand(),

                        new ConditionalCommand(
                                new PathCommand(parkPath, 1)
                                        .alongWith(new SequentialCommandGroup(
                                                new WaitCommand(500),
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
