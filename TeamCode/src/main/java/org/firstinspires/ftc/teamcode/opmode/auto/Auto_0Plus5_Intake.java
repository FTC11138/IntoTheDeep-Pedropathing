package org.firstinspires.ftc.teamcode.opmode.auto;

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
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;

@Config
@Autonomous(name = "0+5 Intake", preselectTeleOp = "Solo")
public class Auto_0Plus5_Intake extends LinearOpMode {

    public static double grab2X = 8.7;
    public static double grab2Y = 40;
    public static double grab2Degrees = -90;

    public static double grab3X = 9.3;
    public static double grab3Y = 40;
    public static double grab3Degrees = -90;

    public static double grab4X = 9.5;
    public static double grab4Y = 40;
    public static double grab4Degrees = -90;

    public static double grab5X = 9.7;
    public static double grab5Y = 40;
    public static double grab5Degrees = -90;


    public static double preGrab2X = 18;
    public static double preGrab2Y = 40;
    public static double preGrab2Degrees = -90;

    public static double preGrab3X = 16;
    public static double preGrab3Y = 40;
    public static double preGrab3Degrees = -90;

    public static double preGrab4X = 16;
    public static double preGrab4Y = 40;
    public static double preGrab4Degrees = -90;

    public static double preGrab5X = 16;
    public static double preGrab5Y = 40;
    public static double preGrab5Degrees = -90;


    public static double intake1X = 34.5;
    public static double intake1Y = 41.5;
    public static double intake1Degrees = -60;
    public static int intake1Ext = 300;

    public static double intake2X = 40;
    public static double intake2Y = 32;
    public static double intake2Degrees = -90;
    public static int intake2Ext = 100;

    public static double intake3X = 41;
    public static double intake3Y = 22;
    public static double intake3Degrees = -90;
    public static int intake3Ext = 0;

    public static double eject1X = 32;
    public static double eject1Y = 28.5;
    public static double eject1Degrees = -150;
    public static int eject1Ext = 800;

    public static double eject2X = 32;
    public static double eject2Y = 18.4;
    public static double eject2Degrees = -180;
    public static int eject2Ext = 700;

    public static double eject3X = 9;
    public static double eject3Y = 42;
    public static double eject3Degrees = -90;
    public static int eject3Ext = 700;


    public static double score1x = 37;
    public static double score1y = 74;
    public static double score1angle = 90;

    public static double score2x = 37;
    public static double score2y = 71;
    public static double score2angle = 90;

    public static double score3x = 37;
    public static double score3y = 68;
    public static double score3angle = 90;

    public static double score4x = 37;
    public static double score4y = 65;
    public static double score4angle = 90;

    public static double score5x = 38;
    public static double score5y = 62;
    public static double score5angle = 90;

    public static double parkX = 11;
    public static double parkY = 38;
    public static double parkDegrees = 180;

    public static double slowSpeed = 1;

    public static int outtakeTime = 00;



    public static Path preload;

    public static Path intakeSamplePath1, intakeSamplePath2, intakeSamplePath3;
    public static Path ejectSamplePath1, ejectSamplePath2, ejectSamplePath3;

    public static Path intakeToScore2Path, score2ToIntakePath, preGrabtoGrab2Path;
    public static Path intakeToScore3Path, score3ToIntakePath, preGrabtoGrab3Path;
    public static Path intakeToScore4Path, score4ToIntakePath, preGrabtoGrab4Path;
    public static Path intakeToScore5Path, preGrabtoGrab5Path;
    public static Path parkPath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.redObsv;
        Pose scorePose = PoseConstants.Score.redChamber;

        Pose parkPose = new Pose(parkX, parkY, Math.toRadians(parkDegrees));

        Pose grab2Pose = new Pose(grab2X, grab2Y, Math.toRadians(grab2Degrees));
        Pose grab3Pose = new Pose(grab3X, grab3Y, Math.toRadians(grab3Degrees));
        Pose grab4Pose = new Pose(grab4X, grab4Y, Math.toRadians(grab4Degrees));
        Pose grab5Pose = new Pose(grab5X, grab5Y, Math.toRadians(grab5Degrees));

        Pose preGrab2Pose = new Pose(preGrab2X, preGrab2Y, Math.toRadians(preGrab2Degrees));
        Pose preGrab3Pose = new Pose(preGrab3X, preGrab3Y, Math.toRadians(preGrab3Degrees));
        Pose preGrab4Pose = new Pose(preGrab4X, preGrab4Y, Math.toRadians(preGrab4Degrees));
        Pose preGrab5Pose = new Pose(preGrab5X, preGrab5Y, Math.toRadians(preGrab5Degrees));

        Pose intake1Pose = new Pose(intake1X, intake1Y, Math.toRadians(intake1Degrees));
        Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Degrees));
        Pose intake3Pose = new Pose(intake3X, intake3Y, Math.toRadians(intake3Degrees));

        Pose eject1Pose = new Pose(eject1X, eject1Y, Math.toRadians(eject1Degrees));
        Pose eject2Pose = new Pose(eject2X, eject2Y, Math.toRadians(eject2Degrees));
        Pose eject3Pose = new Pose(eject3X, eject3Y, Math.toRadians(eject3Degrees));

        Pose score1Pose = new Pose(score1x, score1y, Math.toRadians(score1angle));
        Pose score2Pose = new Pose(score2x, score2y, Math.toRadians(score2angle));
        Pose score3Pose = new Pose(score3x, score3y, Math.toRadians(score3angle));
        Pose score4Pose = new Pose(score4x, score4y, Math.toRadians(score4angle));
        Pose score5Pose = new Pose(score5x, score5y, Math.toRadians(score5angle));


        preload = buildPath(startPose, score1Pose);


        intakeSamplePath1 = buildCurve(score1Pose, intake1Pose, new Point(0, 74.000));

        ejectSamplePath1 = buildPath(intake1Pose, eject1Pose);

        intakeSamplePath2 = buildPath(eject1Pose, intake2Pose);

        ejectSamplePath2 = buildPath(intake2Pose, eject2Pose);

        intakeSamplePath3 = buildPath(eject2Pose, intake3Pose);

        ejectSamplePath3 = buildPath(intake3Pose, grab2Pose);

        preGrabtoGrab2Path = buildPath(preGrab2Pose, grab2Pose);



        intakeToScore2Path = buildCurve(grab2Pose, score2Pose, new Point(17.6, 66.5), 0.5);

        score2ToIntakePath = buildCurve(score2Pose, grab3Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);

        preGrabtoGrab3Path = buildPath(preGrab3Pose, grab3Pose);

        intakeToScore3Path = buildCurve(preGrab3Pose, score3Pose, new Point(17.6, 66.5), 0.5);

        score3ToIntakePath = buildCurve(score3Pose, grab4Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);

        preGrabtoGrab4Path = buildPath(preGrab4Pose, grab4Pose);

        intakeToScore4Path = buildCurve(grab4Pose, score4Pose, new Point(17.6, 66.5), 0.5);

        score4ToIntakePath = buildCurve(score4Pose, grab5Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);

        preGrabtoGrab5Path = buildPath(preGrab5Pose, grab5Pose);

        intakeToScore5Path = buildCurve(grab5Pose, score5Pose, new Point(17.6, 66.5), new Point(29.9, 41), 0.5);


        parkPath = buildPath(score5Pose, parkPose);
    }


    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        ElapsedTime timer = new ElapsedTime();

        Globals.IS_AUTO = true;
        DriveConstants.pathEndTimeoutConstraint = 100;

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


                        new PathCommand(intakeSamplePath1)
                                .alongWith(new SequentialCommandGroup(
                                        new IntakePushOutCommand(intake1Ext),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
//                        new WaitCommand(200),
                        new ExtensionJumpCommand(1),
                        new WaitCommand(300),

                        new PathCommand(ejectSamplePath1)
                                .alongWith(new ExtensionPositionCommand(eject1Ext)),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                        new WaitCommand(outtakeTime),

                        new PathCommand(intakeSamplePath2)
                                .alongWith(new SequentialCommandGroup(
                                        new IntakePushOutCommand(intake2Ext)
                                )),
//                        new WaitCommand(200),
                        new ExtensionJumpCommand(1),
                        new WaitCommand(400),

                        new PathCommand(ejectSamplePath2)
                                .alongWith(new ExtensionPositionCommand(eject2Ext)),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                        new WaitCommand(outtakeTime),

                        new PathCommand(intakeSamplePath3)
                                .alongWith(new SequentialCommandGroup(
                                        new IntakePushOutCommand(intake3Ext)
                                )),
//                        new WaitCommand(200),
                        new ExtensionJumpCommand(1),
                        new WaitCommand(600),

                        new PathCommand(ejectSamplePath3),

//                        new PathCommand(preGrabtoGrab2Path, slowSpeed)
//                                .alongWith(new IntakePushOutCommand(eject2Ext)),
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
//                        new PathCommand(preGrabtoGrab3Path, slowSpeed),
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
//                        new PathCommand(preGrabtoGrab4Path, slowSpeed),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore4Path),
                        new SpecimenDepositCommand(),

                        new PathCommand(score4ToIntakePath, 1)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
//                        new PathCommand(preGrabtoGrab5Path, slowSpeed),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(200),

                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore5Path),
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