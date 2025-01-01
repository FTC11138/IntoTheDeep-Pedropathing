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
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathTimeoutChangeCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
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
@Autonomous(name = "0+5 Drag", preselectTeleOp = "Solo")
public class Auto_0Plus5_Drag extends LinearOpMode {

    public static double grab2X = 9.3;
    public static double grab2Y = 38;
    public static double grab2Degrees = -90;

    public static double grab3X = 8.5;
    public static double grab3Y = 38;
    public static double grab3Degrees = -90;

    public static double grab4X = 8.5;
    public static double grab4Y = 38;
    public static double grab4Degrees = -90;

    public static double grab5X = 8.5;
    public static double grab5Y = 38;
    public static double grab5Degrees = -90;

    public static double preGrab2X = 10.5;
    public static double preGrab2Y = 44;
    public static double preGrab2Degrees = -90;

    public static double preGrab3X = 9.3;
    public static double preGrab3Y = 44;
    public static double preGrab3Degrees = -90;

    public static double preGrab4X = 9.3;
    public static double preGrab4Y = 44;
    public static double preGrab4Degrees = -90;

    public static double preGrab5X = 9.3;
    public static double preGrab5Y = 44;
    public static double preGrab5Degrees = -90;





    public static double dragStart1X = 37.4;
    public static double dragStart1Y = 41.7;
    public static double dragStart1Degrees = -55;
    public static int dragStart1Ext = 1000;

    public static double dragStart2X = 37.4;
    public static double dragStart2Y = 31.7;
    public static double dragStart2Degrees = -55;
    public static int dragStart2Ext = 1000;

    public static double dragStart3X = 37.4;
    public static double dragStart3Y = 21.7;
    public static double dragStart3Degrees = -55;
    public static int dragStart3Ext = 1000;


    public static double dragEnd1X = 17.7;
    public static double dragEnd1Y = 41.7;
    public static double dragEnd1Degrees = -100;

    public static double dragEnd2X = 17.7;
    public static double dragEnd2Y = 31.7;
    public static double dragEnd2Degrees = -100;

    public static double dragEnd3X = 17.7;
    public static double dragEnd3Y = 21.7;
    public static double dragEnd3Degrees = -100;




    public static double score1x = 36;
    public static double score1y = 73;
    public static double score1angle = 90;

    public static double score2x = 36;
    public static double score2y = 69;
    public static double score2angle = 90;

    public static double score3x = 36.5;
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
    public static double slowSpeed = 0.4;


    public static Path preload;

    public static Path toDragPath;

    public static Path sampleDragPath1, sampleDragPath2, sampleDragPath3;
    public static Path sampleDragPath1Back, sampleDragPath2Back;

    public static Path dragToIntakePath;

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

        Pose dragStart1Pose = new Pose(dragStart1X, dragStart1Y, Math.toRadians(dragStart1Degrees));
        Pose dragStart2Pose = new Pose(dragStart2X, dragStart2Y, Math.toRadians(dragStart2Degrees));
        Pose dragStart3Pose = new Pose(dragStart3X, dragStart3Y, Math.toRadians(dragStart3Degrees));

        Pose dragEnd1Pose = new Pose(dragEnd1X, dragEnd1Y, Math.toRadians(dragEnd1Degrees));
        Pose dragEnd2Pose = new Pose(dragEnd2X, dragEnd2Y, Math.toRadians(dragEnd2Degrees));
        Pose dragEnd3Pose = new Pose(dragEnd3X, dragEnd3Y, Math.toRadians(dragEnd3Degrees));

        Pose score1Pose = new Pose(score1x, score1y, Math.toRadians(score1angle));
        Pose score2Pose = new Pose(score2x, score2y, Math.toRadians(score2angle));
        Pose score3Pose = new Pose(score3x, score3y, Math.toRadians(score3angle));
        Pose score4Pose = new Pose(score4x, score4y, Math.toRadians(score4angle));
        Pose score5Pose = new Pose(score5x, score5y, Math.toRadians(score5angle));


        preload = buildPath(startPose, score1Pose);



        toDragPath = buildCurve(score1Pose, dragStart1Pose, new Point(0, 74.000));

        sampleDragPath1 = buildPath(dragStart1Pose, dragEnd1Pose);
        sampleDragPath1Back = buildPath(dragEnd1Pose, dragStart1Pose);

        sampleDragPath2 = buildPath(dragStart2Pose, dragEnd2Pose);
        sampleDragPath2Back = buildPath(dragEnd2Pose, dragStart3Pose);

        sampleDragPath3 = buildPath(dragStart3Pose, dragEnd3Pose);

        dragToIntakePath = buildPath(dragEnd3Pose, preGrab2Pose);


        


        preGrabtoGrab2Path = buildPath(preGrab2Pose, grab2Pose);



        intakeToScore2Path = buildCurve(grab2Pose, score2Pose, new Point(17.6, 66.5), 0.5);

        score2ToIntakePath = buildCurve(score2Pose, preGrab3Pose, new Point(17.6, 66.5), new Point(29.9, 45), 0.5);

        preGrabtoGrab3Path = buildPath(preGrab3Pose, grab3Pose);

        intakeToScore3Path = buildCurve(preGrab3Pose, score3Pose, new Point(17.6, 66.5), 0.5);

        score3ToIntakePath = buildCurve(score3Pose, preGrab4Pose, new Point(17.6, 66.5), new Point(29.9, 45), 0.5);

        preGrabtoGrab4Path = buildPath(preGrab4Pose, grab4Pose);

        intakeToScore4Path = buildCurve(preGrab4Pose, score4Pose, new Point(17.6, 66.5), 0.5);

        score4ToIntakePath = buildCurve(score4Pose, preGrab5Pose, new Point(17.6, 66.5), new Point(29.9, 45), 0.5);

        preGrabtoGrab5Path = buildPath(preGrab5Pose, grab5Pose);

        intakeToScore5Path = buildCurve(preGrab5Pose, score5Pose, new Point(17.6, 66.5), 0.5);


        parkPath = buildPath(score5Pose, parkPose);

    }


    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        ElapsedTime timer = new ElapsedTime();

        Globals.IS_AUTO = true;
        DriveConstants.pathEndTimeoutConstraint = 250;

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

                        new PathTimeoutChangeCommand(100),
                        new PathCommand(preload),
                        new SpecimenDepositCommand(),



                        new PathCommand(toDragPath)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1700),
                                        new ExtensionPositionCommand(dragStart1Ext),
                                        new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),

                        new PathCommand(sampleDragPath1),
                        new PathCommand(sampleDragPath1Back)
                                .alongWith(new SequentialCommandGroup(
                                        new ExtensionPositionCommand(dragStart2Ext),
                                        new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE)
                                )),
                        new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE),
                        new WaitCommand(300),

                        new PathCommand(sampleDragPath2),
                        new PathCommand(sampleDragPath2Back)
                                .alongWith(new SequentialCommandGroup(
                                        new ExtensionPositionCommand(dragStart3Ext),
                                        new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE)
                                )),
                        new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE),
                        new WaitCommand(300),

                        new PathCommand(sampleDragPath3),

                        new PathCommand(dragToIntakePath).alongWith(new IntakePullBackCommand()),


                        new PathCommand(preGrabtoGrab2Path, slowSpeed),



                        new SpecimenGrabCommand(),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new PathCommand(intakeToScore2Path),
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

                        new PathCommand(score4ToIntakePath, 1)
                                .alongWith(new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB)
                                )),
                        new PathCommand(preGrabtoGrab5Path, slowSpeed),
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
