package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleAlignCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Disabled
@Autonomous(name = "5+0", preselectTeleOp = "Solo")
public class Auto_5Plus0 extends LinearOpMode {

    public static double scoreX = 14;
    public static double scoreY = 126;
    public static double scoreDegrees = -45;
    public static Pose score;


    public static double sample1x = 33;
    public static double sample1y = 109;
    public static double sample1degrees = 60;
    public static int sample1ext = 750;

    public static double sample2x = 39;
    public static double sample2y = 115;
    public static double sample2degrees = 90;
    public static int sample2ext = 650;

    public static double sample3x = 38;
    public static double sample3y = 123;
    public static double sample3degrees = 90;
    public static int sample3ext = 500;

    public static double sample4x = 72;
    public static double sample4y = 100;
    public static double sample4degrees = -90;
    public static int sample4ext = 250;


    public static Path preload;
    public static Path sample1Path, sample1ScorePath;
    public static Path sample2Path, sample2ScorePath;
    public static Path sample3Path, sample3ScorePath;
    public static Path sample4Path, sample4ScorePath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.redBasket;
        Pose scorePose = new Pose(scoreX, scoreY, Math.toRadians(scoreDegrees));
        score = new Pose(scoreX, scoreY, Math.toRadians(scoreDegrees));

        Pose sample1Pose = new Pose(sample1x, sample1y, Math.toRadians(sample1degrees));
        Pose sample2Pose = new Pose(sample2x, sample2y, Math.toRadians(sample2degrees));
        Pose sample3Pose = new Pose(sample3x, sample3y, Math.toRadians(sample3degrees));
        Pose sample4Pose = new Pose(sample4x, sample4y, Math.toRadians(sample4degrees));

        // Preload path
        preload = buildPath(startPose, scorePose);

        // Sample 1 paths
        sample1Path = buildPath(scorePose, sample1Pose);
        sample1ScorePath = buildPath(sample1Pose, scorePose);

        // Sample 2 paths
        sample2Path = buildPath(scorePose, sample2Pose);
        sample2ScorePath = buildPath(sample2Pose, scorePose);

        // Sample 3 paths
        sample3Path = buildPath(scorePose, sample3Pose);
        sample3ScorePath = buildPath(sample3Pose, scorePose);

        // Sample 4 paths
        sample4Path = buildCurve(scorePose, sample4Pose, new Point(75, 119));
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

        robot.setPose(PoseConstants.Start.redBasket);
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
                        new PathCommand(preload)
                                .alongWith(new LiftUpCommand()),
                        new DropSampleCommand(),

                        new PathCommand(sample1Path)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample1ext),
                        new WaitCommand(500),
                        new PathCommand(sample1ScorePath)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakePullBackCommand(),
                                                new SampleTransferCommand(),
                                                new WaitCommand(200),
                                                new LiftUpCommand(),
                                                new WaitCommand(700)
                                        )
                                ),
                        new DropSampleCommand(),

                        new PathCommand(sample2Path)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample2ext),
                        new WaitCommand(500),
                        new PathCommand(sample2ScorePath)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakePullBackCommand(),
                                                new SampleTransferCommand(),
                                                new WaitCommand(200),
                                                new LiftUpCommand(),
                                                new WaitCommand(700)
                                        )
                                ),
                        new DropSampleCommand(),

                        new PathCommand(sample3Path)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample3ext),
                        new WaitCommand(500),
                        new PathCommand(sample3ScorePath)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakePullBackCommand(),
                                                new SampleTransferCommand(),
                                                new WaitCommand(200),
                                                new LiftUpCommand(),
                                                new WaitCommand(700)
                                        )
                                ),
                        new DropSampleCommand(),

                        new PathCommand(sample4Path)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new WaitCommand(1000),
                                        new IntakePushOutCommand(sample4ext)
                                )),

                        new ExtensionJumpCommand(1, sample4ext),
                        new WaitCommand(500),
                        new PathCommand(sample4ScorePath)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakePullBackCommand(),
                                                new SampleTransferCommand(),
                                                new WaitCommand(700),
                                                new LiftUpCommand(),
                                                new WaitCommand(700)
                                        )
                                ),
                        new DropSampleCommand(),

                        new PathCommand(sample4Path)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new WaitCommand(1500),
                                        new IntakePushOutCommand(sample4ext)
                                )),

                        new SampleAlignCommand(),

                        new ExtensionJumpCommand(1, 1000),
                        new WaitCommand(500),

                        new LinePositionCommand(score)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakePullBackCommand(),
                                                new SampleTransferCommand(),
                                                new WaitCommand(700),
                                                new LiftUpCommand(),
                                                new WaitCommand(700)
                                        )
                                ),
                        new DropSampleCommand(),

                        new LiftDownCommand()

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
