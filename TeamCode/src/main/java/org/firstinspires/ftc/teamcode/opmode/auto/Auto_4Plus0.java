package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.Ascent1Command;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.CurvePositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "4+0", preselectTeleOp = "Solo")
public class Auto_4Plus0 extends LinearOpMode {

    public static Pose redBasketAngle = PoseConstants.Score.redBasketAngle;
    public static double sample1x = 35;
    public static double sample1y = 108;
    public static double sample1degrees = 60;
    public static int sample1ext = 750;

    public static double sample2x = 41;
    public static double sample2y = 115;
    public static double sample2degrees = 90;
    public static int sample2ext = 650;

    public static double sample3x = 39.5;
    public static double sample3y = 123;
    public static double sample3degrees = 90;
    public static int sample3ext = 500;

    @Override
    public void runOpMode() throws InterruptedException {
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

        Pose sample1 = new Pose(sample1x, sample1y, Math.toRadians(sample1degrees));
        Pose sample2 = new Pose(sample2x, sample2y, Math.toRadians(sample2degrees));
        Pose sample3 = new Pose(sample3x, sample3y, Math.toRadians(sample3degrees));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
                        new LinePositionCommand(redBasketAngle)
                                .alongWith(new LiftUpCommand()),
                        new DropSampleCommand(),

                        new LinePositionCommand(sample1)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample1ext),
                        new WaitCommand(500),
                        new LinePositionCommand(redBasketAngle)
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

                        new LinePositionCommand(sample2)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample2ext),
                        new WaitCommand(500),
                        new LinePositionCommand(redBasketAngle)
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

                        new LinePositionCommand(sample3)
                                .alongWith(new SequentialCommandGroup(
                                        new LiftDownCommand(),
                                        new IntakePushOutCommand(0)
                                )),

                        new ExtensionJumpCommand(1, sample3ext),
                        new WaitCommand(500),
                        new LinePositionCommand(redBasketAngle)
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

                        new LinePositionCommand(new Pose(60, 108, Math.toRadians(90)))
                                .alongWith(new LiftDownCommand())

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
