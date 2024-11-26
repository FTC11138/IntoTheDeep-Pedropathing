package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.Ascent1Command;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.CurvePositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "3+0", preselectTeleOp = "Solo")
public class Auto_3Plus0 extends LinearOpMode {

    public static Pose redBasketAngle = PoseConstants.Score.redBasketAngle;
    public static double sample1x = -37;
    public static double sample1y = -27.5;
    public static double sample1degrees = 180;
    public static int sample1ext = 650;

    public static double sample2x = -47;
    public static double sample2y = -27.5;
    public static double sample2degrees = 180;
    public static int sample2ext = 500;

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

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
                        new LinePositionCommand(redBasketAngle)
                                .alongWith(new LiftUpCommand()),
                        new WaitCommand(500),
                        new WaitCommand(500),
                        new DropSampleCommand(),

                        new LiftDownCommand(),
                        new IntakePushOutCommand(0),

                        new CurvePositionCommand(sample1, 0, Math.toRadians(90)),
                        new ExtensionJumpCommand(1, sample1ext),
                        new WaitCommand(1000),
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
                        new WaitCommand(1000),
                        new WaitCommand(500),
                        new DropSampleCommand(),

                        new LiftDownCommand(),
                        new IntakePushOutCommand(0),

                        new CurvePositionCommand(sample2, 0, Math.toRadians(90)),
                        new ExtensionJumpCommand(1, sample2ext),
                        new WaitCommand(1000),
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
                        new WaitCommand(1000),
                        new WaitCommand(500),
                        new DropSampleCommand(),

                        new LiftDownCommand(),

                        new CurvePositionCommand(new Pose(-18, -12, Math.toRadians(0)), Math.toRadians(90), Math.toRadians(-30))
                                .alongWith(new Ascent1Command())
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
