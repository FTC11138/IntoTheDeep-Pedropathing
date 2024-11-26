package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.Ascent1Command;
import org.firstinspires.ftc.teamcode.commands.drivecommand.LinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.CurvePositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Autonomous(name = "1+0", preselectTeleOp = "Solo")
public class Auto_1Plus0 extends LinearOpMode {


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

        waitForStart();

        robot.setPose(PoseConstants.Start.redBasket);

        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        robot.depositSubsystem.updateBucketState(DepositSubsystem.BucketState.INTAKE);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
                        new LinePositionCommand(PoseConstants.Score.redBasketAngle)
                                .alongWith(new LiftUpCommand()),
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
