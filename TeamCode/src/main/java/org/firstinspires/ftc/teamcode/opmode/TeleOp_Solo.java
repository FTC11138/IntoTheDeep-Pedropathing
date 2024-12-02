package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftMidCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenGrabCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Localizer;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp (name = "Solo")
public class TeleOp_Solo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;

    Pose currentPose;
    double heading;
    double fieldCentricOffset;

    boolean lastLeftTrigger;
    boolean lastRightTrigger;

    boolean lastA;
    boolean lastB;
    boolean lastX;
    boolean lastY;

    boolean lastLeftBumper;
    boolean lastRightBumper;

    boolean lastDpadUp;
    boolean lastDpadDown;
    boolean lastDpadLeft;
    boolean lastDpadRight;

    boolean lastRightStickButton;
    boolean lastLeftStickbutton;

    boolean lastLiftChangeJoystickUp;
    boolean lastLiftChangeJoystickDown;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        robot.setPose(robot.data.currentPose);

        data.stopIntaking();
        data.stopScoring();
        data.setSampleUnloaded();

        switch (Globals.ALLIANCE) {
            case RED:
                fieldCentricOffset = Math.toRadians(90);
                robot.intakeSubsystem.leds.setPattern(Constants.redPattern);
                break;
            case BLUE:
                fieldCentricOffset = Math.toRadians(-90);
                robot.intakeSubsystem.leds.setPattern(Constants.bluePattern);
                break;
        }

        robot.startTeleopDrive();

    }

    @Override
    public void run() {

        cs.run();
        robot.periodic();

        robot.update();
        currentPose = robot.getPose();
        heading = -currentPose.getHeading();

        robot.setTeleOpMovementVectors(
                -gamepad1.left_stick_y * (data.scoring || data.intaking ? 0.5 : 1),
                -gamepad1.left_stick_x * (data.scoring || data.intaking ? 0.5 : 1),
                -gamepad1.right_stick_x * (data.scoring || data.intaking ? 0.7 : 1),
                false
        );

        boolean liftChangeJoystickUp = gamepad1.right_stick_y < -0.8;
        boolean liftChangeJoystickDown = gamepad1.right_stick_y > 0.8;

        if (robot.data.intaking) {
            if (liftChangeJoystickUp && !lastLiftChangeJoystickUp) {
                cs.schedule(new ExtensionJumpCommand(1));
            } else if (liftChangeJoystickDown && !lastLiftChangeJoystickDown) {
                cs.schedule(new ExtensionJumpCommand(-1));
            }

//            if (robot.intakeSubsystem.getCurrentColor() == IntakeSubsystem.DetectedColor.RED && Globals.ALLIANCE == Globals.Alliance.BLUE ||
//                    robot.intakeSubsystem.getCurrentColor() == IntakeSubsystem.DetectedColor.BLUE && Globals.ALLIANCE == Globals.Alliance.RED) {
//                cs.schedule(new SampleEjectCommand());
//            }
        }

        lastLiftChangeJoystickUp = liftChangeJoystickUp;
        lastLiftChangeJoystickDown = liftChangeJoystickDown;

        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean y = g1.getButton(GamepadKeys.Button.Y);
        boolean leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean dpadUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean dpadDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadLeft = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean rightStickButton = g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean leftStickButton = g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);

        scheduleCommand(lastA, a, new SequentialCommandGroup(
                new DropSampleCommand(),
                new WaitCommand(400),
                new LiftDownCommand()
        ));
        scheduleCommand(lastB, b, new LiftDownCommand());
        scheduleCommand(lastX, x, new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT));
        scheduleCommand(lastY, y, new LiftMidCommand());

        scheduleCommand(lastLeftBumper, leftBumper, new SampleEjectCommand());
        scheduleCommand(lastRightBumper, rightBumper, new LiftUpCommand());

        scheduleCommand(lastDpadDown, dpadDown, new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB));
        scheduleCommand(lastDpadUp, dpadUp, new ConditionalCommand(
                new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.LOW),
                new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                () -> robot.specimenSubsystem.getSpecimenLiftState() == SpecimenSubsystem.SpecimenLiftState.HIGH
        ));

        scheduleCommand(lastDpadLeft, dpadLeft, new SpecimenGrabCommand()
                .andThen(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH)));
        scheduleCommand(lastDpadRight, dpadRight, new SpecimenDepositCommand());

        if (leftStickButton) {
            Globals.LIMITS = false;
            cs.schedule(new LiftPowerCommand(-0.5));
        } else if (lastLeftStickbutton) { // if just released set power back to 0
            Globals.LIMITS = true;
            cs.schedule(new LiftPowerCommand(0));
        }

        scheduleCommand(lastRightStickButton, rightStickButton, new LiftResetCommand());



        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastRightStickButton = rightStickButton;
        lastLeftStickbutton = leftStickButton;


        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (leftTrigger && !lastLeftTrigger) {
            cs.schedule(new IntakePushOutCommand(Constants.extIntake));
        }

        if (rightTrigger && !lastRightTrigger) {
            cs.schedule(new IntakePullBackCommand().andThen(new SampleTransferCommand()));
        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;

        robot.updateData();
        robot.write();

    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            cs.schedule(command);
        }
    }
}
