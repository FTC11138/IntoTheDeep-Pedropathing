package org.firstinspires.ftc.teamcode.commands.drivecommand;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.createPathBetweenPoses;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class LinePositionCommand extends CommandBase {

    public Pose startPose;
    public Pose endPose;

    private Robot robot = Robot.getInstance();

    public LinePositionCommand(Pose target) {
        this.endPose = target;
    }

    @Override
    public void initialize() {
        this.startPose = robot.getPose();

        robot.followPath(createPathBetweenPoses(startPose, endPose));
    }

    @Override
    public void execute() {
        robot.update();
        robot.getDashboardPoseTracker().update();
    }

    @Override
    public boolean isFinished() {
        return !robot.isBusy();
    }

}
