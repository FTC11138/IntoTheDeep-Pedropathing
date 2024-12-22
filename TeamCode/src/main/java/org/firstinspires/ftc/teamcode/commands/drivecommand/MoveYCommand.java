package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class MoveYCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();

    private final double speed, distance;

    public MoveYCommand(double distance) {
        this.distance = Math.toRadians(distance);
        this.speed = 1;
    }

    public MoveYCommand(double distance, double speed) {
        this.distance = Math.toRadians(distance);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.setMaxPower(speed);

        Path path = new Path(new BezierLine(
                new Point(robot.getPose()),
                new Point(robot.getPose().getX(), robot.getPose().getY()+distance)
        ));
        path.setConstantHeadingInterpolation(robot.getPose().getHeading());

        robot.followPath(path, false);
    }

    @Override
    public void execute() {
        robot.update();
        robot.getDashboardPoseTracker().update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !robot.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        robot.setMaxPower(1);
    }
}
