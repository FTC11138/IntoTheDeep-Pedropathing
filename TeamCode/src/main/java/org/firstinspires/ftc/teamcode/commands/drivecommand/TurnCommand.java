package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class TurnCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();

    private final double speed, angle;

    public TurnCommand(double degrees) {
        this.angle = Math.toRadians(degrees);
        this.speed = 1;
    }

    public TurnCommand(double degrees, double speed) {
        this.angle = Math.toRadians(degrees);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.setMaxPower(speed);

        Path path = new Path(new BezierLine(
                new Point(robot.getPose()),
                new Point(robot.getPose())
        ));
        path.setLinearHeadingInterpolation(robot.getPose().getHeading(), robot.getPose().getHeading() + angle);

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
