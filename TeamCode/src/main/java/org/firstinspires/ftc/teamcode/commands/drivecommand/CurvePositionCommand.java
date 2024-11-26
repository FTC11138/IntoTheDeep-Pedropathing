package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class CurvePositionCommand extends CommandBase {

    public Point startPoint;
    public Point endPoint;
    public double st;
    public double et;

    private Robot robot = Robot.getInstance();

    public CurvePositionCommand(Pose target, double st, double et) {
        this.endPoint = new Point(target);
    }

    @Override
    public void initialize() {
        this.startPoint = new Point(robot.getPose());

        Path path = new Path(new BezierCurve(startPoint, endPoint));
        path.setLinearHeadingInterpolation(st, et);

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
}
