package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveConstants;

public class PathCommand extends CommandBase {

    private final Path path;

    private final Robot robot = Robot.getInstance();

    private final double speed;

    public PathCommand(Path path) {
        this.path = path;
        this.speed = 1;
    }

    public PathCommand(Path path, double speed) {
        this.path = path;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.setMaxPower(speed);
        robot.followPath(path, false);
    }

    @Override
    public void execute() {
        robot.update();
    }

    @Override
    public boolean isFinished() {
//        double xError = Math.abs(robot.getPose().getX() - path.getPath(0).getLastControlPoint().getX());
//        double yError = Math.abs(robot.getPose().getY() - path.getPath(0).getLastControlPoint().getY());
//        double headingError = Math.abs(robot.getPose().getHeading() - path.getPath(0).getHeadingGoal(1));

//        return xError < Constants.pathEndXTolerance && yError < Constants.pathEndYTolerance && headingError < Constants.pathEndHeadingTolerance;
        return !robot.isBusy();
    }

}
