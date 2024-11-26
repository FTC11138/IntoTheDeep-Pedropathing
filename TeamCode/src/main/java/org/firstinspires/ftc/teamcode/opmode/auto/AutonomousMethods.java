package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutonomousMethods {

    /**
     * Creates a path between two poses.
     *
     * @param pose1 The starting pose
     * @param pose2 The ending pose
     * @return The created path
     */
    public static Path createPathBetweenPoses(Pose pose1, Pose pose2) {
        // Create points using only the x and y from the poses
        Point point1 = new Point(pose1.getX(), pose1.getY());
        Point point2 = new Point(pose2.getX(), pose2.getY());

        // Create the path using a Bezier line
        Path path = new Path(new BezierLine(point1, point2));

        // Set linear heading interpolation using the poses' angles
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        return path;
    }

}
