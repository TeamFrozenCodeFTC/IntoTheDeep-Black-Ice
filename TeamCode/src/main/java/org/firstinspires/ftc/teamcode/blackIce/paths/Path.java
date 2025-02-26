package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public class Path extends Movement {
    double[][] points;

    public double constantHeading = 0;
    public double headingOffset = 0;
    public boolean isConstantHeading = false;

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;
    }

    /**
     * Make the robot have a constant heading along the path.
     */
    public Path setConstantHeading(double newConstantHeading) {
        isConstantHeading = true;
        constantHeading = newConstantHeading;
        return this;
    }

    /**
     * Make the robot have a offset heading along the path.
     */
    public Path setHeadingOffset(double newHeadingOffset) {
        isConstantHeading = false;
        headingOffset = newHeadingOffset;
        return this;
    }

    /**
     * Make the robot follow the curve.
     */
    public void runCurve(Movement movementBuild) {
        Robot robot = Robot.getInstance();

        // Set previous Point
        Target.setTarget(Target.previousHeading, points[0][0], points[0][1]);

        double[] endPoint = points[points.length - 1];

        for (int i = 1; i <= points.length; i++) {
            robot.loopUpdate();

            double[] point = points[i];

            double inchesLeftOnPath = (points.length - i) * Constants.Curve.INCHES_PER_POINT;
            if (inchesLeftOnPath < Util.getVectorMagnitude(
                Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
                break;
            }

            double[] pointHeading;
            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= points.length) {
                pointHeading = endPoint;
            }
            else {
                pointHeading = points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
            }

            double targetHeading;
            if (isConstantHeading) {
                targetHeading = constantHeading;
            } else {
                targetHeading = Math.toDegrees(
                    Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
                ) + headingOffset;
            }

            // Hacky Solution for allowing things to be build for curve
            Target.setTarget(targetHeading, point[0], point[1]);
            movementBuild.path = null; // otherwise gets stuck in loop
            movementBuild.moveThrough().continuePowerAfter().run();
        }

        robot.loopUpdate();

        double targetHeading;
        if (isConstantHeading) {
            targetHeading = constantHeading;
        } else {
            targetHeading = Math.toDegrees(
                Math.atan2(endPoint[1] - points[points.length - 2][1],
                    endPoint[0] - points[points.length - 2][0])
            ) + headingOffset;
        }

        Target.setTarget(targetHeading, endPoint[0], endPoint[1]);
        movementBuild.path = null; // otherwise gets stuck in loop
        movementBuild.stopAtPosition().run();

        Drive.zeroPower();
    }
}