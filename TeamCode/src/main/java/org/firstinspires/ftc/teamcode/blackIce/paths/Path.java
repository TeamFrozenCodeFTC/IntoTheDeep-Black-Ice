package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public class Path {
    double[][] points;

    public double constantHeading = 0;
    public double headingOffset = 0;
    public boolean isConstantHeading = false;

    private final double endingHeading;

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;

        this.endPoint = points[points.length - 1];

        if (isConstantHeading) {
            endingHeading = constantHeading;
        } else {
            endingHeading = Math.toDegrees(
                Math.atan2(endPoint[1] - points[points.length - 2][1],
                    endPoint[0] - points[points.length - 2][0])
            ) + headingOffset;
        }
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

    public Path setMovementParameters(Movement movementParameters) {
        movementBuild = movementParameters;
        return this;
    }

    private int i = 1;
    private boolean holding = false;
    private boolean completed = false;
    private final double[] endPoint;
    private Movement movementBuild;

    public void update() {
        if (holding) {
            movementBuild.update();
            if (!movementBuild.isNotCompleted()) {
                completed = true;
            }
            return;
        }

        Target.updatePosition();

        double[] point = points[i];

        Follower.telemetry.addData("i", i);
        Follower.telemetry.update();

        double inchesLeftOnPath = (points.length - i) * Constants.Curve.INCHES_PER_POINT;
        if (inchesLeftOnPath < Util.getVectorMagnitude(
            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
            stopAndHoldEndPoint();
            return;
        }

        double targetHeading;
        if (isConstantHeading) {
            targetHeading = constantHeading;
        } else {
            double[] pointHeading;
            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= points.length) {
                pointHeading = endPoint;
            }
            else {
                pointHeading = points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
            }

            targetHeading = Math.toDegrees(
                Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
            ) + headingOffset;
        }

        // Hacky Solution for allowing things to be build for curve
        Target.setTarget(targetHeading, point[0], point[1]);
        movementBuild.path = null; // otherwise gets stuck in loop
        // maybe should not have it wait until the next point?
        movementBuild.moveThrough().waitForMovement();

        this.i += 1;
        if (i >= points.length) {
            stopAndHoldEndPoint();
        }
    }

    public void stopAndHoldEndPoint() {
        Target.setTarget(endingHeading, endPoint[0], endPoint[1]);
        movementBuild.path = null; // otherwise gets stuck in loop
        movementBuild.stopAtPosition();
        holding = true;
    }

    /**
     * Make the robot follow the curve.
     */
    public void runCurve(Movement movementBuild) {
        this.movementBuild = movementBuild;

        // Set previous Point
        Target.setTarget(Target.previousHeading, points[0][0], points[0][1]);

        Follower.telemetry.addData("start", 1);
        Follower.telemetry.update();

        while (!completed) {
            update();
        }

        Follower.telemetry.addData("completed", 1);
        Follower.telemetry.update();

        Drive.zeroPower();
    }
//
//    public void endPath() {
//        movementBuild.update();
//
//        double targetHeading;
//        if (isConstantHeading) {
//            targetHeading = constantHeading;
//        } else {
//            targetHeading = Math.toDegrees(
//                Math.atan2(endPoint[1] - points[points.length - 2][1],
//                    endPoint[0] - points[points.length - 2][0])
//            ) + headingOffset;
//        }
//
//        Target.setTarget(targetHeading, endPoint[0], endPoint[1]);
//        movementBuild.path = null; // otherwise gets stuck in loop
//        movementBuild.stopAtPosition().waitForMovement();
//
//        Drive.zeroPower();
//    }

//    /**
//     * Make the robot follow the curve.
//     */
//    public void runCurve(Follower movementBuild) {
//        // Set previous Point
//        Target.setTarget(Target.previousHeading, points[0][0], points[0][1]);
//
//        double[] endPoint = points[points.length - 1];
//
//        for (int i = 1; i <= points.length; i++) {
//            movementBuild.update();
//
//            double[] point = points[i];
//
//            double inchesLeftOnPath = (points.length - i) * Constants.Curve.INCHES_PER_POINT;
//            if (inchesLeftOnPath < Util.getVectorMagnitude(
//                Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
//                break;
//            }
//
//            double[] pointHeading;
//            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= points.length) {
//                pointHeading = endPoint;
//            }
//            else {
//                pointHeading = points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
//            }
//
//            double targetHeading;
//            if (isConstantHeading) {
//                targetHeading = constantHeading;
//            } else {
//                targetHeading = Math.toDegrees(
//                    Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
//                ) + headingOffset;
//            }
//
//            // Hacky Solution for allowing things to be build for curve
//            Target.setTarget(targetHeading, point[0], point[1]);
//            movementBuild.path = null; // otherwise gets stuck in loop
//            movementBuild.moveThrough().continuePowerAfter().run();
//        }
//
//        movementBuild.update();
//
//        double targetHeading;
//        if (isConstantHeading) {
//            targetHeading = constantHeading;
//        } else {
//            targetHeading = Math.toDegrees(
//                Math.atan2(endPoint[1] - points[points.length - 2][1],
//                    endPoint[0] - points[points.length - 2][0])
//            ) + headingOffset;
//        }
//
//        Target.setTarget(targetHeading, endPoint[0], endPoint[1]);
//        movementBuild.path = null; // otherwise gets stuck in loop
//        movementBuild.stopAtPosition().run();
//
//        Drive.zeroPower();
//    }
}