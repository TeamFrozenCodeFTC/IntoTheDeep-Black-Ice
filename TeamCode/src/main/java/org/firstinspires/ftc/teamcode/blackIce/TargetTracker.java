package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.tests.SquareRunTest;
import org.firstinspires.ftc.teamcode.blackIce.ErrorMargin;
import org.firstinspires.ftc.teamcode.util.Util;

public class TargetTracker {
    Robot robot;

    public TargetTracker(Robot robot) {
        this.robot = robot;
    }

    public ErrorMargin defaultErrorMargin = new ErrorMargin(1, 1, 3);
//    public ErrorMargin wideErrorMargin = new ErrorMargin(5, 1.5, 1.5);

    public double heading;
    public double x;
    public double y;

    public double previousX;
    public double previousY;

    public double headingError;
    public double xError;
    public double yError;
    public double distanceToTarget;

    public double totalDistanceToTarget;
    public double previousHeading;

//    public double lateralBrakingDistance;
//    public double forwardBrakingDistance;
//    public double brakingDistance;
//
//    public double xBrakingDistance;
//    public double yBrakingDistance;

//    public void setPreviousPoint(double targetX, double targetY) {
//        this.previousX = targetX;
//        this.previousY = targetY;
//    }

    public void setTarget(double targetHeading, double targetX, double targetY) {
        previousHeading = this.heading;
        previousX = this.x;
        previousY = this.y;

        this.heading = targetHeading;
        this.x = targetX;
        this.y = targetY;

        updatePosition();

        totalDistanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
    }

    public void updatePosition() {
        robot.odometry.update();
        headingError = Util.simplifyAngle(heading - robot.odometry.heading);
        xError = x - robot.odometry.x;
        yError = y - robot.odometry.y;
        distanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

        robot.telemetry.addData("velocity", robot.odometry.velocity);
        robot.telemetry.update();
    }

    public boolean isNotWithinErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError) > errorMargin.y ||
            Math.abs(xError) > errorMargin.x ||
            Math.abs(headingError) > errorMargin.degrees
        );
    }

    public boolean isWithinBrakingErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError - robot.odometry.yBrakingDistance) < errorMargin.y &&
            Math.abs(xError - robot.odometry.xBrakingDistance) < errorMargin.x &&
            Math.abs(headingError) < errorMargin.degrees
        );
    }

//    public double[] fieldVectorToLocalVector(double x, double y) {
//        // positive heading is counterclockwise
//        double headingRadians = Math.toRadians(heading);
//        double cos = Math.cos(headingRadians);
//        double sin = Math.sin(headingRadians);
//        double localX = (x * cos + y * sin); // clockwise rotation
//        double localY = (-x * sin + y * cos);
//
//        return new double[] {localX, localY};
//    }
//
//    public double[] localVectorToFieldVector(double x, double y) {
//        // positive heading is counterclockwise
//        double headingRadians = Math.toRadians(heading);
//        double cos = Math.cos(headingRadians);
//        double sin = Math.sin(headingRadians);
//        double fieldX = (-x * cos - y * sin); // counter clockwise rotation
//        double fieldY = (x * sin - y * cos);
//
//        return new double[] {fieldX, fieldY};
//    }
//
//    private void estimateBrakingDistances() {
//        double[] localVelocity = fieldVectorToLocalVector(robot.odometry.xVelocity, robot.odometry.yVelocity);
//        forwardBrakingDistance = 0.00130445 * Math.pow(localVelocity[0], 2) + 0.0644448 * localVelocity[0] + 0.0179835;
//        lateralBrakingDistance = 0.00130445 * Math.pow(localVelocity[1], 2) + 0.0644448 * localVelocity[1] + 0.0179835;
//
//        brakingDistance = Math.sqrt(Math.pow(forwardBrakingDistance, 2) + Math.pow(lateralBrakingDistance, 2));
//
//        double[] fieldVelocity = localVectorToFieldVector(forwardBrakingDistance, lateralBrakingDistance);
//        xBrakingDistance = fieldVelocity[0];
//        yBrakingDistance = fieldVelocity[1];
//    }
//
    public boolean isWithinBrakingDistance2() {
        boolean isInSameQuadrant = Math.signum(xError) == Math.signum(robot.odometry.xBrakingDistance)
            || Math.signum(yError) == Math.signum(robot.odometry.yBrakingDistance);
        return distanceToTarget <= robot.odometry.brakingDistance && isInSameQuadrant;
        // force it to be going in the right direction
        // return xError < robot.odometry.xBrakingDistance && yError < robot.odometry.yBrakingDistance;
    }

    public boolean isWithinBrakingDistance() {
        boolean isInSameQuadrant = Math.signum(xError) == Math.signum(robot.odometry.xBrakingDistance)
            || Math.signum(yError) == Math.signum(robot.odometry.yBrakingDistance);
        boolean brakingDistanceGoesPastTarget =
            (Math.abs(xError) < Math.abs(robot.odometry.xBrakingDistance)
            && Math.abs(yError) < Math.abs(robot.odometry.yBrakingDistance)); // the && could also be ||

        return isInSameQuadrant && brakingDistanceGoesPastTarget;
    }

    // (for movePast)
    public boolean isWithinBrakingDistance5() {
        boolean isInSameQuadrant = Math.signum(xError) == Math.signum(robot.odometry.xBrakingDistance)
            || Math.signum(yError) == Math.signum(robot.odometry.yBrakingDistance);
        boolean brakingDistanceGoesPastTarget =
            (Math.abs(xError) < 0
                && Math.abs(yError) < 0); // the && could also be ||

        return isInSameQuadrant && brakingDistanceGoesPastTarget;
    }

    // for moveTo - within the error margin
    public boolean isWithinBrakingDistance6() {
//        return (
//            Math.abs(yError) > 0.5 ||
//            Math.abs(xError) > 0.5 ||
//            Math.abs(headingError) > 3
//        );
        return (
            Math.abs(yError) > 0.5 ||
                Math.abs(xError) > 0.5 ||
                Math.abs(headingError) > 3
        );
    }
    // if it has to slow down then jumpt to next movement
    // try reversing the sign on brakiing distance if that is more accurate
//
//    public boolean xTooFar() {
//        // Same sign
//        if (Math.signum(xError) != Math.signum(robot.odometry.xBrakingDistance)) {
//            return false;
//        }
//        //
//        else if (xError < 0 && Math.abs(xError) < robot.odometry.xBrakingDistance) {
//            return true;
//        }
//        else return xError > 0 && xError < Math.abs(robot.odometry.xBrakingDistance);
//    }
//
//    public boolean yTooFar() {
//        // Same sign
//        if (Math.signum(yError) != Math.signum(robot.odometry.yBrakingDistance)) {
//            return false;
//        }
//        //
//        else if (yError < 0 && Math.abs(yError) < robot.odometry.yBrakingDistance) {
//            return true;
//        }
//        else return yError > 0 && yError < Math.abs(robot.odometry.yBrakingDistance);
//    }
//
//    public boolean isWithinBrakingDistance3() {
//        return xTooFar() || yTooFar();
//    }
}
