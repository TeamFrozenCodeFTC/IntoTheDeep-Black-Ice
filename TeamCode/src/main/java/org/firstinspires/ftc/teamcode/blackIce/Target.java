package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public final class Target {
    public static ErrorMargin defaultErrorMargin = new ErrorMargin(1, 1, 3);
    // make stopAtPosition wait until velocity is less than x instead of based of error margin
    // + help if gets stuck

    public static double heading;
    public static double x;
    public static double y;

    public static double previousX;
    public static double previousY;

    public static double headingError;
    public static double xError;
    public static double yError;
    public static double distanceToTarget;

    public static double totalDistanceToTarget;
    public static double previousHeading;

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

    public static void setTarget(double targetHeading, double targetX, double targetY) {
        previousHeading = heading;
        previousX = x;
        previousY = y;

        heading = targetHeading;
        x = targetX;
        y = targetY;

        updatePosition();
    }

    public static void updatePosition() {
        Odometry.update();
        headingError = Util.simplifyAngle(heading - Odometry.heading);
        xError = x - Odometry.x;
        yError = y - Odometry.y;
        //distanceToTarget = 100;
    }

    public static boolean isNotWithinErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError) > errorMargin.y ||
            Math.abs(xError) > errorMargin.x ||
            Math.abs(headingError) > errorMargin.degrees
        );
    }

    public static boolean isWithinBrakingErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError - Odometry.yBrakingDistance) < errorMargin.y &&
            Math.abs(xError - Odometry.xBrakingDistance) < errorMargin.x &&
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

}
