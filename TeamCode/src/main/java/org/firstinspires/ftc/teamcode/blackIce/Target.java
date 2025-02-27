package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public final class Target {
    public static ErrorMargin defaultErrorMargin = new ErrorMargin(1, 1, 3);

    public static double heading;
    public static double x;
    public static double y;

    public static double previousX;
    public static double previousY;

    public static double headingError;
    public static double xError;
    public static double yError;

    public static double totalDistanceToTarget;
    public static double previousHeading;

    public static void setTarget(double targetHeading, double targetX, double targetY) {
        previousHeading = heading;
        previousX = x;
        previousY = y;

        heading = targetHeading;
        x = targetX;
        y = targetY;

        totalDistanceToTarget = Util.getVectorMagnitude(previousX - x, previousY - y);

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
}
