package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public final class Target {
    public static ErrorMargin defaultErrorMargin = new ErrorMargin(.5, .5, 3);

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
        heading = targetHeading;

        setTarget(targetX, targetY);
    }

    public static void setTarget(double targetX, double targetY) {
        previousX = x;
        previousY = y;

        x = targetX;
        y = targetY;

        xDelta = x - previousX;
        yDelta = y - previousY;

        updatePosition();
    }

    static double headingCos;
    static double headingSin;

    // Target - previousTarget
    public static double yDelta;
    public static double xDelta;

    public static void updatePosition() {
        Odometry.update();
        headingError = Util.simplifyAngle(heading - Odometry.heading);
        xError = x - Odometry.x;
        yError = y - Odometry.y;

        double headingRadians = Math.toRadians(Odometry.heading);
        headingCos = Math.cos(headingRadians);
        headingSin = Math.sin(headingRadians);
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
