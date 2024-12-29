package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.ErrorMargin;
import org.firstinspires.ftc.teamcode.util.Util;

public abstract class TargetTracker extends Robot {
    public ErrorMargin defaultErrorMargin = new ErrorMargin(2, 0.5, 0.5);
//    public ErrorMargin wideErrorMargin = new ErrorMargin(5, 1.5, 1.5);

    public double targetHeading;
    public double targetX;
    public double targetY;

    public double headingError;
    public double xError;
    public double yError;
    public double distanceToTarget;

    public double totalDistanceToTarget;
    public double previousHeading;

    public void setTarget(double heading, double x, double y) {
        previousHeading = targetHeading;

        targetHeading = heading;
        targetX = x;
        targetY = y;

        updatePosition();

        totalDistanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
    }

    public void updatePosition() {
        odometry.update();
        headingError = Util.simplifyAngle(targetHeading - odometry.heading);
        xError = targetX - odometry.x;
        yError = targetY - odometry.y;
        distanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
    }

    public boolean isNotWithinErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError) > errorMargin.y ||
            Math.abs(xError) > errorMargin.x ||
            Math.abs(headingError) > errorMargin.degrees
        );
    }
}
