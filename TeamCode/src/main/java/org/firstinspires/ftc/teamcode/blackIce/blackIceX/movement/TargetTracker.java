package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.ErrorMargin;
import org.firstinspires.ftc.teamcode.util.Util;

public class TargetTracker {
    Robot robot;

    public TargetTracker(Robot robot) {
        this.robot = robot;
    }

    public ErrorMargin defaultErrorMargin = new ErrorMargin(2, 0.5, 0.5);
//    public ErrorMargin wideErrorMargin = new ErrorMargin(5, 1.5, 1.5);

    public double heading;
    public double x;
    public double y;

    public double headingError;
    public double xError;
    public double yError;
    public double distanceToTarget;

    public double totalDistanceToTarget;
    public double previousHeading;

    public void setTarget(double targetHeading, double targetX, double targetY) {
        previousHeading = this.heading;

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
    }

    public boolean isNotWithinErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError) > errorMargin.y ||
            Math.abs(xError) > errorMargin.x ||
            Math.abs(headingError) > errorMargin.degrees
        );
    }
}
