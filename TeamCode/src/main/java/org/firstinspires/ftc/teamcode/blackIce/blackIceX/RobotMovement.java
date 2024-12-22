package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


// TODO path system, better turning
public abstract class RobotMovement extends Robot {
    final int TILE = 24;
    final int ROBOT = 18;
    final int HALF_OF_ROBOT = ROBOT / 2;
    final int EDGE_OF_TILE = TILE - ROBOT;

    public double targetHeading;
    public double targetX;
    public double targetY;

    public double headingError;
    public double xError;
    public double yError;
    public double distanceToTarget;

    public double powerMult = 1;

    final double MAX_POWER = 1;
    final double MAX_TURN_POWER = 1;
    final double HEADING_POWER = 0.02;

    final double LINEAR_INCH_SLOW_DOWN = 5;
    // if robot ever gets stuck maybe add a I in PID

    final double CONTROLLABLE_VELOCITY = 15;

    public ErrorMargin defaultErrorMargin = new ErrorMargin(2, 0.5, 0.5);

    public ErrorMargin wideErrorMargin = new ErrorMargin(5, 1.5, 1.5);

    /**
     * Moves the robot straight with a set power for the given seconds. (Includes turn correction).
     */
    public void goStraightForSeconds(double seconds, double power) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updatePosition();
            goStraight(power);
        }
    }

    /**
     * Brakes the robot for the given seconds.
     */
    public void brakeForSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updatePosition();
            brake();
        }
    }

    /**
     * Moves the robot to a given position and keeps its momentum for the next step.
     */
    public void goThroughPosition(double heading, double x, double y) {
        targetHeading = heading;
        targetX = x;
        targetY = y;

        updatePosition();

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            goTowardTarget();
            updatePosition();
        }
    }

    /**
     * Moves the robot to a given position and brakes.
     */
    public void brakeToPosition(double heading, double x, double y, ErrorMargin errorMargin) {
        brakeToPosition_(heading, x, y, errorMargin);
    }

    /**
     * Moves the robot to a given position and brakes.
     */
    public void brakeToPosition(double heading, double x, double y) {
        brakeToPosition_(heading, x, y, defaultErrorMargin);
    }

    /**
     * Locks the robot at the last target pose for the given seconds.
     */
    public void holdFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updatePosition();
            goTowardTarget();
        }

        updatePosition();
    }

    private void brakeToPosition_(double heading, double x, double y, ErrorMargin errorMargin) {
        targetHeading = heading;
        targetX = x;
        targetY = y;

        updatePosition();

        while (opModeIsActive() && isNotWithinErrorMargin(errorMargin)) {
            double stoppingDistance = estimateStoppingDistance();

            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
                // try braking using negative powers
                brake();
            } else {
                goTowardTarget();
            }

            updatePosition();
        }
    }

    public double estimateStoppingDistance() {
        return 0.00130445 * Math.pow(odometry.velocity, 2) + 0.0644448 * odometry.velocity + 0.0179835;
        // 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498;
    }

    public double getTurnCorrection() {
        return Math.min(
                MAX_TURN_POWER,
                headingError * HEADING_POWER);
    }

    public double getTurn() { // TODO test
        double estimatedTime = distanceToTarget / odometry.velocity;
        // headingVelocity should
        // estimatedTime = headingError / x
        // x = headingError / estimatedTime
        // 20degrees / 2seconds

        double targetHeadingVelocity = headingError / estimatedTime;
        double headingVelocityError = (targetHeadingVelocity - odometry.headingVelocity);

        return headingVelocityError * 0.05;
    }

    public boolean isNotWithinErrorMargin(ErrorMargin errorMargin) {
        return (
            Math.abs(yError) > errorMargin.y ||
            Math.abs(xError) > errorMargin.x ||
            Math.abs(headingError) > errorMargin.degrees
        );
    }

    public void updatePosition() {
        odometry.update();
        headingError = simplifyAngle(targetHeading - odometry.heading);
        xError = targetX - odometry.x;
        yError = targetY - odometry.y;
        distanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
    }

    public void brake() {
        setAllWheelPowersTo(0);
    }

    public void setAllWheelPowersTo(double power) {
        frontLeftWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
        frontRightWheel.setPower(power);
    }


    private void goTowardTarget() {
        powerWheels(applyCorrection(normalize(localToGlobal(
                odometry.heading - 90 ,// + 90, // +90 for starting orientation of hub
                -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN,
                (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN
        ))));
    }

    public void forceTowardTarget() {
        double rotation = odometry.heading - 90;
        double x1 = -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN;
        double y1 = (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN;

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        double x;
        double y;

        if (odometry.velocity > 10 && odometry.xVelocity > odometry.yVelocity) {
            x = xx;
            // (totalY / totalX + ?) = (x/y)
            y = yy + ((yy/xx) * (xx+odometry.xVelocity) - (yy+odometry.yVelocity));
        }
        else {
            x = yy + ((xx/yy) * (yy+odometry.yVelocity) - (xx+odometry.xVelocity));
            y = yy;
        }

        powerWheels(applyCorrection(normalize(new double[] {y-x, y+x,y+x, y-x})));
    }

    public double[] applyCorrection(double[] powers) {
        double turnCorrection = getTurnCorrection();

        for (int i = 0; i < powers.length; i++) {
            double total = powers[i];
            if (i < 2) {
                total -= turnCorrection;
            }
            else {
                total += turnCorrection;
            }
            powers[i] = clampPower(total) * MAX_POWER * powerMult;
        }

        return normalize(powers);
    }

    public void goStraight(double power) {
        powerMult = power;
        goTowardTarget();
        powerMult = 1;
    }

    public void powerWheels(double[] powers) {
        frontLeftWheel.setPower(powers[0]);
        backLeftWheel.setPower(powers[1]);
        frontRightWheel.setPower(powers[2]);
        backRightWheel.setPower(powers[3]);
    }

    public double[] localToGlobal(double rotation, double x1, double y1) {
        // Gives the powers of wheels to go to a certain vector

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        return new double[]{yy - xx, yy + xx, yy + xx, yy - xx};
    }

    private double clampPower(double x) {
        return Math.max(-1, Math.min(x, 1));
    }

    public double[] normalize(double[] a) {
        double maxPower = 1.0;
        for (double value : a) {
            maxPower = Math.max(maxPower, Math.abs(value));
        }

        return new double[]{
                a[0] / maxPower,
                a[1] / maxPower,
                a[2] / maxPower,
                a[3] / maxPower
        };
    }

    private double simplifyAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}