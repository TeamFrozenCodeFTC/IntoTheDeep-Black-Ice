package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


// TODO path system, better turning
public abstract class RobotMovement extends Robot {
    final int TILE = 24;
    final int ROBOT = 18;
    final int HALF_OF_ROBOT = ROBOT / 2;
    final int EDGE_OF_TILE = TILE - ROBOT;
    final double ROBOT_TURN_RADIUS = Math.sqrt(Math.pow(HALF_OF_ROBOT, 2) * 2);

    final double LINEAR_INCH_SLOW_DOWN = 5;
    final double CONTROLLABLE_VELOCITY = 5;

    final double MAX_POWER = 1;
    final double MAX_TURN_POWER = 1;
    final double HEADING_POWER = 0.02;

    public double targetHeading = 0;
    public double targetX;
    public double targetY;

    public double headingError;
    public double xError;
    public double yError;
    public double distanceToTarget;

    public double totalDistanceToTarget;
    public double previousHeading;

    public double powerMult = 1;

    public ErrorMargin defaultErrorMargin = new ErrorMargin(2, 0.5, 0.5);

    public ErrorMargin wideErrorMargin = new ErrorMargin(5, 1.5, 1.5);

    public void setTarget(double heading, double x, double y) {
        previousHeading = targetHeading;

        targetHeading = heading;
        targetX = x;
        targetY = y;

        updatePosition();

        totalDistanceToTarget = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
    }

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
     * Moves the robot straight with a set power for the given seconds. (Includes turn correction).
     */
    public void slideForSeconds(double seconds, double power) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updatePosition();
            slide(power);
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
        setTarget(heading, x, y);

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
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(errorMargin)) {
            double stoppingDistance = odometry.brakingDistance;

            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
                // try braking using negative powers
                brake();
            } else {
                forceTowardTarget2(0, 5);
            }

            updatePosition();
        }
    }

    public void quickBrakeTo(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            double stoppingDistance = odometry.brakingDistance;

            if (distanceToTarget <= stoppingDistance) {
                if (odometry.velocity > 5) {
                    // try braking using negative powers
                    brake();
                }
                // else!?
                else {
                    return;
                }
            } else {
                forceTowardTarget2(0, 5);
            }

            updatePosition();
        }
    }

    public void quickBrakeTo2(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            double stoppingDistance = odometry.brakingDistance;

            if (distanceToTarget <= stoppingDistance) {
                if (odometry.velocity > 10) {
                    // try braking using negative powers
                    brake();
                }
                // else!?
                else {
                    return;
                }
            } else {
                forceTowardTarget2(0, 1);
            }

            updatePosition();
        }
    }

    public double getTurnCorrection() {
        return Math.min(
                MAX_TURN_POWER,
                headingError * HEADING_POWER);
    }

    public double getTurn() {
        double turnPower = headingError * 0.03;

        // Ensure smooth turning proportional to progress toward the target
        double progressFactor = Math.min(1, (totalDistanceToTarget - distanceToTarget) / totalDistanceToTarget + 0.2);
        turnPower *= progressFactor; // Minimum effort to keep turning

        return turnPower;

//        if (Math.abs(headingError) < 10) { // Adjust threshold as needed
//            //return 0; // No correction if error is negligible
//            return headingError * 0.03;
//        }
//
//        double estimatedTime = distanceToTarget / odometry.velocity;
//        double targetHeadingVelocity = headingError / estimatedTime;
//        double headingVelocityError = (targetHeadingVelocity - odometry.headingVelocity);
//
//        return headingVelocityError * 0.01; // was 0.01
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

    public void goTowardTarget() {
        powerWheels(applyCorrection(normalize(localToGlobal(
                odometry.heading - 90 ,// + 90, // +90 for starting orientation of hub
                -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN,
                (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN
        ))));
    }

    public void smoothVelToTarget() {
        // for some reason is the smoothest (only works when going backwards)
        double rotation = odometry.heading - 90;
        double x1 = -(targetX - odometry.x);
        double y1 = (targetY - odometry.y);

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        double currentXVel = odometry.xBrakingDistance;
        double currentYVel = odometry.yBrakingDistance;

        double x = xx - currentXVel; // switch to positive for going forwards
        double y = yy - currentYVel;

        powerWheels(applyCorrection(normalize(new double[] {y-x, y+x,y+x, y-x})));
    }

    public void forceTowardTarget2(double velocityFactor, double linearFactor) {
        double rotation = -odometry.heading;

        double currentXVel = Math.signum(odometry.xVelocity) *(
                0.00130445 * Math.pow(odometry.xVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
        double currentYVel = Math.signum(odometry.yVelocity) * (
                0.00130445 * Math.pow(odometry.yVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);

//        if (currentXVel >= currentYVel) {
//            currentXVel -= currentYVel;
//            currentYVel = 0;
//        }
//        else {
//            currentYVel -= currentXVel;
//            currentXVel = 0;
//        }
        double addToY = 0;
        double addToX = 0;
        if (currentXVel > currentYVel) {
            // (y/x)=(yv+?/xv)
            addToY = ((targetY - odometry.y)/(targetX - odometry.x))*currentXVel-currentYVel;
        }
        else {
            addToX = ((targetY - odometry.y)/(targetX - odometry.x))*currentXVel-currentYVel;
        }

        // if x is greater)


        // switch and add?
//        double x1 = (targetX - odometry.x) - currentXVel * velocityFactor;
//        double y1 = (targetY - odometry.y) - currentYVel * velocityFactor;
        double x1 = (targetX - odometry.x) + addToX * velocityFactor;
        double y1 = (targetY - odometry.y) + addToY * velocityFactor;

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double x = (x1 * cos - y1 * sin) / linearFactor;
        double y = (x1 * sin + y1 * cos) / linearFactor;

        powerWheels(applyCorrection(normalize(new double[] {x-y, x+y, x+y, x-y})));
//        double rotation = -odometry.heading;
//
//        double currentXVel = Math.signum(odometry.xVelocity) *(
//                0.00130445 * Math.pow(odometry.xVelocity, 2)
//                        + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
//        double currentYVel = Math.signum(odometry.yVelocity) * (
//                0.00130445 * Math.pow(odometry.yVelocity, 2)
//                        + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);
//
////        if (currentXVel >= currentYVel) {
////            currentXVel -= currentYVel;
////            currentYVel = 0;
////        }
////        else {
////            currentYVel -= currentXVel;
////            currentXVel = 0;
////        }
//        if (currentXVel >= currentYVel) {
//            currentXVel -= currentYVel;
//            currentYVel = 0;
//            // (y/x)=(yv+?/xv)
//            // addToY = (y/x)*xv-yv
//        }
//        else {
//            currentYVel -= currentXVel;
//            currentXVel = 0;
//        }
//
//        // if x is greater)
//
//
//        // switch and add?
////        double x1 = (targetX - odometry.x) - currentXVel * velocityFactor;
////        double y1 = (targetY - odometry.y) - currentYVel * velocityFactor;
//        double x1 = (targetX - odometry.x) + currentYVel * velocityFactor;
//        double y1 = (targetY - odometry.y) + currentXVel * velocityFactor;
//
//        double cos = Math.cos(Math.toRadians(rotation));
//        double sin = Math.sin(Math.toRadians(rotation));
//        double x = (x1 * cos - y1 * sin) / linearFactor;
//        double y = (x1 * sin + y1 * cos) / linearFactor;
//
//        powerWheels(applyCorrection(normalize(new double[] {x-y, x+y, x+y, x-y})));

    }

    public void forceTowardTarget3() {
        double rotation = -odometry.heading;

        double currentXVel = Math.signum(odometry.xVelocity) * (
                0.00130445 * Math.pow(odometry.xVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
        double currentYVel = Math.signum(odometry.yVelocity) * (
                0.00130445 * Math.pow(odometry.yVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);

        double x1 = targetX - (odometry.x - currentXVel*5);
        double y1 = targetY - (odometry.y);

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double x = (x1 * cos - y1 * sin) / 5;
        double y = (x1 * sin + y1 * cos) / 5;

        powerWheels(applyCorrection(normalize(new double[] {x-y, x+y, x+y, x-y})));
    }

    public void directionChange(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            //forceTowardTarget2(1, 5); // 5?
            //forceTowardTarget3();
            smoothVelToTarget();
            updatePosition();
        }
    }

    public void accelerate(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            forceTowardTarget2(0, 5);
            updatePosition();
        }
    }

    public void accelerate2(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            if (distanceToTarget < odometry.brakingDistance) {
                return;
            }
            forceTowardTarget2(0, 5);
            updatePosition();
        }
    }

    public double[] applyCorrection(double[] powers) {
        double turnCorrection = getTurn();

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
//        powerMult = power;
//        goTowardTarget();
//        powerMult = 1;
        powerWheels(applyCorrection((new double[] {power, power, power, power})));
    }

    public void slide(double power) {
        powerWheels(applyCorrection((new double[] {power, -power, -power, power})));
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

    public double clampPower(double x) {
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

//    public double simplifyAngle(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
    public double simplifyAngle(double angle) {
        angle = (angle + 180) % 360;  // Shift range to 0-360
        if (angle < 0) angle += 360; // Handle negative modulo result
        return angle - 180;          // Shift range back to -180 to 180
    }

    public void rotateToTarget(double heading2, double x2, double y2) {
        setTarget(heading2, x2, y2);

        while (opModeIsActive()) {
            if (distanceToTarget <= odometry.brakingDistance) {
                if (odometry.velocity > 5) {
                    // try braking using negative powers
                    updatePosition();
                    brake();
                    continue;
                }
                else {
                    return;
                }
            }
            double rotation = -odometry.heading;

            double x1 = (targetX - odometry.x);
            double y1 = (targetY - odometry.y);

            double cos = Math.cos(Math.toRadians(rotation));
            double sin = Math.sin(Math.toRadians(rotation));
            double x = (x1 * cos - y1 * sin) / 5;
            double y = (x1 * sin + y1 * cos) / 5;

            double[] powers = normalize(new double[] {x-y, x+y, x+y, x-y});

            double targetAngleCorrection;

            if (distanceToTarget < 25) {
                targetAngleCorrection = getTurnCorrection();
            }
            else {
                double degrees = Math.toDegrees(Math.atan2(y1, x1)) - targetHeading + 90;
                targetAngleCorrection = simplifyAngle(degrees - odometry.heading) * 0.015;
                telemetry.addData("targetAngleCorrection", targetAngleCorrection);
                telemetry.addData("targetAngle", degrees);
                telemetry.addData("heading", odometry.heading);
                telemetry.update();
            }

            for (int i = 0; i < powers.length; i++) {
                double total = powers[i];
                if (i < 2) {
                    total -= targetAngleCorrection;
                }
                else {
                    total += targetAngleCorrection;
                }
                powers[i] = clampPower(total) * MAX_POWER * powerMult;
            }

            powerWheels(normalize(powers));

            updatePosition();
        }
    }
}