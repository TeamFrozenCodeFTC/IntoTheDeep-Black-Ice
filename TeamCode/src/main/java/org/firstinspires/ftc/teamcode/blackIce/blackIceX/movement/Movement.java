package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;////package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.teamcode.blackIce.blackIceX.ErrorMargin;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.HeadingCorrection;
import org.firstinspires.ftc.teamcode.util.Util;


public abstract class Movement extends HeadingCorrection {
    // have cancel if not moving, and have cancel for teleOp,
    // combine to one func with opModeIsActive()
    /**
     * Moves to the the given position and stops.
     * <p>
     * 1. Accelerates
     * 2. Brakes
     * 3. Adjusts
     * <p>
     * Useful for stopping a precise location.
     */
    public void stopAtPosition(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance && odometry.velocity > 10) {
                brake();
            } else {
                moveTowardTarget(5);
            }

            updatePosition();
        }
    }

    /**
     * Moves to the the given position and brakes until traveling at the desired velocity.
     */
    public void quickBrakeTo(double heading, double x, double y, double continuingVelocity) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance) {
                if (odometry.velocity > continuingVelocity) {
                    brake();
                }
                else {
                    break;
                }
            } else {
                moveTowardTarget(5);
            }

            updatePosition();
        }
    }

    /**
     * Moves to the the given position and continues its momentum.
     */
    public void moveTo(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance) {
                break;
            }
            moveTowardTarget(1);

            updatePosition();
        }
    }

//    public void backIntoWall() {
//        // While neither touch sensors are pressed...
//        do {
//            targetY = 24+24-(24-18);
//            updatePosition();
//            goStraight(-0.3);
//        } while (!touchRight.isPressed() && !touchLeft.isPressed());
//    }

     /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the slide
     * and forwards/backwards.
     */
    public double[] fieldVectorToLocalWheelPowers(double x, double y, double divisor) {
        // positive heading is counterclockwise
        double heading = Math.toRadians(odometry.heading);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double localForwards = (x * cos + y * sin) / divisor; // clockwise rotation
        double localSlide = (-x * sin + y * cos) / divisor;

        return Util.normalize(new double[]
                {localForwards-localSlide, localForwards+localSlide,
                        localForwards+localSlide, localForwards-localSlide}
        );
    }

    public void moveTowardTarget(double linearSlow) {
        double currentXVel = Math.signum(odometry.xVelocity) * (
                0.00130445 * Math.pow(odometry.xVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
        double currentYVel = Math.signum(odometry.yVelocity) * (
                0.00130445 * Math.pow(odometry.yVelocity, 2)
                        + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);

//        double estimatedStoppedX;
//        double estimatedStoppedY;
//
//        // Makes sure the velocity doesn't overpower
//        if (Math.abs(currentXVel) < Math.abs(xError) * linearSlow / 2) {
//            estimatedStoppedX = (odometry.x + currentXVel * linearSlow);
//        }
//        else {
//            estimatedStoppedX = odometry.x;
//        }
//        if (Math.abs(currentYVel) < Math.abs(yError) * linearSlow / 2) {
//            estimatedStoppedY = (odometry.y + currentYVel * linearSlow);
//        }
//        else {
//            estimatedStoppedY = odometry.y;
//        }
        double x1 = xError - currentXVel * linearSlow;
        double y1 = yError - currentYVel * linearSlow;

        if (x1/linearSlow < 1) {
            x1 = xError;
        }
        if (y1/linearSlow < 1) {
            y1 = yError;
        }

        double[] powers = fieldVectorToLocalWheelPowers(x1, y1, linearSlow);

        powerWheels(applyTurnCorrection(powers));}
//    public void moveTowardTarget(double linearSlow) {
//
//
//
//        double x1 = xError;
//        double y1 = yError;
//
//        double[] powers = fieldVectorToLocalWheelPowers(x1, y1);
//
//        powerWheels(applyTurnCorrection(powers));
//    }

}
