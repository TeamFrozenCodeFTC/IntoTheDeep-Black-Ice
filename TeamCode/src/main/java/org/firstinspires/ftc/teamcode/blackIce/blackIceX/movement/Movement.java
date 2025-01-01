package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;////package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.blackIceX.ErrorMargin;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.HeadingCorrection;
import org.firstinspires.ftc.teamcode.util.Util;


public abstract class Movement extends HeadingCorrection {
    // have cancel if not moving, and have cancel for teleOp,
    // combine to one func with opModeIsActive()

    // set to position always with intake but just limit power

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

        while (isNotInterrupted() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance && odometry.velocity > 10) {
                drive.brake();
            } else {
                moveTowardTarget(5);
            }

            updatePosition();
        }
    }

    public void holdPositionFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (isNotInterrupted() && timer.seconds() < seconds) {
            updatePosition();
            moveTowardTarget(5);
        }
    }

    /**
     * Moves to the the given position and brakes until traveling at the desired velocity.
     */
    public void quickBrakeTo(double heading, double x, double y, double continuingVelocity) {
        setTarget(heading, x, y);

        while (isNotInterrupted() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance) {
                if (odometry.velocity > continuingVelocity) {
                    drive.brake();
                }
                else {
                    break;
                }
            } else {
                moveTowardTarget(1);
            }

            updatePosition();
        }
    }

    /**
     * Moves to the the given position and continues its momentum.
     */
    public void moveTo(double heading, double x, double y) {
        setTarget(heading, x, y);

        while (isNotInterrupted() && isNotWithinErrorMargin(defaultErrorMargin)) {
            if (distanceToTarget <= odometry.brakingDistance) {
                break;
            }
            moveTowardTarget(1);

            updatePosition();
        }
    }

    public void backIntoWall(double power) {
        // While neither touch sensors are pressed...
        do {
            updatePosition();
            drive.power(applyTurnCorrection(drive.backward(power), turnOverMovement()));
        } while (isNotInterrupted() && !touchRight.isPressed() && !touchLeft.isPressed());
    }

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
        double x1 = xError - odometry.xBrakingDistance * linearSlow;
        double y1 = yError - odometry.yBrakingDistance * linearSlow;

        if (x1/linearSlow < 1) {
            x1 = xError;
        }
        if (y1/linearSlow < 1) {
            y1 = yError;
        }

        double[] powers = fieldVectorToLocalWheelPowers(x1, y1, linearSlow);

        drive.power(applyTurnCorrection(powers, turnOverMovement()));
    }

    public void shiftLeft() {
        // for some reason is the smoothest (only works when going backwards)
        double rotation = odometry.heading - 90;
        double x1 = -(targetX - odometry.x);
        double y1 = (targetY - odometry.y);

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        double x = xx - odometry.xBrakingDistance; // switch to positive for going forwards
        double y = yy - odometry.yBrakingDistance;

        drive.power(applyTurnCorrection(Util.normalize(new double[] {y-x, y+x,y+x, y-x}), turnOverMovement()));
    }
}

