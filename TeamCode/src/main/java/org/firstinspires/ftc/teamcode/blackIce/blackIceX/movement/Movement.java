package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;////package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Util;


public class Movement {
    // have cancel if not moving, and have cancel for teleOp,
    // combine to one func with opModeIsActive()

    // set to position always with intake but just limit power

    Robot robot;
    public TargetTracker target;
    public HeadingCorrection headingCorrection;

    public Movement(Robot robot) {
        this.robot = robot;
        this.target = new TargetTracker(robot);
        this.headingCorrection = new HeadingCorrection(this);
    }

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
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance && robot.odometry.velocity > 10) {
                robot.drive.brake();
            } else {
                moveTowardTarget(5);
            }

            target.updatePosition();
        }
    }

    public void holdPositionFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted() && timer.seconds() < seconds) {
            target.updatePosition();
            moveTowardTarget(5);
        }
    }

    /**
     * Moves to the the given position and brakes until traveling at the desired velocity.
     */
    public void quickBrakeTo(double heading, double x, double y, double continuingVelocity) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance) {
                if (robot.odometry.velocity > continuingVelocity) {
                    robot.drive.brake();
                }
                else {
                    break;
                }
            } else {
                moveTowardTarget(1);
            }

            target.updatePosition();
        }
    }

    /**
     * Moves to the the given position and continues its momentum.
     */
    public void moveTo(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance) {
                break;
            }
            moveTowardTarget(1);

            target.updatePosition();
        }
    }

    public void backIntoWall(double power) {
        // While neither touch sensors are pressed...
        do {
            target.updatePosition();
            robot.drive.power(headingCorrection.applyTurnCorrection(robot.drive.backward(power), headingCorrection.turnOverMovement()));
        } while (robot.isNotInterrupted() && !robot.touchRight.isPressed() && !robot.touchLeft.isPressed());
    }

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public double[] fieldVectorToLocalWheelPowers(double x, double y, double divisor) {
        // positive heading is counterclockwise
        double heading = Math.toRadians(robot.odometry.heading);
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
        double x1 = target.xError - robot.odometry.xBrakingDistance * linearSlow;
        double y1 = target.yError - robot.odometry.yBrakingDistance * linearSlow;

        // combine projected powers target.xError - robot.odometry.xBrakingDistance
        // target.yError - robot.odometry.yBrakingDistance
        // 

        // test without this
        if (x1/linearSlow < 1) {
            x1 = target.xError;
        }
        if (y1/linearSlow < 1) {
            y1 = target.yError;
        }

        double[] powers = fieldVectorToLocalWheelPowers(x1, y1, linearSlow);

        robot.drive.power(headingCorrection.applyTurnCorrection(powers, headingCorrection.turnOverMovement()));
    }

    public void shiftLeft() {
        // for some reason is the smoothest (only works when going backwards)
        double rotation = robot.odometry.heading - 90;
        double x1 = -target.xError;
        double y1 = target.yError;

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        double x = xx - robot.odometry.xBrakingDistance; // switch to positive for going forwards
        double y = yy - robot.odometry.yBrakingDistance;

        robot.drive.power(headingCorrection.applyTurnCorrection(Util.normalize(new double[] {y-x, y+x,y+x, y-x}), headingCorrection.turnOverMovement()));
    }
}

