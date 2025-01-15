package org.firstinspires.ftc.teamcode.blackIce;////package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Util;


public class Movement {
    Robot robot;
    public TargetTracker target;
    public HeadingCorrection headingCorrection;

    public Movement(Robot robot) {
        this.robot = robot;
        this.target = new TargetTracker(robot);
        this.headingCorrection = new HeadingCorrection(this);
    }

    public void holdPosition() {
        totalErrorX += target.xError;
        totalErrorY += target.yError;

        double X = totalErrorX * 0 + target.xError * 0.5;
        double Y = totalErrorY * 0 + target.yError * 0.5;

        double[] powers = fieldVectorToLocalWheelPowers(X, Y, 1);

        robot.drive.power(headingCorrection.applyTurnCorrection(powers, headingCorrection.turnOverMovement()));
    }

    private double totalErrorX = 0;
    private double totalErrorY = 0;
    public void stopAtPosition(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted()) {
            if (robot.drive.isStuck()) {
                robot.telemetry.addData("Robot was stuck", "exited movement");
                robot.telemetry.update();
                break;
            }
            else if (target.isWithinErrorMargin(target.defaultErrorMargin) && robot.odometry.velocity < 1) {
                robot.telemetry.addData("Robot reached its position", "exited");
                robot.telemetry.update();
                break;
            }
            else if (target.distanceToTarget <= robot.odometry.brakingDistance && robot.odometry.velocity > 5) {
                robot.telemetry.addData("Robot is", "braking");
                robot.telemetry.update();

                robot.drive.brake();
                robot.loopUpdate();
                continue;
            }

            holdPosition();

            robot.drive.brake();
            robot.loopUpdate();
        }
        totalErrorX = 0;
        totalErrorY = 0;
    }
//
//    private double totalErrorX = 0;
//    private double totalErrorY = 0;
    public void stopAtPositionPI(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (timer.seconds() > 2 && robot.odometry.velocity < 0.5) {
                break;
            }
            if (target.distanceToTarget <= robot.odometry.brakingDistance) {
                if (robot.odometry.velocity > 10) {
                    robot.drive.brake();
                }
                else {
                    totalErrorX += target.xError;
                    totalErrorY += target.yError;

                    double X = totalErrorX * 0.01 + target.xError * 0.1;
                    double Y = totalErrorY * 0.01 + target.yError * 0.1;

                    double[] powers = fieldVectorToLocalWheelPowers(X, Y, 1);

                    robot.drive.power(headingCorrection.applyTurnCorrection(powers, headingCorrection.turnOverMovement()));
                }
            }
            else {
                moveTowardTarget(1, headingCorrection.turnOverMovement());
            }

            target.updatePosition();
        }
        totalErrorX = 0;
        totalErrorY = 0;
        robot.drive.brake();
    }

    public void stopAtPositionOnWall(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin) && !robot.drive.isStuck()) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance && robot.odometry.velocity > 10) {
                robot.drive.brake();
            } else {
                moveTowardTarget(5, headingCorrection.turnOverMovement());
            }

            robot.loopUpdate();
        }
    }

    public void holdPositionFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted() && timer.seconds() < seconds) {
            robot.loopUpdate();
            moveTowardTarget(5, headingCorrection.locked());
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
                moveTowardTarget(1, headingCorrection.turnOverMovement());
            }

            robot.loopUpdate();
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
            moveTowardTarget(1, headingCorrection.turnOverMovement());

            robot.loopUpdate();
        }
    }

    public void movePast(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            moveTowardTarget(1, headingCorrection.turnOverMovement());

            robot.loopUpdate();
        }
    }

    public void moveAgainstWall(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance || robot.drive.isStuck()) {
                break;
            }
            moveTowardTarget(1, headingCorrection.turnOverMovement());

            robot.loopUpdate();
        }
    }

    /**
     * Moves to the the given position and continues its momentum.
     */
    public void turnAndMoveTo(double heading, double x, double y) {
        target.setTarget(heading, x, y);

        while (robot.isNotInterrupted() && target.isNotWithinErrorMargin(target.defaultErrorMargin)) {
            if (target.distanceToTarget <= robot.odometry.brakingDistance) {
                break;
            }
            moveTowardTarget(1, headingCorrection.locked());

            robot.loopUpdate();
        }
    }

    public void backIntoWall(double power) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        // While neither touch sensors are pressed...
        do {
            robot.loopUpdate();
            robot.drive.power(headingCorrection.applyTurnCorrection(robot.drive.backward(power), headingCorrection.turnOverMovement()));
        } while (timer.seconds() < 0.5 && robot.isNotInterrupted() && !robot.touchRight.isPressed() && !robot.touchLeft.isPressed());
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

    public void moveTowardTarget(double linearSlow, double turnMethod) {
        double x1 = target.xError - robot.odometry.xBrakingDistance * linearSlow;
        double y1 = target.yError - robot.odometry.yBrakingDistance * linearSlow;


        // test without this because of early exit
        if (x1/linearSlow < 1) {
            x1 = target.xError;
        }
        if (y1/linearSlow < 1) {
            y1 = target.yError;
        }

        double[] powers = fieldVectorToLocalWheelPowers(x1, y1, linearSlow);

        robot.drive.power(headingCorrection.applyTurnCorrection(powers, turnMethod));
    }
}

