package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;

public class Drive {
    Robot robot;

    public Drive(Robot op) {
        this.robot = op;
    }

    public void power(double[] powers) {
        robot.frontLeftWheel.setPower(powers[0]);
        robot.backLeftWheel.setPower(powers[1]);
        robot.frontRightWheel.setPower(powers[2]);
        robot.backRightWheel.setPower(powers[3]);
    }

    public void brake() {
        power(forward(0));
    }

    public double[] forward(double power) {
        return new double[] {power, power, power, power};
    }

    public double[] backward(double power) {
        return forward(-power);
    }

    public double[] slideLeft(double power) {
        return new double[] {power, -power, -power, power};
    }

    public double[] slideRight(double power) {
        return slideLeft(-power);
    }

    public double[] turnClockwise(double power) {
        return new double[] {power, power, -power, -power};
    }

    public double[] turnCounterclockwise(double power) {
        return turnClockwise(-power);
    }

    public double[] combine(double[] powers1, double[] powers2) {
        return normalize(new double[] {
                powers1[0] + powers2[0], powers1[1] + powers2[1],
                powers1[2] + powers2[2], powers1[3] + powers2[3]
        });
    }

    public double[] combineMax(double[] powers1, double[] powers2, double max) {
        return downScaleTo(max,
            new double[] {
            powers1[0] + powers2[0], powers1[1] + powers2[1],
            powers1[2] + powers2[2], powers1[3] + powers2[3]
        });
    }

    public double[] multiply(double[] powers1, double multiplier) {
        return normalize(new double[] {
                powers1[0] * multiplier, powers1[1] * multiplier,
                powers1[2] * multiplier, powers1[3] * multiplier
        });
    }

    public void brakeFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted() && timer.seconds() < seconds) {
            brake();
        }
    }

    private long stuckStartTime = 0; // Time when the robot starts potentially being stuck
    private final long STUCK_THRESHOLD_TIME = 250; // Time in milliseconds to determine if stuck
    private final double MIN_VELOCITY = 1; // Minimum velocity to consider the robot "moving"

    public boolean isStuck() {
        boolean wheelsMoving = wheelsAreMoving();
        double currentVelocity = robot.odometry.velocity;

        if (currentVelocity < MIN_VELOCITY && wheelsMoving) {
            if (stuckStartTime == 0) {
                // Record the time when the robot might be stuck
                stuckStartTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - stuckStartTime > STUCK_THRESHOLD_TIME) {
                // If stuck for longer than the threshold, return true
                return true;
            }
        } else {
            // Reset the timer if conditions for "stuck" are not met
            stuckStartTime = 0;
        }
        return false;
    }

    public boolean wheelsAreMoving() {
        return robot.frontLeftWheel.getPower() != 0 ||
                robot.backLeftWheel.getPower() != 0 ||
                robot.frontRightWheel.getPower() != 0 ||
                robot.backRightWheel.getPower() != 0;
    }

    private double[] normalize(double[] powers) {
        double maxPower = 1.0;
        for (double value : powers) {
            maxPower = Math.max(maxPower, Math.abs(value));
        }

        return new double[]{
                powers[0] / maxPower,
                powers[1] / maxPower,
                powers[2] / maxPower,
                powers[3] / maxPower
        };
    }

    public double[] downScaleTo(double maxScaleTo, double[] powers) {
        double maxPower = 0;
        for (double value : powers) {
            maxPower = Math.max(maxPower, Math.abs(value));
        }

        // Avoid upscaling array
        if (maxPower <= maxScaleTo) {
            return powers;
        }

        double scaleDownBy = maxScaleTo / maxPower;

        for (int i = 0; i < powers.length; i++) {
            powers[i] *= scaleDownBy;
        }

        return powers;
    }



    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public double[] fieldVectorToLocalWheelPowers(double x, double y) {
        // positive heading is counterclockwise
        double heading = Math.toRadians(robot.odometry.heading);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double localForwards = (x * cos + y * sin); // clockwise rotation
        double localSlide = (-x * sin + y * cos);

        return Util.normalize(new double[]
                {localForwards-localSlide, localForwards+localSlide,
                 localForwards+localSlide, localForwards-localSlide}
        );
    }
}