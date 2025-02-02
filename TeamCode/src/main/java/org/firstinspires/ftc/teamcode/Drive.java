package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public void zeroPowerFloat() {
        robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void zeroPowerBrake() {
        robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void zeroPower() {
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
            zeroPower();
        }
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