package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

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

    public void brakeFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (robot.isNotInterrupted() && timer.seconds() < seconds) {
            brake();
        }
    }

    public boolean isStuck() {
        return robot.odometry.velocity == 0 && wheelsAreMoving();
    }

    public boolean wheelsAreMoving() {
        return robot.frontLeftWheel.getPower() != 0 ||
                robot.backLeftWheel.getPower() != 0 ||
                robot.frontRightWheel.getPower() != 0 ||
                robot.frontRightWheel.getPower() != 0;
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
}