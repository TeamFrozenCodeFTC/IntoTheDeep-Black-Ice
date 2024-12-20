package org.firstinspires.ftc.teamcode.blackIceX;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

public abstract class BlackIce extends Robot {

    static final double MAX_POWER = 1;
    static final double MAX_TURN_POWER = 1;

    /**
     * Amount of power applied per degree of rotation off
     */
    static final double TURN_POWER = 0.03;

    /**
     * After how many inches where it starts slowing down
     */
    static final double SLOW_FACTOR = 15;

//    0: 66.014 9.815
//            1: 60.088 8.878
//            2: 53.388 7.167
//            3: 48.390 6.041
//            4: 38.811 4.502
//            5:33.624 3.631
//            6: 25.629 2.448
//            7: 18.813 1.821
//            8:10.799 0.919
//            9: 5.555 0.365
//            10:0.000 0.000

    public double robotHeading;
    public double x;
    public double y;

    public double[] convertToGlobal(double rotation, double x1, double y1) {
        // Gives the powers of wheels to go to a certain vector

        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        return new double[]{yy - xx, yy + xx, yy + xx, yy - xx};
    }

    public void setPosition(double startingHeading, double startingX, double startingY) {
        odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startingX,
                startingY,
                AngleUnit.DEGREES,
                startingHeading
        ));
    }
    // fully break at vel^2 formula until vel is like 15 or something

    public double[] getMecanumWheelPower(
            double targetHeading,
            double targetX,
            double targetY
    ) {
        double distance = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow((targetY - y), 2));
        double velocity = Math.abs(odometry.getVelocity().getX(DistanceUnit.INCH))
                + Math.abs(odometry.getVelocity().getY(DistanceUnit.INCH));
        double stoppingDistance = 0.00130445 * Math.pow(velocity, 2) + 0.0644448 * velocity + 0.0179835;
//        double stoppingDistance = 0.00439157 * Math.pow(velocity, 2) + 0.0985017 * velocity - 0.0700498 + 2;

        double[] powers;
        if (distance <= stoppingDistance && velocity > 15) {
            telemetry.addData("STOPPED", stoppingDistance);

//            double turnCorrection = Math.min(0.5, simplifyAngle(targetHeading - robotHeading) *0.04);
//            powers = new double[] {-turnCorrection+1,+1,turnCorrection+1,+1}; // right two a bit negative // 1!??
            return new double[]{0, 0, 0, 0};
        } else {
            powers = normalize(convertToGlobal(
                    robotHeading + 90,
                    (targetX - x) / 5,
                    -(targetY - y) / 5));
            double turnCorrection = Math.min(
                    MAX_TURN_POWER,
                    simplifyAngle(targetHeading - robotHeading) * TURN_POWER);

            return normalize(new double[]{
                    (powers[0] - turnCorrection) * MAX_POWER,
                    (powers[1] + turnCorrection) * MAX_POWER,
                    (powers[2] - turnCorrection) * MAX_POWER,
                    (powers[3] + turnCorrection) * MAX_POWER
            });
        }
    }

    public double[] getMecanumWheelPower2(
            double targetHeading,
            double targetX,
            double targetY
    ) {
        double[] powers = normalize(convertToGlobal(
                robotHeading + 90,
                (targetX - x) / 5,
                -(targetY - y) / 5));
        double turnCorrection = Math.min(
                MAX_TURN_POWER,
                simplifyAngle(targetHeading - robotHeading) * TURN_POWER);

        return normalize(new double[]{
                (powers[0] - turnCorrection) * MAX_POWER,
                (powers[1] + turnCorrection) * MAX_POWER,
                (powers[2] - turnCorrection) * MAX_POWER,
                (powers[3] + turnCorrection) * MAX_POWER
        });
    }

////        double[] powers = normalize(convertToGlobal(
////                robotHeading + 90,
////                (targetX - x) / 15,
////                -(targetY - y) / 15));
//
////        LinearEquation equ = new LinearEquation(0,0., 180, 1);
//        double turnCorrection = Math.min(
//                MAX_TURN_POWER,
//                simplifyAngle(targetHeading - robotHeading) * TURN_POWER);
//
//        return normalize(new double[]{
//                (powers[0] - turnCorrection) * MAX_POWER,
//                (powers[1] + turnCorrection) * MAX_POWER,
//                (powers[2] - turnCorrection) * MAX_POWER,
//                (powers[3] + turnCorrection) * MAX_POWER
//        });

    double[] normalize2(double[] a) {
        double maxPower = 0.1;
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

    double[] normalize(double[] a) {
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

    public double simplifyAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void stopWheels() {
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
    }

    public void updatePosition() {
        odometry.update();
        Pose2D pos = odometry.getPosition();
        robotHeading = pos.getHeading(AngleUnit.DEGREES);
        x = pos.getX(DistanceUnit.INCH);
        y = pos.getY(DistanceUnit.INCH);
    }

    public void stepPosition(double targetHeading, double targetX, double targetY) {
        updatePosition();

        double[] powers = getMecanumWheelPower(targetHeading, targetX, targetY);

        frontLeftWheel.setPower(powers[0]);
        frontRightWheel.setPower(powers[1]);
        backLeftWheel.setPower(powers[2]);
        backRightWheel.setPower(powers[3]);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("robotHeading", robotHeading);
        telemetry.addData("frontLeft", powers[0]);
        telemetry.addData("backLeft", powers[1]);
        telemetry.addData("frontRight", powers[2]);
        telemetry.addData("backRight", powers[3]);
        telemetry.update();
    }

    double headingTarget;
    double xTarget;
    double yTarget;

    public void goToPosition(double targetHeading, double targetX, double targetY) {
        headingTarget = targetHeading;
        xTarget = targetX;
        yTarget = targetY;
        while (
            Math.abs(targetY - y) > 0.5 ||
            Math.abs(targetX - x) > 0.5 ||
                    // 90 - 90
            Math.abs(simplifyAngle(targetHeading - robotHeading)) > 2
        ) {
            stepPosition(targetHeading, targetX, targetY);
        }
    }

    public void holdFor(double seconds) {
        long endTime = System.currentTimeMillis() + (long)(seconds * 1000);

        // Keep running stepPosition until the time expires
        while (System.currentTimeMillis() < endTime) {
            stepPosition(headingTarget, xTarget, yTarget);
        }

        stopWheels();
    }
}