package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.GoBildaPinpointDriver;

public abstract class Autonomous extends Robot {
    GoBildaPinpointDriver odo;

    double robotHeading;
    double x;
    double y;

    public void initOdo() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-36, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public double[] convertToGlobal(double rotation, double x1, double y1) {
        // Gives the powers of wheels to go to a certain vector
        
        double cos = Math.cos(Math.toRadians(rotation));
        double sin = Math.sin(Math.toRadians(rotation));
        double xx = x1 * cos - y1 * sin;
        double yy = x1 * sin + y1 * cos;

        return new double[]{yy-xx, yy+xx, yy+xx, yy-xx};
    }

    double POWER_CONSTANT = 0.2;

    public double[] getMecanumWheelPower(
            double targetHeading,
            double targetX,
            double targetY
    ) {
        // rotate forward powers towards point
//        double[] powers = normalize(convertToGlobal(
//                robotHeading + 90, // targetHeading?
//                targetX - x,
//                -(targetY - y)));
        double[] powers = normalize(convertToGlobal(
                robotHeading + 90, // targetHeading?
                targetX - x,
                -(targetY - y)));

//
//        telemetry.addData("distanceOffX", targetX - x);
//        telemetry.addData("distanceOffY", targetY - y);
//
//
        double turnCorrection = Math.min(0.5, simplifyAngle(targetHeading - robotHeading) * 0.05);

        double frontLeftPower = powers[0] - turnCorrection;
        double frontRightPower = powers[1] + turnCorrection;
        double backLeftPower = powers[2] - turnCorrection;
        double backRightPower = powers[3] + turnCorrection;
//
//        telemetry.addData("turn Correction", turnCorrection);

        return normalize(new double[]{
                frontLeftPower * POWER_CONSTANT,
                frontRightPower * POWER_CONSTANT,
                backLeftPower * POWER_CONSTANT,
                backRightPower * POWER_CONSTANT
        });
    }

    double[] normalize(double[] a) {
        double maxPower = 1.0;
        for (double value : a) {
            maxPower = Math.max(maxPower, Math.abs(value));
        }
//
//        double maxPower = Math.max(1.0,
//                Math.max(Math.abs(a[0]),
//                        Math.max(Math.abs(a[1]),
//                        Math.max(Math.abs(a[2]), Math.abs(a[3]))
//                        )));
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

    public void stopWheels() {
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
    }

    public void goToPosTurn(double turn, double x1, double y1) {
        while (
                Math.abs(y1 - y) > 0.5 ||
                Math.abs(x1 - x) > 0.5 ||
                Math.abs(simplifyAngle(turn - robotHeading)) > 2
        ) {
            odo.update();
            Pose2D pos = odo.getPosition();
            robotHeading = pos.getHeading(AngleUnit.DEGREES);
            x = pos.getX(DistanceUnit.INCH);
            y = pos.getY(DistanceUnit.INCH);

            double[] powers = getMecanumWheelPower(turn, x1, y1);

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

        stopWheels();
    }
}