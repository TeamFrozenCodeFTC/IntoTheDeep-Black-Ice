//package org.firstinspires.ftc.teamcode.blackIce;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.ViperSlide;
//import org.firstinspires.ftc.teamcode.util.LinearEquation;
//
//public abstract class RobotMovement extends Robot {
//    public double[] convertToGlobal(double rotation, double x1, double y1) {
//        // Gives the powers of wheels to go to a certain vector
//
//        double cos = Math.cos(Math.toRadians(rotation));
//        double sin = Math.sin(Math.toRadians(rotation));
//        double xx = x1 * cos - y1 * sin;
//        double yy = x1 * sin + y1 * cos;
//
//        return new double[]{yy - xx, yy + xx, yy + xx, yy - xx};
//    }
//
//    public void setPosition(double startingHeading, double startingX, double startingY) {
//        odometry.setPosition(new Pose2D(
//                DistanceUnit.INCH,
//                startingX,
//                startingY,
//                AngleUnit.DEGREES,
//                startingHeading
//        ));
//    }
//
//    public void setStartingPosition(double startingHeading, double startingX, double startingY) {
//        setPosition(startingHeading, startingX, startingY);
//    }
//
//    // fully break at vel^2 formula until vel is like 15 or something
//
//    public double[] getMecanumWheelPower(
//            double targetHeading,
//            double targetX,
//            double targetY
//    ) {
//        //double distance = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow((targetY - y), 2)) + 15;
//        double velocity = Math.abs(odometry.getVelocity().getX(DistanceUnit.INCH))
//                + Math.abs(odometry.getVelocity().getY(DistanceUnit.INCH));
//        double[] powers = normalize(convertToGlobal(
//                robotHeading + 90,
//                (targetX - x) / (Math.pow(velocity, 2)*0.015+17),
//                -(targetY - y) / (Math.pow(velocity, 2)*0.015+17)));
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
//    }
//
//    double[] normalize(double[] a) {
//        double maxPower = 1.0;
//        for (double value : a) {
//            maxPower = Math.max(maxPower, Math.abs(value));
//        }
//
//        return new double[]{
//                a[0] / maxPower,
//                a[1] / maxPower,
//                a[2] / maxPower,
//                a[3] / maxPower
//        };
//    }
//
//    public double simplifyAngle(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
//
//    public void stopWheels() {
//        backLeftWheel.setPower(0);
//        backRightWheel.setPower(0);
//        frontLeftWheel.setPower(0);
//        frontRightWheel.setPower(0);
//    }
//
//    public void updatePosition() {
//        odometry.update();
//        Pose2D pos = odometry.getPosition();
//        robotHeading = pos.getHeading(AngleUnit.DEGREES);
//        x = pos.getX(DistanceUnit.INCH);
//        y = pos.getY(DistanceUnit.INCH);
//    }
//
//    public void stepPosition(double targetHeading, double targetX, double targetY) {
//        updatePosition();
//
//        double[] powers = getMecanumWheelPower(targetHeading, targetX, targetY);
//
//        frontLeftWheel.setPower(powers[0]);
//        frontRightWheel.setPower(powers[1]);
//        backLeftWheel.setPower(powers[2]);
//        backRightWheel.setPower(powers[3]);
//
//        telemetry.addData("x", x);
//        telemetry.addData("y", y);
//        telemetry.addData("robotHeading", robotHeading);
//        telemetry.addData("frontLeft", powers[0]);
//        telemetry.addData("backLeft", powers[1]);
//        telemetry.addData("frontRight", powers[2]);
//        telemetry.addData("backRight", powers[3]);
//        telemetry.update();
//    }
//
//    double headingTarget;
//    double xTarget;
//    double yTarget;
//
//    public void goToPosition(double targetHeading, double targetX, double targetY) {
//        headingTarget = targetHeading;
//        xTarget = targetX;
//        yTarget = targetY;
//        while (
//                Math.abs(targetY - y) > 0.5 ||
//                        Math.abs(targetX - x) > 0.5 ||
//                        // 90 - 90
//                        Math.abs(simplifyAngle(targetHeading - robotHeading)) > 2
//        ) {
//            stepPosition(targetHeading, targetX, targetY);
//        }
//    }
//
//    public void holdFor(double seconds) {
//        long endTime = System.currentTimeMillis() + (long)(seconds * 1000);
//
//        // Keep running stepPosition until the time expires
//        while (System.currentTimeMillis() < endTime) {
//            stepPosition(headingTarget, xTarget, yTarget);
//        }
//
//        stopWheels();
//    }
//}