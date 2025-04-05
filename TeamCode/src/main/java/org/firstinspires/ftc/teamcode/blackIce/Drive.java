package org.firstinspires.ftc.teamcode.blackIce;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public final class Drive {
    private Drive() {}

    public static DcMotor frontLeftWheel;
    public static DcMotor backLeftWheel;
    public static DcMotor frontRightWheel;
    public static DcMotor backRightWheel;

    private static DcMotor[] motors;

    public static void init(HardwareMap hardwareMap) {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);

        backRightWheel = hardwareMap.get(DcMotor.class, "backRight");
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

        motors = new DcMotor[] {frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel};
    }

    public static void power(double[] powers) {
        frontLeftWheel.setPower(powers[0]);
        backLeftWheel.setPower(powers[1]);
        frontRightWheel.setPower(powers[2]);
        backRightWheel.setPower(powers[3]);
    }

    public static void zeroPowerFloatMode() {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public static void zeroPowerBrakeMode() {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public static void zeroPower() {
        power(forward(0));
    }

    public static double[] forward(double power) {
        return new double[] {power, power, power, power};
    }

    public static double[] backward(double power) {
        return forward(-power);
    }

    public static double[] slideLeft(double power) {
        return new double[] {power, -power, -power, power};
    }

    public static double[] slideRight(double power) {
        return slideLeft(-power);
    }

    public static double[] turnClockwise(double power) {
        return new double[] {power, power, -power, -power};
    }

    public static double[] turnCounterclockwise(double power) {
        return turnClockwise(-power);
    }

    public static double[] combine(double[] powers1, double[] powers2) {
        return normalize(new double[] {
                powers1[0] + powers2[0], powers1[1] + powers2[1],
                powers1[2] + powers2[2], powers1[3] + powers2[3]
        });
    }

    public static double[] combineMax(double[] powers1, double[] powers2, double max) {
        return downScaleTo(max,
            new double[] {
            powers1[0] + powers2[0], powers1[1] + powers2[1],
            powers1[2] + powers2[2], powers1[3] + powers2[3]
        });
    }

    public static double[] multiply(double[] powers1, double multiplier) {
        return normalize(new double[] {
                powers1[0] * multiplier, powers1[1] * multiplier,
                powers1[2] * multiplier, powers1[3] * multiplier
        });
    }

    public static void brakeFor(double seconds) {
//        Robot robot = Robot.getInstance();
//
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (robot.isNotInterrupted() && timer.seconds() < seconds) {
//            zeroPower();
//        }
    }

    static double[] normalize(double[] powers) {
        double maxPower = 1; // WAS 0 WAS 1 (for only downscaling)
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

    public static double[] downScaleTo(double maxScaleTo, double[] powers) {
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
    public static double[] fieldVectorToLocalWheelPowers(double[] vector) {
        return robotVectorToLocalWheelPowers(fieldVectorToRobotVector(vector));
    }

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public static double[] fieldVectorToLocalWheelPowers(double vectorX, double vectorY) {
        return robotVectorToLocalWheelPowers(fieldVectorToRobotVector(vectorX, vectorY));
    }

    public static double[] fieldVectorToRobotVector(double[] fieldVector) {
        // positive heading is counterclockwise
//        double heading = Math.toRadians(Odometry.heading);
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
        double localForwards = (fieldVector[0] * Target.headingCos + fieldVector[1] * Target.headingSin); // clockwise rotation
        double localSlide = (-fieldVector[0] * Target.headingSin + fieldVector[1] * Target.headingCos);

        return new double[]{localForwards, localSlide};
    }

    public static double[] fieldVectorToRobotVector(double fieldX, double fieldY) {
        double localForwards = (fieldX * Target.headingCos + fieldY * Target.headingSin); // clockwise rotation
        double localSlide = (-fieldX * Target.headingSin + fieldY * Target.headingCos);

        return new double[]{localForwards, localSlide};
    }

    public static double[] robotVectorToFieldVector(double[] robotVector) { // TODO convert double[] to two doubles
        // Positive heading is counterclockwise
//        double heading = Math.toRadians(Odometry.heading);
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
        double fieldX = (robotVector[0] * Target.headingCos - robotVector[1] * Target.headingSin); // Counterclockwise rotation
        double fieldY = (robotVector[0] * Target.headingSin + robotVector[1] * Target.headingCos);

        return new double[]{fieldX, fieldY};
    }

    static double[] robotVectorToLocalWheelPowers(double[] robotVector) {
        double localForwards = robotVector[0];
        double localSlide = robotVector[1];

        double upRightDirection = localForwards+localSlide;
        double downLeftDirection = localForwards-localSlide;

        return Util.normalize(new double[]
            {downLeftDirection, upRightDirection,
             upRightDirection,  downLeftDirection}
        );
    }
}