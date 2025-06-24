//package org.firstinspires.ftc.teamcode.blackIce.drive;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
///**
// * Encapsulates the drivetrain of the robot and provides methods
// * to control its motors. It includes initialization, power control, and
// * zero-power behavior settings.
// */
//public final class Drive {
//    private Drive() {}
//
//    public static DcMotorEx frontLeftWheel;
//    public static DcMotorEx backLeftWheel;
//    public static DcMotorEx frontRightWheel;
//    public static DcMotorEx backRightWheel;
//
//    private static DcMotorEx[] motors;
//
//    /**
//     * Initializes the drivetrain motors.
//     */
//    public static void init(HardwareMap hardwareMap) {
//        // TODO change these to your actual motor names, and change the directions as needed.
//        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "frontLeft");
//
//        frontLeftWheel.setDirection(DcMotorEx.Direction.REVERSE);
//
//        backLeftWheel = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backLeftWheel.setDirection(DcMotorEx.Direction.FORWARD);
//
//        frontRightWheel = hardwareMap.get(DcMotorEx.class, "frontRight");
//        frontRightWheel.setDirection(DcMotorEx.Direction.FORWARD);
//
//        backRightWheel = hardwareMap.get(DcMotorEx.class, "backRight");
//        backRightWheel.setDirection(DcMotorEx.Direction.FORWARD);
//
//        motors = new DcMotorEx[] {frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel};
//    }
//
//    /**
//     * Sets the power for each motor in the drivetrain based on the provided
//     * DrivePowers object.
//     */
//    static void power(DrivePowers powers) {
//        frontLeftWheel.setPower(powers.frontLeftPower);
//        backLeftWheel.setPower(powers.backLeftPower);
//        frontRightWheel.setPower(powers.frontRightPower);
//        backRightWheel.setPower(powers.backRightPower);
//    }
//
//    /**
//     * Sets all motors to zero-power float mode, allowing the robot to coast
//     * when no power is applied.
//     */
//    public static void zeroPowerFloatMode() {
//        for (DcMotorEx motor : motors) {
//            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        }
//    }
//
//    /**
//     * Sets all motors to zero-power brake mode, causing the robot to resist
//     * movement when no power is applied.
//     */
//    public static void zeroPowerBrakeMode() {
//        for (DcMotorEx motor : motors) {
//            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        }
//    }
//
//    /**
//     * Sets all motors to zero power.
//     */
//    public static void zeroPower() {
//        power(DrivePowers.forward(0));
//    }
//
////    /**
////     * Brakes for the given amount of seconds.
////     */
////    @Deprecated
////    public static void brakeFor(double seconds) {
////        zeroPower();
////
////        ElapsedTime timer = new ElapsedTime();
////
////        timer.reset();
////        while (Follower.opMode.opModeIsActive() && timer.seconds() < seconds) {
////            Follower.opMode.idle();
////        }
////    }
//}