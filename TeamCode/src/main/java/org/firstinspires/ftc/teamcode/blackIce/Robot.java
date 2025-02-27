//package org.firstinspires.ftc.teamcode.blackIce;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.odometry.Odometry;
//
//public class Robot {
//    public void loopUpdate() {
//        Target.updatePosition();
//    }
//
//    private static Robot robot = null;
//    public LinearOpMode opMode;
//
//    private Robot(LinearOpMode opMode) {
//
//    }
//
//    public static Robot getInstance() {
////        if (robot == null) {
////            robot = new Robot();
////        }
//        return robot;
//    }
//
//    public static void init(LinearOpMode opMode) {
//        robot = new Robot(opMode);
//        Odometry.init(opMode.hardwareMap);
//        Drive.init(opMode.hardwareMap);
//    }
//
//    public boolean gamepadHasInterrupted() {
//        return opMode.gamepad1.dpad_down;
//    }
//
//    /**
//     * Makes sure opMode is running and that the controller has not canceled the movement.
//     */
//    public boolean isNotInterrupted() {
//        return !gamepadHasInterrupted() && opMode.opModeIsActive();
//    }
//}
