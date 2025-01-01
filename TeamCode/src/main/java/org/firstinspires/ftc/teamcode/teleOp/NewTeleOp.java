//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.blackIce.blackIceX.RobotMovement;
//import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Movement;
//
//
//@TeleOp
//public class RelativeControls extends Movement {
//    @Override
//    public void runOpMode() {
//        initRobot();
//
//        SampleControls specimenControls = new SampleControls(this);
//        //RelativeWheelControls relativeWheelControls = new RelativeWheelControls(this);
//
//        timer = new ElapsedTime();
//
//        waitForStart();
//
//        new Thread(() -> {
//            while (opModeIsActive()) {
//                specimenControls.control();
//            }
//        }).start();
//
//        timer.reset();
//
//        while (opModeIsActive()) {
//            //relativeWheelControls.control();
//            if (gamepad1.triangle) {
//                odometry.resetHeading();
//            }
//
//            double heading;
//            double x;
//            double y;
//
//            if (gamepad1.right_trigger != 0 ||
//                    (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)) {
//                moveTowardTarget(1);
//                continue;
//            }
//            else {
//                setTarget(odometry.heading, odometry.x, odometry.y);
//            }
////            else {
////                heading = odometry.heading + gamepad1.right_stick_x*40;
////                x = odometry.x + gamepad1.left_stick_x*3; // target?
////                y = odometry.y - gamepad1.left_stick_y*3;
////            }
//
////
////            moveTowardTarget(5);
////
//            updatePosition();
//            drive.power(drive.combine(
//                    drive.turnClockwise(gamepad1.right_stick_x),
//                    fieldVectorToLocalWheelPowers(
//                            gamepad1.left_stick_x,
//                            gamepad1.left_stick_y,
//                            2
//                    )));
//
//
//            telemetry.update();
//
//            if (120-timer.seconds() < 30 && 120-timer.seconds() > 29) {
//                gamepad1.rumble(0.5, 0.5, 100);
//            }
//        }
//    }
//}
