//package org.firstinspires.ftc.teamcode.teleOp.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@Disabled
//@TeleOp(group="Tests")
//public class _ViperTest extends Robot {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initHardware();
//
//        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        intakeExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        double maxTicks = 4365.0;
//        double minTicks = 4170.0;
//        double m = -1.0 / (maxTicks - minTicks);
//        double b = -1.0 * m * maxTicks;
//        boolean sweeperDown = false;
//        boolean slideUp = false;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            if (gamepad1.dpad_up) {
//                slideUp = true;
//            } else if (gamepad1.dpad_down) {
//                slideUp = false;
//            }
//
//            if (slideUp) { // up
//                if(viperSlideMotor.getCurrentPosition() < minTicks) {
//                    viperSlideMotor.setPower(1.0);
//                } else {
//                    viperSlideMotor.setPower(m * viperSlideMotor.getCurrentPosition() + b);
//                }
//            } else if (!slideUp && viperSlideMotor.getCurrentPosition() >= 20) {
//                viperSlideMotor.setPower(-1.0);
//            } else {
//                viperSlideMotor.setPower(0);
//            }
//
//            if(gamepad1.dpad_right && intakeExtender.getCurrentPosition() <= 3490) {
//                intakeExtender.setPower(-1.0);
//            } else if(gamepad1.dpad_left && intakeExtender.getCurrentPosition() >= 10) {
//                intakeExtender.setPower(1.0);
//            } else {
//                intakeExtender.setPower(0);
//            }
//
//            if(gamepad1.triangle) {
//                dumperServo.setPosition(0.5);
//            } else {
//                dumperServo.setPosition(0.17);
//            }
//
//            if(gamepad1.right_bumper) {
//                sweeperDown = true;
//            } else if (gamepad1.left_bumper) {
//                sweeperDown = false;
//            }
//            if(sweeperDown) {
//                sweeperRotator.setPosition(0.92);
//            } else {
//                sweeperRotator.setPosition(0.31);
//            }
//
//            if(gamepad1.circle) {
//                sweeper.setPower(1.0);
//            } else if(gamepad1.square) {
//                sweeper.setPower(-1.0);
//            } else {
//                sweeper.setPower(0);
//            }
//
//            telemetry.addData("vertical ticks", viperSlideMotor.getCurrentPosition());
//            telemetry.addData("horizontal ticks", intakeExtender.getCurrentPosition());
//            telemetry.update();
//        }
//
//    }
//}