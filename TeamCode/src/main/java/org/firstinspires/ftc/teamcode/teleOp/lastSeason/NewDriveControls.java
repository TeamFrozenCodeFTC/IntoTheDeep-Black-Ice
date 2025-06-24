//package org.firstinspires.ftc.teamcode.teleOp.lastSeason;
//
//import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.robot.DrivePowers;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//public class NewDriveControls {
//    Robot2 robot;
//
//    double x = 2.25;
//
//    public NewDriveControls(Robot2 robot) {
//        this.robot = robot;
//    }
//
//    double powerEquation(double power) {
//        return Math.pow(power, NewControls.POWER);
//    }
//
//    void control() {
//        if (robot.gamepad1.right_stick_button) {
//            PinpointLocalizer.setPosition(90, PinpointLocalizer.x, PinpointLocalizer.y);
//        }
////        // Reset Odometry in observation zone corner
////        else if (robot.gamepad1.circle) {
////            Odometry.setPosition(90,24*3-9,0);
////        }
////        else if (robot.gamepad1.square) {
////            robot.viperSlide.liftRobot();
////        }
////        else if (robot.gamepad1.cross) {
////            robot.viperSlide.liftersIn();
////        }
////        else if (robot.gamepad1.touchpad_finger_1) {
////            robot.viperSlideMotor.setPower(-0.3);
////            robot.viperSlideMotor.setTargetPosition(-5000);
////        }
////        else if (robot.gamepad1.left_bumper) {
////            robot.viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            robot.viperSlideMotor.setTargetPosition(0);
////            robot.viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        }
//       // else if (robot.gamepad1.dpad_up) {
////            robot.movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);
////            robot.movement.buildMovement(90, TILE + HALF_OF_ROBOT, -3)
////                .stopAtPosition()
////                .setMaxPower(0.4)
////                .runTimeout(0.3);
////
////            robot.viperSlide.clawGrab();
////            robot.sleep(200);
////            robot.viperSlide.bottomBasketRaise();
////            robot.sleep(250);
////
////            robot.viperSlide.upperChamberRaise();
////
////            robot.movement.moveThrough(-90, x, 30);
////
////            robot.movement.buildMovement(-90, x, 32)
////                .stopAtPosition()
////                .setMaxPower(0.5)
////                .runTimeout(0.7);
////
////            robot.viperSlide.upperChamberPull();
////            robot.viperSlide.waitForExtension();
////
////            robot.viperSlide.clawOut();
////            robot.viperSlide.lower();
////
////            x -= 1.5;
////            robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       // }
//        //else if (robot.gamepad1.dpad_right) {
////            robot.movement.moveThrough(-90, 38, 57);
////            robot.intake.armOut();
////            robot.movement.stopAtPosition(-90, 48, 13);
////            robot.intake.spinSweeperOut();
////
////            ElapsedTime timer = new ElapsedTime();
////            timer.reset();
////            while (timer.seconds() < 0.2) {
////                robot.idle();
////            }
////
////            robot.intake.stopSweeper();
////
////            robot.frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////            robot.backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       // }
//
//
//        controllerRelativeMovement()
//            .add(pivot())
//            .scale(.5)
//            .applyPowers();
//
////        Drive.power(
////            DrivePowers.scaleToMax(
////                DrivePowers.combine(
////                    pivot(),
////                    controllerRelativeMovement()
////                ),
////                speedFactor
////            )
////        );
//    }
//
//    private DrivePowers pivot() {
//        double power = powerEquation(robot.gamepad1.right_stick_x) * 0.7;
//
//        return DrivePowers.turnClockwise(power);
//    }
//
//    boolean firstInput = false;
//
//    private DrivePowers controllerRelativeMovement() {
//        double xStick = powerEquation(robot.gamepad1.left_stick_x);
//        double yStick = -powerEquation(robot.gamepad1.left_stick_y);
//
////        robot.gamepad1.rumble(Math.max(xStick, yStick), Math.max(xStick, yStick), 100);
//
//        if (!firstInput && (xStick != 0 || yStick != 0)) {
//            firstInput = true;
//            robot.timer.reset();
//        }
//
//        return new Vector(xStick, yStick).fieldVectorToLocalWheelPowers();
//    }
//}
