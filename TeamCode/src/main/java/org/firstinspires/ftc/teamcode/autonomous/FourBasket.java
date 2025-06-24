//
//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.EXTRA_TURN_RADIUS;
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.configConstants.Constants.Measurement.ROBOT_TURN_RADIUS;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous
//public class FourBasket extends Robot {
//    public void dumpSample() {
//        viperSlide.upperBasketRaise();
//
//        movement.stopAtPosition(
//            72, -59.1,
//            7);
//        viperSlide.waitForExtension();
//        movement.stopAtPosition(
//            72, -59.1,
//            5.7);
//
//        viperSlide.dump();
//        viperSlide.clawOut();
//        sleep(1000);
//
//        movement.stopAtPosition(
//            72, -55,
//            5.6);
//        viperSlide.lower();
//    }
//
//    @Override
//    public void runOpMode() {
//        initRobot();
//
//        viperSlide.bucketDown();
//        intake.armIn2();
//
//        waitForStart();
//
//        odometry.setPosition(90, -TILE - HALF_OF_ROBOT - EDGE_OF_TILE + 1, 0);
//
//        dumpSample();
//        viperSlide.lower();
//
//        movement.moveThrough(
//            -180, -TILE - HALF_OF_ROBOT - 2,
//            EDGE_OF_TILE + HALF_OF_ROBOT);
//        movement.stopAtPosition(
//            -180, -34, 38);
//        intake.armOut();
//        sleep(750);
//        intake.spinSweeperBy(0.75);
//        movement.stopAtPosition(
//            -180, -34-3,
//            38);
//        sleep(500);
//        intake.stopSweeper();
//        intake.armIn2();
//        sleep(1200);
//        sweeper.setPower(-0.5);
//        sleep(900);
//        intake.spinSweeperIn();
//        sleep(150);
//        intake.stopSweeper();
//
//        dumpSample();
//
//        movement.stopAtPosition(
//            -180, -44.2,
//            38);
//        intake.armOut();
//        sleep(750);
//        intake.spinSweeperBy(0.75);
//        movement.stopAtPosition(
//            -180, -44.2-3,
//            38);
//        sleep(500);
//        intake.stopSweeper();
//        intake.armIn2();
//        sleep(1200);
//        sweeper.setPower(-0.5);
//        sleep(1200);
//        intake.spinSweeperIn();
//        sleep(150);
//        intake.stopSweeper();
//
//        dumpSample();
//
//        movement.stopAtPosition(-90+45, -33.5, 48);
//
//        while (opModeIsActive()) {
//            idle();
//        }
//
////
////        movement.movePast(
////            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 3,
////            EXTRA_TURN_RADIUS + 3);
////
////        viperSlide.lower();
////        movement.stopAtPositionPI(
////            -180, -TILE - HALF_OF_ROBOT, TILE - HALF_OF_ROBOT);
////        movement.quickBrakeTo(
////            -180, -TILE - HALF_OF_ROBOT - 3,
////            3*12+1, 5);
////        movement.quickBrakeTo(
////            90, -TILE - HALF_OF_ROBOT - 3,
////            3*12+1, 5);
////        intake.stopSweeper();
////
////        intake.armIn();
////        viperSlide.clawOut();
////        sleep(500);
//
//    }
//}