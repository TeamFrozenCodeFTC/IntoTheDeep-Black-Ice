
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Basket4 extends Robot {

    public void dumpSample() {
        viperSlide.upperBasketRaise();

        movement.stopAtPositionPI(
            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 2,
            EXTRA_TURN_RADIUS + 2);
        viperSlide.waitForExtension();
        movement.stopAtPositionPI(
            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 2.5,
            EXTRA_TURN_RADIUS - 0.5);


        viperSlide.dump();
        sleep(1000);
//        movement.movePast(
//            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 4,
//            EXTRA_TURN_RADIUS + 2);


    }
    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.bucketDown();
        intake.armIn();

        waitForStart();

        odometry.setPosition(90, -TILE - HALF_OF_ROBOT - EDGE_OF_TILE + 1, 0);

        dumpSample();
        viperSlide.lower();

        movement.moveTo(
            -180, -TILE - HALF_OF_ROBOT - 2,
            EDGE_OF_TILE + HALF_OF_ROBOT);
        movement.stopAtPositionPI(
            -180, -TILE - HALF_OF_ROBOT - 1,
            EDGE_OF_TILE + HALF_OF_ROBOT + TILE - 1.5);
        intake.armOut();
        sleep(750);
        intake.spinSweeperBy(0.75);
        movement.stopAtPositionPI(
            -180, -TILE - HALF_OF_ROBOT - 4.5,
            EDGE_OF_TILE + HALF_OF_ROBOT + TILE - 1.5);
        sleep(500);
        intake.stopSweeper();
        intake.armIn();
        sleep(2000);
        sweeper.setPower(-0.5);
        sleep(900);
        intake.spinSweeperIn();
        sleep(150);
        intake.stopSweeper();

        dumpSample();
        viperSlide.lower();

        movement.moveTo(
            -180, -TILE - HALF_OF_ROBOT - 2,
            EDGE_OF_TILE + HALF_OF_ROBOT);
        movement.stopAtPositionPI(
            -180, -TILE - HALF_OF_ROBOT - 6 - 4,
            EDGE_OF_TILE + HALF_OF_ROBOT + TILE - 1.5);
        intake.armOut();
        sleep(750);
        intake.spinSweeperBy(0.75);
        movement.stopAtPositionPI(
            -180, -TILE - HALF_OF_ROBOT - 6 - 7,
            EDGE_OF_TILE + HALF_OF_ROBOT + TILE - 1.5);

        sleep(500);
        intake.stopSweeper();
        intake.armIn();
        sleep(2000);
        sweeper.setPower(-0.5);
        sleep(900);
        intake.spinSweeperIn();
        sleep(150);
        intake.stopSweeper();

        dumpSample();
        viperSlide.lower();

        movement.stopAtPositionPI(
            90, -TILE - HALF_OF_ROBOT - 2,
            EDGE_OF_TILE + HALF_OF_ROBOT);

        while (opModeIsActive()) {
            idle();
        }

//
//        movement.movePast(
//            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 3,
//            EXTRA_TURN_RADIUS + 3);
//
//        viperSlide.lower();
//        movement.stopAtPositionPI(
//            -180, -TILE - HALF_OF_ROBOT, TILE - HALF_OF_ROBOT);
//        movement.quickBrakeTo(
//            -180, -TILE - HALF_OF_ROBOT - 3,
//            3*12+1, 5);
//        movement.quickBrakeTo(
//            90, -TILE - HALF_OF_ROBOT - 3,
//            3*12+1, 5);
//        intake.stopSweeper();
//
//        intake.armIn();
//        viperSlide.clawOut();
//        sleep(500);
    }
}