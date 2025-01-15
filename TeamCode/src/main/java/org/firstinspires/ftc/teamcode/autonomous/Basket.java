
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Basket extends Robot {
    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.bucketDown();
        intake.armIn();

        waitForStart();

        odometry.setPosition(90, -TILE - HALF_OF_ROBOT - EDGE_OF_TILE, 0);

        viperSlide.upperBasketRaise();
        movement.quickBrakeTo(
            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 2,
            EXTRA_TURN_RADIUS, 5);

        viperSlide.waitForExtension();
        viperSlide.dump();
        sleep(1000);
        movement.moveTo(
            45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 1,
            EXTRA_TURN_RADIUS + 5);
        viperSlide.lower();
        movement.quickBrakeTo(
            -180, -TILE - HALF_OF_ROBOT,
            TILE, 5);
        movement.quickBrakeTo(
            -180, -TILE - HALF_OF_ROBOT - 3,
            3*12+1, 5);
        movement.quickBrakeTo(
            90, -TILE - HALF_OF_ROBOT - 3,
            3*12+1, 5);
        intake.stopSweeper();

        intake.armIn();
        viperSlide.clawOut();
        sleep(500);
    }
}