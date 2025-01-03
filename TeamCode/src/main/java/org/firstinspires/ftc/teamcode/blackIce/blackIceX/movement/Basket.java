
package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;

import static org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Constants.Measurement.ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Constants.Measurement.ROBOT_TURN_RADIUS;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Movement;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Basket extends Robot {
    @Override
    public void runOpMode() {
        initRobot();

        waitForStart();

        odometry.setPosition(90, TILE, 0);

        viperSlide.topBasketRaise();
        movement.stopAtPosition(45, ROBOT_TURN_RADIUS + 2, ROBOT_TURN_RADIUS + 2);

        viperSlide.waitForExtension();
        viperSlide.dump();
        sleep(1000);
    }
}