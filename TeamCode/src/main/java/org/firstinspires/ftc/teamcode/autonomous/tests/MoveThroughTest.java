package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class MoveThroughTest extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        odometry.setPosition(90, 0, 0);
        movement.target.setTarget(90, 0, 0);
        // test what happens without set target

        movement.moveThrough(90, 0, 24);
        movement.moveThrough(90, 0, 0);
        movement.moveThrough(90, 0, 24);
    }
}
