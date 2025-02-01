package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class SquareStopTest extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        odometry.setPosition(90, 0, 0);

        movement.stopAtPosition(90, 0, 36);
        movement.stopAtPosition(90, 48, 36);
        movement.stopAtPosition(90, 48, 0);
        movement.stopAtPosition(90, 0, 0);
    }
}
