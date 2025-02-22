package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class SquarePastTest extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        odometry.setPosition(90, 0, 0);

        movement.movePast(90, 0, 24);
        movement.movePast(90, 48, 24);
        movement.movePast(90, 48, 0);
        movement.movePast(90, 0, 0);
        
    }
}
