package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class MaxVelocityTest extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        Odometry.setPosition(0, 0, 0);

        new Movement(48, 0, 0)
            .stopAtPosition()
            .setMaxVelocity(20)
            .run();

        new Movement(48, 0, -180)
            .stopAtPosition()
            .setMaxHeadingVelocity(20)
            .run();
}}