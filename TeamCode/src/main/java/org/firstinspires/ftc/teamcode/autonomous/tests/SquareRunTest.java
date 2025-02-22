package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;

//@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class SquareRunTest extends Robot {
//
//    public static double EXIT = 1;
//    public static double ADJUST_POWER = 1;

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        odometry.setPosition(90, 0, 0);

        movement.moveTo(90, 0, 24, 0.8);
        movement.moveTo(90, 48, 24, 0.8);
        movement.moveTo(90, 48, 0, 0.8);
        movement.moveTo(90, 0, 0, 0.8);

    }
}
