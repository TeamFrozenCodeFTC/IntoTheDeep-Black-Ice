package org.firstinspires.ftc.teamcode.autonomous.tests;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class ExtendIntake extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        odometry.setPosition(90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);

        intake.fullyExtend();
        intake.armOut();

        movement.stopAtPosition(62, 31.813, 15.765);
//
        //sweeperRotator.getController().pwmDisable();

        movement.buildMovement(-30, 31.813, 15.765)
            .moveThrough()
            .setMaxPower(0.3)
            .run();

        intake.armUp();

        movement.stopAtPosition(62, 31.813, 15.765);
        intake.armOut();
        movement.stopAtPosition(42, 35.86, 20);

        //sweeperRotator.getController().pwmDisable();

        movement.buildMovement(-35, 35.86, 20)
            .moveThrough()
            .setMaxPower(0.3)
            .run();
    }
}