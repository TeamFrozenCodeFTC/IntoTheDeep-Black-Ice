package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class StopAtPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        Movement m = MovementBuilder.stopAtPosition(0, 24, 90)
            .build();

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        m.waitForMovement();
    }
}