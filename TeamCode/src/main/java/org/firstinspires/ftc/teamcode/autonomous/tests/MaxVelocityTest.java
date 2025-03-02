package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Point;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class MaxVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        Odometry.setPosition(0, 0, 0);

        new Point(48, 0, 0)
            .stopAtPosition()
            .setMaxVelocity(30)
            .build()
            .waitForMovement();
}}