package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class ForwardTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        Follower.telemetry.addData("previousX", Target.previousX);
        Follower.telemetry.addData("previousY", Target.previousY);
        Follower.telemetry.update();

        // more mass = more friction?
        // more mass = more momentum?
        // say braking friction is -30in/s^2 for mass of 40 lb

        MovementBuilder.moveThrough(0, 12, 90)
            .build()
            .waitForMovement();

        Follower.telemetry.addData("power", Drive.backLeftWheel.getPower());
        Follower.telemetry.update();

        MovementBuilder.moveThrough(0, 24, 90)
            .setToBrakeAfter()
            .build()
            .waitForMovement();

        while (opModeIsActive()) {
            idle();
        }
    }
}