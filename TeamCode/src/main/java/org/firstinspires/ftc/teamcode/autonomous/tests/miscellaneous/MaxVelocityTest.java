package org.firstinspires.ftc.teamcode.autonomous.tests.miscellaneous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.DriveCorrection;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class MaxVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();
        // TODO cant run twice

        Odometry.setPosition(0, 0, 0);

        Follower.telemetry.addData("a pre Target.previousY", Target.previousY);
        Follower.telemetry.addData("a pre Target.previousX", Target.previousX);
        Follower.telemetry.addData("a pre Target.Y", Target.y);
        Follower.telemetry.addData("a pre Target.X", Target.x);
        Follower.telemetry.addData("a pre Target.xError", Target.xError);
        Follower.telemetry.addData("a pre Target.yError", Target.yError);

        Follower.telemetry.addData("odo x", Odometry.x);
        Follower.telemetry.addData("odo y", Odometry.y);
        Follower.telemetry.update();

//        MovementBuilder.moveThrough(48, 0, 0)
//            .setMaxVelocity(40)
//            .setDriveCorrection(DriveCorrection.velocityConstraints)
//            .setToBrakeAfter()
//            .build()
//            .waitForMovement();
}}