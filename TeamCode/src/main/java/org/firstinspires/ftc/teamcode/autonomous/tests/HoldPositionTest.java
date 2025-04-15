package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuild;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.blackIce.paths.BezierCurve;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class HoldPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        Odometry.setPosition(0, 0, 0);

        final Movement m = MovementBuilder.stopAtPosition(0, 0, 0).build();

        waitForStart();

        m.waitUntil(() -> false);

        Follower.telemetry.addData("finished", 0);
        Follower.telemetry.update();
    }
}
// v0 - encoder on backRight wheel + imu

// v0.9 - simple linear, point to point movement
// (took awhile to get a good stopAtPosition and MoveThrough)
// for stopping at a position tried PIDs but our one of odometry wheels was malfunctioning
// original either 100% or 100% 0 power brake,
// but then allowed adjusting position while braking,

// After 2024-2025 season, v1 - bezier curves,
// documentation, modular movements

// v1.5 making path following more accurate and immune to pushing, debugging,
// tried to making path following as little arbitrary as possible.
// go to every 1-2 inch points on a curve


// more points cause oscillations do due the robot not having enough loop speed. the robot goes past
// the point and then gets stuck skipping the point but the robot is already going too fast



//
