package org.firstinspires.ftc.teamcode.development.loopTimes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class UpdateLoopTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        Movement m = MovementBuilder.stopAtPosition(0, 0, 90)
            .build().start();

        LoopTime.getLoopTime(m::update);
    }
}