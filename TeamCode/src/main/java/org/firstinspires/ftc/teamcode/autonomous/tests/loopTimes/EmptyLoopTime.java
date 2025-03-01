package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.tests.loopTimes.LoopTime;
import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class EmptyLoopTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        LoopTime.getLoopTime(Target::updatePosition);
    }
}