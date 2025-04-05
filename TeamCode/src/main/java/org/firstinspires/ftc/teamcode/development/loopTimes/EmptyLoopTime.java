package org.firstinspires.ftc.teamcode.development.loopTimes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.Target;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class EmptyLoopTime extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();

        LoopTime.getLoopTime(Target::updatePosition);
    }
}