package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Point;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class StopAtPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        Movement m = new Point(0, 24, 90)
            .stopAtPosition().build();

        m.start();
        resetRuntime();

        while (opModeIsActive()) {
            double oldTime = getRuntime();
            m.update();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;

            Follower.telemetry.addData("frequency", frequency);
            Follower.telemetry.update();
            //double oldTime = getRuntime();
        }

    }
} // 0.12
//0.11999999999999998