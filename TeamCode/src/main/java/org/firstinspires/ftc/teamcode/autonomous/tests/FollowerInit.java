package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.blackIce.paths.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class FollowerInit extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        final Path curve = new BezierCurve(
            new double[][]
                {{ 1.68073593,  0.29761905},
                    {14.74350649, 23.73376623},
                    {55.08441558, 31.03354978},
                    {81.20995671, 28.72835498},
                    {77.36796537,  0.29761905},
                    {56.23701299,  3.75541126},
                    {34.33766234,  3.75541126}}
        )
            .setLinearHeadingInterpolation(90,0);
        final Movement path = curve.build();

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        //path.start();
        path.waitForMovement();
    }
}
