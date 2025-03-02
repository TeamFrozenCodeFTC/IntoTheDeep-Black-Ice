package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Point;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.paths.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class BezierLineTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        //m.waitForMovement();
        path.waitForMovement();
    }


    private static final Movement m = new Point(42,17)
        .moveThrough()
        .build();

    // Recommended to build all movements and paths outside of runOpMode and as `static final`
    // and with private keyword if not needed anywhere else
    private static final Movement path = new BezierCurve(
        new double[][]
            {{ 1.68073593,  0.29761905},
                {14.74350649, 23.73376623},
                {55.08441558, 31.03354978},
                {81.20995671, 28.72835498},
                {77.36796537,  0.29761905},
                {56.23701299,  3.75541126},
                {34.33766234,  3.75541126}}
    ).build();

}

// after last tournament,
// - we added the goals of creating bezier curves
// - differentiating forward vs lateral braking distance, (+ adding quadratic regression into java)
// - documenting our code for others

// 2/21/2025 - made bezier curves work by updating moveThrough function to incorporate
// lines parallel --- to the previous point instead of L's or |__

// now make constants better and tune