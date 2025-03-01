package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class BezierLineTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        Odometry.setPosition(90, 0, 0);

        BezierCurve java = new BezierCurve( // was static
            new double[][]
                {{ 1.68073593,  0.29761905},
                {14.74350649, 23.73376623},
                {55.08441558, 31.03354978},
                {81.20995671, 28.72835498},
                {77.36796537,  0.29761905},
                {56.23701299,  3.75541126},
                {34.33766234,  3.75541126}}
        );

        //new Movement(38, 7).stopAtPosition().waitForMovement();
        new Movement(42,17).moveThrough().floatAfter().waitForMovement();
        //new PathFollower(java).waitForPath(); // .complete() -> PathFollower?

        //new Movement(1,2).setMovementExit(() -> true).waitForMovement(() -> {});




    }
}

// after last tournament,
// - we added the goals of creating bezier curves
// - differentiating forward vs lateral braking distance, (+ adding quadratic regression into java)
// - documenting our code for others

// 2/21/2025 - made bezier curves work by updating moveThrough function to incorporate
// lines parallel --- to the previous point instead of L's or |__

// now make constants better and tune