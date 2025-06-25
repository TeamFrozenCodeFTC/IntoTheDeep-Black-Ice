package org.firstinspires.ftc.teamcode.blackIce.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequenceConstructor;

@Autonomous(group="Black-Ice Examples")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        PathSequenceConstructor constructor = follower.pathSequenceConstructor();

        waitForStart();
        
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("heading", follower.motionState.heading);
            telemetry.addData("brakingDisplacement", follower.motionState.getPredictedStoppedPosition().subtract(follower.motionState.position));
            telemetry.addData("fieldRelativeVelocity", follower.motionState.fieldRelativeVelocity);
            telemetry.addData("robotRelativeVelocity", follower.motionState.robotRelativeVelocity);
            telemetry.update();
        }
        
        // TODO path Maker op-mode
        // TODO motor/servo opmodes (just find online)
    }
}

//        BezierCurve c = new BezierCurve(
//            new double[][]{{1.68073593,  0.29761905},
//                {14.74350649, 23.73376623},
//                {55.08441558, 31.03354978},
//                {81.20995671, 28.72835498},
//                {77.36796537,  0.29761905},
//                {56.23701299,  3.75541126},
//                {34.33766234,  3.75541126}});
//        Path path = new Path(c)
//            .setTargetVelocity(60);
//
//        Path toSubmersible = constructor.toPoint(new Vector(10, 10));
//        Path pastSubmersible = constructor.toPoint(new Vector(15, 10));
//
//        follower.beginFollowing(toSubmersible, pastSubmersible);
//
//        Path multiSegmentPath = constructor.buildSegments()
//            .toPoint(new Vector(15, 10))
//            .toPoint(new Vector(15, 15))
//            .build()
//            .setLinearHeadingInterpolation(0, 90);
//
//        //
//
//        // use previous pose when you add to macro?
//        // Macro.addPath(pastSubmersible)
//
//        Path turn = constructor.turnTo(90);
//        Path turnSlow = turn.copy().setTargetAngularVelocity(45);
//        Path toPoint = constructor.toPoint(new Vector(1, 2));
//        // .build()
//
//        // add actions...?
//
//        MacroSequence autoMacro = new MacroSequence(follower)
//            .addPath(toScore)
//            .addAction(() -> arm.raise())
//            .addWait(0.5)
//            .addPath(toPark);

// does the macro need to be on thing?
// currentMacro.update()

// startTeleOpMacro
// setMacroCancelCondition()
// Macro.run(?) BUILDER?
// follower.startMacro()
//
