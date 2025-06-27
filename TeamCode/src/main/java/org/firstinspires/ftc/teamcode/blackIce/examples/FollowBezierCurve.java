package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.action.Condition;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequenceConstructor;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Autonomous(group="Black-Ice Examples")
public class FollowBezierCurve extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        follower.whileFollowing(follower::telemetryDebug);

        PathSequenceConstructor constructor = follower.pathSequenceConstructor();
        BezierCurve c = new BezierCurve(new double[][]{{0,0}, {24, 24}, {12, 48}});
        Logger.debug("starting", c.calculatePointAt(0));
        Logger.debug("starting0.5", c.calculatePointAt(0.5));
        Logger.debug("starting1", c.calculatePointAt(1));
        Logger.debug("derivativeAt0", c.calculateFirstDerivative(0));
        Logger.debug("derivativeAt0.5", c.calculateFirstDerivative(0.5));
        Logger.debug("derivativeAt1", c.calculateFirstDerivative(1));
        
        Logger.debug("second derivativeAt0", c.calculateSecondDerivative(0));
        Logger.debug("second derivativeAt0.5", c.calculateSecondDerivative(0.5));
        Logger.debug("second derivativeAt1", c.calculateSecondDerivative(1));
        
        Logger.debug("closestPointTo24,24", c.calculateClosestPointTo(new Vector(24,24), 0).getPoint());
        Logger.debug("closestTangentTo24,24",
            c.calculateClosestPointTo(new Vector(24,24), 0).getTangentVector());
        Logger.debug("closestTTo24,24",
            c.calculateClosestPointTo(new Vector(24,24), 0).getTValue());
        Logger.debug("length",c.length());
        
//        Path path = constructor.toPoint(new Vector(24, 24))
//            .stopAtEnd()
//            .maximizeSpeed()
////            .setConstantHeading(90)
//            .holdUntil(Condition.NEVER);

        
        Path path = constructor.curve(new double[][]{{0,0}, {36, 36}, {12, 48}})
            .setDeceleration(80) // if going 60inches a second 30 will decelerate in 2 seconds
            // TODO set deceleration seconds
//            .setEndingVelocity(22)
//            .setEndingVelocityCruiseDistance(40)
            //.maximizeSpeed()
            //.setConstantHeading(90)
            .holdUntil(Condition.NEVER);

        waitForStart();

        follower.beginFollowing(path);
        
        while (follower.isFollowingPath() && opModeIsActive()) {
            follower.update();
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