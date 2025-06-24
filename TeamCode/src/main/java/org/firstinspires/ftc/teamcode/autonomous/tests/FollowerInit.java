//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.blackIce.old.BezierCurveOld;
//import org.firstinspires.ftc.teamcode.blackIce.old.PathOld;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class FollowerInit extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        final PathOld curve = new BezierCurveOld(
//            new double[][]
//                {{ 1.68073593,  0.29761905},
//                    {14.74350649, 23.73376623},
//                    {55.08441558, 31.03354978},
//                    {81.20995671, 28.72835498},
//                    {77.36796537,  0.29761905},
//                    {56.23701299,  3.75541126},
//                    {34.33766234,  3.75541126}}
//        )
//            .setLinearHeadingInterpolation(90,0);
//        final Movement path = curve.build();
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        //path.start();
//        path.waitForMovement();
//    }
//}
