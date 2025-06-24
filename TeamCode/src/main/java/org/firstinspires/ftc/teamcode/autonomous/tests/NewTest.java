//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.blackIce.paths.segments.BezierCurve;
//import org.firstinspires.ftc.teamcode.blackIce.paths.PathConfig;
//
//// WORKING VERSION
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class NewTest extends LinearOpMode {
//
//
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        final BezierCurve path = new BezierCurve(
//            new double[][]
//                {{ 1.68073593,  0.29761905},
//                    {14.74350649, 23.73376623},
//                    {55.08441558, 31.03354978},
//                    {81.20995671, 28.72835498},
//                    {77.36796537,  0.29761905},
//                    {56.23701299,  3.75541126},
//                    {34.33766234,  3.75541126}}
//        );
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        PathConfig follower = new PathConfig()
//            .waitUntil(() -> false);
//
//        new PathFollower(path).waitUntil();
//        path.waitUntil(() -> false);
//
//        Follower.telemetry.addData("finished", 0);
//        Follower.telemetry.update();
//    }
//}
//// v0 - encoder on backRight wheel + imu
//
//// v0.9 - simple linear, point to point movement
//// (took awhile to get a good stopAtPosition and MoveThrough)
//// for stopping at a position tried PIDs but our one of odometry wheels was malfunctioning
//// original either 100% or 100% 0 power brake,
//// but then allowed adjusting position while braking,
//
//// After 2024-2025 season, v1 - bezier curves,
//// documentation, modular movements
//
//// v1.5 making path following more accurate and immune to pushing, debugging,
//// tried to making path following as little arbitrary as possible.
//// go to every 1-2 inch points on a curve
//
//
//// more points cause oscillations do due the robot not having enough loop speed. the robot goes past
//// the point and then gets stuck skipping the point but the robot is already going too fast
//
//
//
////
