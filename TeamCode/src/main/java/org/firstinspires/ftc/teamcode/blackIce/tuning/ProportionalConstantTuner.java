//TODO
//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.HeadingCorrection;
//
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
//import org.firstinspires.ftc.teamcode.blackIce.paths.BezierCurve;
//import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
//import org.firstinspires.ftc.teamcode.util.Util;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class ProportionalConstantTuner extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        waitForStart();
//
//        Odometry.setPosition(90, 0, 0);
//
//        Drive.zeroPowerFloatMode();
//
//        double distance = 40;
//        double increment = 2;
//
//        //for (int i = 1; i <= 24 / increment; i++) {
//        for (double[] point : path.points) {
//            Target.setTarget(90, point[0], point[1]);
//            double targetYError = Target.y - Target.previousY;
//            double targetXError = Target.x - Target.previousX;
//            while (opModeIsActive()) {
//                Target.updatePosition();
//
//                boolean isPastPoint = (Target.previousY - Odometry.y - Odometry.yBrakingDistance)
//                    * targetYError
//                    + (Target.previousX - Odometry.x - Odometry.xBrakingDistance)
//                    * targetXError < 0;
//
//                if (isPastPoint) {
//                    break;
//                }
//
//                double xPower = (Target.xError - Odometry.xBrakingDistance);
//                double yPower = (Target.yError - Odometry.yBrakingDistance);
//
////                if (Vector.getMagnitude(xPower, yPower) < 1) {
////                    break;
////                }
//
//
//                Drive.power(Drive.combineMax(Util.normalize(
//                        Drive.fieldVectorToLocalWheelPowers(
//                            Vector.setMagnitudeToOne(xPower, yPower)
////                        new double[]{
////                            xPower,
////                            yPower
////                        }
//                        )), Drive.turnCounterclockwise(HeadingCorrection.locked.calculateTurnPower()), 1)
//                );
//
////                Follower.telemetry.addData("i", i);
////                Follower.telemetry.update();
//
////                Follower.telemetry.addData("odometryX", Odometry.x);
////                Follower.telemetry.addData("odometryY", Odometry.y);
////                Follower.telemetry.addData("targetX", Target.x);
////                Follower.telemetry.addData("targetY", Target.y);
////
////                Follower.telemetry.addData("xPower", (Target.xError - Odometry.xBrakingDistance));
////                Follower.telemetry.addData("yPower", (Target.yError - Odometry.yBrakingDistance));
////
////                Follower.telemetry.addData("drivePower1",
////                    movement.driveCorrection.calculateDrivePowers()[0]);
////                Follower.telemetry.addData("drivePower2",
////                    movement.driveCorrection.calculateDrivePowers()[1]);
////                Follower.telemetry.addData("drivePower3",
////                    movement.driveCorrection.calculateDrivePowers()[2]);
////                Follower.telemetry.addData("drivePower4",
////                    movement.driveCorrection.calculateDrivePowers()[3]);
////
////                Follower.telemetry.addData("i", i);
////                Follower.telemetry.update();
//            }
//
////            Follower.telemetry.addData("i2", i);
////            Follower.telemetry.update();
//        }
//
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
//// originally had like it would go to every 2 inch points on a curve
//// but now it just goes to the closest point it can go full power to without braking
