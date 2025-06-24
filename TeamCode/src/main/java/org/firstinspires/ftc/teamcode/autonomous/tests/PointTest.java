//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.paths.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.HeadingCorrection;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuild;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
//import org.firstinspires.ftc.teamcode.blackIce.old.BezierCurve;
//
//// WORKING VERSION
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tests")
//public class PointTest extends LinearOpMode {
//
//    private static final BezierCurve path = new BezierCurve(
//        new double[][]
//            {{1.68073593, 0.29761905},
//                {14.74350649, 23.73376623},
//                {55.08441558, 31.03354978},
//                {81.20995671, 28.72835498},
//                {77.36796537, 0.29761905},
//                {56.23701299, 3.75541126},
//                {34.33766234, 3.75541126}}
//    );
//
//    boolean finished;
//    double targetYError;
//    double targetXError;
//
//    public void update() {
//        Target.updatePosition();
//
////        finished = (Target.previousY - Odometry.y - Odometry.yBrakingDistance)
////            * targetYError
////            + (Target.previousX - Odometry.x - Odometry.xBrakingDistance)
////            * targetXError < 0;
//        finished = m.isAtGoal.condition();
//
//        if (finished) {
//            return;
//        }
//        double xPower = (Target.xError - Odometry.xBrakingDistance);
//        double yPower = (Target.yError - Odometry.yBrakingDistance);
//
////                if (Vector.getMagnitude(xPower, yPower) < 1) {
////                    break;
////                }
//
//        m.moveTowardTarget();
////        Drive.power(Drive.combineMax(
////                Vector.scaleToMax(Drive.fieldVectorToLocalWheelPowers(
////                    xPower, yPower
////                ), 1),
////            Drive.turnCounterclockwise(HeadingCorrection.locked.calculateTurnPower()), 1)
////        );
//    }
//
//    MovementBuild m;
//
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
//            m = MovementBuilder.moveThrough(point[0], point[1], 90);
//
//            m.setHeadingCorrection(HeadingCorrection.locked);
//            m.setDriveCorrection(() ->
//                Drive.fieldVectorToLocalWheelPowers(
//                    (Target.xError - Odometry.xBrakingDistance),
//                    (Target.yError - Odometry.yBrakingDistance)
//                )
//            );
//            m.setTotalCorrection(() -> Drive.power(
//                Drive.combineMax(
//                    m.getDrivePowerCorrection(),
//                    m.getHeadingPowerCorrection(),
//                    1
//                )
//            ));
//
//            // Makes robot move always full power
//            m.setDrivePowerScaling(drivePowers -> Vector.scaleToMax(drivePowers, 1));
//
//            m.setToContinuePowerAfter();
//
//            m.isAtGoal = () -> {
//                boolean isPastPoint = (Target.previousY - Odometry.y - Odometry.yBrakingDistance)
//                    * Target.yDelta
//                    + (Target.previousX - Odometry.x - Odometry.xBrakingDistance)
//                    * Target.xDelta < 0;
//
//                return isPastPoint;
//            };
//
//            m.build().waitForMovement();
//
////            Target.setTarget(90, point[0], point[1]);
//////            targetYError = Target.y - Target.previousY;
//////            targetXError = Target.x - Target.previousX;
////            finished = m.isAtGoal.condition();
////            m.start();
////            while (opModeIsActive() && !finished) {
////                update();
////            }
//
////                Follower.telemetry.addData("i", i);
////                Follower.telemetry.update();
////
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
//        }
//
////            Follower.telemetry.addData("i2", i);
////            Follower.telemetry.update();
//
//
////
////        Follower.telemetry.addData("finished", 0);
////        Follower.telemetry.update();
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
////
