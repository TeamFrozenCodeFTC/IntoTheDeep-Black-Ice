//package org.firstinspires.ftc.teamcode.folder.tuning;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.folder.Movement;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous
//public class LateralBrakingTuner extends Movement {
//    public double[] getStoppingDistance(double power) {
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < 0.6) {
//            updatePosition();
//            drive.power(applyTurnCorrection(drive.slideLeft(power), locked()));
//        }
//
//        double startingX = odometry.y;
//        double xVelocity = Math.abs(odometry.yVelocity);
//
//        drive.brakeFor(2);
//        updatePosition();
//
//        double newDistance = odometry.y;
//
//        double stoppingDistance = Math.abs(newDistance - startingX);
//
//        return new double[] {xVelocity, stoppingDistance};
//    }
//
//    public String stringify(double[] array) {
//        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
//    }
//
//    @Override
//    public void runOpMode() {
//        initRobot();
//        waitForStart();
//
//        double points = 10;
//
//        for (int i = 0; i <= points; i++) {
//            double direction = i % 2 == 0 ? 1 : -1;
//
//            double[] point = getStoppingDistance(direction * ((1-1*(i/points))+0.2));
//            telemetry.addData(Integer.toString(i), stringify(point));
//        }
//
//        telemetry.update();
//
//        while (opModeIsActive()) {
//            idle();
//        }
//    }
//}
//
////0:49.904 6.873
////
////
////        1: 47.927 6.006
////
////
////        2: 50.546 6.584
////
////
////        3:44.856 5.439
////
////
////        4:41.788 4.864
////
////
////        5:37.369 3.990
////
////
////        6:30.381 3.170
////
////
////        7:24.6052.327
////
////
////        8:19.479 1.688
////
////
////        9:13.147 1.013
////
////
////        10:6.358 0.406