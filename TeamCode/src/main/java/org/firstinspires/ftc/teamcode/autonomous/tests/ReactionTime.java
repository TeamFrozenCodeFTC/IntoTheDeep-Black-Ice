//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.blackIce.robot.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuild;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilds;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class ReactionTime extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        Follower.telemetry.addData("odometry1", PinpointLocalizer.x);
//        Follower.telemetry.addData("odometry1", PinpointLocalizer.y);
//
//        double increment = .4;
//
//        MovementBuild[] movements = new MovementBuild[40];
//        for (int i = 1; i <= movements.length; i++) {
//            movements[i-1] = MovementBuilds.moveThrough(0,increment * i,90);
//                //.build();
//        }
//
////        Follower.telemetry.addData("odometryX", Odometry.x);
////        Follower.telemetry.addData("odometryY", Odometry.y);
////        Follower.telemetry.addData("targetX", Target.x);
////        Follower.telemetry.addData("targetY", Target.y);
////        Follower.telemetry.update();
//
////        Movement movement1 = MovementBuilder.moveThrough(0,-0.3,90)
////            .brakeAfter().build();
////
////        movement1.waitForMovement();
//
//        for (MovementBuild movement : movements) {
//            //movement.waitForMovement();
//            //movement.loop();
////            movement.start();
////            while (opModeIsActive() && !movement.isFinished()) {
////                movement.update();
////            }
////            movement.finish();
//        }
//        Follower.telemetry.addData("odometryX", PinpointLocalizer.x);
//        Follower.telemetry.addData("odometryY", PinpointLocalizer.y);
//        Follower.telemetry.addData("x braking distance", PinpointLocalizer.xBrakingDistance);
//        Follower.telemetry.addData("y braking distance", PinpointLocalizer.yBrakingDistance);
//        Follower.telemetry.update();
//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < 3) {
//            Drive.zeroPower();
//        }
//    }
//}
