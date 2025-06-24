//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.robot.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilds;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class MultiMoveThroughTest extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        Follower.telemetry.addData("previousX", Target.previousX);
//        Follower.telemetry.addData("previousY", Target.previousY);
//        Follower.telemetry.update();
//
//        MovementBuilds.moveThrough(0, 12, 90)
//            .build()
//            .waitForMovement();
//
//        Follower.telemetry.addData("power", Drive.backLeftWheel.getPower());
//        Follower.telemetry.update();
//
//        MovementBuilds.moveThrough(0, 24, 90)
//            .setToBrakeAfter()
//            .build()
//            .waitForMovement();
//
//        while (opModeIsActive()) {
//            idle();
//        }
//    }
//}