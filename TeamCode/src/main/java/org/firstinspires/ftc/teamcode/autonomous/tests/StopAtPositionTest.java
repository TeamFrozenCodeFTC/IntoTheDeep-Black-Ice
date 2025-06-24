//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilds;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class StopAtPositionTest extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        Movement m = MovementBuilds.stopAtPosition(0, 24, 90)
//            .build();
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        m.waitForMovement();
//    }
//}