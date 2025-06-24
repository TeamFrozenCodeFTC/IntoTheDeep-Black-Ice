//package org.firstinspires.ftc.teamcode.development.loopTimes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
//import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilds;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class UpdateLoopTime extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Follower.initAuto(this);
//
//        waitForStart();
//
//        PinpointLocalizer.setPosition(90, 0, 0);
//
//        Movement m = MovementBuilds.stopAtPosition(0, 0, 90)
//            .build().start();
//
//        LoopTime.getLoopTime(m::update);
//    }
//}