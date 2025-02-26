//package org.firstinspires.ftc.teamcode.autonomous.tests;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.blackIce.Movement;
//import org.firstinspires.ftc.teamcode.odometry.Odometry;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
//public class PIDTest extends Robot {
//    @Override
//    public void runOpMode() {
//        initRobot();
//        waitForStart();
//
//        Odometry.setPosition(0, 0, 0);
//
//        Movement.stopAtPosition(0, 48, 0);
//        telemetry.addData("x", Odometry.x);
//        telemetry.addData("y", Odometry.y);
//        telemetry.update();
//    }
//}