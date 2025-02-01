//
//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_SUBMERSIBLE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
//public class SlideYellowSamples extends Robot {
//    @Override
//    public void runOpMode() {
//        initRobot();
//
//        sweeperRotator.getController().pwmDisable();
//        viperSlide.clawGrab();
//
//        waitForStart();
//
//        odometry.setPosition(-180, -TILE-HALF_OF_ROBOT, 0);
//
//        double sampleY = TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 3.5;
//
//        movement.moveTo(-180, -TILE-HALF_OF_ROBOT-2, sampleY);
//        movement.moveTo(-180, -TILE-HALF_OF_ROBOT+2, sampleY);
//        movement.turnAndMoveTo(-135, -TILE-TILE-ROBOT_TURN_RADIUS, ROBOT_TURN_RADIUS);
//    }
//}