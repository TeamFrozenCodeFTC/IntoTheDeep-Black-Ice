//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
//public class Specimen5 extends Robot {
//    public void hangSpecimen(double x) {
//        viperSlide.upperChamberRaise();
//        movement.quickBrakeTo(-90, x, 26, 10);
//
//        viperSlide.waitForExtension();
//
//        // While neither touch sensors are pressed...
//        movement.backIntoWall(0.3);
//
//        viperSlide.upperChamberPull();
//        //odometry.setHeading(-90);
//        //odometry.setY(31);
//        viperSlide.waitForExtension();
//        viperSlide.clawOut();
//        viperSlide.lower();
//    }
//
//    public void getSpecimen() {
//        movement.quickBrakeTo(90, TILE + HALF_OF_ROBOT, 3, 10);
//
//        movement.backIntoWall(0.3);
//
//        viperSlide.clawGrab();
//        sleep(200);
//        viperSlide.bottomBasketRaise();
//        sleep(250);
//    }
//
//    @Override
//    public void runOpMode() {
//        initRobot();
//
//        intake.armIn();
//        viperSlide.clawGrab();
//        //viperSlide.maxInitRaise();
//
//        waitForStart();
//
//        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0); // -90
//
//        viperSlide.upperChamberRaise();
//
//        movement.stopAtPosition(-90, 10.25, 27);
//
//        viperSlide.waitForExtension();
//
//        drive.power(drive.forward(-0.3));
//        sleep(500);
//        viperSlide.upperChamberPull();
//        viperSlide.waitForExtension();
//        viperSlide.clawOut();
//
//        viperSlide.lower();
//        movement.moveTo(-90, 10.25, 26); // Back up
//        movement.moveTo(-90, 35, 26); // Go past submersible
//        movement.stopAtPosition(-90, 40, 36); // Get Sample 1
//        movement.turnAndMoveTo(-180, 47, 7.75);
//
////        movement.reverseDirection(-180, 47, 11);
////        movement.turnAndMoveTo(-90, 46, 21); //
////
////        movement.stopAtPositionQuick(-90, 47.5, 35, 3); // line up with sample 2
////        movement.movePast(-90, 53, 35); // hook sample 2
////        movement.turnAndMoveTo(-180, 53, 7.75); // push sample in
//
//        movement.moveTo(-180, 47, 11);
//        movement.turnAndMoveTo(-90, 46, 21); //
//
//        movement.moveTo(-90, 47.5, 35); // line up with sample 2
//        movement.movePast(-90, 53, 35); // hook sample 2
//        movement.turnAndMoveTo(-180, 53, 7.75); // push sample in
//
//
//
//
//
//
////        // 2
////        getSpecimen();
////        hangSpecimen(-0.5);
////
////        // 3
////        getSpecimen();
////        hangSpecimen(-0.5+1.5); // 1
////
////        // 4
////        getSpecimen();
////        hangSpecimen(3.5); // technically should be 2.5
////
////        // movement.moveTo(-45, TILE*2, 8);
////        movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 0);
//
//    }
//}