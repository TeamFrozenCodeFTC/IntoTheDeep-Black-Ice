//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
//public class SlideAll4Specimen extends Robot {
//    public void hangSpecimen(double x) {
//        viperSlide.upperChamberRaise();
//        movement.quickBrakeTo(-90, x, 28, 10);
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
//        odometry.setPosition(-90, 0, 0); // -90
//
//        hangSpecimen(-2);
//
//        //sweeperRotator.getController().pwmDisable();
//
//        // Move away from submersible
//        movement.moveTo(-90, 4, TILE + EDGE_OF_TILE - 3);
//        // Turn and go towards sample
//        movement.moveTo(-180, TILE + 4, TILE + EDGE_OF_TILE - 4);
//
//        // Hook 2nd Sample
//        movement.moveTo(-180, TILE * 2 - 6, TILE + EDGE_OF_TILE + HALF_OF_ROBOT);
//        // Push 2nd Sample
//        movement.moveTo(90, TILE * 2 - 5, 3);
//
//        // Moves straight out of observation zone
//        movement.moveTo(90, TILE * 2 - 5, 12);
//
//        // Aligns with 3th Sample
//        movement.quickBrakeTo(-180, TILE * 2 - 1, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2, 5);
//        // Hook 3rd Sample
//        movement.moveTo(-180, TILE * 2 + 5, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2);
//        // Push 3rd Sample
//        movement.moveTo( 90, TILE * 2, 3);
//
////        // Moves straight out of observation zone
////        movement.moveTo(90, TILE * 2, 12);
//
////        // Getting Specimen
////        movement.quickBrakeTo(0, TILE * 3 - HALF_OF_ROBOT - ROBOT_TURN_RADIUS, ROBOT_TURN_RADIUS, 5);
//
//        // 2
//        getSpecimen();
//        hangSpecimen(-0.5);
//
//        // 3
//        getSpecimen();
//        hangSpecimen(-0.5+1.5); // 1
//
//        // 4
//        getSpecimen();
//        hangSpecimen(3.5); // technically should be 2.5
//
//        // movement.moveTo(-45, TILE*2, 8);
//        // movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 0);
//        movement.stopAtPositionPI(90, TILE + HALF_OF_ROBOT, 0);
//        while (opModeIsActive()) {
//            movement.holdPosition();
//        }
//    }
//}