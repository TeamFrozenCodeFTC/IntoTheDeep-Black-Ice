//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;
//
//// Load specimen upside down
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
//public class HangSpecimenTest extends Robot {
//    public void hangSpecimen(double x) {
//        viperSlide.upperChamberRaise();
//
//        movement.stopAtPosition(-90, x, 29);
//
//        viperSlide.waitForExtension();
//
//        viperSlide.upperChamberPull();
//        movement.buildMovement(-90, x, 32)
//            .stopAtPosition()
//            .setMaxPower(0.5)
//            .setMovementExit(() -> !viperSlide.isExtended())
//            .run();
//
//        viperSlide.clawOut();
//        viperSlide.lower();
//    }
//
//    public void getSpecimen() {
//        movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);
//        movement.buildMovement(90, TILE + HALF_OF_ROBOT, -3)
//            .stopAtPosition()
//            .setMaxPower(0.3)
//            .runTimeout(0.2);
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
//
//        waitForStart();
//
//        // Start with claw facing submersible
//        odometry.setPosition(-90, 0, TILE + EDGE_OF_TILE);
//
//        getSpecimen();
//        hangSpecimen(0);
//
//        getSpecimen();
//        hangSpecimen(0);
//
//        getSpecimen();
//        hangSpecimen(0);
//    }
//}