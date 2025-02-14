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
//public class HangUpsideDown extends Robot {
//    public void hangSpecimen(double x) {
//        viperSlide.upperChamberRaise();
//
//        movement.moveThrough(-90, x, 30);
//
//        movement.buildMovement(-90, x, 32)
//            .stopAtPosition()
//            .setMaxPower(0.5)
//            .runTimeout(0.5);
//
//        viperSlide.upperChamberPull();
//        viperSlide.waitForExtension();
//
//        viperSlide.clawOut();
//        viperSlide.lower();
//    }
//
//    public void getSpecimen() {
//        movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);
//        movement.buildMovement(90, TILE + HALF_OF_ROBOT, -3)
//            .stopAtPosition()
//            .setMaxPower(0.4)
//            .runTimeout(0.3);
//
//        viperSlide.clawGrab(); // claw not holding
//        sleep(200);
//        viperSlide.bottomBasketRaise();
//        sleep(250);
//    }
//
//    MovementBuild toSubmersibleMovement;
//    public void hangFirstUpsideDown() { // make more consistent
//        // Start with claw facing submersible
//        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);
//
//        viperSlide.raise(2430);
//
//        toSubmersibleMovement = movement.buildMovement(-90, 10.25, 30.5)
//            .stopAtPosition()
//            .setHeadingCorrection(movement.headingCorrections.locked)
//            .setMovementExit(() -> {
//                double progress = (double) (1903 - viperSlideMotor.getCurrentPosition()) / 1903;
//                toSubmersibleMovement.setMaxPower(progress+0.1);
//                return viperSlide.isExtended();
//            });
//        toSubmersibleMovement.run();
//
//        viperSlide.clawOut();
//
//        movement.moveThrough(-90, 10.25, 25); // Back up
//
//        viperSlide.lower();
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
////        // Start with claw facing submersible
////        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);
////
////        viperSlide.raise(2370);
////
////        movement.stopAtPosition(-90, 10.25, 30);
////
////        movement.buildMovement(-90, 10.25, 30)
////            .moveTo(0)
////            .setHeadingCorrection(movement.headingCorrections.locked)
////            .setMovementExit(() -> viperSlide.isExtendedPast(1903))
////            .run();
////
////        movement.buildMovement(-90, 10.25, 32)
////            .moveTo(0)
////            .setMaxPower(0.5)
////            .setHeadingCorrection(movement.headingCorrections.locked)
////            .setMovementExit(() -> viperSlide.isExtended())
////            .run();
////
//////        movement.buildMovement(-90, 10.25, 32)
//////            .stopAtPosition() // Moves through the position
//////            .setMaxPower(0.5) // Sets the maximum power to 50%
//////            // Makes the robot turn over the whole movement instead of turning instantly
//////            .setHeadingCorrection(movement.headingCorrections.turnOverMovement)
//////            // Makes the robot hold its position until the slide is raised
//////            .setMovementExit(() -> viperSlide.isExtended())
//////            .run(); // Runs the movement
////
////        viperSlide.clawOut();
////
////        movement.moveThrough(-90, 10.25, 25); // Back up
//
//        //viperSlide.lower();
//        hangFirstUpsideDown();
//
//        movement.moveThrough(-90, 32, 26); // Go past submersible
//        movement.stopAtPosition(-90, 40, 36); // Get Sample 1
//        movement.turnAndMoveThrough(-180, 47, 7.75); // push sample 1
//
//        movement.moveThrough(-180, 47, 12);
//        movement.turnAndMoveThrough(-90, 46, 21); // turn and go
//
//        movement.stopAtPosition(-90, 47.5, 36); // line up with sample 2
//        movement.moveThrough(-90, 50+2, 36);
//        movement.turnAndMoveThrough(-180, 56, 7.75); // push sample in
//
//        movement.moveThrough(-180, 56, 12);
//        movement.buildMovement(90, 58.5, 36)
//            .stopAtPosition()
//            .setHeadingCorrection(movement.headingCorrections.locked)
//            .run();
//
//        movement.moveThrough(90, 60, 36);
//        intake.armIn();
//        movement.stopAtPosition(90, 60, 4);
//
//        drive.power(drive.backward(0.4));
//        sleep(400);
////
//        viperSlide.clawGrab();
////        // may need sleep here
//        viperSlide.upperChamberRaise();
////
//        //movement.stopAtPosition(-90, 10.25-1.5, 28);
//        hangSpecimen(10.25-1);
//        getSpecimen();
//
//        hangSpecimen(10.25-1);
//        getSpecimen();
//
//        hangSpecimen(10.25-1);
//        getSpecimen();
//
//        hangSpecimen(10.25-1);
//    }
//}