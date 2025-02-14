//package org.firstinspires.ftc.teamcode.autonomous;
//
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
//import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;
//
//// Load specimen upside down
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
//public class FirstSpecimen extends Robot {
//    MovementBuild toSubmersibleMovement;
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
//        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);
//
//        viperSlide.raise(2400);
//
//        toSubmersibleMovement = movement.buildMovement(-90, 10.25, 31)
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
//
//        movement.moveThrough(-90, 32, 26); // Go past submersible
//    }
//}