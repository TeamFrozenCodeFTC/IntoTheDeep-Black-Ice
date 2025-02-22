package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;

// Load specimen upside down
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
public class FourSpecimen extends Specimen {
    @Override
    public void runOpMode() {
        initRobot();

        intake.armIn();
        viperSlide.clawOut();

        waitForStart();

        odometry.setPosition(-90, 24, 0);
        movement.target.setTarget(-90, 0, 0);

        movement.moveThrough(-90, 12, 0); // push yellow sample
        movement.stopAtPosition(-90, 40, 36); // Get Sample 1
        movement.turnAndMoveThrough(-180, 47, 7.75); // push sample 1

        movement.moveThrough(-180, 47, 12);
        movement.turnAndMoveThrough(-90, 46, 21); // turn and go

        movement.stopAtPosition(-90, 47.5, 36); // line up with sample 2
        movement.moveThrough(-90, 50+2, 36);
        movement.turnAndMoveThrough(-180, 56, 7.75); // push sample in

        movement.moveThrough(-180, 56, 12);
        movement.buildMovement(90, 58.5, 36)
            .stopAtPosition()
            .setHeadingCorrection(movement.headingCorrections.locked)
            .run();

        movement.moveThrough(90, 60, 36);
        movement.moveThrough(90, 60, 10);

        drive.power(drive.backward(0.3));
        sleep(400);

        viperSlide.clawGrab();
        sleep(200);
        viperSlide.upperChamberRaise();
        sleep(250);

        movement.moveThrough(-90, 11, 23);

        hangSpecimen(10.25-1.5);
        getSpecimen();

        hangSpecimen(10.25-3);
        getSpecimen();

        hangSpecimen(10.25-4.5);
        getSpecimen();

        hangSpecimen(10.25-6);
        movement.stopAtPosition(-90-45, TILE + EDGE_OF_TILE, 3);
        
    }
}