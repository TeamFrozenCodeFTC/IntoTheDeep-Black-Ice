package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;

// Load specimen upside down
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
public class SpaceSpecimen extends Specimen {
    @Override
    public void runOpMode() {
        initRobot();

        intake.armIn();
        viperSlide.clawGrab();

        waitForStart();

        hangFirstUpsideDown(10.25);

        movement.moveThrough(-90, 32, 26); // Go past submersible
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

        hangSpecimen(10.25-2);
        getSpecimen();

        hangSpecimen(10.25-4);
        getSpecimen();

        hangSpecimen(10.25-6);
        getSpecimen();

        hangSpecimen(10.25-8);

        movement.stopAtPosition(-90-45, TILE + EDGE_OF_TILE, 3);
    }
}