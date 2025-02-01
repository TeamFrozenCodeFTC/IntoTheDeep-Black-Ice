package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;

// Load specimen upside down
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
public class X extends Robot {
    public void hangSpecimen(double x) {
        viperSlide.upperChamberRaise();

        movement.stopAtPosition(-90, x, 26);

        viperSlide.waitForExtension();

        viperSlide.upperChamberPull();
        movement.buildMovement(-90, x, 30)
            .stopAtPosition()
            .setMaxPower(0.3)
            .setMovementExit((MovementBuild movementBuild) -> !viperSlide.isExtended())
            .run();

        viperSlide.clawOut();
        viperSlide.lower();
    }

    public void getSpecimen() {
        movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);
        movement.buildMovement(90, TILE + HALF_OF_ROBOT, -3)
            .stopAtPosition()
            .setMaxPower(0.3)
            .runTimeout(0.2);
        viperSlide.clawGrab();
        sleep(200);
        viperSlide.bottomBasketRaise();
        sleep(250);
    }

    @Override
    public void runOpMode() {
        initRobot();

        intake.armIn();
        viperSlide.clawGrab();

        waitForStart();

        // Start with claw facing submersible
        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);

        viperSlide.upperChamberRaise();

        movement.stopAtPosition(-90, 10.25, 28);

        viperSlide.waitForExtension();
        viperSlide.clawOut();

        movement.moveThrough(-90, 10.25, 25); // Back up

        viperSlide.lower();

        movement.moveThrough(-90, 32, 26); // Go past submersible
        movement.stopAtPosition(-90, 40, 36); // Get Sample 1
        movement.turnAndMoveThrough(-180, 47, 7.75); // push sample 1

        movement.moveThrough(-180, 47, 12);
        movement.turnAndMoveThrough(-90, 46, 21); // turn and go

        movement.stopAtPosition(-90, 47.5, 36); // line up with sample 2
        movement.moveThrough(-90, 50, 36);
        movement.turnAndMoveThrough(-180, 56, 7.75); // push sample in

        movement.moveThrough(-180, 56, 12);
        movement.buildMovement(90, 58.5, 36)
            .stopAtPosition()
            .setHeadingCorrection(movement.headingCorrections.locked)
            .run();

        movement.moveThrough(90, 60, 36);
        intake.armIn();
        movement.stopAtPosition(90, 60, 4);

        drive.power(drive.backward(0.3));
        sleep(300);

        viperSlide.clawGrab();
        // may need sleep here
        viperSlide.upperChamberRaise();

        hangSpecimen(10-1.5);

        getSpecimen();
        hangSpecimen(10-3);

        getSpecimen();
        hangSpecimen(10-3.5);

        getSpecimen();
        hangSpecimen(10-4.5);
    }
}