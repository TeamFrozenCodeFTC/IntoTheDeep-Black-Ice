package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;

public abstract class Specimen extends Robot {
    public void hangSpecimen(double x) {
        viperSlide.upperChamberRaise();

        movement.moveThrough(-90, x, 30);

        movement.buildMovement(-90, x, 32)
            .stopAtPosition()
            .setMaxPower(0.5)
            .runTimeout(0.5);

        viperSlide.upperChamberPull();
        viperSlide.waitForExtension();

        viperSlide.clawOut();
        viperSlide.lower();
    }

    public void getSpecimen() {
        movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);
        movement.buildMovement(90, TILE + HALF_OF_ROBOT, -3)
            .stopAtPosition()
            .setMaxPower(0.4)
            .runTimeout(0.3);

        viperSlide.clawGrab(); // claw not holding
        sleep(200);
        viperSlide.bottomBasketRaise();
        sleep(250);
    }

    MovementBuild toSubmersibleMovement;
    public void hangFirstUpsideDown() { // make more consistent
        // Start with claw facing submersible
        odometry.setPosition(-90, HALF_OF_ROBOT + EDGE_OF_TILE, 0);

        viperSlide.raise(2430);

        toSubmersibleMovement = movement.buildMovement(-90, 10.25, 30.5)
            .stopAtPosition()
            .setHeadingCorrection(movement.headingCorrections.locked)
            .setMovementExit(() -> {
                double progress = (double) (2100 - viperSlideMotor.getCurrentPosition()) / 2100;
                toSubmersibleMovement.setMaxPower(progress+0.1);
                return viperSlide.isExtended();
            });
        toSubmersibleMovement.run();

        viperSlide.clawOut();

        movement.moveThrough(-90, 10.25, 25); // Back up

        viperSlide.lower();
    }
}