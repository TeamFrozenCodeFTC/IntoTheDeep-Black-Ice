
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
public class Specimen5Partner extends Robot {
    public void hangSpecimen(double x) {
        viperSlide.upperChamberRaise();
        movement.quickBrakeTo(-90, x, 28, 10);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        movement.backIntoWall(0.3);

        odometry.setHeading(-90);

        viperSlide.upperChamberPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();
    }

    public void getSpecimen() {
        movement.quickBrakeTo(90, TILE + HALF_OF_ROBOT, 3, 10);

        movement.backIntoWall(0.3);

        odometry.setY(0);

        viperSlide.clawGrab();
        sleep(200);
        viperSlide.bottomBasketRaise();
        sleep(250);
    }

    @Override
    public void runOpMode() {
        initRobot();

        sweeperRotator.getController().pwmDisable();

        waitForStart();

        odometry.setPosition(90, TILE, 0);

        // Push yellow sample towards alliance
        movement.moveAgainstWall(90, TILE - 5, 0);

        double sampleY = TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 3.5;

        // Align with 1st Sample
        movement.moveTo(0, TILE + HALF_OF_ROBOT + EDGE_OF_TILE, sampleY);
        // Hook first Sample
        movement.moveTo(0, TILE + HALF_OF_ROBOT + EDGE_OF_TILE + 3, sampleY);
        // Turn and Push Sample
        movement.turnAndMoveTo(-90, TILE*2, 5);

        // Back out of observation zone without turning
        movement.moveTo(-90, TILE*2, 5 + HALF_OF_ROBOT);
        // Turn and align with 2nd sample
        movement.turnAndMoveTo(0, TILE*2, sampleY);

        // Hook 2nd sample
        movement.moveTo(0, TILE*2 + 4, sampleY);
        // Push 2nd Sample
        movement.turnAndMoveTo(-90, TILE + HALF_OF_ROBOT, 5);

        // Back out of observation zone without turning
        movement.moveTo(-90, TILE*2 + HALF_OF_ROBOT, 5 + HALF_OF_ROBOT);

        // Align with 3rd Sample
        movement.turnAndMoveTo(0, TILE*2 + HALF_OF_ROBOT, sampleY);
        // Hook 3rd Sample
        movement.moveAgainstWall(0, TILE*3 - HALF_OF_ROBOT, sampleY);
        // Push 3rd Sample
        movement.moveAgainstWall(0, TILE*3 - HALF_OF_ROBOT, 3);

        movement.moveTo(0, TILE*3 - HALF_OF_ROBOT - 4, 3);

        getSpecimen();
        viperSlide.upperChamberRaise();
        movement.quickBrakeTo(-90, 4, 28, 10);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        movement.backIntoWall(0.3);

        movement.stopAtPositionOnWall(-90, -4, 32);

        viperSlide.upperChamberPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();

        getSpecimen();
        hangSpecimen(-2);

        getSpecimen();
        hangSpecimen(-0.5);

        getSpecimen();
        hangSpecimen(1);
    }
}