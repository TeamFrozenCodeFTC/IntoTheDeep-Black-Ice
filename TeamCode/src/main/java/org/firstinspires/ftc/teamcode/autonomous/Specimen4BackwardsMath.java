
package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_SUBMERSIBLE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.ROBOT_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Specimen")
public class Specimen4BackwardsMath extends Robot {
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
        viperSlide.clawGrab();

        waitForStart();

        odometry.setPosition(-90, 0, 0);

        hangSpecimen(-2);

        movement.moveTo(-90, 0, TILE + EDGE_OF_TILE - EXTRA_TURN_RADIUS - 1);
        movement.moveTo(0, TILE + HALF_OF_ROBOT, TILE + EDGE_OF_TILE - EXTRA_TURN_RADIUS - 1);

        double sampleY = TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 3.5;

        // Align with 1st Sample
        movement.moveTo(0, TILE + HALF_OF_ROBOT + EDGE_OF_TILE, sampleY);
        // Hook first Sample
        movement.moveTo(0, TILE + HALF_OF_ROBOT + EDGE_OF_TILE + 5, sampleY);
        // Turn and Push Sample
        movement.turnAndMoveTo(-90, TILE*2, 5);

        // Back out of observation zone without turning
        movement.movePast(-90, TILE*2, ROBOT_TURN_RADIUS);
        // Turn and align with 2nd sample
        movement.turnAndMoveTo(0, TILE*2, sampleY);

        // Hook 2nd sample
        movement.moveTo(0, TILE*2 + 4, sampleY);
        // Push 2nd Sample
        movement.turnAndMoveTo(-90, TILE + HALF_OF_ROBOT, 5);

        // Back out of observation zone without turning
        movement.movePast(-90, TILE*2 + HALF_OF_ROBOT, 5 + HALF_OF_ROBOT);

        // Align with 3rd Sample
        movement.turnAndMoveTo(-180, TILE*2 + HALF_OF_ROBOT, sampleY);
        // Hook 3rd Sample
        movement.moveAgainstWall(-180, TILE*3 - HALF_OF_ROBOT, sampleY);
        // Push 3rd Sample
        movement.moveAgainstWall(-180, TILE*3 - HALF_OF_ROBOT, 3);

        movement.moveTo(0, TILE*3 - HALF_OF_ROBOT - 4, 3);

        getSpecimen();
        hangSpecimen(-0.5);

        getSpecimen();
        hangSpecimen(1);

        getSpecimen();
        hangSpecimen(2.5);
    }
}