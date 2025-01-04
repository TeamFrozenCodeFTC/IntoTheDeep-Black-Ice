package org.firstinspires.ftc.teamcode.oldBlackIce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class SpecimenOpto extends RobotMovement {
    public void hangSpecimen(double x) {
        viperSlide.topBarRaise();
        rotateToTarget(-90, x, 30);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        do {
            targetY = 24+24-(24-18);
            updatePosition();
            goStraight(-0.3);
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.topBarPull();
        odometry.setHeading(-90);
        //odometry.setY(31);
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();
    }

    public void getSpecimen() {
        do {
            targetY = -3;
            updatePosition();
            goStraight(-0.3); // velocity instead of power?
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.clawGrab();
        sleep(200);
        viperSlide.bottomBasketRaise();
        sleep(250);

        // arc?
        //goThroughPosition(90+15, TILE * 2 - 5, 5);
        //goThroughPosition(180, TILE * 2 - 5 - 5, 11);
    }

    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.clawGrab();
        viperSlide.maxInitRaise();

        waitForStart();

        odometry.setPosition(-90, 0, 0); // -90

        hangSpecimen(0);

        // Move away from submersible
        accelerate(-90, 4, TILE + EDGE_OF_TILE - 1.5);
        // Turn and Go towards sample
//        LINEAR_INCH_SLOW_DOWN = 10;
        accelerate(-180, TILE, TILE + EDGE_OF_TILE - 3);

        // 1st sample
        directionChange(-180, TILE * 2 - 8, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2);
        quickBrakeTo( 90, TILE * 2 - 5, 3);

        // 2nd sample + 12
        accelerate(90, TILE * 2 - 5, 12);
        quickBrakeTo(-180, TILE * 2 - 1, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 6);
        directionChange(-180, TILE * 2 + 5, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2);
        quickBrakeTo( 90, TILE * 2, 3);

        accelerate(90, TILE * 2, 12);
        quickBrakeTo(0, TILE * 2 + 5, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 6);
        quickBrakeTo(0, TILE * 3 - HALF_OF_ROBOT, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 6);

        quickBrakeTo(0, TILE * 3 - HALF_OF_ROBOT, 0);

        // Getting Specimen
        quickBrakeTo(0, TILE * 3 - HALF_OF_ROBOT - ROBOT_TURN_RADIUS, ROBOT_TURN_RADIUS);
        quickBrakeTo(90, TILE * 2 - 5, 3);

        // 2
        getSpecimen();
        hangSpecimen(0);

        // 3
        rotateToTarget(90, TILE * 2 - 5, 3);
        getSpecimen();
        hangSpecimen(0);

        // 4
        rotateToTarget(90, TILE * 2 - 5, 3);
        getSpecimen();
        hangSpecimen(0);

        // 5
        rotateToTarget(90, TILE * 2 - 5, 3);
        getSpecimen();
        hangSpecimen(0);
    }
}