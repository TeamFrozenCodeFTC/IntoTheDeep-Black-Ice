package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxSpecimen extends RobotMovement {
    public void hangSpecimen(double x) {
        viperSlide.topBarRaise();
        brakeToPosition(-90, x, 30, wideErrorMargin);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        do {
            targetY = 24+24-(24-18);
            updatePosition();
            goStraight(0.4);
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.topBarPull();
        odometry.setHeading(-90);
        //odometry.setY(31);
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();
    }

    public void goAroundSubmersible() {
        // Move away from submersible
        goThroughPosition(-90, 4, TILE + EDGE_OF_TILE - 1.5);
        // Turn and Go towards sample
        brakeToPosition(-180, TILE + HALF_OF_ROBOT, TILE + EDGE_OF_TILE - 3);
    }

    public void getSpecimen() {
        do {
            targetY = -3;
            updatePosition();
            goStraight(0.2); // velocity instead of power?
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.clawGrab();
        sleep(250);
        viperSlide.bottomBasketRaise();
        sleep(250);

        // arc?
        goThroughPosition(90+15, TILE * 2 - 5, 5);
        goThroughPosition(180, TILE * 2 - 5 - 5, 11);
    }

    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.clawGrab();
        viperSlide.maxInitRaise();

        waitForStart();

        odometry.setPosition(-90, 0, 0); // -90

        hangSpecimen(0);
        goAroundSubmersible();

        // Hook sample
        goThroughPosition(-180, TILE * 2 - 5, TILE + EDGE_OF_TILE + HALF_OF_ROBOT);
        // Turn and push sample to observation zone
        brakeToPosition(90, TILE * 2 - 5, 3);

        getSpecimen();
        hangSpecimen(2);
        goAroundSubmersible();

        brakeToPosition(-180, TILE * 2 + HALF_OF_ROBOT + 2, TILE + EDGE_OF_TILE + 9);
        // Turn and push sample to observation zone
        goThroughPosition(90, TILE * 2 + HALF_OF_ROBOT, TILE);
        brakeToPosition(90, TILE * 2 - 5, 3);

        getSpecimen();
        hangSpecimen(4);

        brakeToPosition(90, TILE * 2 - 5, 3);
        getSpecimen();
        hangSpecimen(-2);

        goAroundSubmersible();
        brakeToPosition(90, TILE * 3 - HALF_OF_ROBOT, TILE * 2 + 1);
    }
}


