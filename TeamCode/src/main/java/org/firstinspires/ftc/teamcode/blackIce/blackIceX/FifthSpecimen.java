package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FifthSpecimen extends RobotMovement {

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
        goThroughPosition(-180, TILE + HALF_OF_ROBOT, TILE + EDGE_OF_TILE - 3);
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
        waitForStart();

        odometry.setPosition(-90, 0, 31);

        // Move away from submersible
        goThroughPosition(-90, 4, TILE + EDGE_OF_TILE - 1.5);
        // Turn and Go towards sample
        goThroughPosition(-180, TILE, TILE + EDGE_OF_TILE - 3);
        goThroughPosition(-180, TILE * 2, TILE * 2 - 2);

        brakeToPosition(-180, TILE * 3 - HALF_OF_ROBOT - 1, TILE * 2 + 1);
    }
}