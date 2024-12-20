package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.teamcode.ViperSlide;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Specimen2 extends RobotMovement {
    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.clawGrab();
        viperSlide.maxInitRaise(); // maximum raise to 18in
        odometry.setPosition(-90, 0, 0);

        waitForStart();

        viperSlide.topBarRaise();
        brakeToPosition(-90, 0, TILE + EDGE_OF_TILE);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        do {
            targetY = TILE + EDGE_OF_TILE + 4;
            updatePosition();
            goStraight(0.5); // velocity instead of power?
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.topBarPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();

        goThroughPosition(-90, 4, TILE + EDGE_OF_TILE - 4);
        brakeToPosition(0, TILE + HALF_OF_ROBOT, TILE + EDGE_OF_TILE - 4);
        holdFor(3);
        goThroughPosition(0, TILE * 3 - HALF_OF_ROBOT, TILE * 3 - HALF_OF_ROBOT - 4);
        goThroughPosition(0, TILE * 3 - HALF_OF_ROBOT + 4, TILE * 3 - HALF_OF_ROBOT - 4);

        brakeToPosition(90, TILE * 3, 0);
        holdFor(3);
    }
}

