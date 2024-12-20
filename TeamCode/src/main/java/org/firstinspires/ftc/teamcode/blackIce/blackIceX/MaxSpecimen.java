package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.teamcode.ViperSlide;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxSpecimen extends RobotMovement {
//    public void go() {
//        double percentage = 1 - (double) viperSlideMotor.getCurrentPosition() / ViperSlide.MAX_TICKS;
//        telemetry.addData("Percentage", percentage);
//        telemetry.update();
//        double power = 0.5 * percentage;
//        goStraight(-power);
//    }

    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.clawGrab();
        viperSlide.maxInitRaise(); // maximum raise to 18in
        odometry.setPosition(-90, 0, 0);

        waitForStart();

        viperSlide.topBarRaise();
        brakeToPosition(-90, 0, 30);

        viperSlide.waitForExtension();

        // While neither touch sensors are pressed...
        do {
            targetY = 24+24-(24-18);
            updatePosition();
            goStraight(0.5); // velocity instead of power?
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.topBarPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();

        goThroughPosition(-90, 4, 28-4);
        brakeToPosition(90, 36, 28-4);
        holdFor(3);

//        goToPosition(-90, 24 + 9, 24);
//        goToPosition(-90, 24+9, 0);

    }
}

