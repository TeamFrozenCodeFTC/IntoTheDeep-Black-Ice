package org.firstinspires.ftc.teamcode.autonomous.tests;

import org.firstinspires.ftc.teamcode.ViperSlide;
import org.firstinspires.ftc.teamcode.autonomous.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TouchSensorTest extends Autonomous {
    public void go() {
        double percentage = 1 - (double) viperSlideMotor.getCurrentPosition() / ViperSlide.MAX_TICKS;
        telemetry.addData("Percentage", percentage);
        telemetry.update();
        double power = 0.5 * percentage;
        angleLock(-power, -power, -power, -power);
    }

    @Override
    public void runOpMode() {
        initRobot();

        viperSlide.clawGrab();

        waitForStart();

        viperSlide.topBarRaise();

        // While neither touch sensors are pressed...
        do {
            go();
        } while (!touchRight.isPressed() && !touchLeft.isPressed());

        viperSlide.waitForExtension();
        viperSlide.topBarPull();
        viperSlide.waitForExtension();
        viperSlide.clawOut();
        viperSlide.lower();

        sleep(3000);
    }
}
