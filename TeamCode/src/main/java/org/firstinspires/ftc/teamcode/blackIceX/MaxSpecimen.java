package org.firstinspires.ftc.teamcode.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ViperSlide;
import org.firstinspires.ftc.teamcode.autonomous.Autonomous;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MaxSpecimen extends BlackIce {

    public void angleLock(
            double frontLeft,
            double backLeft,
            double frontRight,
            double backRight
    ) {
        double turnCorrection = Math.min(
                MAX_TURN_POWER,
                simplifyAngle(headingTarget - robotHeading) * TURN_POWER);

        frontLeftWheel.setPower(frontLeft+turnCorrection);
        backLeftWheel.setPower(backLeft+turnCorrection);
        frontRightWheel.setPower(frontRight-turnCorrection);
        backRightWheel.setPower(backRight-turnCorrection);
    }

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
        waitForStart();

        setPosition(-90, 0, 0);

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

        updatePosition();

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();

        //sleep(5000);

        goToPosition(-90, 4, 28-4);
        goToPosition(90, 36, 28-4);
        holdFor(3);

//        goToPosition(-90, 24 + 9, 24);
//        goToPosition(-90, 24+9, 0);

    }
}

