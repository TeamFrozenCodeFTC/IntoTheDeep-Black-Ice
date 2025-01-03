package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class Controls extends Robot {
    @Override
    public void runOpMode() {
        initRobot();

        SampleControls specimenControls = new SampleControls(this);
        DriveControls relativeWheelControls = new DriveControls(this);

        timer = new ElapsedTime();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                specimenControls.control();
            }
        }).start();

        timer.reset();

        while (opModeIsActive()) {
            relativeWheelControls.control();
            if (gamepad1.triangle) {
                odometry.resetHeading();
            }

            telemetry.update();

            if (120-timer.seconds() < 30 && 120-timer.seconds() > 29) {
                gamepad1.rumble(0.5, 0.5, 100);
            }

            if (120-timer.seconds() < 35 && odometry.x > 4*24 && odometry.y > 1.5*24) {
                gamepad1.rumble(0.5, 0.5, 100);
            }
        }
    }
}
