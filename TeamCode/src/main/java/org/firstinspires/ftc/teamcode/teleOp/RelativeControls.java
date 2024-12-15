package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class RelativeControls extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        viperSlide.lower();

        SampleControls specimenControls = new SampleControls(this);
        RelativeWheelControls relativeWheelControls = new RelativeWheelControls(this);

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                specimenControls.control();
            }
        }).start();

        while (opModeIsActive()) {
            relativeWheelControls.control();
            telemetry.update();
        }
    }
}
