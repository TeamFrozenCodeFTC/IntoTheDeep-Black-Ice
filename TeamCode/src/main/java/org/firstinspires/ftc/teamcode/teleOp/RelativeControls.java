package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.RobotMovement;


@TeleOp
public class RelativeControls extends RobotMovement {
    @Override
    public void runOpMode() {
        initRobot();

        SampleControls specimenControls = new SampleControls(this);
        RelativeWheelControls relativeWheelControls = new RelativeWheelControls(this);

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
            telemetry.update();

            if (120-timer.seconds() < 30 && 120-timer.seconds() > 29) {
                gamepad1.rumble(0.5, 0.5, 100);
            }
        }
    }
}
