package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class NewControls extends Robot {
    @Override
    public void runOpMode() {
        initRobot();

        odometry.setPosition(90, TILE + HALF_OF_ROBOT, 0);

        SampleControls specimenControls = new SampleControls(this);
        NewDriveControls relativeWheelControls = new NewDriveControls(this);

        timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        odometry.setPosition(90, TILE + HALF_OF_ROBOT, 0);

        while (opModeIsActive()) {
            specimenControls.control();
            relativeWheelControls.control();

            telemetry.addData("viper slide power", viperSlideMotor.getPower());
            telemetry.update();

            if (120-timer.seconds() < 30 && 120-timer.seconds() > 29) {
                gamepad1.rumble(1, 1, 100);
                gamepad2.rumble(1, 1, 100);
            }

            loopUpdate();
//            else if (120-timer.seconds() < 35 && odometry.x > 4*24 && odometry.y > 1.5*24) {
//                gamepad1.rumble(0.5, 0.5, 100);
//            }
        }
    }
}
