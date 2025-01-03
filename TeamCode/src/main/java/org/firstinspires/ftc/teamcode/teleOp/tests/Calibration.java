package org.firstinspires.ftc.teamcode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@TeleOp(group="Debugging")
public class Calibration extends Robot {
    @Override
    public void runOpMode() {
        initRobot();

        waitForStart();
        resetTicks(backRightWheel);

        while(opModeIsActive()) {
            double power = 0.7;

            if (gamepad1.right_stick_y != 0) {
                power = gamepad1.right_stick_y;
            }
            if (gamepad1.triangle) {
                resetTicks(backRightWheel);
            }

            if (gamepad1.dpad_up) {
                frontLeftWheel.setPower(power);
                frontRightWheel.setPower(power);
                backLeftWheel.setPower(power);
                backRightWheel.setPower(power);
            }
            else if (gamepad1.dpad_down) {
                frontLeftWheel.setPower(-power);
                frontRightWheel.setPower(-power);
                backLeftWheel.setPower(-power);
                backRightWheel.setPower(-power);
            }
            else if (gamepad1.dpad_left) {
                frontLeftWheel.setPower(power);
                frontRightWheel.setPower(-power);
                backLeftWheel.setPower(-power);
                backRightWheel.setPower(power);
            }
            else if (gamepad1.dpad_right) {
                frontLeftWheel.setPower(-power);
                frontRightWheel.setPower(power);
                backLeftWheel.setPower(power);
                backRightWheel.setPower(-power);
            }
            else {
                frontLeftWheel.setPower(0);
                frontRightWheel.setPower(0);
                backLeftWheel.setPower(0);
                backRightWheel.setPower(0);
            }

            telemetry.addData("Ticks Traveled", backRightWheel.getCurrentPosition());
            telemetry.addData("Power:", power);

            telemetry.update();
        }
    }
}
