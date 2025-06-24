package org.firstinspires.ftc.teamcode.blackIce.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PIDF Test", group = "Testing")
@Config
public class PIDF_Test extends LinearOpMode {
    private DcMotorEx motor;

    // These values can be changed live via FTC Dashboard or DS config UI
    public static double p = 1.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    public static int targetPosition = 1000;  // Encoder ticks

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1.0);  // Full power (let PIDF handle speed)

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            // Update PIDF live
            PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);

            // Update target position if changed
            motor.setTargetPosition(targetPosition);

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.addData("Error", targetPosition - motor.getCurrentPosition());
            telemetry.addData("At Target?", !motor.isBusy());
            telemetry.update();

            sleep(50);  // Loop delay
        }
    }
}