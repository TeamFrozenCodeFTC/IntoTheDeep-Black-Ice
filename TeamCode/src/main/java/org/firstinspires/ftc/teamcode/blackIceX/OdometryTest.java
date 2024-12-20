package org.firstinspires.ftc.teamcode.autonomous.custom;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.blackIceX.BlackIce;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class OdometryTest extends BlackIce {
    @Override
    public void runOpMode() {
        initOdometry();
        waitForStart();

        while (opModeIsActive()) {
            updatePosition();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", robotHeading);

            double[] powers = getMecanumWheelPower(0, 0, 0);
            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("backLeft", powers[1]);
            telemetry.addData("frontRight", powers[2]);
            telemetry.addData("backRight", powers[3]);

            telemetry.update();
        }

    }
}