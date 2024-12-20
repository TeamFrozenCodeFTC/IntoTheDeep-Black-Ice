package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class OdometryTest extends Robot {
    @Override
    public void runOpMode() {
        initOdometry();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();

            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);
            telemetry.addData("heading", odometry.heading);

            telemetry.update();
        }

    }
}