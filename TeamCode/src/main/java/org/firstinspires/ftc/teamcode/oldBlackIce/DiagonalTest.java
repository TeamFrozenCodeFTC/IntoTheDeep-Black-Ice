package org.firstinspires.ftc.teamcode.oldBlackIce;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DiagonalTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(90, 0, 0);

        brakeToPosition(0, 10, 10);

        brake();

        while (opModeIsActive()) {
            telemetry.addData("X", odometry.x);
            telemetry.addData("y", odometry.y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();
        }
    }
}