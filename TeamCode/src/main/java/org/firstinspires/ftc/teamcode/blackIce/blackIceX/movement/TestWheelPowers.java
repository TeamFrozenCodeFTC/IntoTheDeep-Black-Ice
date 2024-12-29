package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TestWheelPowers extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        setTarget(0, 0, 0);

        while (opModeIsActive()) {
            updatePosition();

            double[] powers = vectorToLocalWheelPowers(xError, yError);

            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);
//
            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("backLeft", powers[1]);
            telemetry.addData("frontRight", powers[2]);
            telemetry.addData("backRight", powers[3]);


            telemetry.update();
        }

    }
}