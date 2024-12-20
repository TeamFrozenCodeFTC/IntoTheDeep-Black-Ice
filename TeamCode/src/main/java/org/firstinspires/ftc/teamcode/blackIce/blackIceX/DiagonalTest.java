package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DiagonalTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(90, 0, 0);

        brakeToPosition(0, 10, 10);
    }
}