package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Test180 extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        brakeToPosition(0, -20, 0);
        brakeToPosition(180, 20, 0);
    }
}