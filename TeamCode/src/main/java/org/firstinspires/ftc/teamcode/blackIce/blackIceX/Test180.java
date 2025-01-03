package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Test180 extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        quickBrakeTo2(0, -20, 0);
        quickBrakeTo2(180, 20, 0);
        holdFor(5);
    }
}