package org.firstinspires.ftc.teamcode.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Test180 extends BlackIce {
    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        goToPosition(0, -20, 0);
        goToPosition(180, 20, 0);
    }
}