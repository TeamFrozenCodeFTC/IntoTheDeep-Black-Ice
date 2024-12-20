package org.firstinspires.ftc.teamcode.blackIceX;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DiagonalTest extends BlackIce {
    @Override
    public void runOpMode() {
        initOdo();
        initWheels();
        waitForStart();

        setPosition(90, 0, 0);

        goToPosition(0, 10, 10);
    }
}