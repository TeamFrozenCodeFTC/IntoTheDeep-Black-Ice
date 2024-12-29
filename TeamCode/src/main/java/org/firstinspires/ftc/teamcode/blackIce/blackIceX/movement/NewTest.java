package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Movement;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class NewTest extends Movement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        moveTo(0, -20, 0);
        stopAtPosition(180, 20, 0);
//        sleep(3000);
    }
}