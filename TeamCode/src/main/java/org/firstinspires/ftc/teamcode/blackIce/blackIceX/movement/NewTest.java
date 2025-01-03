package org.firstinspires.ftc.teamcode.blackIce.blackIceX;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement.Movement;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class NewTest extends Robot {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        movement.moveTo(0, -20, 0);
        movement.stopAtPosition(180, 20, 0);
//        sleep(3000);
    }
}