package org.firstinspires.ftc.teamcode.autonomous.tests;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class BackwardTurnTest extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        movement.moveTo(0, -20, 0);
        movement.stopAtPositionPI(180, 20, 0);
//        sleep(3000);
    }
}