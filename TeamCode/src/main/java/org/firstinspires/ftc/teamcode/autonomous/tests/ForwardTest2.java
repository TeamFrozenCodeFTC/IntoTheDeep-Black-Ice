package org.firstinspires.ftc.teamcode.autonomous.tests;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class ForwardTest2 extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        Odometry.setPosition(90, 0, 0);

        Movement.stopAtPosition(90, 0, 24);
        
    }
}