package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class ForwardDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.init(this);

        waitForStart();

        while (opModeIsActive()) {
            Drive.power(Drive.forward(0.4));
        }
    }
}