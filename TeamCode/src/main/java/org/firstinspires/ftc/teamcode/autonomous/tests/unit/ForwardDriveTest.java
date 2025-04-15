package org.firstinspires.ftc.teamcode.autonomous.tests.unit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;


/**
 * Test to see if the robot's wheels are all working in the correct directions.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group="Tests")
public class ForwardDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower.initAuto(this);

        waitForStart();

        while (opModeIsActive()) {
            DrivePowers.forward(0.4)
                .applyPowers();
        }
    }
}