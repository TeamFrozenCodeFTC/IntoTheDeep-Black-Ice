package org.firstinspires.ftc.teamcode.blackIce.examples;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

@TeleOp(group="Black-Ice Examples")
public class ApplyPowerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        waitForStart();
        
        follower.drivetrain.followVector(new Vector(0.6, 0.7), 0);
        follower.waitUntilOpModeStop();
    }
}