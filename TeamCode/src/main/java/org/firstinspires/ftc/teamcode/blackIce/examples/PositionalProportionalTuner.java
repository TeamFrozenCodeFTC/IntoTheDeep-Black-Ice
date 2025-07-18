package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;

@Config
@Autonomous
public class PositionalProportionalTuner extends LinearOpMode {
    public static double kP = 0.5;
    public static double kD = 0.0;
    
    // might want different kP and kD for lateral vs forward
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        
        follower.drivetrain.zeroPowerFloatMode();
        
        waitForStart();
        
        while (opModeIsActive()) {
            follower.update();
            // predictive + kD brake term
            follower.drivetrain.followVector(follower.motionState.position.times(-kP)
                .minus((follower.motionState.velocity.times(kD))), 0);
        }
    }
}