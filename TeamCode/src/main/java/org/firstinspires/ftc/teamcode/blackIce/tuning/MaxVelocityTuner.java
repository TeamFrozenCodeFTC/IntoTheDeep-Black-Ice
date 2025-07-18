package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

@TeleOp
public class MaxVelocityTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        
        waitForStart();
        
        follower.drivetrain.followVector(Vector.FORWARD.times(0.5), 0);
        
        sleep(2000);
        
        // velocity at end point with zero deceleration
        // vs velocity at end point with current velocity
        
        follower.update();
        follower.drivetrain.zeroPower();
        double maxVelocity = follower.getMotionState().speed;
        telemetry.addData("maxVelocity", maxVelocity);
        telemetry.addData("Your feedforward should be about", 1/maxVelocity);
        // 0.0158 for 63v
        telemetry.update();
        
        follower.waitUntilOpModeStop();
    }
}
