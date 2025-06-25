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
        
        follower.getDrivetrain().driveTowards(Vector.FORWARD, 0);
        
        sleep(1000);
        
        follower.update();
        follower.getDrivetrain().zeroPower();
        double maxVelocity = follower.getMotionState().velocityMagnitude;
        telemetry.addData("maxVelocity", maxVelocity);
        telemetry.addData("Your feedforward should be about", 1/maxVelocity);
        telemetry.update();
        
        follower.waitUntilOpModeStop();
    }
}
