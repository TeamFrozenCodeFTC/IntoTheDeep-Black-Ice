package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Config
@Autonomous
public class ZigZagDecel extends LinearOpMode {
    public static long TIME = 700;
   
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        
        follower.drivetrain.zeroPowerFloatMode();
        
        waitForStart();
        
        // cruise at velocity, coast at zero power, apply -0.1 power to brake
        // to stop faster than zero power deceleration you have to apply negative power
        
        follower.drivetrain.followVector(Vector.FORWARD, 0);
        
        sleep(TIME);
        
        follower.update();
        double startingDistance = follower.motionState.position.dotProduct(Vector.FORWARD);
        double maxVelocity = follower.motionState.velocity.dotProduct(Vector.FORWARD);

        int i = 0;
        while (opModeIsActive()) {
            i++;
            follower.update();
            Logger.debug("velocity", follower.motionState.velocity.dotProduct(Vector.FORWARD));
            
            if (i % 4 == 0) {
                Logger.debug("negative", follower.motionState.velocity.times(-0.015));
                follower.drivetrain.followVector(
                    follower.motionState.velocity.times(-0.015).withMaxMagnitude(0.3), 0);
            }
            else {
                Logger.debug("zero");
                follower.drivetrain.followVector(new Vector(0, 0), 0);
            }
            if (follower.motionState.velocity.dotProduct(Vector.FORWARD) < 0.005) {
                break;
            }
        }
        
        follower.update();
        double endingDistance = follower.motionState.position.dotProduct(Vector.FORWARD);
        double stoppingDistance = Math.abs(endingDistance - startingDistance);
        
        Logger.info("Starting distance", startingDistance);
        Logger.info("Ending distance", endingDistance);
        Logger.info("Stopping distance", stoppingDistance);
        Logger.info("Max velocity", maxVelocity);
        
        // Step 4: Log linearity ratio
        double ratio = stoppingDistance / maxVelocity;
        Logger.info("Stopping distance per velocity (d/v)", ratio);
        
        double deceleration = (maxVelocity * maxVelocity) / (2 * stoppingDistance);
        Logger.info("Deceleration (inches/s/s)", deceleration);
    }
}

