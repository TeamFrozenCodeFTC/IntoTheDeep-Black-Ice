package org.firstinspires.ftc.teamcode.blackIce.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.QuadraticLinearBrakingModel;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Config
@Autonomous
public class MinimalPowerDeceleration extends LinearOpMode {
    public static double POWER = -0.05;
    public static double DIRECTION = 1;
    public static double CONSTRAINT = 0.01;
    public static long TIME = 700;
    public static double MAX_POWER = 0.3;

    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));
        
        follower.drivetrain.zeroPowerFloatMode();
        
        waitForStart();
        
        // cruise at velocity, coast at zero power, apply -0.1 power to brake
        // to stop faster than zero power deceleration you have to apply negative power
        
        follower.drivetrain.followVector(Vector.FORWARD.times(DIRECTION), 0);
        
        follower.update();
        double startingDistance = follower.motionState.position.dotProduct(Vector.FORWARD);
        double maxVelocity = follower.motionState.velocity.dotProduct(Vector.FORWARD);
        
        while (opModeIsActive()) {
            follower.update();
            Logger.debug("velocity", follower.motionState.velocity.dotProduct(Vector.FORWARD));
            follower.drivetrain.followVector(follower.motionState.velocity.times(POWER).withMaxMagnitude(MAX_POWER), 0);
            
            
            if (follower.motionState.velocity.dotProduct(Vector.FORWARD) < CONSTRAINT) {
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


//Starting distance: 19.212394173689717
//Ending distance: 28.14022995355561
//Stopping distance: 8.927835779865894
//Max velocity: 53.746236966350885
//Stopping distance per velocity (d/v): 0.16611089973527596
//Deceleration (inches/s/s): 161.77817666391556

//Ending distance: 14.94032611997109
//Stopping distance: 6.464485709122787
//Max velocity: 43.29764298566683
//Stopping distance per velocity (d/v): 0.14930340922397964
//Deceleration (inches/s/s): 144.9988423262099



//Starting distance: 30.543181652159205
//Ending distance: 38.55254406065453
//Stopping distance: 8.009362408495328
//Max velocity: 57.55631093903789
//Stopping distance per velocity (d/v): 0.13915697997007886
//Deceleration (inches/s/s): 206.80353565956045





//Starting distance: 30.611444908802905
//Ending distance: 38.71324674351009
//Stopping distance: 8.101801834707185
//Max velocity: 59.01369590458908
//Stopping distance per velocity (d/v): 0.1372868062323337
//Deceleration (inches/s/s): 214.92850450872464
//Starting distance: 8.405242679625983
//Ending distance: 13.589598888487327
//Stopping distance: 5.1843562088613435
//Max velocity: 45.88091031772884
//Stopping distance per velocity (d/v): 0.1129959317057852
//Deceleration (inches/s/s): 203.02018676739587
//Starting distance: 4.580040879136934
//Ending distance: 8.617396917868787
//Stopping distance: 4.037356038731852
//Max velocity: 35.99884513794907
//Stopping distance per velocity (d/v): 0.11215237664598784
//Deceleration (inches/s/s): 160.49078144630158
//Starting distance: 4.582875769908034
//Ending distance: 8.350560345987635
//Stopping distance: 3.767684576079601
//Max velocity: 36.388355165015994
//Stopping distance per velocity (d/v): 0.10354094212265681
//Deceleration (inches/s/s): 175.7196449009976
//Starting distance: 30.139006366879922
//Ending distance: 37.72900438684178
//Stopping distance: 7.5899980199618575
//Max velocity: 59.38151682455709
//Stopping distance per velocity (d/v): 0.1278175167264005
//Deceleration (inches/s/s): 232.29021477418493

//Starting distance: 0.5826224680021992
//Ending distance: 2.0184848815437375
//Stopping distance: 1.4358624135415383
//Max velocity: 14.413460558793677
//Stopping distance per velocity (d/v): 0.09961954713682664
//Deceleration (inches/s/s): 72.3425320283624

//Starting distance: 42.6208159679503
//Ending distance: 50.862090343565455
//Stopping distance: 8.241274375615156
//Max velocity: 59.93857556440699
//Stopping distance per velocity (d/v): 0.13749533248015705
//Deceleration (inches/s/s): 217.9658555793418

// -velocity * k