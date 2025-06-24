package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import android.util.Log;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.MotionProfile;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.pidf.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

public class WheelPowersCalculator { // or DriveCalculator
    public final MotionProfile motionProfile;
    public final TranslationalVectorCalculator translatingPowers;
    
    public final BrakingCalculator brakingPowers;
    public final HeadingPowerCalculator headingCorrectionPowers;
    
    /**
     * Responsible for turning the robot and making sure it is facing the correct direction.
     */
    public final PIDFController headingPIDF;
    
    /**
     * Responsible holding a given pose and giving translational power for the
     * robot to stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    public final PIDFController poseStabilizationPIDF;
    
    /**
     * Responsible giving translational power.
     */
    public final PIDFController translationalPIDF;

    /**
     * Responsible accelerating and decelerating the robot's velocity.
     */
    public final PIDFController drivePIDF;
    
    public WheelPowersCalculator(
        MotionProfile motionProfile,
        TranslationalVectorCalculator translationalDrive,
        BrakingCalculator brakingPowers,
        HeadingPowerCalculator headingPowers,
        PIDFController headingPIDF,
        PIDFController poseStabilizationPIDF, PIDFController translationalPIDF,
        PIDFController drivePIDF
    ) {
        this.motionProfile = motionProfile;
        this.translatingPowers = translationalDrive;
        this.brakingPowers = brakingPowers;
        this.headingCorrectionPowers = headingPowers;
        this.headingPIDF = headingPIDF;
        this.poseStabilizationPIDF = poseStabilizationPIDF;
        if (translationalPIDF == null) {
            this.translationalPIDF = poseStabilizationPIDF;
        }
        else {
            this.translationalPIDF = translationalPIDF;
        }
        this.drivePIDF = drivePIDF;
    }
    
    public Vector computeDrivePowerVector(PathFollowContext context) {
        Vector drivePower;
        if (motionProfile == null) {
            drivePower = context.closestPoint.getTangentVector();
            Logger.debug("driving along path at full power");
        }
        else {
            Vector targetDriveVelocity =
                // closest point to robot?
                motionProfile.computeTargetVelocity(context.closestPoint.getTangentVector(),
                    context.motionState, context.distanceToEnd);
            drivePower = drivePIDF.run(
                targetDriveVelocity,
                context.motionState.fieldRelativeVelocity,
                context.motionState.deltaTime
            );
            Logger.debug("targetDriveVelocity", targetDriveVelocity);
            Logger.debug("currentVelocity", context.motionState.fieldRelativeVelocity);
            Logger.debug("drivePower", drivePower);
        }
        return drivePower.withMaxMagnitude(1);
    }
    //right now I am getting the closest point on the path to the robot but I also have the
    // closest point to the robot predicted position which is just position + the braking displacement relative to how fast it is going. I have a drive vector that accelerates and slows the robot down. Once the closest point to the robot's braking displacement is the end point it removes the drive vector and switches to a positional PID of (endPoint - predictedStoppedPosition). But I also have a transitional vector that currently goes from the closest point to the robot on the path but if the robot ever overshoots along the path the transitional isn't predictive and wont react until it happens. I have tried making the proportional (closestPointToBraking - predictedBrakingPosition) but when the robot decelerates it makes the closestPoint to braking really unpredictable and it vibrates around making the robot shake when It decelerates near the end point. are there any other solutions?
    public void accelerate(Drivetrain drivetrain,
                           PathFollowContext context) {
//        Vector translationalPower;
//        if (motionProfile.isDecelerating(context.closestPoint.getTangentVector(),
//            context.motionState, context.distanceToEnd)) {
//            Logger.debug("decelerating");
//            translationalPower = poseStabilizationPIDF.run(
//                context.closestPointToRobot.getPoint(),
//                context.motionState.position,// context
//                // .closestPoint.getPoint().subtract
//                // (context.motionState.position) Use current position instead of predicted? then it
//                // would be forward power.
//                context.motionState.deltaTime
//            );
//        }
//        else {
//
//        }
        Vector translationalPower = translationalPIDF.run(
            context.closestPoint.getPoint(),
            context.motionState.getPredictedStoppedPosition(),// context
            // .closestPoint.getPoint().subtract
            // (context.motionState.position) Use current position instead of predicted? then it
            // would be forward power.
            context.motionState.deltaTime
        );
        
        Logger.debug("translationalPower", translationalPower);
        Logger.debug("trans closestPoint", context.closestPoint.getPoint());
        Logger.debug("trans predictedPosition", context.motionState.getPredictedStoppedPosition());
        double transitionalMag = translationalPower.computeMagnitude();
        
        Vector drivePower = computeDrivePowerVector(context);
        
        Vector totalPower;
        if (transitionalMag < 1) {
            totalPower = translationalPower.add(drivePower.withMaxMagnitude(1 - transitionalMag));
        }
        else {
            totalPower = translationalPower.withMagnitude(1);
        }

        Logger.debug("totalDrivePower", totalPower);
        drivetrain.driveTowards(
            totalPower,
            computeHeadingCorrectionPower(context)
        );
    }
    
    public double computeHeadingCorrectionPower(PathFollowContext context) {
        Logger.debug("targetHeading",
            Math.toDegrees(context.headingInterpolator.interpolate(context.closestPoint)));
        Logger.debug("currentHeading", Math.toDegrees(context.motionState.heading));
        return headingPIDF.run(
            context.headingInterpolator.interpolate(context.closestPoint),
            context.motionState.heading,
            context.motionState.deltaTime
        );
    }
    
    public void positionalHold(Drivetrain drivetrain,
                               PathFollowContext context) {

        drivetrain.driveTowards(
            poseStabilizationPIDF.run(
                context.closestPoint.getPoint(),
                context.motionState.getPredictedStoppedPosition(),
                context.motionState.deltaTime
            ).toRobotVector(context.motionState.heading),
            computeHeadingCorrectionPower(context)
        );
    }
}


//        Vector targetVelocity = brakingPowers.computeTargetVector(context);
//        Follower.getInstance().telemetry.addData("targetVelocity", targetVelocity);
//
//        if (targetVelocity == null) {
//            isNull = true;
//        }
//        if (isNull) {
//            drivetrain.driveTowards(
//                poseStabilizationPIDF.run(
//                    TranslationalVectorCalculator.toEndPose.computeTargetVector(context),
//                    context.motionState.deltaTime
//                ),
//                computeHeadingCorrectionPower(context)
//            );
//        }
//        else {
//            drivetrain.driveTowards(
//                drivePIDF.run(
//                    targetVelocity,
//                    context.motionState.deltaTime
//                ).add(PIDFController.createProportionalFeedforward(0.1, 0).run(
//                    TranslationalVectorCalculator.toClosestPose.computeTargetVector(context),
//                    context.motionState.deltaTime
//                )),
//                computeHeadingCorrectionPower(context)
//            );
//        }
