package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

/**
 * Converts target velocities and positions into drive powers and runs them.
 */
public class DrivePowerController {
    /**
     * Responsible for turning the robot and making sure it is facing the correct direction.
     */
    public final PIDController headingPID;
    
    /**
     * Responsible holding a given pose and giving translational power for the
     * robot to stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    public final PIDController poseStabilizationPID;
    
    /**
     * Responsible giving translational power.
     */
    public final PIDController translationalPID;

    /**
     * Responsible accelerating and decelerating the robot's velocity.
     */
    public final PIDFController drivePIDF;
    
    public DrivePowerController(
        PIDController headingPID,
        PIDController poseStabilizationPID,
        PIDController translationalPID,
        PIDFController drivePIDF
    ) {
        this.headingPID = headingPID;
        this.poseStabilizationPID = poseStabilizationPID;
        if (translationalPID == null) {
            this.translationalPID = poseStabilizationPID;
        }
        else {
            this.translationalPID = translationalPID;
        }
        this.drivePIDF = drivePIDF;
    }
    
    public Vector computeDrivePowerVector(PathFollowContext context) {
        Vector drivePower;
        if (context.path.getMotionProfile() == null) {
            drivePower = context.closestPointToRobot.getTangentVector();
            Logger.debug("driving along path at full power");
        }
        else {
            Vector targetDriveVelocity =
                // closest point to robot?
                context.path.getMotionProfile().computeTargetVelocity(context.closestPointToRobot.getTangentVector(),
                    context.motionState, context.distanceToEnd);
            drivePower = drivePIDF.run(
                targetDriveVelocity,
                context.motionState.fieldRelativeVelocity,
//                Kinematics.predictNextLoopVelocity(
//                    context.motionState.fieldRelativeVelocity,
//                    context.motionState.toFieldRelativeVector(context.motionState.robotRelativeVelocity
//                        .subtract(context.motionState.previousRobotRelativeVelocity))),
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

        // for paths with tight turns, con: will 0.25 inches off the path on turns
        Vector translationalPower = translationalPID.run(
            context.closestPoint.getPoint(),
            context.motionState.getPredictedStoppedPosition(),
//            context.closestPointToRobot.getPoint(),
//            context.motionState.position.add(Kinematics.predictNextLoopVelocity(
//                context.motionState.fieldRelativeVelocity,
//                context.motionState.toFieldRelativeVector(context.motionState.robotRelativeVelocity
//                    .subtract(context.motionState.previousRobotRelativeVelocity))).times(context.motionState.deltaTime)),
            context.motionState.deltaTime
        );
        
        Logger.debug("translationalPower", translationalPower);
        Logger.debug("trans closestPoint", context.closestPoint.getPoint());
        Logger.debug("trans predictedPosition", context.motionState.getPredictedStoppedPosition());
        double transitionalMag = translationalPower.computeMagnitude();
        
        Vector drivePower = computeDrivePowerVector(context);
        Vector totalPower = translationalPower.withMaxMagnitude(1).add(drivePower);
        
//        Vector totalPower;
//        if (transitionalMag < 1) {
//            totalPower = translationalPower.add(drivePower.withMaxMagnitude(1 - transitionalMag));
//        }
//        else {
//            totalPower = translationalPower.withMagnitude(1);
//        }
//        double translationalMag = translationalPower.computeMagnitude();
//
//// How much magnitude is left after using full translational?
//        double remaining = Math.sqrt(Math.max(0, 1 - translationalMag * translationalMag));
//
//// Scale drive to fit remaining budget
//        Vector scaledDrive = drivePower.withMaxMagnitude(remaining);
//
//// Now add both
//        Vector totalPower = translationalPower.add(scaledDrive);
//
        Logger.debug("totalDrivePower", totalPower);
        drivetrain.driveTowards(
            context.motionState.makeRobotRelative(totalPower),
            computeHeadingCorrectionPower(context)
        );
    }
    
    public double computeHeadingCorrectionPower(PathFollowContext context) {
        Logger.debug("targetHeading",
            Math.toDegrees(context.headingInterpolator.interpolate(context.closestPoint)));
        Logger.debug("currentHeading", Math.toDegrees(context.motionState.heading));
        return headingPID.run(
            context.headingInterpolator.interpolate(context.closestPoint),
            context.motionState.heading,
            context.motionState.deltaTime
        );
    }
    
    public void positionalHold(Drivetrain drivetrain,
                               PathFollowContext context) {

        drivetrain.driveTowards(
            context.motionState.makeRobotRelative(poseStabilizationPID.run(
                context.closestPoint.getPoint(),
                context.motionState.getPredictedStoppedPosition(),
                context.motionState.deltaTime
            )),
            computeHeadingCorrectionPower(context)
        );
    }
    
    public boolean shouldBrake(PathFollowContext context) {
        return context.motionState.makeRobotRelative(poseStabilizationPID.run(
            context.closestPoint.getPoint(),
            context.motionState.getPredictedStoppedPosition(),
            context.motionState.deltaTime
        )).computeMagnitude() < 1;
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
