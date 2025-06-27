package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
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
        } else {
            this.translationalPID = translationalPID;
        }
        this.drivePIDF = drivePIDF;
    }
    
    Double previousTargetVelocity = null; // needs to be reset for next path
    
    public Vector computeDrivePowerVector(PathFollowContext context) {
        // take in maxDrivePower
        Vector drivePower;
        if (context.path.getMotionProfile() == null) {
            drivePower = context.closestPointToRobot.getTangentVector();
            Logger.debug("driving along path at full power");
        } else {
            /* Allows wheels to reverse direction while decelerating */
            boolean allowReverseDeceleration = true;
            /* Uses zero power when decelerating. */
            boolean zeroPowerDeceleration = false;
            
            double targetVelocity =
                context.path.getMotionProfile().computeTargetVelocity(context.motionState,
                    context.distanceToEnd);
            
            double currentTangentialVelocity = context.motionState.fieldRelativeVelocity
                .dotProduct(context.closestPoint.getTangentVector());
            // or turn to current heading but then make stronger?

//            boolean isDecelerating = targetVelocity - currentTangentialVelocity < 0.5;
            double drivePower1;
//            if (!context.path.getMotionProfile().isDecelerating(context.motionState,  context
//            .distanceToEnd)) {
            if (previousTargetVelocity == null || targetVelocity > previousTargetVelocity) {
                if (zeroPowerDeceleration) {
                    drivePower1 = 0;
                } else {
                    drivePower1 =
                        drivePIDF.run(
                            targetVelocity - currentTangentialVelocity,
                            context.motionState.deltaTime
                        );
                }
            } else {
                drivePower1 =
                    drivePIDF.run(
                        targetVelocity,
                        currentTangentialVelocity,
                        context.motionState.deltaTime
                    );
            }
            previousTargetVelocity = targetVelocity;
            
            if (!allowReverseDeceleration) {
                drivePower1 = Math.max(0, drivePower1);
            }
            
            drivePower = context.closestPoint.getTangentVector().withMagnitude(drivePower1);
            
            Logger.debug("targetVelocity", targetVelocity);
            Logger.debug("currentVelocity", context.motionState.fieldRelativeVelocity);
            Logger.debug("drivePower", drivePower);
        }
        //drivePower = context.closestPointToRobot.getTangentVector();
        return drivePower.withMaxMagnitude(1);
    }
    
    //right now I am getting the closest point on the path to the robot but I also have the
    // closest point to the robot predicted position which is just position +  the braking
    // displacement relative to how fast it is going. I have a drive vector that accelerates and
    // slows the robot down. Once the closest point to the robot's braking displacement is the
    // end point it removes the drive vector and switches to a positional PID of (endPoint -
    // predictedStoppedPosition). But I also have a transitional vector that currently goes  from
    // the closest point to the robot on the path but if the robot ever overshoots along the path
    // the transitional isn't predictive and wont react until it happens. I have tried making the
    // proportional (closestPointToBraking - predictedBrakingPosition) but when the robot
    // decelerates it makes the closestPoint to braking really unpredictable and it vibrates
    // around making the robot shake when It decelerates near the end point. are there any other
    // solutions?
    public void accelerate(Drivetrain drivetrain,
                           PathFollowContext context) {
        
        Vector translationalPower = translationalPID.run(
            context.closestPoint.getPoint(),
            context.motionState.getPredictedStoppedPosition(),
            context.motionState.deltaTime
        );
        
        Logger.debug("translationalPower", translationalPower);
        Logger.debug("trans closestPoint", context.closestPoint.getPoint());
        Logger.debug("trans predictedPosition", context.motionState.getPredictedStoppedPosition());
        Vector drivePower = computeDrivePowerVector(context);
        
        // Step 1: Prioritize translational power
        double translationalMag = translationalPower.computeMagnitude();
        translationalMag = Math.min(translationalMag, 1.0); // Clamp just in case
        Vector scaledTranslational = translationalPower.withMagnitude(translationalMag);

// Step 2: Allocate turning power from remaining budget
        double remainingAfterTranslational =
            Math.sqrt(Math.max(0, 1 - translationalMag * translationalMag));
        double rawTurnPower = computeHeadingCorrectionPower(context);
        double clampedTurnPower =
            Math.copySign(Math.min(Math.abs(rawTurnPower), remainingAfterTranslational),
                rawTurnPower);

// Step 3: Allocate remaining budget to drive power
        double remainingAfterTurn = Math.sqrt(Math.max(0,
            1 - clampedTurnPower * clampedTurnPower - translationalMag * translationalMag));
        Vector scaledDrive = drivePower.withMaxMagnitude(remainingAfterTurn);

// Step 4: Combine translational and drive vectors
        Vector totalPower = scaledTranslational.add(scaledDrive);

// Step 5: Drive the robot
        Logger.debug("totalDrivePower", totalPower);
        drivetrain.driveTowards(
            context.motionState.makeRobotRelative(totalPower),
            clampedTurnPower
        );
//
//        double translationalMag = translationalPower.computeMagnitude();
//
//        double remaining = Math.sqrt(Math.max(0, 1 - translationalMag * translationalMag));
//        double turnPower = Math.min(remaining, computeHeadingCorrectionPower(context));
//
//        Vector scaledDrive = drivePower.withMaxMagnitude(Math.sqrt(Math.max(0,
//            1 - turnPower * turnPower)));
//
//        Vector totalPower = translationalPower.add(scaledDrive);

//        Logger.debug("totalDrivePower", totalPower);
//        drivetrain.driveTowards(
//            context.motionState.makeRobotRelative(totalPower),
//            turnPower
//        );
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

//    public boolean shouldBrake(PathFollowContext context) {
//        return context.motionState.makeRobotRelative(poseStabilizationPID.run(
//            context.closestPoint.getPoint(),
//            context.motionState.getPredictedStoppedPosition(),
//            context.motionState.deltaTime
//        )).computeMagnitude() < 1;
//    }
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


//     Vector adjustedDeceleration = new Vector(-Math.abs(60),
//                -Math.abs(60) * 1.2);
/// /               Follower.getInstance().drivetrain.adjustDirectionalEffort(
/// /                    Vector.fromScalar(-Math.abs(deceleration)));
//            Vector remainingTangentVector =
//                context.motionState.makeRobotRelative(context.closestPointToRobot
//                .getTangentVector().withMagnitude(context.distanceToEnd));
//
////            Vector finalVelocity = Kinematics.getFinalVelocityAtDistance(
////                context.closestPointToRobot.getTangentVector().withMagnitude(context
//.motionState.velocityMagnitude),
////                adjustedDeceleration,
////                remainingTangentVector
////            );

// todo power ramping

//            Vector deceleration = new Vector(-120, -120);
//
//            Vector targetVelocity = context.motionState.robotRelativeVelocity.map(deceleration,
//                (velocity, decel) ->
//                Kinematics.computeVelocityToStop(
//                    context.distanceToEnd,
//                    -Math.abs(decel)
//                )
//            );
//
//
//            drivePower = drivePIDF.run(
//                context.motionState.toFieldRelativeVector(targetVelocity.minus(context
//                .motionState.robotRelativeVelocity)),
//                context.motionState.deltaTime
//            );
// if decelerate with zero power is true Math.max(0, error)
// if no harsh decel (no backward wheels) with zero power is true Math.max(0,
// targetVelocity)

// 30, 40
// 50 - 40

//            double finalVelocity = Kinematics.getFinalVelocityAtDistance(
//                context.motionState.velocityMagnitude,
//                -50,
//                context.distanceToEnd
//            );
//            drivePower =
//                // can have
//                // custom end
//                // velocity
//                drivePIDF.run(
//                    context.pathExecutor.getCurrentSegment().getEndTangent().withMagnitude(0),
//                    context.closestPointToRobot.getTangentVector().withMagnitude(finalVelocity),
//                    context.motionState.deltaTime
//                );

//            double targetVelocity = Kinematics.getCurrentVelocityToStopAtPositionWithDeceleration(
//                context.distanceToEnd,
//                -60
//            );
//            drivePower =
//                drivePIDF.run(
//                    context.closestPointToRobot.getTangentVector().withMagnitude(targetVelocity),
//                context.motionState.fieldRelativeVelocity,
//                context.motionState.deltaTime);

//            Vector targetDriveVelocity =
//                // closest point to robot?
//                context.path.getMotionProfile().computeTargetVelocity(context
//                .closestPointToRobot.getTangentVector(),
//                    context.motionState, context.distanceToEnd);
//            drivePower = drivePIDF.run(
//                targetDriveVelocity,
//                context.motionState.fieldRelativeVelocity,
////                Kinematics.predictNextLoopVelocity(
////                    context.motionState.fieldRelativeVelocity,
////                    context.motionState.toFieldRelativeVector(context.motionState
   // .robotRelativeVelocity
////                        .subtract(context.motionState.previousRobotRelativeVelocity))),
//                context.motionState.deltaTime
//            );
//            Logger.debug("velocityAtEnd", Kinematics.getFinalVelocityAtDistance(
//                context.motionState.robotRelativeVelocity,
//                adjustedDeceleration,
//                remainingTangentVector
//            ));
//            Logger.debug("targetVelocity", targetVelocity);