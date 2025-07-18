package org.firstinspires.ftc.teamcode.blackIce.follower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blackIce.controller.PIDController;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.controller.PIDFController;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.QuadraticLinearBrakingModel;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathState;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Config
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
    
    public static double P = 0.5;
    public static double D = 0.07;
    public static double DD = 0;
    
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

    double previousDistanceRemaining = 0;

    public Vector computeDrivePowerVector(PathState pathState, MotionState motionState, Path path) {
        
        // double power = targetVelocity * 0.015 - targetAcceleration * 0.05;
        // targetVelocity = Math.sqrt(2 * 40 * remaining) - velocityAtEnd
        // targetVelocity * 0.015
        // cap power to positive
        
        Vector tangentDirection = pathState.closestPathPointToRobot.getTangentVector();
        
        PIDController controller = new PIDController(0.5, 0, 0.0625); // D = kP * brakingDistance
        
//        double predictedVelocity =
//            (pathState.tangentialVelocity * 2 - motionState.toFieldRelativeVector(motionState.previousRobotRelativeVelocity)
//                .dotProduct(tangentDirection));
//        double predictedDistanceRemaining =
//            pathState.distanceRemaining * 2 - previousDistanceRemaining;
//        previousDistanceRemaining = pathState.distanceRemaining;

        double tangentDisplacementToEnd;
        if (pathState.distanceRemaining == 0) {
            tangentDisplacementToEnd =
                pathState.closestPathPointToRobot.getPoint()
                    .minus(motionState.position)
                    .dotProduct(pathState.closestPathPointToRobot.getTangentVector());
        }
        else {
            tangentDisplacementToEnd = pathState.distanceRemaining;
        }
        
        double power = controller.runPIDFromError(tangentDisplacementToEnd,
            pathState.tangentialVelocity);
        boolean isBraking = power < 1;

        if (isBraking) {
            Logger.debug("power", power);
            
            double maxReversePower = 0.3;
            boolean isOpposingMotion = pathState.tangentialVelocity * power < 0;

            double clampedPower = power;
            if (isOpposingMotion) {
                if (power < 0) {
                    clampedPower = Math.max(power, -maxReversePower);
                } else {
                    clampedPower = Math.min(power, maxReversePower);
                }
            }
            
            Logger.debug("clampedPower", clampedPower);
            
            return tangentDirection.withMagnitude(clampedPower);
        }

        return tangentDirection;
        // negative powers trigger internal emf, resulting in force proportional to velocity,
        // making stopping distance linear instead of quadratic
        
        // Aggressive PID controller
//            double power =
//                tangentDisplacementToEnd * 0.5 - pathState.tangentialVelocity * 0.0625;
//
//            double brakingDistance = pathState.tangentialVelocity * 0.125;
//            double power =
//                (tangentDisplacementToEnd - brakingDistance) * 0.5;
        
        // full power or a velocity PIDF     to maintain to follow velocity profile
        
        // Then if you want a smoother deceleration use a positional zero power cruise phase.
        // tangentDisplacementToEnd  * 0.5 - floatingDistance * 0.1

        // Note after all the testing I did: The robot cannot decelerate in between a negative
        // power brake (~ -200 inches/s/s) the zero power float deceleration (~ -40 inches/s/s).
        
        // For higher rpm wheels or heavier robots these ranges are probably closer together.
        // Without decelerating at 60 inches/s float distance is around 45 inches
        // With Braking EMF at 60 inches/s float distance is less than 10 inches. It turns
        // braking from quadratic into linear.
    }
    
    public Vector computeTranslationalPowerVector(PathState pathState, MotionState motionState,
                                                  Path path) {
        return translationalPID.runPID(
            pathState.closestPathPointToPredictedStop.getPoint(),
            motionState.getPredictedStoppedPosition(),
            motionState.deltaTime
        ).times(0.001);
    }
    
    public void accelerate(Drivetrain drivetrain,
                           PathState pathState, Path path, MotionState motionState) {
        
//        Vector translationalPower = translationalPID.runPID(
//            pathState.closestPathPointToPredictedStop.getPoint(),
//            motionState.getPredictedStoppedPosition(),
//            motionState.deltaTime
//        ).times(0.001);
        Vector translationalPower = computeTranslationalPowerVector(pathState, motionState, path);
        
        Logger.debug("translationalPower", translationalPower);
        Logger.debug("trans closestPoint", pathState.closestPathPointToPredictedStop.getPoint());
        Logger.debug("trans predictedPosition", motionState.getPredictedStoppedPosition());
        Vector drivePower = computeDrivePowerVector(pathState, motionState, path);
        
        // Step 1: Prioritize translational power
        double translationalMag = translationalPower.computeMagnitude();
        translationalMag = Math.min(translationalMag, 1.0); // Clamp just in case
        Vector scaledTranslational = translationalPower.withMagnitude(translationalMag);

// Step 2: Allocate turning power from remaining budget
        double remainingAfterTranslational =
            Math.sqrt(Math.max(0, 1 - translationalMag * translationalMag));
        double rawTurnPower = computeHeadingCorrectionPower(path, pathState, motionState);
        double clampedTurnPower =
            Math.copySign(Math.min(Math.abs(rawTurnPower), remainingAfterTranslational),
                rawTurnPower);

//        Vector driveResidual = drivePower.subtract(
//            scaledTranslational.projectOnto(drivePower)
//        );
//        double remaining = Math.sqrt(Math.max(0, 1 - scaledTranslational.computeMagnitudeSquared() - clampedTurnPower^2));
//        Vector scaledDrive = driveResidual.withMagnitude(Math.min(driveResidual.computeMagnitude(), remaining));


// Step 3: Allocate remaining budget to drive power
        double remainingAfterTurn = Math.sqrt(Math.max(0,
            1 - clampedTurnPower * clampedTurnPower - translationalMag * translationalMag));
        Vector scaledDrive = drivePower.withMaxMagnitude(remainingAfterTurn);

// Step 4: Combine translational and drive vectors
        Vector totalPower = scaledTranslational.plus(scaledDrive);

// Step 5: Drive the robot
        Logger.debug("totalDrivePower", totalPower);
        Logger.debug("CRASH DEBUG Driving DrivePower: ", totalPower);
        Logger.debug("CRASH DEBUG Driving Mag DrivePower: ", totalPower.computeMagnitude());
        drivetrain.followVector(
            motionState.makeRobotRelative(totalPower),
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
    
    public double computeHeadingCorrectionPower(Path path, PathState pathState,
                                                MotionState motionState) {
        double targetHeading =
            path.headingInterpolator.interpolate(pathState.closestPathPointToPredictedStop);
        return headingPID.runPID(
            targetHeading,
            motionState.heading,
            motionState.deltaTime
        );
    }
    //        double brakingDistance = new QuadraticLinearBrakingModel(0.001, 0.08) // more empirical
//            // also can use different braking distances for forward/vs lateral
//            .getStoppingDistanceWithVelocity(pathState.tangentialVelocity);
    ////                    - pathState.tangentialVelocity * 0.0375
    ////                    - Math.abs(pathState.tangentialVelocity) * pathState.tangentialVelocity * 0.00075;
//        double brakingDistance =
//            Math.signum(pathState.tangentialVelocity)
//                * Math.abs(pathState.tangentialVelocity) * pathState.tangentialVelocity * 0.003;
//    double velocityToFitAcceleration;
//        if (twoTimesAcceleration == null) {
//        velocityToFitAcceleration = Double.POSITIVE_INFINITY;
//    }
//        else {
//        double distanceTraveled = totalDistance - distanceRemaining;
//        velocityToFitAcceleration = Math.sqrt(twoTimesAcceleration * distanceTraveled);
//    }
//
//    double maxVelocityToFitAcceleration = Math.min(maxVelocity, velocityToFitAcceleration);
//
//        if (twoTimesDeceleration == null) {
//        return maxVelocityToFitAcceleration;
//    }
//
//    double distanceToDecelerate = distanceRemaining - endingVelocityCruiseDistance;
//        if (distanceToDecelerate <= 0) {
//        return endingVelocity;
//    }
//    // TODO graph current vs target velocity
//    double rawVelocity =
//        Math.sqrt(endingVelocitySquared + twoTimesDeceleration * distanceToDecelerate);
//        return Math.min(maxVelocityToFitAcceleration, rawVelocity);
    
    public void decelerate(Drivetrain drivetrain, Path path, PathState pathState,
                           MotionState motionState) {
        
    }
    
    Vector previousPower2 = new Vector(-0.1, -0.1);
    
    public void positionalHold(Drivetrain drivetrain, Path path, PathState pathState,
                               MotionState motionState) {
        
        // -0.01 * velocity (this is a D term)
        // at
        
//        Vector targetPower = translationalPID.runPID(
//            pathState.closestPathPointToPredictedStop.getPoint(),
//            motionState.getPredictedStoppedPosition(),
//            motionState.deltaTime
//        ).withMaxMagnitude(1);
        
        // D term is just braking, simulates zero power brake mode
        
        double kBrake = 0.05;
        Vector targetPower = translationalPID.runPID(
            pathState.closestPathPointToRobot.getPoint(),
            motionState.position,
            motionState.deltaTime
        ).minus(motionState.velocity.times(0.05)); // tune this kBrake constant then find the deceleration it gives to use as stopping distance.

        // resists change in error, slows the robot down, prevents overshoot and oscillations.
        // these are all correct but they are somewhat misleading in terms of positional PIDs.
        // The D term is simply the braking term. It is what makes the robot brake just enough to
        // land on the target. Without it the robot would overshoot, then oscillate until it
        // eventually settles to a stopped position.
        
//        Vector targetPower = translationalPID.runPID(
//            pathState.closestPathPointToPredictedStop.getPoint(),
//            motionState.getPredictedStoppedPosition(),
//            motionState.deltaTime
//        );
        //
        //
        Logger.debug("TARGET-POWER", targetPower);
//
//        Logger.debug("DD targetPower", targetPower);
//
//        double maxDelta = 0.1;
//        Vector delta = targetPower.minus(previousPower2);
//
//        Logger.debug("DD Delta targetPower", delta);
//
//        // Clamp delta per component
//        double clampedX = Math.max(-maxDelta, Math.min(maxDelta, delta.getX()));
//        double clampedY = Math.max(-maxDelta, Math.min(maxDelta, delta.getY()));
//
//        Vector clampedPower = new Vector(previousPower2.getX() + clampedX,
//            previousPower2.getY() + clampedY);
//        previousPower2 = clampedPower;
//
//        Logger.debug("DD clampedPower", clampedPower);
//
        drivetrain.followVector(
            targetPower,
            computeHeadingCorrectionPower(path, pathState, motionState)
        );
//        // this is only really braking force, not holding force if proportional is0.1 and mult is
//        // 10x
//
//        // start at 10x and then go to 1x
//
//        // 0.1 -> 0.5
//
//        Vector error =
//            pathState.closestPathPointToPredictedStop2.getPoint().minus(motionState.getPredictedStoppedPosition());
//
//        // braking distance 0 -> 0.5
//        // braking distance ? -> 0.1
//
//        // error * xkP = 1 or 0?
//
//        // make the proportional as high as possible without exceeding 0.5, so that the power is
//        // just below 1?
//        double highestProportional = Math.max(0.1, Math.min(0.5, 1 - error.computeMagnitude()));
//        // 0.5, 1 - 2.4 -> -1.4 -> 0.1kP
//        // 0.5, 1 - 1.5 -> -0.5 -> 0.1kP
//        // 0.5, 1 - 0.8 -> 0.2 -> 0.2kP
//        // 0.5, 1 - 0.5 -> 0.2 -> 0.5kP
//        // 0.5, 1 - 0.2 -> 0.8 -> 0.5kP
//
//        Vector drivePower = motionState.makeRobotRelative(error.times(0.5));
//
////        Vector drivePower = motionState.makeRobotRelative(poseStabilizationPID.runPID(
////            pathState.closestPathPointToPredictedStop2.getPoint(),// TODO test
////            motionState.getPredictedStoppedPosition(),
////            motionState.deltaTime
////        )); // fade out the braking distance when closer?
////
//        // increase proportional if braking distance is small.
//        // todo test with just velocity PID of like 150 deceleration with correct magnitude
//        //  proportions
//
////        Vector drivePower = motionState.makeRobotRelative(poseStabilizationPID.runPID(
////            adjustedError,
////            motionState.deltaTime
////        ));
//        Logger.debug("CRASH DEBUG Braking DrivePower: ", drivePower);
//        Logger.debug("CRASH DEBUG Braking Mag DrivePower: ", drivePower.computeMagnitude());
//        drivetrain.driveTowards(
//            drivePower.withMaxMagnitude(1),
//            computeHeadingCorrectionPower(path, pathState, motionState)
//        );
    }
    // has about a 5% of making your robot go in full power in a random direction for 5 seconds
    
//    double maxPowerDeltaPerSecond = 3.0; // tune this: e.g., full reversal in ~0.67s
//    double maxDeltaPerStep = maxPowerDeltaPerSecond * deltaTime;
//    double clampedPower = clampRate(prevPower, targetPower, maxDeltaPerStep);
//
//    private double clampRate(double current, double target, double maxDelta) {
//        return Math.max(Math.min(target, current + maxDelta), current - maxDelta);
//    }


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


//    public Vector computeDrivePowerVector(PathState pathState, MotionState motionState, Path path) {
//        // problems with a simple velocity PIDF
//        // 1. requires tuning
//        // 2. can be unstable
//        // 3. doesn't account for the robots momentum when decelerating.
//        // 4. is only reactive when decelerating (not predictive with momentum)
//
//        // TODO implement trapezoidal profile with accel, cruise, decel, cruise
//        // can coast Through path if distance is less than maxDecel
//        // how to setDeceleration but also coast through?
//        // how to endingVelocity of 15 but with stop? when is this useful? how do implement?
//        // already implemented with translational
//        // todo cruise ending velocity
//
//        double deceleration = 150;
//        double endingVelocity = 0;
//        double maxVelocity = 999;
//        double endingVelocityCruiseDistance = 0;
//
//        double stoppingDistance =
//            (endingVelocity * endingVelocity - pathState.tangentialVelocity * pathState.tangentialVelocity) / (2 * -deceleration);
//
//        Logger.debug("AA targetVelocity",
//            Math.sqrt(2 * deceleration * pathState.distanceRemaining));
//        Logger.debug("AA currentVelocity", pathState.tangentialVelocity);
//
//        Vector tangentDirection = pathState.closestPathPointToRobot.getTangentVector();
//        if (pathState.distanceRemaining > stoppingDistance && !isBraking) {
//            return tangentDirection;
//        }
//        else {
//            // just set a max power for the translational
//            // then have cruise at 0 power for "deceleration"
//
//            isBraking = true;
//            // -300 = -1
//            // -150 = -0.5
////            double reactionDistance =
////                (pathState.distanceRemaining - (0 - pathState.tangentialVelocity * pathState.tangentialVelocity) / (2 * -300)); // -250 or whatever deceleration is when power is negative
////            double targetVelocity =
////                Math.copySign(Math.sqrt(2 * deceleration * Math.abs(reactionDistance)),
////                    reactionDistance);
//            double drivePower = (0 -
//                Kinematics.getFinalVelocityAtDistance(
//                    pathState.tangentialVelocity,
//                    -150,
//                    Math.max(0, pathState.distanceRemaining - (0 - pathState.tangentialVelocity * pathState.tangentialVelocity) / (2 * -300))
//                )) * 0.05;
//
////            Logger.debug("reactionDistance", reactionDistance);
////            Logger.debug("target2Velocity", targetVelocity);
////            Logger.debug("AA reactionDistance", reactionDistance);
////            double drivePower = targetVelocity * 0.015;
//            Logger.debug("Drive2Power", drivePower);
//            return tangentDirection.times(drivePower);
////            return tangentDirection.times(Math.min(0, Math.max(-0.2,
////                (pathState.distanceRemaining - stoppingDistance) * 0.2 - 0.01)));
//        }





//        // - velocity.minus(velocityAtEnd)
//        double brakingPower = 0.10; // 0 no brake (hard to stop) -> 1 (will cause the robot to go
//        // from +1 power to -1 instantly often crashing the robot)
//        double kBrake = 0.25;
//        return tangentDirection.times(Math.max(-0.2,
//            (pathState.distanceRemaining - stoppingDistance) * kBrake - 0.01));

//        Vector tangentDirection = pathState.closestPathPointToRobot.getTangentVector();
//
//        double targetVelocity = Math.sqrt(2 * deceleration * pathState.distanceRemaining);
//        // targetVelocity-velocityAtEnd * 0.015
//        // smoooooth: targetVelocity * targetVelocity - pathState.tangentialVelocity * pathState.tangentialVelocity
////        double velocityAtEndIfNothing =
////            Kinematics.getFinalVelocityAtDistance(pathState.tangentialVelocity, -40, pathState.distanceRemaining);
////        double endVelocity =
////            Kinematics.getFinalVelocityAtDistance(pathState.tangentialVelocity, -50,
////                pathState.distanceRemaining);
//
//        // double drivePower = 0 - endVelocity
//        double drivePower = Math.max(-0.1,
//            (pathState.distanceRemaining - endingVelocityCruiseDistance - stoppingDistance) * 0.1);
////        if (previousPower != null) {
////            double delta = drivePower - previousPower;
////            if (Math.abs(delta) > 0.1) {
////                drivePower = previousPower + Math.copySign(0.1, delta);
////            }
////        }
////        previousPower = drivePower;
//        Logger.debug("drivePower", drivePower);
//        // 0.6 - 1 = -0.4
//
//       // double drivePower =
////            (pathState.distanceRemaining - endingVelocityCruiseDistance -
////                (targetVelocity * targetVelocity - pathState.tangentialVelocity * pathState.tangentialVelocity) / (2 * -40)) * 0.05;
//
//
//            // control end velocity?
//           //+ (targetVelocity) * 0.015;
//        // -40 = 0
//           // + (targetVelocity - (pathState.tangentialVelocity - velocityAtEndIfNothing)) * 0.015;
//
//        Logger.debug("BBB targetVelocity", targetVelocity);
//        Logger.debug("BBB currentVelocity", pathState.tangentialVelocity);
//        Logger.debug("BBB stoppingDistance", stoppingDistance);
//        Logger.debug("BBB pathState.distanceRemaining", pathState.distanceRemaining);
//        // TODO make translation have maxDecel reaction of like 150 instead of braking displacement
//        return tangentDirection.times(drivePower);
// }

//    public Vector computeDrivePowerVector(PathState pathState, MotionState motionState, Path path) {
//        double rawDrivePower;
//        boolean isDecelerating;
//
//        Double targetVelocity = path.computeTargetVelocityAt(pathState.distanceRemaining);
//
//// TODO test braking or better yet this but with zero power distance
//        double currentVelocity = pathState.tangentialVelocity;
//        double brakingDistance = currentVelocity * currentVelocity / (2 * 40);
//        double brakingError = Math.max(0, brakingDistance - pathState.distanceRemaining);
//        Logger.debug("brakingError", brakingError);
//        double brakingTerm = 0.2 * brakingError;
//
//        if (targetVelocity == null) {
//            rawDrivePower = 1;
//            isDecelerating = false;
//        } else {
//            double currentTangentialVelocity = motionState.fieldRelativeVelocity
//                .dotProduct(pathState.closestPathPointToPredictedStop.getTangentVector());
//
//            isDecelerating =
//                previousTargetVelocity == null || targetVelocity > previousTargetVelocity;
//            previousTargetVelocity = targetVelocity;
//
//            if (isDecelerating && path.behavior.decelerateWithoutFeedforward) {
//                rawDrivePower = drivePIDF.runPID(targetVelocity,
//                    currentTangentialVelocity,
//                    motionState.deltaTime) + brakingTerm;
//            } else {
//                rawDrivePower = drivePIDF.runPIDF(
//                    targetVelocity,
//                    currentTangentialVelocity,
//                    motionState.deltaTime
//                ) + brakingTerm;
//            }
//
//            Logger.debug("targetVelocity", targetVelocity);
//            Logger.debug("currentVelocity", pathState.tangentialVelocity);
//        Logger.debug("drivePower", rawDrivePower);
//        double drivePower = path.behavior.drivePowerModifier.modifyDrivePower(rawDrivePower,
//            isDecelerating);
//        }
//        // keep target velocity tho when not decelerating or it is accelerating
//        double velocityToStopWithDeceleration; // this but with Feedforward
//        double stoppingDistance =
//            (0 - pathState.tangentialVelocity * pathState.tangentialVelocity) / (2 * -50);
//
//        Logger.debug("stoppingDistance", stoppingDistance);
//
//        Vector tangentDirection = pathState.closestPathPointToRobot.getTangentVector();
//        double drivePower =
//            (pathState.distanceRemaining - stoppingDistance) * 0.1;
//
//        // cap drivePower with 0.05*maxVelocity with PIDF
//        Logger.debug("drivePower", drivePower);
//        // drive only do constant targetVelocity
//        // keep translations but have max acceleration reaction that it uses but without the
//        // robot servely braking
//        // then have drive switch to a decelerating one that is (pathState.distanceRemaining - stoppingDistance) * 0.1
//        // dont need different axis cause it is target StoppingDistance and then translational
//        // still holds the position and might even be able to hold with strong proportional if
//        // braking distance is less viotile.
//
//        return tangentDirection.times(drivePower + 0.015 * targetVelocity);
//    }