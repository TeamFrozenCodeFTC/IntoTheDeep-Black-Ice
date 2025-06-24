package org.firstinspires.ftc.teamcode.blackIce.motion;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

/**
 * Represents a snapshot of the robot's motion in terms of its
 * position, heading, delta time, and velocity.
 * Used to store repeatedly used information about the robot's motion.
 * <p>
 * Managed by the {@link MotionTracker}.
 */
public class MotionState {
    public final Vector position;

    public final double heading; // always in radians
    public final double angularVelocity;

    public final Vector fieldRelativeVelocity;
    public final Vector robotRelativeVelocity;
    public final double velocityMagnitude;

    public final Vector previousRobotRelativeVelocity;
    
    /** The time elapsed since the last update. */
    public final double deltaTime;
    
    private Vector predictedStoppedPosition = null;

    public MotionState(
        Vector position,
        double headingRadians,
        double angularVelocity,
        double deltaTime,
        Vector fieldRelativeVelocity,
        double velocityMagnitude,
        Vector robotRelativeVelocity,
        Vector previousRobotRelativeVelocity
    ) {
        this.position = position;
        this.heading = headingRadians;
        this.angularVelocity = angularVelocity;
        this.deltaTime = deltaTime;
        this.fieldRelativeVelocity = fieldRelativeVelocity;
        this.velocityMagnitude = velocityMagnitude;
        this.robotRelativeVelocity = robotRelativeVelocity;
        this.previousRobotRelativeVelocity = previousRobotRelativeVelocity;
    }
    
    /**
     * Predict what the robot's position would be if it slammed on it's brakes right now.
     */
    public Vector getPredictedStoppedPosition() {
        if (predictedStoppedPosition == null) {
            predictedStoppedPosition = computeStoppedPosition();
        }
        return predictedStoppedPosition;
    }
    
    private Vector computeStoppedPosition() {
        Vector robotVelocity = robotRelativeVelocity;
        
        Vector robotDeltaVelocity = robotVelocity
            .subtract(previousRobotRelativeVelocity);
        
        Vector predictedRobotVelocity =
            Kinematics.predictNextLoopVelocity(robotVelocity, robotDeltaVelocity);
        
        Vector stoppingDisplacement = TuningConstants.BRAKING_DISPLACEMENT
            .getStoppingDistanceWithVelocity(predictedRobotVelocity);
        
        Logger.debug("X localStoppingDisplacement", stoppingDisplacement);
        Logger.debug("X predictedRelativeVelocity", predictedRobotVelocity);
        Logger.debug("X robotRelativeVelocity", robotRelativeVelocity);
        Logger.debug("X fieldRelativeVelocity", fieldRelativeVelocity);
        Logger.debug("X fieldRelativeStoppingDisplacement",
            stoppingDisplacement.toFieldVector(heading));

        return position
            .add(stoppingDisplacement.toFieldVector(heading));
    }
    
    public Vector toFieldRelativeVector(Vector robotRelativeVector) {
        return robotRelativeVector.rotatedBy(heading);
    }
    
    public Vector toRobotRelativeVector(Vector fieldRelativeVector) {
        return fieldRelativeVector.rotatedBy(-heading);
    }
    
//    public double computeCurrentAcceleration() {
//        return Kinematics.getAcceleration(
//            velocityMagnitude - previousRobotRelativeVelocity.computeMagnitude(),
//            deltaTime);
//    }
}
