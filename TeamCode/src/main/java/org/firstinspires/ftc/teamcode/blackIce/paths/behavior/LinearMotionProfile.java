package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

public class LinearMotionProfile implements MotionProfile {
    private final double maxVelocity;
    private final double endingVelocity;
    private final Double acceleration; //TODO
    private final Double deceleration;
    private final double endingVelocityCruiseDistance;
    
    // TODO allow earlier decelerations to endingVelocity
    
    /**
     * Accelerates to maxVelocity and then decelerates until it reaches the given
     * ending velocity where it cruises there for endingVelocityCruiseDistance inches. Linearly
     * increases the target velocity while accelerating or decelerating.
     * <p>
     * All parameters allow null.
     * @param acceleration (inches/s^2) How fast the robot accelerates. Must be positive.
     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Must be positive.
     * @param deceleration (inches/s^2) How fast the robot decelerates. Must be positive.
     * @param endingVelocity (inches/s) The velocity the robot decelerates to. Must be positive.
     * @param endingVelocityCruiseDistance (inches) The distance the robot travels at the ending
     *                                     velocity. Usually zero. Must be zero or positive.
     */
    public LinearMotionProfile(Double acceleration, Double maxVelocity,
                               Double deceleration, Double endingVelocity,
                               Double endingVelocityCruiseDistance) {
        this.deceleration = deceleration;
        this.acceleration = acceleration;
        this.maxVelocity = (maxVelocity != null) ? maxVelocity : Double.MAX_VALUE;
        this.endingVelocity = (endingVelocity != null) ? endingVelocity : 0;
        this.endingVelocityCruiseDistance = (endingVelocityCruiseDistance != null) ?
            endingVelocityCruiseDistance : 0;
        
        
        assert this.deceleration == null || this.deceleration > 0 : "Deceleration must be " +
            "positive or" +
            " null.";
        assert this.acceleration == null || this.acceleration > 0 : "Acceleration must be " +
            "positive or null.";
        assert this.endingVelocityCruiseDistance >= 0 : "Ending velocity cruise distance must be " +
            "zero, positive or null.";
        
        assert this.endingVelocity >= 0: "Ending velocity must be positive." + this.endingVelocity;
        assert this.maxVelocity > 0: "Maximum velocity must be positive.";
        if (this.endingVelocity > this.maxVelocity) {
            Logger.warnWithStack("Ending velocity exceeds maximum velocity.");
        }
    }
    
    @Override
    public double computeTargetVelocity(MotionState motionState, double distanceToEnd) { //TODO
        // pass in pathState so can use totalDistanceToEnd for acceleration
        
        if (deceleration == null) {
            return maxVelocity;
        }
        
        double distanceToDecelerate = distanceToEnd - endingVelocityCruiseDistance;
        if (distanceToDecelerate <= 0) {
            return endingVelocity;
        }
        
        double rawVelocity = Kinematics.computeVelocityToReach(
            distanceToDecelerate, -deceleration, endingVelocity);
        return Math.min(maxVelocity, rawVelocity);
    }
    
    @Override
    public boolean isDecelerating(MotionState motionState, double distanceToEnd) {
        if (deceleration == null) {
            return false;
        }
        
        double distanceToDecelerate = distanceToEnd - endingVelocityCruiseDistance;
        if (distanceToDecelerate <= 0) {
            return false;
        }
        
        double rawVelocity = Kinematics.computeVelocityToReach(
            distanceToDecelerate, -deceleration, endingVelocity);
        return rawVelocity < maxVelocity;
    }
}
