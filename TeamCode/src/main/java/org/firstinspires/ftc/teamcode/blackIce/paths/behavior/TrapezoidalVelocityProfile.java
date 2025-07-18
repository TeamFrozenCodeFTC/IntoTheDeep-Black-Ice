package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import static org.firstinspires.ftc.teamcode.blackIce.util.Utils.getOrDefault;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;
import org.firstinspires.ftc.teamcode.blackIce.util.Validator;

/**
 * A velocity profile that has attributes that resemble a trapezoid. Can accelerate, cruise,
 * decelerate and optional continue cruising after decelerating.
 */
public class TrapezoidalVelocityProfile implements VelocityProfile {
    private final double maxVelocity;
    private final @Nullable Double twoTimesAcceleration;
    private final @Nullable Double twoTimesDeceleration;
    private final double endingVelocityCruiseDistance;
    private final double endingVelocitySquared;
    private final double endingVelocity;

    /**
     * Accelerates to maxVelocity and then decelerates until it reaches the given ending velocity
     * where it cruises there for endingVelocityCruiseDistance inches. Linearly increases the target
     * velocity while accelerating or decelerating.
     * <p>
     * All parameters allow null.
     *
     * @param acceleration                 (inches/s^2) How fast the robot accelerates. Must be
     *                                     positive. If null, it will not accelerate.
     * @param maxVelocity                  (inches/s) The maximum velocity the robot can reach. Must
     *                                     be positive. If null, it will not have a maximum
     *                                     velocity.
     * @param deceleration                 (inches/s^2) How fast the robot decelerates. Must be
     *                                     positive. If null, it will not decelerate.
     * @param endingVelocity               (inches/s) The velocity the robot decelerates to. Must be
     *                                     positive. If null, it will end with zero velocity.
     * @param endingVelocityCruiseDistance (inches) The distance the robot travels at the ending
     *                                     velocity. Must be zero, positive, or null. If null, it
     *                                     will be zero.
     */
    public TrapezoidalVelocityProfile(@Nullable Acceleration acceleration,
                                      @Nullable Double maxVelocity,
                                      @Nullable Acceleration deceleration,
                                      @Nullable Double endingVelocity, // finalVelocity
                                      @Nullable Double endingVelocityCruiseDistance) {
        this.maxVelocity =
            Validator.ensureGreaterThanZero(getOrDefault(maxVelocity, Double.POSITIVE_INFINITY));
        this.endingVelocity = Validator.ensureGreaterThanZero(getOrDefault(endingVelocity,
            0));
        this.endingVelocitySquared = this.endingVelocity * this.endingVelocity;
        
        if (deceleration != null) {
            this.twoTimesDeceleration =
                2 * Validator.ensurePositiveSign(deceleration.compute(this.maxVelocity,
                    this.endingVelocity));
        } else {
            this.twoTimesDeceleration = null;
        }
        
        if (acceleration != null) {
            this.twoTimesAcceleration =
                2 * Validator.ensurePositiveSign(acceleration.compute(0, this.maxVelocity));
        } else {
            this.twoTimesAcceleration = null;
        }
        
        this.endingVelocityCruiseDistance =
            Validator.ensureGreaterThanZero(getOrDefault(endingVelocityCruiseDistance, 0));
        
        if (this.endingVelocity > this.maxVelocity) {
            Logger.warnWithStack("Ending velocity exceeds maximum velocity.");
        }
    }
    
    @Override
    public double computeTargetVelocity(double totalDistance, double distanceRemaining) {
        double velocityToFitAcceleration;
        if (twoTimesAcceleration == null) {
            velocityToFitAcceleration = Double.POSITIVE_INFINITY;
        }
        else {
            double distanceTraveled = totalDistance - distanceRemaining;
            velocityToFitAcceleration = Math.sqrt(twoTimesAcceleration * distanceTraveled);
        }
        
        double maxVelocityToFitAcceleration = Math.min(maxVelocity, velocityToFitAcceleration);

        if (twoTimesDeceleration == null) {
            return maxVelocityToFitAcceleration;
        }
        
        double distanceToDecelerate = distanceRemaining - endingVelocityCruiseDistance;
        if (distanceToDecelerate <= 0) {
            return endingVelocity;
        }
// TODO graph current vs target velocity
        double rawVelocity =
            Math.sqrt(endingVelocitySquared + twoTimesDeceleration * distanceToDecelerate);
        return Math.min(maxVelocityToFitAcceleration, rawVelocity);
    }
    
    /**
     * A type of acceleration by accelerating or decelerating between velocities either by time
     * (s), distance (inch) or a constant (inches/s^2).
     */
    @FunctionalInterface
    public interface Acceleration {
        double compute(double startingVelocity, double endingVelocity);
        
        static Acceleration constant(double acceleration) {
            return (startingVelocity, endingVelocity) -> acceleration;
        }
        
        static Acceleration toReachInTime(double seconds) {
            return (startingVelocity, endingVelocity) -> Kinematics.computeAccelerationForTime(
                startingVelocity, seconds, endingVelocity);
        }
        
        static Acceleration toReachOverDistance(double distance) {
            return (startingVelocity, endingVelocity) -> Kinematics.computeAccelerationForDistance(
                startingVelocity, distance, endingVelocity);
        }
    }
}
