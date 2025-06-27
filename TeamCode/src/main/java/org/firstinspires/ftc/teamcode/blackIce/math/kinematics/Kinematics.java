package org.firstinspires.ftc.teamcode.blackIce.math.kinematics;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

// Got into pedro path code: https://github.com/Pedro-Pathing/PedroPathing/pull/36
/**
 * Implementation of kinematic equations for decelerating the robot at desired decelerations.
 * <pre>
 * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
 * v<sub>f</sub> = v<sub>i</sub> + a·△t
 * </pre>
 * Where:<br>
 * <ul>
 *   <li><b>v<sub>f</sub></b> = final velocity</li>
 *   <li><b>v<sub>i</sub></b> = initial velocity</li>
 *   <li><b>a</b> = acceleration</li> (always negative in this case)
 *   <li><b>d</b> = distance (equations arranged in code for positive and negative)</li>
 * </ul>
 */
public class Kinematics {
    /**
     * Get the final velocity at a given distance with the deceleration.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for vi
     * v<sub>i</sub> = sqrt(v<sub>f</sub><sup>2</sup> - 2·a·d)
     * Set v<sub>f</sub> to 0 to stop
     * v<sub>i</sub> = sqrt(0 - 2·a·d)
     * v<sub>i</sub> = sqrt(-2·a·d)
     * </pre>
     */
    public static double computeVelocityToStop(
        double directionalDistance,
        double deceleration
    ) {
        return Math.signum(directionalDistance)
            * Math.sqrt(-2 * deceleration * Math.abs(directionalDistance));
    }
    
    /**
     * Calculate the velocity the robot would be going after traveling the given distance
     * as it decelerates.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for v<sub>f</sub>
     * v<sub>f</sub> = sqrt(v<sub>i</sub><sup>2</sup> + 2·a·d)
     * </pre>
     */
    public static double getFinalVelocityAtDistance(
        double currentVelocity,
        double deceleration,
        double directionalDistance
    ) {
        double vfSquared = currentVelocity * currentVelocity + 2 * deceleration * directionalDistance;

        // If vf^2 goes negative, that means we overshot the zero crossing and are reversing direction
        if (vfSquared < 0) {
            return Math.sqrt(-vfSquared) * Math.signum(deceleration);
        }

        // Otherwise preserve direction of motion (positive or negative)
        double vf = Math.sqrt(vfSquared);
        return (currentVelocity >= 0) ? vf : -vf;
        
//        return Math.signum(directionalDistance) * Math.sqrt(
//            currentVelocity * currentVelocity + 2 * deceleration * Math.abs(directionalDistance));
    }
    
    public static Vector getFinalVelocityAtDistance(
        Vector currentVelocity,
        Vector deceleration,
        Vector directionalDistance
    ) {
        return new Vector(
            Kinematics.getFinalVelocityAtDistance(currentVelocity.getX(), deceleration.getX(),
                directionalDistance.getX()),
            Kinematics.getFinalVelocityAtDistance(currentVelocity.getY(), deceleration.getY(),
                directionalDistance.getY())
        );
    }
    
    
    public static double computeVelocityToReach(
        double directionalDistance,
        double deceleration,
        double targetVelocity
    ) {
        return Math.signum(directionalDistance)
            * Math.sqrt(targetVelocity * targetVelocity - 2 * deceleration * Math.abs(directionalDistance));
    }
    
    // for accelerating, getTargetVelocityToGo 60 inches/second
    
    /**
     * Get the final velocity at a given distance with the deceleration.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for vi
     * v<sub>i</sub> = sqrt(v<sub>f</sub><sup>2</sup> - 2·a·d)
     * Set v<sub>f</sub> to 0 to stop
     * v<sub>i</sub> = sqrt(0 - 2·a·d)
     * v<sub>i</sub> = sqrt(-2·a·d)
     * </pre>
     */
    public static Vector computeVelocityToStop(
        Vector displacementVector,
        double deceleration
    ) {
        return displacementVector.map(
            (displacement) ->
                computeVelocityToStop(displacement, deceleration)
        );
    }
    
    public static double getDistanceToVelocity(
        double velocity,
        double deceleration,
        double targetVelocity
    ) {
        return (targetVelocity * targetVelocity - velocity * velocity) / (2 * deceleration);
    }
    

    public static double getStoppingDistance(
        double velocity,
        double deceleration
    ) {
        return getDistanceToVelocity(velocity, deceleration, 0);
    }

    public static double getStoppingDistance(
        double velocity,
        BrakingDistanceModel model
    ) {
        return model.getStoppingDistanceWithVelocity(velocity);
    }

    /**
     * Calculates the distance needed to stop at a given velocity with the deceleration.
     * Returns true if the robot would overshoot the target distance.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for d
     * d = -(v<sub>f</sub><sup>2</sup> - v<sub>i</sub><sup>2</sup>) / (2·a)
     * distanceToTarget > d
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Always negative (very general braking value -120 inch/s^2)
     */
    public static Vector getStoppingDistance(Vector velocityVector, double deceleration) {
        return velocityVector.map(
            (vel) -> getStoppingDistance(vel, deceleration)
        );
    }

    /**
     * Calculates the distance needed to stop at a given velocity with the deceleration.
     * Returns true if the robot would overshoot the target distance.
     *
     * <pre>
     * v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d
     * Solve for d
     * d = -(v<sub>f</sub><sup>2</sup> - v<sub>i</sub><sup>2</sup>) / (2·a)
     * distanceToTarget > d
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Always negative (very general braking value -120 inch/s^2)
     * @param distance Always positive.
     */
    public static boolean isOvershooting(double velocity, double deceleration, double distance) {
        double stoppingDistance = getStoppingDistance(velocity, deceleration);
        return stoppingDistance > distance;
    }

    public static double predictNextLoopVelocity(double velocity, double deltaVelocity) {
        return velocity + deltaVelocity;
    }

    public static Vector predictNextLoopVelocity(Vector velocityVector, Vector deltaVelocityVector) {
        return velocityVector.add(deltaVelocityVector);
    }

    /**
     * <pre>
     * d = v<sub>i</sub> · △t + ½ · a · △t<sup>2</sup>
     * d = v · △t + ½ · (v - v<sub>prev</sub>) · △t
     * Equivalent to: d = (v + v<sub>prev</sub>) / 2 · △t
     * </pre>
     */
    public static double getNextDistanceTraveled(
        double deltaVelocity,
        double deltaTime
    ) {
        return deltaVelocity / 2 * deltaTime;
    }

    /**
     * <pre>
     * d = v<sub>i</sub> · △t + ½ · a · △t<sup>2</sup>
     * d = v · △t + ½ · (v - v<sub>prev</sub>) · △t
     * Equivalent to: d = (v + v<sub>prev</sub>) / 2 · △t
     * </pre>
     */
    public static Vector getNextDistanceTraveled(
        Vector deltaVelocityVector,
        double deltaTime
    ) {
        return deltaVelocityVector.map(
            dv -> getNextDistanceTraveled(dv, deltaTime)
        );
    }

//    /**
//     * <pre>
//     * d = v<sub>i</sub> · △t + ½ · a · △t<sup>2</sup>
//     * d = v · △t + ½ · (v - v<sub>prev</sub>) · △t
//     * Equivalent to: d = (v + v<sub>prev</sub>) / 2 · △t
//     * </pre>
//     */
//    public static double getDistanceToStop(
//        double velocity,
//        double deceleration
//    ) {
//        return changeInVelocity / 2 * deltaTime;
//    }
    // v<sub>f</sub><sup>2</sup> = v<sub>i</sub><sup>2</sup> + 2·a·d

    /**
     * <pre>
     * v<sub>f</sub> = v<sub>i</sub> + a·△t
     * a = (v<sub>f</sub> - v<sub>i</sub>) / △t
     * </pre>
     */
    public static double getAcceleration(
        double changeInVelocity,
        double deltaTime
    ) {
        return changeInVelocity / deltaTime;
    }
}
