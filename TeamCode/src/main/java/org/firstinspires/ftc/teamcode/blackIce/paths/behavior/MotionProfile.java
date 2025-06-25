package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;

//ᴘʏmon | 18535 Frozen Code — 1:42 PM
//but the gist of it is this:
//Tune the robot so you get an equation in the form y = ax^2 + bx to convert velocity (x) into braking displacement (y) where a is basically the robot’s deceleration and b is the braking force. The way I tune this is by mimicking the braking behavior when the motors are at zero power with zero power brake mode.
//It then uses this velocity to braking distance formula to predict where the robot would be if it slammed on its brakes right now. If the closest point to this braking displacement is the end point on the path, then robot knows to brake as hard as it can and it basically just switches to a strong positional proportional (endPoint - predicted position) where the predicted position is position + braking displacement. And if you don’t want any braking then instead of braking it just immediately skips to the next path.

@FunctionalInterface
public interface MotionProfile {
    /**
     * @param remainingTangentVector The tangent vector scaled to the remaining distance to the
     *                               end of the path.
     */
    Vector computeTargetVelocity(Vector tangent, MotionState motionState,
                                 double distanceToEnd);
    
    default boolean isDecelerating(Vector tangent, MotionState motionState,
                                   double distanceToEnd) {
        double ds = 0.01;

        double remainingNow = distanceToEnd;
        double remainingAhead = distanceToEnd - ds;

        Vector velNow = this.computeTargetVelocity(tangent, motionState, remainingNow);
        Vector velNext = this.computeTargetVelocity(tangent, motionState, remainingAhead);

        Vector deltaVelocity = velNext.subtract(velNow);
        Vector approxAccel = deltaVelocity.divide(ds);

        return velNow.dotProduct(approxAccel) < 0;
    }
    
    /**
     *
     * @param deceleration (inches/s^2) How fast the robot decelerates. Should be positive. 60 is
     *                     slow deceleration while 120+ is faster deceleration.
     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Do NOT set this to
     *                   a large number like 999. This will cause transitional correction to not
     *                    work.
     */
    static MotionProfile deceleration(double deceleration, double maxVelocity) {
        return (tangent, motionState, distanceRemaining) ->
            MotionProfile.deceleration(deceleration).computeTargetVelocity(tangent, motionState, distanceRemaining)
                .withMaxMagnitude(maxVelocity);
    }
    
    /**
     *
     * @param deceleration (inches/s^2) How fast the robot decelerates. Should be positive. 60 is
     *                     slow deceleration while 120+ is faster deceleration.
     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Do NOT set this to
     *                   a large number like 999. This will cause transitional correction to not
     *                    work.
     */
    static MotionProfile deceleration(double deceleration) {
        return (tangent, motionState, distanceRemaining) -> {
            
            Vector adjustedDeceleration = new Vector(-Math.abs(deceleration),
                -Math.abs(deceleration) * 1.2);
//               Follower.getInstance().drivetrain.adjustDirectionalEffort(
//                    Vector.fromScalar(-Math.abs(deceleration)));
            Vector remainingTangentVector =
                motionState.makeRobotRelative(tangent.withMagnitude(distanceRemaining));
            
            Vector x =
                motionState.robotRelativeVelocity.subtract(Kinematics.getFinalVelocityAtDistance(
                motionState.robotRelativeVelocity,
                new Vector(-37, -59),
                remainingTangentVector
            )); // how much velocity the robot is going to lose when it gets to the end
            // is the velocity drop that the robot must undergo to stop or slow to vf at the
            // target position

//            x tells you how much velocity you lose naturally by coasting to the target point.
//
//                It represents the velocity that will disappear just by letting go (zero power, no motor effort).

            return motionState.toFieldRelativeVector(remainingTangentVector.map(adjustedDeceleration,
                Kinematics::getCurrentVelocityToStopAtPositionWithDeceleration
            ).subtract(x));
        };
    }
    
    /**
     * Travels along the path at a constant velocity until the last moment where it needs to brake.
     */
    static MotionProfile constantVelocity(double velocity) {
        return (tangent, motionState, distanceToEnd)
            -> tangent.withMagnitude(velocity);
    }
    
    /**
     * Travels along the path at full power until the last moment where it needs to brake.
     */
    MotionProfile maximizeSpeed = null;
}
