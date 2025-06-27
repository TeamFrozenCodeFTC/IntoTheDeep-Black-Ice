package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

import java.util.Locale;

//ᴘʏmon | 18535 Frozen Code — 1:42 PM
//but the gist of it is this:
//Tune the robot so you get an equation in the form y = ax^2 + bx to convert velocity (x) into braking displacement (y) where a is basically the robot’s deceleration and b is the braking force. The way I tune this is by mimicking the braking behavior when the motors are at zero power with zero power brake mode.
//It then uses this velocity to braking distance formula to predict where the robot would be if it slammed on its brakes right now. If the closest point to this braking displacement is the end point on the path, then robot knows to brake as hard as it can and it basically just switches to a strong positional proportional (endPoint - predicted position) where the predicted position is position + braking displacement. And if you don’t want any braking then instead of braking it just immediately skips to the next path.

/**
 * A function that provides the target velocity along the path to fit a certain motion profile.
 */
@FunctionalInterface
public interface MotionProfile {
    double computeTargetVelocity(MotionState motionState, double distanceToEnd);

    
    /**
     * Travels along the path at a constant velocity until the last moment where it needs to brake.
     */
    static MotionProfile constantVelocity(double velocity) {
        return (motionState, distanceToEnd) -> velocity;
    }
    
    /**
     * Travels along the path at full speed until the last moment where it needs to brake.
     */
    MotionProfile maximizeSpeed = null;
    
    default boolean isDecelerating(MotionState motionState,
                                   double distanceToEnd) {
        double ds = 0.01;
        
        double remainingNow = distanceToEnd;
        double remainingAhead =  Math.max(0.0, distanceToEnd - ds);
        
        double velNow = this.computeTargetVelocity(motionState, remainingNow);
        double velNext = this.computeTargetVelocity(motionState, remainingAhead);
        
        double deltaVelocity = velNext - velNow;
        double approxAccel = deltaVelocity / ds;
        
        return approxAccel < -1e-6;
    }


//    /**
//     * Decelerates until it reaches the given minimum velocity without exceeding the maxVelocity.
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Must be positive.
//     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Must be positive.
//     * @param endingVelocity (inches/s) The velocity the robot decelerates to. Must be positive.
//     */
//    static MotionProfile constantDeceleration(
//        double deceleration,
//        Double maxVelocity,
//        Double endingVelocity
//    ) {
//        assert deceleration >= 0 : "Deceleration must be positive.";
//
//        final double acceleration = -Math.abs(deceleration);
//        final double maxVel = (maxVelocity != null) ? maxVelocity : Double.MAX_VALUE;
//        final double minVel = (endingVelocity != null) ? endingVelocity : Double.MIN_VALUE;
//
//        assert minVel > maxVel : "Ending velocity cannot exceed maximum velocity.";
//        assert minVel >= 0: "Ending velocity must be positive.";
//        assert maxVel >= 0: "Maximum velocity must be positive.";
//
//        return (motionState, distanceToEnd) -> {
//            double rawVelocity = Kinematics.computeVelocityToStop(distanceToEnd, acceleration);
//            return Math.min(maxVel, Math.max(minVel, rawVelocity));
//        };
//    }
//
//    /**
//     * Decelerates at the end of the path with the given deceleration.
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Must be positive.
//     */
//    static MotionProfile constantDeceleration(double deceleration) {
//        return constantDeceleration(deceleration, null, null);
//    }
//
//    /**
//     * Decelerates at the end of the path to reach the ending velocity with the given
//     * deceleration.
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Must be positive.
//     * @param endingVelocity (inches/s) The velocity the robot decelerates to. Must be positive.
//     */
//    static MotionProfile constantDecelerationToVelocity(double deceleration,
//                                                        double endingVelocity) {
//        return constantDeceleration(deceleration, null, endingVelocity);
//    }
//
//    /**
//     * Decelerates at the end of the path with the given deceleration while not exceeding the max
//     * velocity.
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Must be positive.
//     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Must be positive.
//     */
//    static MotionProfile constantDeceleration(double deceleration, double maxVelocity) {
//        return constantDeceleration(deceleration, maxVelocity, null);
//    }
}


//    /**
//     *
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Should be positive. 60 is
//     *                     slow deceleration while 120+ is faster deceleration.
//     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Do NOT set this to
//     *                   a large number like 999. This will cause transitional correction to not
//     *                    work.
//     */
//    static MotionProfile deceleration(double deceleration, double maxVelocity) {
//        return (tangent, motionState, distanceRemaining) ->
//            MotionProfile.deceleration(deceleration).computeTargetVelocity(tangent, motionState, distanceRemaining)
//                .withMaxMagnitude(maxVelocity);
//    }
//
//    /**
//     *
//     * @param deceleration (inches/s^2) How fast the robot decelerates. Should be positive. 60 is
//     *                     slow deceleration while 120+ is faster deceleration.
//     * @param maxVelocity (inches/s) The maximum velocity the robot can reach. Do NOT set this to
//     *                   a large number like 999. This will cause transitional correction to not
//     *                    work.
//     */
//    static MotionProfile deceleration(double deceleration) {
//        return (tangent, motionState, distanceRemaining) -> {
//
//            Vector adjustedDeceleration = new Vector(-Math.abs(deceleration),
//                -Math.abs(deceleration) * 1.2);
////               Follower.getInstance().drivetrain.adjustDirectionalEffort(
////                    Vector.fromScalar(-Math.abs(deceleration)));
//            Vector remainingTangentVector =
//                motionState.makeRobotRelative(tangent.withMagnitude(distanceRemaining));
//
//            Vector x =
//                motionState.robotRelativeVelocity.minus(Kinematics.getFinalVelocityAtDistance(
//                motionState.robotRelativeVelocity,
//                new Vector(-37, -59),
//                remainingTangentVector
//            )); // how much velocity the robot is going to lose when it gets to the end
//            // is the velocity drop that the robot must undergo to stop or slow to vf at the
//            // target position
//
////            x tells you how much velocity you lose naturally by coasting to the target point.
////
////                It represents the velocity that will disappear just by letting go (zero power, no motor effort).
//
//            return motionState.toFieldRelativeVector(remainingTangentVector.map(adjustedDeceleration,
//                Kinematics::computeVelocityToStop
//            ));//.subtract(x));
//        };
//    }