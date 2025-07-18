package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathState;

//ᴘʏmon | 18535 Frozen Code — 1:42 PM
//but the gist of it is this:
//Tune the robot so you get an equation in the form y = ax^2 + bx to convert velocity (x) into braking displacement (y) where a is basically the robot’s deceleration and b is the braking force. The way I tune this is by mimicking the braking behavior when the motors are at zero power with zero power brake mode.
//It then uses this velocity to braking distance formula to predict where the robot would be if it slammed on its brakes right now. If the closest point to this braking displacement is the end point on the path, then robot knows to brake as hard as it can and it basically just switches to a strong positional proportional (endPoint - predicted position) where the predicted position is position + braking displacement. And if you don’t want any braking then instead of braking it just immediately skips to the next path.

/**
 * A function that provides the target velocity along the path to fit a certain velocity at a
 * certain point along the path.
 */
@FunctionalInterface
public interface VelocityProfile {
    double computeTargetVelocity(double totalDistance, double distanceRemaining);
    
    
    /**
     * Travels along the path at a constant velocity until the last moment where it needs to brake.
     */
    static VelocityProfile constantVelocity(double velocity) {
        return (totalDistance, distanceRemaining) -> velocity;
    }
    
    /**
     * Travels along the path at full speed until the last moment where it needs to brake.
     */
    VelocityProfile maximizeSpeed = null;
}