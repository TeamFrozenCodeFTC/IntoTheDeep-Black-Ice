package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;

@FunctionalInterface
public interface VelocityController {
    Vector computeError(PathFollowContext context);
    
    static VelocityController controlEndVelocity(double endVelocity, double deceleration) {
        return context -> {
            double finalVelocity = Kinematics.getFinalVelocityAtDistance(
                context.motionState.velocityMagnitude,
                -Math.abs(deceleration),
                context.distanceToEnd
            );
            return context.pathExecutor.getCurrentSegment().getEndTangent().withMagnitude(endVelocity).minus(
                context.closestPointToRobot.getTangentVector().withMagnitude(finalVelocity));
        };
    }
    
    static VelocityController controlTargetVelocity(double deceleration) {
        
        return context -> {
            double targetVelocity = Kinematics.computeVelocityToStop(
                context.distanceToEnd,
                -Math.abs(deceleration)
            );
            double currentTangentialVelocity = context.motionState.fieldRelativeVelocity.dotProduct(context.closestPointToRobot.getTangentVector());
            
            return context.closestPointToRobot.getTangentVector().withMagnitude(targetVelocity - currentTangentialVelocity);
        };
    }
    
    // power ramping, power shut off / float to end
}
