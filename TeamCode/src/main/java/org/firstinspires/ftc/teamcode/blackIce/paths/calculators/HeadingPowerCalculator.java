package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;

@FunctionalInterface
public interface HeadingPowerCalculator {
    // the amount of power to turn clockwise
    double computeTargetHeading(PathFollowContext context);

    HeadingPowerCalculator maxSpeedProportional = (context)
        -> {
        double turnPower = context.headingInterpolator.interpolate(context.closestPoint);
        
        return turnPower;
    };
    
    // TODO include angular velocity (or make angular velocity in terms of t/distance )
//    HeadingPowerCalculator PD = (motionState, closestPoint, path,
//                                                   headingInterpolator)
//        -> headingInterpolator.interpolate(closestPoint) - motionState.heading * 0.03;
}
