package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

/**
 * Finds the robot relative vector to accelerate and follow the path.
 */
@FunctionalInterface
public interface DriveVectorCalculator extends VectorCalculatorComponent {
    
    DriveVectorCalculator drive = (context) -> {

        Vector distanceVectorToEnd =
            context.closestPoint.getTangentVector().withMagnitude(context.distanceToEnd);
        
        Vector targetVelocity = new Vector(
            Kinematics.getCurrentVelocityToStopAtPositionWithDeceleration(
                distanceVectorToEnd.getX(),
                    context.path.getDeceleration()),
            Kinematics.getCurrentVelocityToStopAtPositionWithDeceleration(
                distanceVectorToEnd.getY(),
                context.path.getDeceleration() * 1.1)
        ).withMaxMagnitude(context.path.getMaxVelocity());
        
        Logger.debug("targetVelocity", targetVelocity.computeMagnitude());
        Logger.debug("currentVelocity", context.motionState.fieldRelativeVelocity.computeMagnitude());

        return targetVelocity;
    };
    

}


//        double lookAheadInches = 1;
//
// this is not needed for point curve direct
//        Vector lookAheadPoint =
//            context.pathExecutor.getCurrentSegment().calculatePointAt(MathFunctions.clamp0To1(
//                context.closestPoint.getTValue() + lookAheadInches / context.pathExecutor.getCurrentSegment().length()));
//
//        Vector driveVector = lookAheadPoint.subtract(
//            context.motionState.getPredictedStoppedPosition()
//        );
// TODO find distance from point laterally
// + tangent
// braking profile but with target velocity of 60

//        Vector offset = Vector.calculateForwardAndLateralOffset(
//            context.motionState.getPredictedStoppedPosition(),
//            context.motionState.heading,
//            context.closestPoint.getPoint() // = endPoint
//        );
//        double horizontalDistanceToPath = offset.getY();


//        Vector velocityTargetToStopAtEnd = TuningConstants.BRAKING_DISPLACEMENT
//            .getTargetVelocityToStopAtDistance(offset)
//            .withMaxMagnitude(context.path.getMaxVelocity());
//
//        Vector velocityError = velocityTargetToStopAtEnd
//            .subtract(context.motionState.robotRelativeVelocity);
//
//        return velocityError.multiply(TuningConstants.kP);
//
//        return driveVector.withMagnitude(velocityProportional.compute(context)).toRobotVector(context.motionState.heading);