//package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;
//
//import static org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants.BRAKING_DISPLACEMENT;
//
//import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
//import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.math.kinematics.Kinematics;
//
//@FunctionalInterface
//public interface BrakingCalculator extends VectorCalculatorComponent {
//    BrakingCalculator velocityController =
//        (context) -> {
//            Vector offset = Vector.calculateForwardAndLateralOffset(
//                context.motionState.position,
//                context.motionState.heading,
//                context.endPoint.getPoint()
//            );
//
//            Vector offsetDir = offset.normalized();
//            double projectedSpeed = context.motionState.robotRelativeVelocity.dot(offsetDir);
//
//            if (projectedSpeed < 0) {
//                return null;
//            }
//
//            Vector projectedVelocity = offsetDir.times(projectedSpeed);
//
//            Vector robotVelocity = context.motionState.robotRelativeVelocity;
//
//            Vector robotDeltaVelocity = robotVelocity
//                .subtract(context.motionState.previousRobotRelativeVelocity);
//
//            Vector predictedRobotVelocity =
//                Kinematics.predictNextLoopVelocity(robotVelocity, robotDeltaVelocity);
//
//            // how much speed the robot is going to lose at the end point
//            double powerSaving = 0;
//            Vector finalVelocityAtEnd =
//                context.motionState.robotRelativeVelocity.subtract(Kinematics.getFinalVelocityAtDistance(
//                    projectedVelocity,
//                    new Vector(-41, -60),
//                    offset
//            )).times(powerSaving);
//
//            Vector velocityTargetToStopAtEnd = Kinematics.getCurrentVelocityToStopAtPositionWithDeceleration(
//            offset, context.path.getDeceleration())
//            .withMaxMagnitude(context.path.getMaxVelocity());
//
//            //.toFieldVector(context.motionState.heading);
//
//        Follower.getInstance().telemetry.addData("deltaTime (s)", context.motionState.deltaTime);
//            Follower.getInstance().telemetry.addData("targetVelocity", velocityTargetToStopAtEnd);
//
//            return velocityTargetToStopAtEnd
//                .subtract(predictedRobotVelocity)
//                .subtract(finalVelocityAtEnd);
//            return Vector.fromScalar(1);
//    };
//
//    BrakingCalculator positionalController =
//        (context) -> {
//            Vector robotVelocity = context.motionState.robotRelativeVelocity;
//
//            Vector robotDeltaVelocity = robotVelocity
//                .subtract(context.motionState.previousRobotRelativeVelocity);
//
//            Vector predictedRobotVelocity =
//                Kinematics.predictNextLoopVelocity(robotVelocity, robotDeltaVelocity);
//
//            Vector stoppingDisplacement =
//                BRAKING_DISPLACEMENT.getStoppingDistanceWithVelocity(predictedRobotVelocity);
//
//            Vector predictedStoppedPosition = context.motionState.position
//                .add(stoppingDisplacement.toFieldVector(context.motionState.heading));
//
//            return context.closestPoint.getPoint().subtract(predictedStoppedPosition);
//        };
//}
