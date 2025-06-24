package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Condition;
import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Action;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.PathSegment;

/**
 * A Path that follows path segments (can be extended with actions).
 * Caries state and individual parameters.
 * Should be stateless, pathExecutor should have state.
 * Immutable data - segments
 * Mutable behavior - actions
 */
public class Path extends PathConfig<Path> {
    public static final double NO_TIMEOUT = -1.0;
    
    private final PathSegment[] segments;
    private final PathSegment lastSegment;
    
    private final Vector endPoint;
    
    public final double length;
    
    public Path(PathConfig<?> startingConfig, PathSegment... segments) {
        super(startingConfig);
        
        if (segments == null || segments.length == 0) {
            throw new IllegalArgumentException("Path must have at least one segment.");
        }
        
        this.segments = segments;
        this.lastSegment = this.segments[this.segments.length - 1];
        this.endPoint = lastSegment.getEndPoint();
        double total = 0;
        for (PathSegment pathSegment : segments) {
            total += pathSegment.length();
        }
        this.length = total;
    }
    
    public Path(PathSegment... segments) {
        super();
        
        if (segments == null || segments.length == 0) {
            throw new IllegalArgumentException("Path must have at least one segment.");
        }
        
        this.segments = segments;
        this.lastSegment = this.segments[this.segments.length - 1];
        this.endPoint = lastSegment.getEndPoint();
        double total = 0;
        for (PathSegment pathSegment : segments) {
            total += pathSegment.length();
        }
        this.length = total;
    }
    
    public Path copy() {
        return new Path(this, getSegments());
    }
    public Path withConfig(PathConfig<?> startingConfig) {
        return new Path(startingConfig, getSegments());
    }
    
    public PathSegment[] getSegments() {
        return segments.clone();
    }
    
    public double getNumberOfSegments() {
        return segments.length;
    }

    public Pose getEndPose() {
        return new Pose(
            endPoint,
            getHeadingInterpolator().interpolate(lastSegment.getEndSegmentPoint())
        );
    }
    
    // todo timeouts

    private final ActionLoop actionLoop = new ActionLoop();
    
    /**
     * Continue this path and hold the end pose until the condition is true. This method sets the
     * "stopAtEnd" equal to true inorder to hold the end pose.
     * <pre><code>
     * .holdUntil(() -> robot.getLiftPosition() > 100) // hold until lift is above 100
     * </code></pre>
     * If the condition is true before the path is finished, the robot will continue until it
     * reaches the end of the path.
     */
    public Path holdUntil(Condition condition) {
        stopAtEnd();
        actionLoop.canFinishWhen(condition);
        return this;
    }
    
    /**
     * Immediately cancel this path when the condition is true.
     * <pre><code>
     * .cancelWhen(() -> robot.getLiftPosition() > 100) // stop when lift is above 100
     * </code></pre>
     */
    public Path cancelWhen(Condition condition) {
        actionLoop.cancelWhen(condition);
        return this;
    }
    
    /**
     * Calls the given action every loop while this path is being followed.
     * <pre><code>
     * .whileFollowing(() -> myLift.updatePosition()) // update lift position while following
     * </code></pre>
     */
    public Path whileFollowing(Action action) {
        actionLoop.onLoop(action);
        return this;
    }

    /**
     * Executes the action once the condition is true.
     * <pre><code>
     * path.doOnceWhen(slide::isRaised, () -> telemetry.addLine("Slide has raised!"));
     * </code></pre>
     */
    public Path doOnceWhen(Condition condition, Action executable) {
        final boolean[] hasRan = {false};
        return this
            .onStart(() -> hasRan[0] = false)
            .whileFollowing(() -> {
                if (!hasRan[0] && condition.isTrue()) {
                    hasRan[0] = true;
                    executable.execute();
                }
            });
    }

    /**
     * Executes the action every loop the condition is true.
     * <pre><code>
     * path.doWhen(gamepad1::a, slide::raise); // raise slide when A is pressed
     * </code></pre>
     */
    public Path doWhen(Condition condition, Action executable) {
        return this.whileFollowing(() -> executable.executeWhen(condition.isTrue()));
    }
    
    /**
     * Cancel the path when the condition is true, and execute the action.
     * path.(gamepad1::x, gamepad1::rumble); // rumble the controller when X is pressed
     */
    public Path cancelWhen(Condition condition, Action action) {
        actionLoop.cancelWhen(condition, action);
        return this;
    }

    /**
     * Finish the path early when the condition is true.
     * <pre><code>
     * path.earlyFinishWhen(pos.x > 60);
     * </code></pre>
     */
    public Path earlyFinishWhen(Condition condition) {
        actionLoop.earlyExitWhen(condition);
        return this;
    }

    /**
     * Finish the path early when the condition is true and execute an action.
     * <pre><code>
     * path.earlyFinishWhen(pos.x > 60, () -> telemetry.addLine("early finish"));
     * </code></pre>
     */
    public Path earlyFinishWhen(Condition condition, Action action) {
        actionLoop.earlyExitWhen(condition, action);
        return this;
    }

    /**
     * Add an action to be executed when the path starts.
     */
    public Path onStart(Action action) {
        actionLoop.onStart(action);
        return this;
    }

    /**
     * Add an action when the path successfully finishes. This includes early exits
     * but not cancellations.
     */
    public Path onFinish(Action action) { //have separate isAtEndOfPath() method
        actionLoop.onFinish(action);
        return this;
    }
    
    /**
     * Add an action when the path is canceled, either by an opMode stop, or other conditions like macro
     * cancelling buttons.
     */
    public Path onCancel(Action action) {
        actionLoop.onCancel(action);
        return this;
    }
    
    /**
     * Add an action to be executed when the path successfully finishing or is canceled.
     */
    public Path onExit(Action action) {
        actionLoop.onExit(action);
        return this;
    }
    
    public ActionLoop getActionLoop() {
        return actionLoop;
    }
}

//package org.firstinspires.ftc.teamcode.blackIce.paths;
//
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.kinematics.Kinematics;
//import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Pose;
//import org.firstinspires.ftc.teamcode.blackIce.math.MathFunctions;
//import org.firstinspires.ftc.teamcode.blackIce.paths.segments.SegmentPoint;
//import org.firstinspires.ftc.teamcode.blackIce.paths.segments.PathSegment;
//import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;
//
//import java.util.function.DoubleSupplier;
//import java.util.function.Function;
//
///**
// * A Path that follows path segments (can be extended with actions).
// * Caries state and individual parameters.
// * Should be stateless, pathExecutor should have state.
// */
//@Deprecated
//public class Path extends PathConfigurable<Path> {
//    final PathSegment[] segments;
//    private int currentSegmentIndex = 0;
//    private final double length;
//    private final PathSegment lastSegment;
//    private PathSegment currentSegment;
//
//    private final Follower follower;
//    private MotionState motionState;
//
//    double currentSegmentT = 0;
//    double predictedSegmentT = 0.01;
//
//    SegmentPoint closestCurvePointToStoppedPosition;
//    private boolean isBraking = false;
//    private boolean isFinished = false;
//
//    public Path(PathConfigurable<?> copyFrom, PathSegment... segments) {
//        super(copyFrom);
//        this.segments = segments;
//        this.follower = Follower.getInstance();
//        this.motionState = follower.getMotionState();
//        setPathActions();
//
//        double length = 0;
//        for (int i = 0; i < segments.length; i++) {
//            length += segments[i].length();
//        }
//        this.length = length;
//        this.lastSegment = this.segments[this.segments.length - 1];
//        this.currentSegment = this.segments[currentSegmentIndex];
//    }
//
//    // TODO create a builder that puts in a bunch of precalculated lines into a Path
//    // it shouldn't stray away from the path because it should just keep skipping the current Segment
//    // it wont go to the next loop if the robot skips a point
//
//
////    public Path(PathSegment curve, Path copyFrom) {
////        super(copyFrom);
////        this.segments = curve;
////        this.follower = Follower.getInstance();
////        this.motionState = follower.getMotionState();
////    }
//
//    private void setPathActions() {
//        this.withInitialize(this::reset)
//            .withLoop(this::update)
//            .withCondition(() -> isFinished);
//    }
//
//
////    public Path(PathSegment curve) {
////        this(curve, new PathConfigurable<>());
////    }
//
////    public Path copy() {
////        return new Path(segments, this);
////    }
//
////    public Path withConfigFrom(PathConfigurable<?> config) {
////        return new Path(segments, config);
////    }
//
//    /**
//     * Matches the robot's velocity to the progress of another action.
//     * For example if you wanted the robot to reach the end of the path at the same time
//     * as a lift, you could use this method to match the velocity of the robot to the progress.
//     * <pre><code>
//     * .matchVelocityWithProgress(
//     *     () -> currentLiftPosition / targetLiftPosition, // progress of lift
//     *     60 // maxVelocity
//     *  )</code></pre>
//     */
//    @Deprecated // FIXME - if lift is raised, velocity = 0
////    public Path matchVelocityWithProgress(DoubleSupplier getProgress, double maxVelocity) {
////        return this.withLoop(() -> setTargetVelocity(maxVelocity
////                * (1 - getProgress.getAsDouble())));
////    }
//    public Path matchVelocityWithProgress(DoubleSupplier getProgress, double maxVelocity) {
//        return this.withLoop(() -> {
//            double actionCompletePercent = (1 - getProgress.getAsDouble());
//            double pathCompletedPercent = (1 - getPercentAlongPath());
//            this.setTargetVelocity(pathCompletedPercent / actionCompletePercent * maxVelocity);
//        };
//    }
//
////    /**
////     * Sets the heading interpolation for the whole path chain.
////     * <p>
////     * {@inheritDoc}
////     */
////    @Override
////    public Path setHeadingInterpolation(HeadingInterpolator headingInterpolator) {
////        //
////        return super.setHeadingInterpolation((point) ->
////            headingInterpolator.interpolate(new CurvePoint(
////                point.getTValue() + length / (currentSegmentIndex + 1.0), // globalTValue
////
////                // getPercentAlongPath()
////                point.getTangentVector(),
////                point.getPoint())
////            )
////        );
////    }
//
//    @Override
//    public HeadingInterpolator getHeadingInterpolator() {
//        return (point) ->
//            super.getHeadingInterpolator().interpolate(
//                new SegmentPoint(
//                    getPercentAlongPath(),
//                    point.getTangentVector(),
//                    point.getPoint()
//                ));
//    }
//
//    /**
//     * Exit the path when the robot is past a certain position or meets a positional requirement.
//     * <pre><code>
//     * .exitAtPositionBoundary(position -> position.x > 60) // exit when x > 60
//     * </code></pre>
//     * Useful Example: when you want to immediately continue to the next path when the robot
//     * has put a sample in the observation zone.
//     */
//    public Path exitAtPositionBoundary(Function<Vector, Boolean> boundaryCondition) {
//        return this.withCancelCondition(() -> boundaryCondition.apply(motionState.position));
//    }
//
//     // TODO follow wall method
//
//    public Pose getEndPose() {
//        return new Pose(
//            lastSegment.getEndPoint(),
//            getHeadingInterpolator().interpolate(lastSegment.getEndCurvePoint())
//        );
//    }
//
//    private void reset() {
//        currentSegmentT = 0.01;
//        isBraking = false;
//        isFinished = false;
//    }
//
//    /**
//     * Tell whether the robot is currently braking (decelerating) at the end of the path.
//     */
//    public boolean isBraking() {
//        return isBraking;
//    }
//
//    /**
//     * Get the current T value (or percentage) along the entire path.
//     * <p>
//     * 0.00 at start, 0.50 in the middle, 1.00 at end.
//     */
//    public double getPercentAlongPath() {
//        double totalSegments = segments.length;
//        double segmentProgress = currentSegmentT / currentSegment.length();
//        // (1 + (1 / 1) / 5)
//        // have to use different method for point
//        double overallProgress = (currentSegmentIndex + segmentProgress) / totalSegments;
//        return MathFunctions.clamp0To1(overallProgress);
//    }
//
//    void advanceToNextSegment() {
//        currentSegmentIndex++;
//        currentSegment = segments[currentSegmentIndex];
//    }
//
//    public PathSegment getCurrentSegment() {
//        return currentSegment;
//    }
//
//    boolean hasReachedSegmentEnd() {
//        boolean isStopped = motionState.velocityMagnitude < getStoppedVelocityConstraint();
//        boolean isAtParametricEnd = currentSegmentT >= 0.995;
//        return isAtParametricEnd && isStopped;
//    }
//
//    private boolean isOnLastSegment() {
//        return currentSegmentIndex >= segments.length - 1;
//    }
//
////    /**
////     * Predict what the robot's position would be if it slammed on it's brakes right now.
////     */
////    private Vector predictStoppedPosition() {
////        Vector robotVelocity = motionState.robotRelativeVelocity;
////
////        Vector robotDeltaVelocity = robotVelocity
////            .subtract(follower.previousRobotRelativeVelocity); // is there a better way?
////
////        Vector predictedRobotVelocity =
////            Kinematics.predictNextLoopVelocity(robotVelocity, robotDeltaVelocity);
////
////        Vector stoppingDisplacement = TuningConstants.BRAKING_DISPLACEMENT
////            .getStoppingDistanceWithVelocity(predictedRobotVelocity);
////        Vector fieldStoppingDisplacement = stoppingDisplacement.toFieldVector();
////
////        return motionState.position
////            .add(fieldStoppingDisplacement.toFieldVector());
////    }
//
////    private double calculateVelocityAlignmentWithTangent(Vector tangentVector) {
////        return tangentVector
////            .dot(motionState.fieldRelativeVelocity.normalized());
////    }
////
////    /**
////     * Predict the next t value based how much distance the robot vector
////     * is going along the tangent line of the path.
////     */
////    private double getDistanceAlongPath(
////        Vector tangentVector,
////        Vector predictedStoppingDisplacement
////    ) {
////        double percentAlignedOnPath = calculateVelocityAlignmentWithTangent(tangentVector);
////        return predictedStoppingDisplacement.calculateMagnitude() * percentAlignedOnPath;
////    }
////
////    /**
////     * Predict the next t value based how much velocity the robot
////     * is going along the direction of the tangent line of the path.
////     */
////    double predictNextT(Vector tangentVector) {
////        double percentAlignedOnPath = calculateVelocityAlignmentWithTangent(tangentVector);
////        double velocityAlongPath = motionState.velocityMagnitude * percentAlignedOnPath;
////        return currentSegmentT + (velocityAlongPath / length) * motionState.deltaTime;
////    }
//
//
////    private void advanceToNextSegment() {
////        if (isOnLastSegment()) {
////            isFinished = true;
////        }
////        else {
////            currentSegmentIndex++;
////        }
////    }
//
////
////    /**
////     * Calculates the drive powers needed to follow the path.
////     * Uses the robot's predicted stopping position to find the closest point
////     * on the current path segment, then determines the appropriate drive action.
////     */
////    private DrivePowers computeDrivePowers() {
////        double robotSpeed = motionState.velocityMagnitude;
////
////        if (isBraking) {
////            return computeBrakingDrivePowersForSegment();
////        }
////
////        // Where the robot would be if it slammed on it's brakes right now
////        Vector predictedStoppedPosition = predictStoppedPosition();
////
////        return computeDrivePowersForSegment(predictedStoppedPosition, robotSpeed);
////    }
////
////    private DrivePowers computeDrivePowersForSegment(
////        Vector predictedStoppedPosition, double robotSpeed
////    ) {
////        closestCurvePointToStoppedPosition = currentSegment.calculateClosestPointTo(
////            predictedStoppedPosition,
////            this
////        );
////        if (currentSegment.getSegmentType().needsPredictedT) {
////            this.predictedSegmentT = this.predictNextT(closestCurvePointToStoppedPosition.getTangentVector());
////        }
////
////        boolean isOvershooting = closestCurvePointToStoppedPosition.isAtEnd();
////        if (isOvershooting) {
////            if (doesStopAtEnd()) {
////                isBraking = true;
////                return calculateBrakingDrivePowers();
////            }
////            advanceToNextSegment();
////            return computeDrivePowersForSegment(predictedStoppedPosition, robotSpeed);
////        }
////
////        return calculateAcceleratingDrivePowers(
////            closestCurvePointToStoppedPosition.getTValue(), robotSpeed, predictedStoppedPosition
////        );
////    }
////
////    private DrivePowers computeBrakingDrivePowersForSegment() {
////        if (hasReachedSegmentEnd()) { // + is within error margin +
////            // heading is aligned
////            isBraking = false;
////            advanceToNextSegment();
////        }
////        return calculateBrakingDrivePowers();
////    }
////
////    private DrivePowers calculateAcceleratingDrivePowers(
////        double closestT,
////        double robotSpeed,
////        Vector predictedStoppedPosition
////    ) {
////        double lookAheadInches = 1;
////        Vector closestPoint = currentSegment.calculatePointAt(MathFunctions.clamp0To1(
////            closestT + lookAheadInches / currentSegment.length())); // don't need tangent vector
////
////        double targetPower =
////            Math.abs((getTargetVelocity() - robotSpeed) * TuningConstants.kP);
////        // + (targetAcceleration - currentAcceleration) * TuningConstants.kD);
////
////        Vector driveVector = closestPoint.subtract(predictedStoppedPosition);
////
////        return DrivePowers.fromFieldVector(
////            driveVector.multiply(TuningConstants.VELOCITY_SCALING_VECTOR)
////        ).scaleMaxTo(targetPower);
////    }
////
////    private DrivePowers calculateHeadingDrivePowers(CurvePoint closestPoint) {
////        return DrivePowers.turnCounterclockwise(
////            getHeadingInterpolator().interpolate(closestPoint)
////                - motionState.heading
////                * 0.03
////        );
////    }
////
////    private DrivePowers calculateBrakingDrivePowers() {
////        Vector offset = Vector.calculateForwardAndLateralOffset(
////            motionState.position,
////            motionState.heading,
////            currentSegment.getEndPoint()
////        );
////
////        Vector velocityTargetToStopAtEnd = TuningConstants.BRAKING_DISPLACEMENT
////            .getTargetVelocityToStopAtDistance(offset)
////            .withMaxMagnitude(getTargetVelocity());
////
////        Vector velocityError = velocityTargetToStopAtEnd.subtract(
////            motionState.robotRelativeVelocity);
////        Vector targetVelocityDirection = velocityError
////            .multiply(TuningConstants.VELOCITY_SCALING_VECTOR);
////
////        return DrivePowers.fromRobotVector(targetVelocityDirection)
////            .multiply(TuningConstants.kP);
////    }
//}
//
