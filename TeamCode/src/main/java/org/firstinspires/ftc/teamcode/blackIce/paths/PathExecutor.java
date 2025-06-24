package org.firstinspires.ftc.teamcode.blackIce.paths;

import android.util.Log;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.MathFunctions;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.WheelPowersCalculator;
import org.firstinspires.ftc.teamcode.blackIce.paths.calculators.PathFollowContext;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.SegmentPoint;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.PathSegment;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.util.Advancer;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

// pause instead of canceling macros
// because then less detrimental if accidentally stop because there is a resume
// but would this work? how to know when to fully reset

// if macro has been paused for too long then restart macro otherwise resume
// TODO be able to pause path following by just setting wheels to zero power brake mode and
//  setting zero power

public class PathExecutor {
    private final Path path;

    private FollowingState state = FollowingState.FOLLOWING;
    
    private final Advancer<PathSegment> segmentAdvancer;
    private double currentSegmentT = 0.01;
    
    private final WheelPowersCalculator drivePowersCalculator;
    private final Drivetrain drivetrain;
    private final ActionLoop combinedLoop;
    
    public PathExecutor(
        WheelPowersCalculator drivePowersCalculator,
        Path path,
        Drivetrain drivetrain,
        ActionLoop globalActionLoop
    ) {
        this.drivePowersCalculator = drivePowersCalculator;
        this.path = path;
        this.segmentAdvancer = new Advancer<>(path.getSegments());
        this.drivetrain = drivetrain;
        
        // If dynamic follower changes are needed while following paths this combinedLoop
        // construction can be moved to the updateDrivePowersToFollowPath method.
        combinedLoop = globalActionLoop.combineWith(path.getActionLoop());
    }
    
    public ActionLoop getActionLoop() {
        return combinedLoop;
    }
    
    public void start() {
        combinedLoop.start();
    }
    
    public void cancel() {
        combinedLoop.cancel();
    }
    
    public PathSegment getCurrentSegment() {
        return segmentAdvancer.current();
    }
    
    public double getPercentAlongPath() {
        double numberOfSegments = path.getNumberOfSegments();
        double overallProgress = (segmentAdvancer.getIndex() + currentSegmentT) / numberOfSegments;
        return MathFunctions.clamp0To1(overallProgress);
    }
    
    public Path getPath() {
        return path;
    }
    
    // TODO test speed difference of opModeIsActive (has op.idle()) and with opModeHasRequestedStop

    /**
     * Does nothing once path isFinished and it is not holding the end Pose.
     */
    public void updateDrivePowersToFollowPath(MotionState motionState) {
        combinedLoop.loop(); // in follower?
        
        if (combinedLoop.hasCanceled() || combinedLoop.hasFinished()) {
            state = FollowingState.DONE;
            return;
        }
        
        // TODO when state is done make drive powers = 0 so robot doesn't drift
        
//        Follower.getInstance().telemetry.addData("state", getState());
        Follower.getInstance().telemetry.update();
        
        while (true) {
            FollowingState nextState;
            
            if (state == FollowingState.FOLLOWING) {
                nextState = followSegment(motionState);
            }
            else if (state == FollowingState.BRAKING || state == FollowingState.HOLDING) {
                nextState = applyBraking(motionState);
            }
            else if (state == FollowingState.DONE) {
                return;
            }
            else {
                throw new IllegalStateException("Unknown following state: " + state);
            }
            
            if (nextState == state) {
                return;
            }
            
            state = nextState;
        }
    }
    

    private FollowingState followSegment(MotionState motionState) {
        // Note: this is NOT the closest point to the robot, but the closest point to the robot's
        // braking displacement.
        SegmentPoint closestSegmentPoint = getCurrentSegment().calculateClosestPointTo(
            motionState.getPredictedStoppedPosition(),
            currentSegmentT
        );
        SegmentPoint closestPointToRobot = getCurrentSegment().calculateClosestPointTo(
            motionState.position,
            currentSegmentT
        );
        
        currentSegmentT = closestPointToRobot.getTValue();
        PathFollowContext context = new PathFollowContext(
            motionState,
            path,
            closestSegmentPoint,
            closestPointToRobot,
            this,
            (1-getPercentAlongPath())*path.length
        );
        
        // tune P for transitional without stopping distance to ensure oscillating isn't from stopping distance
        Logger.debug("distanceToEnd", (1 - getPercentAlongPath()) * path.length);
        
//        Vector offsetDir =
//            context.endPoint.getPoint().subtract(context.motionState.position).normalized();
//        double projectedSpeed = context.motionState.robotRelativeVelocity.dot(offsetDir);
        // || projectedSpeed < 0 works pretty well
        if (closestSegmentPoint.isAtEnd()) {
            Logger.debug("BRAKING -------------------");
            return handleOvershooting();
        }
        
//        double stoppingDistance = Kinematics.getStoppingDistance(
//            motionState.fieldRelativeVelocity.dot(closestCurvePoint.getTangentVector()),
//            path.getDeceleration() // * decelerationStartMultiplier
//        );
//        double distanceToNextSegment = (1 - currentSegmentT) * getCurrentSegment().length();
//        if (stoppingDistance > distanceToNextSegment) {
//            return handleOvershooting();
//        }
        
        drivePowersCalculator.accelerate(drivetrain, context);
        
        return FollowingState.FOLLOWING;
    }
    
    private FollowingState handleOvershooting() {
        if (path.doesStopAtEnd()) {
            return FollowingState.BRAKING;
        }
        if (segmentAdvancer.isDone()) { // does not listen to actionLoop canFinish because the
            // robot is keeping it's momentum and cannot hold a point
            combinedLoop.finish();
            return FollowingState.DONE; // go to next path as fast as possible
        }
        // Follow next segment
        segmentAdvancer.advance();
        return FollowingState.FOLLOWING;
    }
    
    private FollowingState applyBraking(MotionState motionState) {
        
        if (hasStoppedAtSegmentEnd(motionState)) { // + is within error margin +
            // heading is aligned OR when drive powers are less than 0.1 or something
            state = FollowingState.HOLDING;

            if (!segmentAdvancer.isDone()) {
                segmentAdvancer.advance();
                return FollowingState.FOLLOWING;
            }
            else if (combinedLoop.canFinish()) {
                combinedLoop.finish();
                return FollowingState.DONE;
            }
            Logger.debug("holding -------------------");
            
            // pause
            // cancel = pause and restart state
        }
        
        drivePowersCalculator.positionalHold(drivetrain, new PathFollowContext(
            motionState,
            path,
            getCurrentSegment().getEndSegmentPoint(),
            getCurrentSegment().getEndSegmentPoint(),
            this,
            (1-getPercentAlongPath())*path.length
        ));
        return state;
    }
    
    public FollowingState getState() {
        return state;
    }
    
    private boolean hasStoppedAtSegmentEnd(MotionState motionState) {
        boolean isStopped =
            motionState.velocityMagnitude < path.getStoppedVelocityConstraint()
            && motionState.angularVelocity < path.getStoppedAngularVelocityConstraint();
        boolean isAtParametricEnd = currentSegmentT >= 0.995; // make this a constraint
        return isAtParametricEnd && isStopped;
    }
}


//        switch (state) {
//    case FOLLOWING:
//state = accelerate(motionState);
//                break;
//                    case DONE:
//    break;
//    case BRAKING:
//state = brake(motionState);
//                break;
//                    }