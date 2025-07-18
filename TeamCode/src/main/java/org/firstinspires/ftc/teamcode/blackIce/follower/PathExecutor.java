package org.firstinspires.ftc.teamcode.blackIce.follower;

import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.ActionLoop;
import org.firstinspires.ftc.teamcode.blackIce.paths.FollowingState;
import org.firstinspires.ftc.teamcode.blackIce.paths.ImmutablePathSequence;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathState;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathPoint;
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
    private final DrivePowerController drivePowersCalculator;
    private final Drivetrain drivetrain;
    
    private ActionLoop combinedLoop;
    private Advancer<Path> pathAdvancer;
    public FollowingState state = FollowingState.NOT_FOLLOWING;
    private PathState pathState;
    private ActionLoop globalActionLoop;

    public PathExecutor(
        DrivePowerController drivePowersCalculator,
        Drivetrain drivetrain
    ) {
        this.drivePowersCalculator = drivePowersCalculator;
        this.drivetrain = drivetrain;
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
    
    public Path getCurrentPath() {
        return pathAdvancer.current();
    }
    
    // TODO test speed difference of opModeIsActive (has op.idle()) and with opModeHasRequestedStop
    
//    private FollowingState state = FollowingState.FOLLOWING;
//
//    private double currentParametricTValue = 0.01;

    public void startFollowing(ActionLoop globalActionLoop, ImmutablePathSequence pathSequence) {
        pathAdvancer = pathSequence.advancer();
        
        pathState = new PathState(
            getCurrentPath().geometry.length(),
            0,
            0,
            0,
            getCurrentPath().geometry.getEndPathPoint(),
            getCurrentPath().geometry.getEndPathPoint(),
            closestPathPointToPredictedStop2
        );
   
        this.globalActionLoop = globalActionLoop;
        
        state = FollowingState.FOLLOWING;
        combinedLoop = globalActionLoop.combineWith(getCurrentPath().behavior.actionLoop);
        combinedLoop.start();
     }
     
     private void advance() {
         pathAdvancer.advance();
         state = FollowingState.FOLLOWING;
         combinedLoop = globalActionLoop.combineWith(getCurrentPath().behavior.actionLoop);
         combinedLoop.start();
     }
     
     public void pause() {
        // zero power brake mode
        state = FollowingState.PAUSED;
     }
     
     public void resume() {
        state = FollowingState.FOLLOWING;
//        followingPathTimer.reset();
//        stuckDetectedTimer.reset();
     }
    
    /**
     * Does nothing once path isFinished and it is not holding the end Pose.
     */
    public void updateDrivePowersToFollowPath(MotionState motionState) {
        if (state == FollowingState.PAUSED || state == FollowingState.NOT_FOLLOWING) return;
        
        combinedLoop.loop(); // in follower?
        
        if (combinedLoop.hasCanceled() || combinedLoop.hasFinished()) {
            state = FollowingState.DONE;
            return;
        }
        
        // TODO when state is done make drive powers = 0 so robot doesn't drift

        Follower.getInstance().telemetry.update();
        
        while (true) {
            FollowingState nextState;
            
            if (state == FollowingState.FOLLOWING) {
                nextState = follow(motionState);
            }
            else if (state == FollowingState.BRAKING || state == FollowingState.HOLDING) {
                nextState = applyBraking(motionState);
            }
            else if (state == FollowingState.DONE) {
                break;
            }
            else {
                throw new IllegalStateException("Unknown following state: " + state);
            }
            
            if (nextState == state) {
                break;
            }
            
            state = nextState;
        }
        Logger.debug("XXX currentPosition", motionState.position);
        Logger.debug("XXX predictedPosition", motionState.getPredictedStoppedPosition());
        Logger.debug("XXX closestPointToRobot", closestPathPointToRobot.getPoint());
        Logger.debug("XXX closestPointToBraking", closestPathPointToPredictedStop.getPoint());
    }
    PathPoint closestPathPointToRobot;
    PathPoint closestPathPointToPredictedStop;
    PathPoint closestPathPointToPredictedStop2;

    private FollowingState follow(MotionState motionState) {
        // Note: this is NOT the closest point to the robot, but the closest point to the robot's
        // braking displacement.
        closestPathPointToPredictedStop = getCurrentPath().geometry.computeClosestPathPointTo(
            motionState.getPredictedStoppedPosition().minus(motionState.position).plus(motionState.position),
            pathState.currentTValue
        );
        closestPathPointToPredictedStop2 = getCurrentPath().geometry.computeClosestPathPointTo(
            motionState.getPredictedStoppedPosition().minus(motionState.position).times(10).plus(motionState.position),
            pathState.currentTValue
        );
        closestPathPointToRobot = getCurrentPath().geometry.computeClosestPathPointTo(
            motionState.position,
            pathState.currentTValue
        );
        // Note these are very rough estimates based of parametric t values which are not equal
        // to the length of the curve. 0.5 is not the half way point
        double distanceRemaining =
            getCurrentPath().geometry.computeDistanceRemainingAt(pathState.currentTValue);
        double distanceTraveled =
            getCurrentPath().geometry.computeDistanceRemainingAt(1 - pathState.currentTValue);

        this.pathState = new PathState(
            distanceRemaining,
            closestPathPointToRobot.getTValue(),
            distanceTraveled,
//            motionState.fieldRelativeVelocity
//                .dotProduct(closestPathPointToPredictedStop.getTangentVector()),
            motionState.velocity
                .dotProduct(closestPathPointToRobot.getTangentVector()),
            closestPathPointToRobot,
            closestPathPointToPredictedStop,
            closestPathPointToPredictedStop2
        );
        
        // tune P for transitional without stopping distance to ensure oscillating isn't from stopping distance
        Logger.debug("distanceRemaining", distanceRemaining);
        
        if (closestPathPointToPredictedStop.getTValue() >= 1) {
            return handleOvershooting();
        }
        
        drivePowersCalculator.accelerate(drivetrain, pathState, getCurrentPath(), motionState);
        
        return FollowingState.FOLLOWING;
    }
    
    private FollowingState handleOvershooting() {
        if (!pathAdvancer.isDone()) {
            advance();
            return FollowingState.FOLLOWING;
        }
        if (getCurrentPath().behavior.stopAtEnd) {
            return FollowingState.BRAKING;
        }
        // does not listen to actionLoop canFinish because the
        // robot is keeping it's momentum and cannot hold a point
        combinedLoop.finish();
        return FollowingState.DONE; // go to next path as fast as possible
    }
    
    private FollowingState applyBraking(MotionState motionState) {
        
        if (hasStoppedAtSegmentEnd(motionState)) { // + is within error margin +
            // heading is aligned OR when drive powers are less than 0.1 or something
            state = FollowingState.HOLDING;
            
            if (!pathAdvancer.isDone()) {
                advance();
                return FollowingState.FOLLOWING;
            }
            if (combinedLoop.canFinish()) {
                combinedLoop.finish();
                return FollowingState.DONE;
            }
            Logger.debug("holding -------------------");
            
            // pause
            // cancel = pause and restart state
        }
        this.pathState = new PathState(
            0,
            closestPathPointToRobot.getTValue(),
            getCurrentPath().geometry.length(),
//            motionState.fieldRelativeVelocity
//                .dotProduct(closestPathPointToPredictedStop.getTangentVector()),
            motionState.velocity
                .dotProduct(closestPathPointToRobot.getTangentVector()),
            getCurrentPath().geometry.getEndPathPoint(),
            getCurrentPath().geometry.getEndPathPoint(),
            closestPathPointToPredictedStop2
        );
        drivePowersCalculator.accelerate(drivetrain, pathState, getCurrentPath(), motionState);
        //drivePowersCalculator.positionalHold(drivetrain, getCurrentPath(), pathState, motionState);

        return state;
    }
    
    public FollowingState getState() {
        return state;
    }
    
    private boolean hasStoppedAtSegmentEnd(MotionState motionState) {
        boolean isStopped =
            motionState.speed < getCurrentPath().behavior.stoppedVelocityConstraint
            && motionState.angularVelocity < getCurrentPath().behavior.stoppedAngularVelocityConstraint;
        boolean isAtParametricEnd = pathState.currentTValue >= 0.995; // make this a constraint
        return isAtParametricEnd && isStopped;
    }
}