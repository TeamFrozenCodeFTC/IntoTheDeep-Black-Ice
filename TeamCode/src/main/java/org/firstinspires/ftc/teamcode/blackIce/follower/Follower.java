package org.firstinspires.ftc.teamcode.blackIce.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.action.Condition;
import org.firstinspires.ftc.teamcode.blackIce.action.Action;
import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.motion.MotionTracker;
import org.firstinspires.ftc.teamcode.blackIce.paths.ActionLoop;
import org.firstinspires.ftc.teamcode.blackIce.paths.FollowingState;
import org.firstinspires.ftc.teamcode.blackIce.paths.ImmutablePathSequence;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathBehaviorModifier;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequence;
import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

/**
 * The main class that initializes autonomous and tele-op modes and follows paths.
 */
public class Follower { // TODO add wait func
    private static Follower INSTANCE;
    private static Pose lastOpModePose;
    
    public static Follower getInstance() {
        return INSTANCE;
    }
    
    public static Pose getLastOpModePose() {
        return lastOpModePose;
    }
    
    public final MultipleTelemetry telemetry;
    public final LinearOpMode opMode;
    
    public final Localizer localizer;

    public MotionState motionState;
    private final MotionTracker motionTracker;
    public final Drivetrain drivetrain;

    private PathBehaviorModifier defaultPathBehavior;
    
    private final ElapsedTime followingPathTimer = new ElapsedTime(0);
    private final ElapsedTime stuckDetectedTimer = new ElapsedTime(0);

    public void addDefaultPathBehavior(PathBehaviorModifier config) {
        defaultPathBehavior = defaultPathBehavior.combine(config);
    }
    
    private final PathExecutor pathExecutor;
    private final Pose startingPose;

    public MotionState getMotionState() {
        return motionState;
    }

    public Follower(
        LinearOpMode opMode,
        Pose startingPose,
        FollowerConfig config
    ) {
        this.defaultPathBehavior = config.defaultPathBehavior;
        INSTANCE = this;
        this.opMode = opMode;
        this.startingPose = startingPose;
        
        this.localizer = config.localizer;
        this.motionTracker = new MotionTracker(localizer);
        
        updateMotionState();
        // Path[] path = new PathBuilder(startingPose) || follower.pathBuilder().,
        // .lineTo(new Vector(1,1), new PathBehaviorConfig().stopAtEnd())
        // .lineTo(new Vector(1,1), new PathBehaviorConfig().stopAtEnd())
        // .build()...
        // should Follower have default behavior?
//        this.pathConstructor = new PathSequenceConstructor(startingPose, this.defaultPathBehavior);
//
        this.drivetrain = config.drivetrain;

        this.pathExecutor = new PathExecutor(new DrivePowerController(
            config.headingPID,
            config.positionalPID,
            config.translationalPID,
            config.driveVelocityPIDF
        ), drivetrain);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
//        opMode.telemetry = telemetry;
        
//        addDefaultPathBehavior(path ->
//            path.cancelWhen(() -> {
//                    if (path.getStuckTimeoutSeconds() != NO_TIMEOUT) {
//                        return false;
//                    }
//                    if (motionState.velocityMagnitude > path.getStuckVelocityConstraint()) {
//                        stuckDetectedTimer.reset();
//                        return false;
//                    }
//                    return stuckDetectedTimer.seconds() < path.getStuckTimeoutSeconds();
//                })
//                .cancelWhen(() -> {
//                    if (path.getTimeoutSeconds() != NO_TIMEOUT) {
//                        return false;
//                    }
//                    return followingPathTimer.seconds() < path.getTimeoutSeconds();
//        }));
    }
    
    public Follower(LinearOpMode opMode, Pose startingPose) {
        this(opMode, startingPose, FollowerConstants.defaultFollowerConfig(opMode));
    }
    
    /**
     * Initializes the follower at Pose(0,0,0)
     */
    public Follower(LinearOpMode opMode) {
        this(opMode, new Pose(0,0,0));
    }
    
    public void resume() {
        pathExecutor.resume();
    }
    
    public void pause() {
        pathExecutor.pause();
    }
    
    public boolean isPaused() {
        return pathExecutor.state == FollowingState.PAUSED;
    }
    public boolean isDoneFollowingPath() {
        return pathExecutor.state == FollowingState.DONE;
    }
    
    /**
     * Matches the robot's velocity to the progress of another action.
     * For example if you wanted the robot to reach the end of the path at the same time
     * as a lift, you could use this method to match the velocity of the robot to the progress.
     * <pre><code>
     * .matchVelocityWithProgress(
     *     () -> currentLiftPosition / targetLiftPosition, // progress of lift
     *     60 // maxVelocity
     *  )</code></pre>
     */
//    public Path matchPathVelocityWithProgress(Path path, DoubleSupplier getProgress,
//                                              double maxVelocity) {
//        return path.whileFollowing(() -> {
//            double actionCompletePercent = (1 - getProgress.getAsDouble());
//            double pathCompletedPercent = (1 - getCurrentPathExecutor().getPercentAlongPath());
//            path.setMaxVelocity(pathCompletedPercent / actionCompletePercent * maxVelocity);
//        }); // motionProfile with targetVelocity at t?
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
//    public Path exitPathAtPositionBoundary(Path path,
//                                            Function<Vector, Boolean> boundaryCondition) {
//        return path.cancelWhen(() -> boundaryCondition.apply(getMotionState().position));
//    }
    
    
    /**
     * Set the robot's localizer to the given pose.
     * Also updates the path constructor's current pose so next path starts from here
     * This is useful to reset the robot's position during a match if you know exactly where it is.
     */
    public void setCurrentPose(Pose pose) {
        localizer.setPose(pose);
        //pathConstructor.setCurrentPose(pose); TODO
    }
    
    // TODO make macro cancel current path following and start macro
    
//    public PathConfig<?> newPathConfigurable() {
//        return this.copyConfig();
//    }

//    public Follower beginFollowing(Path... paths) {
//        pathExecutor.startFollowing(actionFollowingLoop, paths);
//        return this;
//    }
    
    public Follower beginFollowing(PathSequence pathSequence) {
        ImmutablePathSequence loadedPathSequence;
        if (pathSequence.hasStartingPose()) {
            loadedPathSequence = pathSequence.build();
        }
        else {
            loadedPathSequence = pathSequence.build(startingPose);
        }
        pathExecutor.startFollowing(actionFollowingLoop, loadedPathSequence);
        return this;
    }
    
//
//    ImmutablePathSequence loadedPathSequence;
//    private boolean hasPreloaded = false;
//
//    public Follower preload(PathSequence... pathSequences) {
//        if (hasPreloaded) {
//            throw new IllegalStateException("Cannot preload more than once.");
//        }
//        hasPreloaded = true;
//        Pose previousTargetPose = startingPose;
//        for (PathSequence pathSequence : pathSequences) {
//            if (pathSequence.hasStartingPose()) {
//                loadedPathSequence = pathSequence.build();
//            }
//            else {
//                loadedPathSequence = pathSequence.build(previousTargetPose);
//            }
//            previousTargetPose = loadedPathSequence.endPose;
//        }
//        return this;
//    }

    public Path getCurrentPath() {
        return pathExecutor.getCurrentPath();
    }
    
    public void updateMotionState() {
        motionTracker.update();
        motionState = motionTracker.getMotionState();
    }
    
    public void update() {
        updateMotionState();
        
        Logger.debug("Updated ---------" + pathExecutor.getState());
        Logger.debug("currentHeadingInit", Math.toDegrees(motionState.heading));
        Logger.debug("hasCanceled", actionFollowingLoop.hasCanceled());
        
        if (isPaused() || actionFollowingLoop.hasCanceled()) {
            return;
        }
        
        pathExecutor.updateDrivePowersToFollowPath(motionState);
    }
    
    /**
     * Wait for the robot to reach the end of the path.
     */
    public void waitForPath() {
        while (!isDoneFollowingPath()) {
            update();
        }
    }
    
//
//    /**
//     * Continue to follow the path until the path is canceled.
//     * Continues to hold the end point in the path if it is not canceled.
//     */
//    public void holdPathUntilCancel() {
//        while (!getCurrentPathExecutor().) {
//            update();
//        }
//    }
    
    private final ActionLoop actionFollowingLoop = new ActionLoop();
    
//    /**
//     * Continues and hold's the current path end pose until the condition is true.
//     * <pre><code>
//     * .holdUntil(() -> robot.getLiftPosition() > 100) // hold until lift is above 100
//     * </code></pre>
//     * If the condition is true before the path is finished, the robot will continue until it
//     * reaches the end of the path.
//     */
//    public Follower holdUntil(Condition conditionIsTrue) {
//        getCurrentPathExecutor().getActionLoop().canFinishWhen(conditionIsTrue);
//        waitForPath();
//        return this;
//    }
    
    /**
     * Cancel and stop following all paths when the condition is true.
     * <pre><code>
     * follower.cancelAllWhen(() -> robot.getLiftPosition() > 100) // stop when lift is above 100
     * </code></pre>
     * Will not return to following paths until the condition is false.
     */
    public Follower cancelAllWhen(Condition condition) {
        drivetrain.zeroPowerBrakeMode();
        drivetrain.zeroPower();
        actionFollowingLoop.cancelWhen(condition);
        return this;
    }
    /**
     * Cancel and stop following all paths when the condition is true and execute the action.
     * <pre><code>
     * follower.cancelAllWhen(gamepad1::x, gamepad1::rumble); // rumble the controller when X is pressed and cancel
     * </code></pre>
     */
    public Follower cancelAllWhen(Condition condition, Action action) {
        drivetrain.zeroPowerBrakeMode();
        drivetrain.zeroPower(); // this needs to be in the actual action
        actionFollowingLoop.cancelWhen(condition, action);
        return this;
    }
    
    // update
//    localizer.update();
//    motionTracker.update();
//    pathExecutor.updateDrivePowersToFollowPath(...);
//
//    if (actionLoop != null && actionLoop.isRunning()) {
//        actionLoop.loop();
//    }

    /**
     * Calls the given action every loop while a path is being followed.
     * <pre><code>
     * follower.onUpdate(() -> myLift.updatePosition()) // update lift position while following
     * </code></pre>
     */
    public Follower whileFollowing(Action action) {
        actionFollowingLoop.onLoop(action);
        return this;
    }
    
    /**
     * Executes the action when the condition is true.
     * <pre><code>
     * follower.doWhen(gamepad1::a, slide::raise); // raise slide whenever A is pressed
     * </code></pre>
     */
    public Follower doWhen(Condition condition, Action executable) {
        return this.whileFollowing(() -> executable.executeWhen(condition.isTrue()));
    }
    
    // Not very many use cases for these:
    /**
     * Add an action to be executed when a path starts.
     */
    public Follower onPathStart(Action action) {
        actionFollowingLoop.onStart(action);
        return this;
    }
    /**
     * Add an action to be executed when the path successfully finishes. This includes early exits
     * but not cancellations.
     */
    public Follower onPathFinish(Action action) {
        actionFollowingLoop.onFinish(action);
        return this;
    }
    /**
     * When a path is canceled, either by an opMode stop, or other conditions like macro
     * cancelling buttons.
     */
    public Follower onPathCancel(Action action) {
        actionFollowingLoop.onCancel(action);
        return this;
    }
    /**
     * When the path is exited, either by successfully finishing or canceling.
     */
    public Follower onPathExit(Action action) {
        actionFollowingLoop.onExit(action);
        return this;
    }

    public void waitUntilOpModeStop() {
        waitUntil(Condition.NEVER);
    }

    public void waitUntil(Condition condition) {
        while (opMode.opModeIsActive() && !condition.isTrue()) {
            opMode.idle();
        }
    }

    /**
     * Initializes the robot for tele-op mode,
     * using the position from the end of the autonomous period.
     */
    public void initTeleOp() {
        localizer.setPose(lastOpModePose);
        motionTracker.update();
        motionState = motionTracker.getMotionState();
        drivetrain.zeroPowerBrakeMode();
    }
    
    public void savePoseForTeleOp() {
        lastOpModePose = new Pose(
            motionState.position,
            Math.toDegrees(motionState.heading)
        );
    }

    public void setHeadingResetButton(Condition gamepadCondition, double headingDegrees) {
        this.doWhen(
            gamepadCondition,
            () -> localizer.setHeading(headingDegrees)
        );
    }

    public void setPoseResetButton(
        Condition gamepadCondition,
        double resetX, double resetY, double resetHeading
    ) {
        this.doWhen(
            gamepadCondition,
            () -> localizer.setPose(resetX, resetY, resetHeading)
        );
    }

    /**
     * Run a basic field-centric tele-op.
     * <p>
     * For driver field-centric
     * <pre><code>
     * follower.fieldCentricTeleOpDrive(
     *     -gamepad1.left_stick_y
     *     -gamepad1.left_stick_x,
     *     -gamepad1.right_stick_x
     * );
     * </code></pre>
     */
    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
        updateMotionState();
        if (opMode.gamepad1.right_stick_button){ // turning
            drivetrain.applyBrakingPowers(motionState.makeRobotRelative(new Vector(forward,
                    lateral)),
                turn);
        }
        else { // simple add
            drivetrain.followVector(motionState.makeRobotRelative(new Vector(forward, lateral)),
                turn); // combine into one with condition
        }
    }

    public void robotCentricDrive(double y, double x, double turn) {
        updateMotionState();
        drivetrain.followVector(new Vector(x, y), turn);
    }
}

