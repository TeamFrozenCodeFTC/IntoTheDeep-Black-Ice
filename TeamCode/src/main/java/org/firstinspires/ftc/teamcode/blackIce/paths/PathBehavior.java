package org.firstinspires.ftc.teamcode.blackIce.paths;

// Black Ice Path Follower

import static org.firstinspires.ftc.teamcode.blackIce.util.Utils.getOrDefault;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.blackIce.action.Action;
import org.firstinspires.ftc.teamcode.blackIce.action.Condition;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.DrivePowerModifier;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.VelocityProfile;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.TrapezoidalVelocityProfile;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

import java.lang.reflect.Field;

/**
 * This is a mutable builder class for the internally used ImmutablePathBehavior.
 */
public class PathBehavior implements Cloneable {
    private Boolean stopAtEnd = null;
    private Boolean preventReverseDeceleration = null;
    private Boolean decelerateWithZeroPower = null;
    private Boolean decelerateWithoutFeedforward = null;
    
    private Double stoppedVelocityConstraint = null;
    private Double stoppedAngularVelocityConstraint = null;

    private Double maxVelocity = null;
    private Double endingVelocity = null;
    @Nullable
    private TrapezoidalVelocityProfile.Acceleration acceleration = null;
    @Nullable
    private TrapezoidalVelocityProfile.Acceleration deceleration = null;
    private Double endingVelocityCruiseDistance = null;
    private VelocityProfile velocityProfile;
    
    private Double stuckVelocityConstraint = null;
    private Double stuckTimeoutSeconds = null;
    private Boolean cancelPathIfStuck = null;
    
    private Double timeoutSeconds = null;
    
    private DrivePowerModifier drivePowerModifier = null;
    
    private final ActionLoop actionLoop = new ActionLoop();
    
    public PathBehavior() {}
    
    ImmutablePathBehavior build() {
        return new ImmutablePathBehavior(
            actionLoop,
            getOrDefault(stopAtEnd, false),
            getOrDefault(decelerateWithoutFeedforward, true),
            getOrDefault(stoppedVelocityConstraint, 1.0),
            getOrDefault(stoppedAngularVelocityConstraint, Math.toRadians(1.0)),
            getOrDefault(stuckVelocityConstraint, 1.0),
            getOrDefault(stuckTimeoutSeconds, 5.0),
            getOrDefault(cancelPathIfStuck, true),
            getOrDefault(timeoutSeconds, 8.0),
            getOrDefault(velocityProfile, null),
            getOrDefault(drivePowerModifier, DrivePowerModifier.none)
        );
    }
    
    // PathBuilder.setDefaultPathBehavior(.holdUntil(...))
    // PathBuilder.addDefaultPathBehavior(.holdUntil(...))
    
    /**
     * Continue this path, stop, and hold the end pose until the condition is true.
     * <pre><code>
     * .holdUntil(() -> robot.getLiftPosition() > 100) // hold until lift is above 100
     * </code></pre>
     * If the condition is true before the path is finished, the robot will continue until it
     * reaches the end of the path.
     */
    public PathBehavior holdUntil(Condition condition) {
        stopAtEnd(); // allow it to not have to stop if the hold until is not active and have it
        // exit by braking distance amount.
        actionLoop.canFinishWhen(condition);
        return this;
    }
    
    /**
     * Immediately cancel this path when the condition is true.
     * <pre><code>
     * .cancelWhen(() -> robot.getLiftPosition() > 100) // stop when lift is above 100
     * </code></pre>
     */
    public PathBehavior cancelWhen(Condition condition) {
        actionLoop.cancelWhen(condition);
        return this;
    }
    
    /**
     * Calls the given action every loop while this path is being followed.
     * <pre><code>
     * .whileFollowing(() -> myLift.updatePosition()) // update lift position while following
     * </code></pre>
     */
    public PathBehavior whileFollowing(Action action) {
        actionLoop.onLoop(action);
        return this;
    }
    
    /**
     * Executes the action once the condition is true.
     * <pre><code>
     * path.doOnceWhen(slide::isRaised, () -> telemetry.addLine("Slide has raised!"));
     * </code></pre>
     */
    public PathBehavior doOnceWhen(Condition condition, Action executable) {
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
    public PathBehavior doWhen(Condition condition, Action executable) {
        return this.whileFollowing(() -> executable.executeWhen(condition.isTrue()));
    }
    
    /**
     * Cancel the path when the condition is true, and execute the action.
     * path.(gamepad1::x, gamepad1::rumble); // rumble the controller when X is pressed
     */
    public PathBehavior cancelWhen(Condition condition, Action action) {
        actionLoop.cancelWhen(condition, action);
        return this;
    }
    
    /**
     * Finish the path early when the condition is true.
     * <pre><code>
     * path.earlyFinishWhen(pos.x > 60);
     * </code></pre>
     */
    public PathBehavior earlyFinishWhen(Condition condition) {
        actionLoop.earlyExitWhen(condition);
        return this;
    }
    
    /**
     * Finish the path early when the condition is true, and execute an action.
     * <pre><code>
     * path.earlyFinishWhen(pos.x > 60, () -> telemetry.addLine("early finish"));
     * </code></pre>
     */
    public PathBehavior earlyFinishWhen(Condition condition, Action action) {
        actionLoop.earlyExitWhen(condition, action);
        return this;
    }
    
    /**
     * Add an action to be executed when the path starts.
     */
    public PathBehavior onStart(Action action) {
        actionLoop.onStart(action);
        return this;
    }
    
    /**
     * Add an action when the path successfully finishes. This includes early exits
     * but not cancellations.
     */
    public PathBehavior onFinish(Action action) { //have separate isAtEndOfPath() method
        actionLoop.onFinish(action);
        return this;
    }
    
    /**
     * Add an action when the path is canceled, either by an opMode stop, or other conditions like macro
     * cancelling buttons.
     */
    public PathBehavior onCancel(Action action) {
        actionLoop.onCancel(action);
        return this;
    }
    
    /**
     * Add an action to be executed when the path successfully finishing or is canceled.
     */
    public PathBehavior onExit(Action action) {
        actionLoop.onExit(action);
        return this;
    }
    
    public static PathBehavior copyOf(PathBehavior behavior) {
        return behavior.clone();
    }
    
    /**
     * If true, prevents the wheels from reversing direction while decelerating.
     */
    public PathBehavior preventReverseDeceleration(boolean isTrue) {
        this.preventReverseDeceleration = isTrue;
        return this;
    }

    /**
     * If true, makes the robot decelerate without using any power.
     */
    public PathBehavior decelerateWithZeroPower(boolean isTrue) {
         this.decelerateWithZeroPower = isTrue;
         return this;
    }
    
    // TODO graph current velocity vs target velocity
    /**
     * If true, makes the robot decelerate without using any power.
     */
    public PathBehavior decelerateWithoutFeedforward(boolean isTrue) {
        this.decelerateWithZeroPower = isTrue;
        return this;
    }
    
    /**
     * Adds a custom drive power for any point along the path.
     */
    public PathBehavior setDrivePowerModifier(DrivePowerModifier drivePowerModifier) {
        this.drivePowerModifier = drivePowerModifier;
        return this;
    }
    
    /**
     * Makes the robot decelerate at the end of the path for the given seconds.
     * <p>
     * Overrides other methods of setting deceleration like {@link #decelerateForDistance} and
     * {@link #decelerate}.
     */
    public PathBehavior decelerateForSeconds(double seconds) {
        return setDeceleration(TrapezoidalVelocityProfile.Acceleration.toReachInTime(seconds));
    }
    
    /**
     * Makes the robot decelerate at the end of the path for the given distance.
     * <p>
     * Overrides other methods of setting deceleration like {@link #decelerateForSeconds} and
     * {@link #decelerate}.
     */
    public PathBehavior decelerateForDistance(double distance) {
        return setDeceleration(TrapezoidalVelocityProfile.Acceleration.toReachOverDistance(distance));
    }
    
    /**
     * Makes the robot decelerate at the end of the path.
     * <p>
     * A higher number slows down faster. If you want to maximize speed and still have the robot stop use use {@link #maximizeSpeed}
     * <p>
     * This method overrides the previously set motionProfile.
     *
     * @param deceleration (inches/s^2) How fast the robot decelerates at the end of the path.
     *                     Sign should be positive. Average value range 30-120ish.
     * @see #setMaxVelocity
     * @see #setAcceleration
     * @see #setEndingVelocity
     */
    public PathBehavior decelerate(double deceleration) {
        return setDeceleration(TrapezoidalVelocityProfile.Acceleration.constant(deceleration));
    }
    
    private PathBehavior setDeceleration(TrapezoidalVelocityProfile.Acceleration deceleration) {
        stopAtEnd();
        this.deceleration = deceleration;
        rebuildLinearMotionProfile();
        return this;
    }
    
    /** Makes the robot stop at the end of the Path. */
    public PathBehavior stopAtEnd() {
        this.stopAtEnd = true;
        return this;
    }
    /** Makes the robot continue its momentum to the next Path. */
    public PathBehavior continueMomentumAtEnd() {
        this.stopAtEnd = false; // deceleration = null?
        return this;
    }
    
    /** The velocity that the robot considers stopped. Used when the robot is done with the path. */
    public PathBehavior setStoppedVelocityConstraint(double stoppedVelocityConstraint) {
        this.stoppedVelocityConstraint = stoppedVelocityConstraint;
        return this;
    }
    /** The turning velocity that the robot considers stopped. Used to tell when the robot is done
     * with the path. */
    public PathBehavior setStoppedAngularVelocityConstraint(double stoppedAngularVelocityConstraint) {
        this.stoppedAngularVelocityConstraint = stoppedAngularVelocityConstraint;
        return this;
    }
    public PathBehavior setStuckVelocityConstraint(double stuckVelocityConstraint) {
        this.stuckVelocityConstraint = stuckVelocityConstraint;
        return this;
    }
    /**
     * The amount of time that the robot must be below the
     * stoppedVelocityConstraint before it is considered stuck.
     */
    public PathBehavior setStuckTimeoutSeconds(double stuckTimeoutSeconds) {
        this.stuckTimeoutSeconds = stuckTimeoutSeconds;
        return this;
    }
    /**
     * Whether or not to cancel the path if the robot is stuck.
     */
    public PathBehavior setCancelPathIfStuck(boolean cancelPathIfStuck) {
        this.cancelPathIfStuck = cancelPathIfStuck;
        return this;
    }
    /**
     * The maximum amount of seconds the path can take before it is canceled. To disable timeout
     * set to null.
     * */
    public PathBehavior setTimeoutSeconds(Double timeoutSeconds) {
        this.timeoutSeconds = timeoutSeconds;
        return this;
    }
//    /**
//     * Set the target heading along the path.
//     * <p>
//     * Common uses:
//     * <pre><code>
//     * // Tangent (default): The robot will always face forward along the direction of the path.
//     * .setHeadingInterpolator(HeadingInterpolator.tangent)
//     * // Linear: Transitions from 0 to 90 degrees along the path.
//     * .setHeadingInterpolator(HeadingInterpolator.linear(0, 90))
//     * // Custom: PathBehaviorConfig is the percentage along the path (0 to 1).
//     * .setHeadingInterpolator(PathBehaviorConfig -> {
//     *     // whatever you want to do here
//     * })
//     *
//     * // To offset a interpolator, you can use the following:
//     * HeadingInterpolator.tangent.offset(90)
//     * // This will follow the path tangent to the side of the robot (or offset by 90 degrees).
//     * </code></pre>
//     *
//     * See {@link HeadingInterpolator} for more interpolators.
//     */
//    public PathBehavior setHeadingInterpolator(HeadingInterpolator headingInterpolator) {
//        this.headingInterpolator = headingInterpolator;
//        return this;
//    }
//    /**
//     * Transitions from one heading to another along the path.
//     */
//    public PathBehavior setLinearHeadingInterpolation(double startHeading, double endHeading) {
//        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading);
//        return this;
//    }
//    /**
//     * Transitions from one heading to another along the path.
//     */
//    public PathBehavior setLinearHeadingInterpolation(double startHeading, double endHeading, double finishT) {
//        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, finishT);
//        return this;
//    }
//    /**
//     * Makes the robot constantly face the given heading (degrees) along the path.
//     */
//    public PathBehavior setConstantHeading(double heading) {
//        this.headingInterpolator = HeadingInterpolator.constant(heading);
//        return this;
//    }
//
//    /**
//     * The robot will always face forward along the direction of the path.
//     */
//    public PathBehavior setHeadingTangent() {
//        this.headingInterpolator = HeadingInterpolator.tangent;
//        return this;
//    }
    
    public PathBehavior cancelPathIfStuck(boolean enable) {
        this.cancelPathIfStuck = enable;
        return this;
    }
    
    /**
     * Sets the motionProfile function that gives the target velocity the robot should be along the
     * path.
     */
    public PathBehavior setVelocityProfile(VelocityProfile velocityProfile) {
        nullifyLinearMotionProfile();
        this.velocityProfile = velocityProfile;
        return this;
    }
    /**
     * Sets how fast the robot accelerates along the path.
     * A higher number accelerates faster, a lower number accelerates slower.
     * Default is no acceleration.
     *
     * @param acceleration (inches/s^2) How fast the robot accelerates. Sign should be positive.
     *                     Average value range 60-120 inches/s^2.
     *
     * @see #setNoAcceleration
     * @see #setDeceleration
     */
    public PathBehavior accelerate(double acceleration) {
        return setAcceleration(TrapezoidalVelocityProfile.Acceleration.constant(acceleration));
    }
    
    private PathBehavior setAcceleration(@Nullable TrapezoidalVelocityProfile.Acceleration acceleration) {
        this.acceleration = acceleration;
        rebuildLinearMotionProfile();
        return this;
    }
    
    /**
     * No acceleration is the default.
     *
     * @see #setAcceleration
     */
    public PathBehavior setNoAcceleration() {
        return setAcceleration(null);
    }
    
    /**
     * Sets the velocity the robot should be after decelerating. Default is zero.
     * <p>
     * If no deceleration is set, default is 60 inches/s^2.
     *
     * @see #setDeceleration
     */
    public PathBehavior setEndingVelocity(double endingVelocity) {
        if (deceleration == null) {
            // default deceleration if not set later
            this.deceleration = TrapezoidalVelocityProfile.Acceleration.constant(60.0);
        }
        this.endingVelocity = endingVelocity;
        rebuildLinearMotionProfile();
        return this;
    }
    
    /**
     * Sets the distance the robot should cruise at the ending velocity after the robot decelerated.
     * Default is zero.
     *
     * @see #setEndingVelocity
     * @see #setDeceleration
     */
    public PathBehavior setEndingVelocityCruiseDistance(double endingVelocityCruiseDistance) {
        this.endingVelocityCruiseDistance = endingVelocityCruiseDistance;
        rebuildLinearMotionProfile();
        return this;
    }
    
    private void rebuildLinearMotionProfile() {
        velocityProfile = new TrapezoidalVelocityProfile(
            acceleration,
            maxVelocity,
            deceleration,
            endingVelocity,
            endingVelocityCruiseDistance
        );
    }
    
    private void nullifyLinearMotionProfile() {
        this.acceleration = null;
        this.deceleration = null;
        this.maxVelocity = null;
        this.endingVelocity = null;
        this.endingVelocityCruiseDistance = null;
    }
    
    /**
     * Makes the robot travel at full speed along the path. If the path is set to stop at the end,
     * the robot will decelerate at the last possible moment to exactly reach the end point.
     * May shake the robot while braking.
     */
    public PathBehavior maximizeSpeed() {
        nullifyLinearMotionProfile();
        this.velocityProfile = VelocityProfile.maximizeSpeed;
        return this;
    }
    
    public boolean doesStopAtEnd() {
        return stopAtEnd;
    }
    public VelocityProfile getVelocityProfile() {
        return velocityProfile;
    }
    public double getStoppedVelocityConstraint() {
        return stoppedVelocityConstraint;
    }
    public double getStoppedAngularVelocityConstraint() {
        return stoppedAngularVelocityConstraint;
    }
    public double getStuckVelocityConstraint() {
        return stuckVelocityConstraint;
    }
    public double getStuckTimeoutSeconds() {
        return stuckTimeoutSeconds;
    }
    public double getTimeoutSeconds() {
        return timeoutSeconds;
    }
    public boolean doesCancelPathIfStuck() {
        return cancelPathIfStuck;
    }

    /**
     * Combines the two path behavior configs into one.
     */
    public PathBehavior mergeWith(PathBehavior override) {
        PathBehavior merged = new PathBehavior();
        for (Field field : PathBehavior.class.getDeclaredFields()) {
            try {
                field.setAccessible(true);
                Object overrideValue = field.get(override);
                Object thisValue = field.get(this);
                Object chosen = getOrDefault(overrideValue, thisValue);
                Logger.debug("Field: " + field.getName() + "; setting value " + thisValue + " To " +
                    "this value: " + overrideValue);
                field.set(merged, chosen);
            } catch (IllegalAccessException e) {
                throw new RuntimeException("Reflection access error on field " + field.getName(), e);
            }
        }
        return merged;
    }
    
    public PathBehavior withDefaults(PathBehavior defaultConfig) {
        PathBehavior merged = new PathBehavior();
        for (Field field : PathBehavior.class.getDeclaredFields()) {
            try {
                field.setAccessible(true);
                Object defaultValue = field.get(defaultConfig);
                Object thisValue = field.get(this);
                Object chosen = getOrDefault(thisValue, defaultValue);
                field.set(merged, chosen);
            } catch (IllegalAccessException e) {
                throw new RuntimeException("Reflection access error on field " + field.getName(), e);
            }
        }
        return merged;
    }
    
    /**
     * Creates a new {@code PathBehaviorConfig} with the same behavior as the given path.
     */
    public static PathBehavior copyBehaviorFrom(Path path) {
        ImmutablePathBehavior behavior = path.behavior;
        PathBehavior config = new PathBehavior();
        try {
            for (Field field : PathBehavior.class.getDeclaredFields()) {
                field.setAccessible(true);
                Object value = field.get(behavior);
                field.set(config, value);
            }
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Reflection error while copying behavior", e);
        }
        return config;
    }
    
    @NonNull
    @Override
    public PathBehavior clone() {
        try {
            return (PathBehavior) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError("PathBehaviorConfig clone failed unexpectedly", e);
        }
    }
}
