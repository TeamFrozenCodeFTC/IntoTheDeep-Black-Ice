package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.MotionProfile;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.LinearMotionProfile;

import java.util.function.Consumer;

/**
 * The path configuration that allows you to set various path parameters and constraints fluently
 * to any subclass.
 *
 * @param <T> the subclass type that extends this configurator. Used for fluent API style which
 *           allows chaining methods.
 *
 * @implNote Mutable for dynamic path behavior.
 */
public class PathConfig<T extends PathConfig<T>> {
    private boolean stopAtEnd = false;

    private Double maxVelocity = null;
    private Double endingVelocity = null;
    private Double acceleration = null;
    private Double deceleration = null;
    private Double endingVelocityCruiseDistance = null;
    private MotionProfile motionProfile;
    
    private double stoppedVelocityConstraint = 1.0;
    private double stoppedAngularVelocityConstraint = 1.0;

    private double stuckVelocityConstraint = 1.0;
    private double stuckTimeoutSeconds = 5.0;
    private boolean cancelPathIfStuck = true;

    private Double timeoutSeconds = 8.0;

    private HeadingInterpolator headingInterpolator = HeadingInterpolator.tangent;
    
    public PathConfig(PathConfig<?> copyFrom) {
        this.stopAtEnd = copyFrom.stopAtEnd;
        this.maxVelocity = copyFrom.maxVelocity;
        this.acceleration = copyFrom.acceleration;
        this.stoppedVelocityConstraint = copyFrom.stoppedVelocityConstraint;
        this.stuckVelocityConstraint = copyFrom.stuckVelocityConstraint;
        this.stuckTimeoutSeconds = copyFrom.stuckTimeoutSeconds;
        this.cancelPathIfStuck = copyFrom.cancelPathIfStuck;
        this.stoppedAngularVelocityConstraint = copyFrom.stoppedAngularVelocityConstraint;
        this.headingInterpolator = copyFrom.headingInterpolator;
        this.timeoutSeconds = copyFrom.timeoutSeconds;
        this.deceleration = copyFrom.deceleration;
    }
    
    /** Default constructor for creating a new PathConfig with default values. */
    public PathConfig() {}
    
    protected T setConstraint(Consumer<PathConfig<?>> setter) {
        setter.accept(this);
        return getThis();
    }
    /** Makes the robot stop at the end of the Path. */
    public T stopAtEnd() {
        return setConstraint((p) -> p.stopAtEnd = true);
    }
    /** Makes the robot continue its momentum to the next Path. */
    public T continueMomentumAtEnd() {
        return setConstraint((p) -> p.stopAtEnd = false);
    }
    /** The velocity that the robot considers stopped. Used when the robot is done with the path. */
    public T setStoppedVelocityConstraint(double stoppedVelocityConstraint) {
        return setConstraint(
            (p) -> p.stoppedVelocityConstraint = stoppedVelocityConstraint);
    }
    /** The turning velocity that the robot considers stopped. Used to tell when the robot is done
     * with the path. */
    public T setStoppedAngularVelocityConstraint(double stoppedAngularVelocityConstraint) {
        return setConstraint(
            (p) -> p.stoppedAngularVelocityConstraint = stoppedAngularVelocityConstraint);
    }
    public T setStuckVelocityConstraint(double stuckVelocityConstraint) {
        this.stuckVelocityConstraint = stuckVelocityConstraint;
        return getThis();
    }
    /**
     * The amount of time that the robot must be below the
     * stoppedVelocityConstraint before it is considered stuck.
     */
    public T setStuckTimeoutSeconds(double stuckTimeoutSeconds) {
        return setConstraint((p) -> p.stuckTimeoutSeconds = stuckTimeoutSeconds);
    }
    /**
     * Whether or not to cancel the path if the robot is stuck.
     */
    public T setCancelPathIfStuck(boolean cancelPathIfStuck) {
        return setConstraint((p) -> p.cancelPathIfStuck = cancelPathIfStuck);
    }
    /**
     * The maximum amount of seconds the path can take before it is canceled. To disable timeout
     * set to null.
     * */
    public T setTimeoutSeconds(Double timeoutSeconds) {
        return setConstraint((p) -> p.timeoutSeconds = timeoutSeconds);
    }
    /**
     * Set the target heading along the path.
     * <p>
     * Common uses:
     * <pre><code>
     * // Tangent (default): The robot will always face forward along the direction of the path.
     * .setHeadingInterpolator(HeadingInterpolator.tangent)
     * // Linear: Transitions from 0 to 90 degrees along the path.
     * .setHeadingInterpolator(HeadingInterpolator.linear(0, 90))
     * // Custom: t is the percentage along the path (0 to 1).
     * .setHeadingInterpolator(t -> {
     *     // whatever you want to do here
     * })
     *
     * // To offset a interpolator, you can use the following:
     * HeadingInterpolator.tangent.offset(90)
     * // This will follow the path tangent to the side of the robot (or offset by 90 degrees).
     * </code></pre>
     *
     * See {@link HeadingInterpolator} for more interpolators.
     */
    public T setHeadingInterpolation(HeadingInterpolator headingInterpolator) {
        return setConstraint((p) -> p.headingInterpolator = headingInterpolator);
    }
    /**
     * Transitions from one heading to another along the path.
     */
    public T setLinearHeadingInterpolation(double startHeading, double endHeading) {
        return setConstraint((p) -> p.headingInterpolator
            = HeadingInterpolator.linear(startHeading, endHeading));
    }
    /**
     * Transitions from one heading to another along the path.
     */
    public T setLinearHeadingInterpolation(double startHeading, double endHeading, double finishT) {
        return setConstraint((p) -> p.headingInterpolator
            = HeadingInterpolator.linear(startHeading, endHeading, finishT));
    }
    /**
     * Makes the robot constantly face the given heading (degrees) along the path.
     */
    public T setConstantHeading(double heading) {
        return setConstraint(p -> p.headingInterpolator
            = HeadingInterpolator.constant(heading));
    }
    
    public T cancelPathIfStuck(boolean enable) {
        return setConstraint(p -> p.cancelPathIfStuck = enable);
    }
    /**
     * Makes the robot decelerate at the end of the path.
     * <p>
     * A higher number slows down faster. If you want to maximize speed and still have the robot stop use use {@link #maximizeSpeed}
     * <p>
     * This method overrides the previously set motionProfile.
     *
     * @param deceleration (inches/s^2) How fast the robot decelerates at the end of the path.
     *                     Sign should be negative. Average value range 30-120ish.
     * @see #setMaxVelocity
     * @see #setAcceleration
     * @see #setEndingVelocity
     */
    public T setDeceleration(double deceleration) {
        stopAtEnd();
        this.deceleration = deceleration;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    public T setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    /**
     * Sets the motionProfile function that gives the target velocity the robot should be along the
     * path.
     */
    public T setMotionProfile(MotionProfile motionProfile) {
        nullifyLinearMotionProfile();
        this.motionProfile = motionProfile;
        return getThis();
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
    public T setAcceleration(double acceleration) {
        this.acceleration = acceleration;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    /**
     * No acceleration is the default.
     *
     * @see #setAcceleration
     */
    public T setNoAcceleration() {
        this.acceleration = null;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    /**
     * Sets the velocity the robot should be after decelerating. Default is zero.
     * <p>
     * If no deceleration is set, default is 60 inches/s^2.
     *
     * @see #setDeceleration
     */
    public T setEndingVelocity(double endingVelocity) {
        if (deceleration == null) {
            // default deceleration if not set later
            this.deceleration = 60.0;
        }
        this.endingVelocity = endingVelocity;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    /**
     * Sets the distance the robot should cruise at the ending velocity after the robot decelerated.
     * Default is zero.
     *
     * @see #setEndingVelocity
     * @see #setDeceleration
     */
    public T setEndingVelocityCruiseDistance(double endingVelocityCruiseDistance) {
        this.endingVelocityCruiseDistance = endingVelocityCruiseDistance;
        rebuildLinearMotionProfile();
        return getThis();
    }
    
    private void rebuildLinearMotionProfile() {
        motionProfile = new LinearMotionProfile(
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
    public T maximizeSpeed() {
        nullifyLinearMotionProfile();
        this.motionProfile = MotionProfile.maximizeSpeed;
        return getThis();
    }
    
    public boolean doesStopAtEnd() {
        return stopAtEnd;
    }
    public MotionProfile getMotionProfile() {
        return motionProfile;
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
    public HeadingInterpolator getHeadingInterpolator() {
        return headingInterpolator;
    }
    
    @SuppressWarnings("unchecked")
    protected T getThis() {
        return (T) this;
    }
}
