//package org.firstinspires.ftc.teamcode.blackIce.paths;
//
//import org.firstinspires.ftc.teamcode.blackIce. action.ActionLoop;
//
//import java.util.function.Consumer;
//
///**
// * The configuration for a path. This is used to set the parameters for the path.
// * <p>
// * This class is used to set the parameters for the path. It is used to set the target velocity,
// * target acceleration, target angular velocity, and other parameters.
// * <p>
// * This class is used to set the parameters for the path. It is used to set the target velocity,
// * target acceleration, target angular velocity, and other parameters.
// *
// * @apiNote Sorry for all the generics. It was the only way I could figure out how to not repeat
// * all the code and documentation between PathParameters and ActionLoop inside PathConfig
// * without having multiple inheritance.
// * My solution was to just have PathConfig have the PathParameters logic
// * and have it extend ActionLoop.
// * I know composition is better than whatever this is,
// * but I didn't want to repeat all the code and documentation and have zero expandability.
// * One alternative would be to to change the way the user interacts with the path and have like
// * a call and response to update the parameters.
// * For example: path.editParameters(parameters -> {...}) or
// * path.editParameters().setTargetVelocity(60).toPath()
// * (this would require parameters to store the path tho)
// * But it isn't as fluent and easy to use.
// */
//public class PathConfig<Subclass extends ActionLoop<Subclass>> extends ActionLoop<Subclass> {
//    boolean stopAtEnd = false;
//    private double targetVelocity = 60;
//    private double targetAcceleration = 60;
//    private double targetAngularVelocity = 60;
//    private double stoppedVelocityConstraint = 1.0;
//    private double stuckVelocityConstraint = 1.0;
//    private double stuckTimeoutSeconds = 5.0;
//
//    private HeadingInterpolator headingInterpolator = HeadingInterpolator.tangent;
//
//    @SuppressWarnings("unchecked")
//    public Subclass getThis() {
//        return (Subclass) this;
//    }
//
//    public Subclass copyConfigFrom(PathConfig<?> other) {
//        this.stopAtEnd = other.stopAtEnd;
//        this.targetVelocity = other.targetVelocity;
//        this.targetAcceleration = other.targetAcceleration;
//        this.targetAngularVelocity = other.targetAngularVelocity;
//        this.stoppedVelocityConstraint = other.stoppedVelocityConstraint;
//        this.stuckTimeoutSeconds = other.stuckTimeoutSeconds;
//        this.headingInterpolator = other.headingInterpolator;
//        return getThis();
//    }
//
//    protected Subclass set(Consumer<PathConfig<?>> setter) {
//        setter.accept(this);
//        return getThis();
//    }
//    /** Make the robot stop at the end of the Path. */
//    public Subclass setToStopAtEnd() {
//        return set((config) -> config.stopAtEnd = true);
//    }
//    /** Make the robot continue it's momentum to the next Path. */
//    public Subclass setToContinueAtEnd() {
//        return set((config) -> config.stopAtEnd = false);
//    }
//    public Subclass setTargetVelocity(double targetVelocity) {
//        return set((config) -> config.targetVelocity = targetVelocity);
//    }
//    public Subclass setTargetAcceleration(double targetAcceleration) {
//        return set((config) -> config.targetAcceleration = targetAcceleration);
//    }
//    /** The max turning velocity of the robot. (degrees/second) */
//    public Subclass setTargetAngularVelocity(double targetAngularVelocity) {
//        return set((config) -> config.targetAngularVelocity = targetAngularVelocity);
//    }
//    /** The velocity that the robot considers stopped. Used when the robot is done with the path. */
//    public Subclass setStoppedVelocityConstraint(double stoppedVelocityConstraint) {
//        return set((config) -> config.stoppedVelocityConstraint = stoppedVelocityConstraint);
//    }
//    /**
//     * The velocity that the robot considers itself to be stuck and not moving.
//     * Triggers that the robot is stuck if the robot is below this velocity for longer than the
//     * zeroVelocityTimeoutSeconds
//     */
//    public Subclass setStuckVelocityConstraint(double stuckVelocityConstraint) {
//        return set((config) -> config.stuckVelocityConstraint = stuckVelocityConstraint);
//    }
//    /**
//     * The amount of time that the robot must be below the
//     * stoppedVelocityConstraint before it is considered stuck.
//     */
//    public Subclass setStuckTimeoutSeconds(double stuckTimeoutSeconds) {
//        return set((config) -> config.stuckTimeoutSeconds = stuckTimeoutSeconds);
//    }
//    public Subclass setHeadingInterpolation(HeadingInterpolator headingInterpolator) {
//        return set((config) -> config.headingInterpolator = headingInterpolator);
//    }
//
//    public double getTargetVelocity() { return targetVelocity; }
//    public double getTargetAcceleration() { return targetAcceleration; }
//    public double getTargetAngularVelocity() { return targetAngularVelocity; }
//    public double getStoppedVelocityConstraint() { return stoppedVelocityConstraint; }
//    public double getStuckTimeoutSeconds() { return stuckTimeoutSeconds; }
//    public boolean doesStopAtEnd() { return stopAtEnd; }
//    public HeadingInterpolator getHeadingInterpolator() { return headingInterpolator; }
//    public double getStuckVelocityConstraint() { return stuckVelocityConstraint; }
//}