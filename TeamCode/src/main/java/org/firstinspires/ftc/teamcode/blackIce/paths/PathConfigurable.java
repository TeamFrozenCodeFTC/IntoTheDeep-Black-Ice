//package org.firstinspires.ftc.teamcode.blackIce.paths;
//
//import org.firstinspires.ftc.teamcode.blackIce.action.Action;
//import org.firstinspires.ftc.teamcode.blackIce.action.ActionLoop;
//import org.firstinspires.ftc.teamcode.blackIce.action.TriggeredAction;
//import org.firstinspires.ftc.teamcode.blackIce.action.Condition;
//import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Executable;
//
//import java.util.List;
//import java.util.function.Consumer;
//
//public class PathConfigurable
//    <Subclass extends PathConfigurable<Subclass>>
//{
//    protected PathParameters parameters;
//
//    public PathConfigurable(PathParameters parameters) {
//        this.parameters = parameters;
//    }
//
//    public PathConfigurable(PathConfigurable<?> configurable) {
//        this.parameters = configurable.copyParameters();
//    }
//
//    public PathConfigurable() {
//        this.parameters = new PathParameters();
//    }
//
//    public PathConfigurable<?> copyConfig() {
//        return new PathConfigurable<>(copyParameters(), copyLoop());
//    }
//
//    @SuppressWarnings("unchecked")
//    public Subclass getThis() {
//        return (Subclass) this;
//    }
//
//    public ActionLoop copyLoop() {
//        return actionLoop.copy();
//    }
//
//    public PathParameters copyParameters() {
//        return parameters.copy();
//    }
//
//    protected Subclass setParameter(Consumer<PathParameters> setter) {
//        setter.accept(this.parameters);
//        return getThis();
//    }
//    /** Make the robot stop at the end of the Path. */
//    public Subclass setToStopAtEnd() {
//        return setParameter((p) -> p.stopAtEnd = true);
//    }
//    /** Make the robot continue it's momentum to the next Path. */
//    public Subclass setToContinueAtEnd() {
//        return setParameter((p) -> p.stopAtEnd = false);
//    }
//    public Subclass setTargetVelocity(double targetVelocity) {
//        return setParameter((p) -> p.targetVelocity = targetVelocity);
//    }
//    public Subclass setTargetAcceleration(double targetAcceleration) {
//        return setParameter((p) -> p.targetAcceleration = targetAcceleration);
//    }
//    /** The max turning velocity of the robot. (degrees/second) */
//    public Subclass setTargetAngularVelocity(double targetAngularVelocity) {
//        return setParameter((p) -> p.targetAngularVelocity = targetAngularVelocity);
//    }
//    /** The velocity that the robot considers stopped. Used when the robot is done with the path. */
//    public Subclass setStoppedVelocityConstraint(double stoppedVelocityConstraint) {
//        return setParameter(
//            (p) -> p.stoppedVelocityConstraint = stoppedVelocityConstraint);
//    }
//    /**
//     * The velocity that the robot considers itself to be stuck and not moving.
//     * Triggers that the robot is stuck if the robot is below this velocity for longer than the
//     * zeroVelocityTimeoutSeconds.
//     */
//    public Subclass setStuckVelocityConstraint(double stuckVelocityConstraint) {
//        return setParameter((p) -> p.stuckVelocityConstraint = stuckVelocityConstraint);
//    }
//    /**
//     * The amount of time that the robot must be below the
//     * stoppedVelocityConstraint before it is considered stuck.
//     */
//    public Subclass setStuckTimeoutSeconds(double stuckTimeoutSeconds) {
//        return setParameter((p) -> p.stuckTimeoutSeconds = stuckTimeoutSeconds);
//    }
//    /**
//     * Whether or not to cancel the path if the robot is stuck.
//     */
//    public Subclass setCancelPathIfStuck(boolean cancelPathIfStuck) {
//        return setParameter((p) -> p.cancelPathIfStuck = cancelPathIfStuck);
//    }
//    /**
//     * Set the target heading along the path.
//     * <p>
//     * Common uses:
//     * <pre><code>
//     * // Tangent (default): The robot will always face forward along the direction of the path.
//     * .setHeadingInterpolator(HeadingInterpolator.tangent)
//     * // Linear: Transitions from 0 to 90 degrees along the path.
//     * .setHeadingInterpolator(HeadingInterpolator.linear(0, 90))
//     * // Custom: t is the percentage along the path (0 to 1).
//     * .setHeadingInterpolator(t -> {
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
//    public Subclass setHeadingInterpolation(HeadingInterpolator headingInterpolator) {
//        return setParameter((p) -> p.headingInterpolator = headingInterpolator);
//    }
//    public Subclass setLinearHeadingInterpolation(double startHeading, double endHeading) {
//        return setParameter((p) -> p.headingInterpolator
//            = HeadingInterpolator.linear(startHeading, endHeading));
//    }
//    public Subclass setConstantHeadingInterpolation(double degrees) {
//        return setParameter((p) -> p.headingInterpolator
//            = HeadingInterpolator.constant(degrees));
//    }
//
//    public Subclass setStuckDetection(boolean enable) {
//        return setParameter((p) -> p.cancelPathIfStuck = enable);
//    }
////
////    protected Subclass setAction(Consumer<ActionLoop> setter) {
////        setter.accept(this.actionLoop);
////        return getThis();
////    }
////
////    /**
////     * Cancel the actions after the action is triggered.
////     */
////    public Subclass addCancellationAction(TriggeredAction action) {
////        return setAction((loop) -> loop.addCancellationAction(action));
////    }
////
////    public Subclass addAction(Action action) {
////        return this
////            .withLoop(action::update)
////            .withInitialize(action::reset);
////    }
////
////    public Subclass addActions(List<Action> actions) {
////        for (Action action : actions) {
////            addAction(action);
////        }
////        return getThis();
////    }
////    /**
////     * Add a condition that will cancel the loop if it becomes true.
////     */
////    public Subclass withCancelCondition(Condition cancelCondition) {
////        return setAction((loop) -> loop.withCancelCondition(cancelCondition));
////    }
////
////    /**
////     * Adds an method that will be called every loop.
////     */
////    public Subclass withLoop(Executable onLoop) {
////        return setAction((loop) -> loop.withLoop(onLoop));
////    }
////    public Subclass withInitialize(Executable onInitialize) {
////        setAction((loop) -> loop.withInitialize(onInitialize));
////        return getThis();
////    }
////
////    /**
////     * Add a condition that will continue the loop until it becomes true.
////     */
////    public Subclass withCondition(Condition condition) {
////        setAction((loop) -> loop.withCondition(condition));
////        return getThis();
////    }
////    public void initialize() {
////        actionLoop.initialize();
////    }
////    public void update() {
////        actionLoop.loop();
////    }
////    public boolean isFinished() {
////        return actionLoop.isFinished();
////    }
////    public void cancel() {
////        actionLoop.cancel();
////    }
////    public boolean isCanceled() {
////        return actionLoop.isCanceled();
////    }
////    public boolean isRunning() {
////        return actionLoop.isRunning();
////    }
//
//    public double getTargetVelocity() { return parameters.targetVelocity; }
//    public double getTargetAcceleration() { return parameters.targetAcceleration; }
//    public double getTargetAngularVelocity() { return parameters.targetAngularVelocity; }
//    public double getStoppedVelocityConstraint() { return parameters.stoppedVelocityConstraint; }
//    public double getStuckTimeoutSeconds() { return parameters.stuckTimeoutSeconds; }
//    public boolean doesStopAtEnd() { return parameters.stopAtEnd; }
//    public HeadingInterpolator getHeadingInterpolator() { return parameters.headingInterpolator; }
//    public double getStuckVelocityConstraint() { return parameters.stuckVelocityConstraint; }
//    public boolean cancelPathIfStuck() { return parameters.cancelPathIfStuck; }
//}
