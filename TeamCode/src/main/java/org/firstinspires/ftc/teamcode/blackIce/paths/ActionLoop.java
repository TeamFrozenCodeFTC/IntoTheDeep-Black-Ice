package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Condition;
import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Action;

/**
 * Note important to exit inside loop
 *     loop.start();
 * while (opModeIsActive() && loop.update()) {
 *     // looks like a loop, but state is updated first
 * }
 * An loop that runs until .finish() or .cancel() is called.
 *
 */
public class ActionLoop {
    private Action onStart = Action.EMPTY;
    private Action onLoop = Action.EMPTY;
    private Action onFinish = Action.EMPTY;
    private Action onCancel = Action.EMPTY;
    
    private Condition canFinishWhen = Condition.ALWAYS;
    private Condition cancelWhen = Condition.NEVER;
    private Condition earlyExitWhen = Condition.NEVER;
    
    private boolean hasCanceled = false;
    private boolean hasFinished = false;
    private boolean isPaused = false;
    
    public void pause() {
        isPaused = true;
    }
    public void resume() {
        isPaused = false;
    }
    
    public boolean isPaused() {
        return isPaused;
    }

    public void start() {
        hasCanceled = false;
        hasFinished = false;
        onStart.execute();
    }
    public void loop() {
        if (isPaused()) return;
        onLoop.execute();
        if (hasCanceled()) cancel();
        if (hasEarlyExited()) finish();
    }
    public void finish() {
        hasFinished = true;
        onFinish.execute();
    }
    public void cancel() {
        hasCanceled = true;
        onCancel.execute();
    }
    
    public void onStart(Action action) { onStart = onStart.andThen(action); }
    public void onLoop(Action action) {
        onLoop = onLoop.andThen(action);
    }
    public void onFinish(Action action) { onFinish = onFinish.andThen(action); }
    public void onCancel(Action action) { onCancel = onCancel.andThen(action); }
    public void doWhen(Condition condition, Action action) {
        onLoop(() -> action.executeWhen(condition.isTrue()));
    }
    public void onExit(Action action) {
        onFinish(action);
        onCancel(action);
    }
    
    public void cancelWhen(Condition condition) {
        cancelWhen = cancelWhen.or(condition);
    }
    public void cancelWhen(Condition condition, Action action) {
        cancelWhen(condition);
        doWhen(condition, action);
    }
    public void earlyExitWhen(Condition condition) {
        earlyExitWhen = earlyExitWhen.or(condition);
    }
    public void earlyExitWhen(Condition condition, Action action) {
        earlyExitWhen(condition);
        doWhen(condition, action);
    }
    public void canFinishWhen(Condition condition) { canFinishWhen = canFinishWhen.and(condition); }
    
    public boolean hasCanceled() { return hasCanceled; }
    public boolean hasFinished() { return hasFinished; }

    public boolean canFinish() {
        return canFinishWhen.isTrue();
    }
    public boolean hasEarlyExited() {
        return earlyExitWhen.isTrue() && canFinish();
    }
    
    public ActionLoop combineWith(ActionLoop other) {
        ActionLoop combined = new ActionLoop();
        
        combined.onStart = this.onStart.andThen(other.onStart);
        combined.onLoop = this.onLoop.andThen(other.onLoop);
        combined.onFinish = this.onFinish.andThen(other.onFinish);
        combined.onCancel = this.onCancel.andThen(other.onCancel);
        
        combined.canFinishWhen = this.canFinishWhen.and(other.canFinishWhen);
        combined.cancelWhen = this.cancelWhen.or(other.cancelWhen);
        combined.earlyExitWhen = this.earlyExitWhen.or(other.earlyExitWhen);
        
        return combined;
    }
    
    /**
     * The loop has been canceled or it has finished either by calling finish() (ignores
     * canFinish), earlyExiting
     */
    public boolean isRunning() {
        return !hasCanceled() && !hasFinished();
    }
    
    public boolean update() {
        loop();
        return isRunning();
    }

    public void run(Condition finishCondition) {
        start();
        while (update()) {
            if (finishCondition.isTrue() && canFinish()) {
                finish();
                break;
            }
        }

    }

//
//    /**
//     * Cancel the path when the condition is true, and execute the action.
//     * path.(gamepad1::x, gamepad1::rumble); // rumble the controller when X is pressed
//     */
//    public Path cancelWhen(Condition condition, Action action) {
//        return this
//            .cancelWhen(condition)
//            .doWhen(condition, action);
//    }
//    // for follower. cancels all current paths and does not run any other paths until the condition is false.
//
//    /**
//     * Finish the path early when the condition is true.
//     * path.earlyFinishWhen(pos.x > 60);
//     */
//    public Path earlyFinishWhen(Condition condition) {
//        earlyFinishWhen = earlyFinishWhen.or(condition);
//        return this;
//    }
//    // not for follower, follower has exitAtPositionBoundary()
//
//    /**
//     * Finish the path early when the condition is true and execute an action.
//     * <pre><code>
//     * path.earlyFinishWhen(pos.x > 60, () -> telemetry.addLine("early finish"));
//     * </code></pre>
//     */
//    public Path earlyFinishWhen(Condition condition, Action action) {
//        return this
//            .earlyFinishWhen(condition)
//            .doWhen(condition, action);
//    }
//    // not for follower, follower has exitAtPositionBoundary()
//
//    /**
//     * Add an action to be executed when the path starts.
//     */
//    public Path onStart(Action path) {
//        onStart = onStart.andThen(path);
//        return this;
//    }
//    // for follower, every time a path is started, it will run the action. but maybe rename onFollowStart()
//
//    /**
//     * Add an action to be executed when the path successfully finishes. This includes early exits
//     * but not cancellations.
//     */
//    public Path onFinish(Action action) { // onCancel?
//        onFinish = onFinish.andThen(action);
//        return this;
//    }
//    // for follower, every time a path is finished, it will run the action. but maybe rename
//    // onFollowFinish()
//
//    /**
//     * When the path is canceled, either by an opMode stop, or other conditions like macro
//     * cancelling buttons.
//     */
//    public Path onCancel(Action action) {
//        onCancel = onCancel.andThen(action);
//        return this;
//    }
//    // for follower, every time a path is canceled, it will run the action. but maybe rename
//    // onFollowCancel()
//
//    /**
//     * When the path is exited, either by successfully finishing or canceling.
//     */
//    public Path onExit(Action action) {
//        return this
//            .onFinish(action)
//            .onCancel(action);
//    }
}
