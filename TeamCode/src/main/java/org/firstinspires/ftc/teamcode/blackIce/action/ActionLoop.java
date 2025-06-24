//package org.firstinspires.ftc.teamcode.blackIce.action;
//
//import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Condition;
//import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Action;
//
//import java.util.Arrays;
//import java.util.List;
//
///**
// * A loop that contains a set of actions.
// *
// * <pre><code>
// * initialize()
// * while (!isFinished() && !isCanceled()) {
// *     loop();
// * }
// * </code></pre>
// */
//public class ActionLoop implements Loop {
//    protected Action onInitialize = Action.EMPTY;
//    protected Action onLoop = Action.EMPTY;
//    protected Condition finishCondition = Condition.NEVER;
//    protected Condition cancelCondition = Condition.NEVER; // !opModeIsActive()
//    protected boolean isCanceled = false;
//
//    public ActionLoop(org.firstinspires.ftc.teamcode.blackIce.action.Action... actions) {
//        addActions(Arrays.asList(actions));
//    }
//
//    public ActionLoop copy() {
//        return new ActionLoop()
//            .withActionLoopFrom(this);
//    }
//
//    /**
//     * Combines the actions of this loop with another loop.
//     */
//    public ActionLoop withActionLoopFrom(ActionLoop loop) {
//        return this
//            .withLoop(loop.onLoop)
//            .withInitialize(loop.onInitialize)
//            .withCondition(loop.finishCondition)
//            .withCancelCondition(loop.cancelCondition);
//    }
//
//    /**
//     * Cancel the actions after the action is triggered.
//     */
//    public void addCancellationAction(TriggeredAction action) {
//        this.cancelCondition = this.cancelCondition.or(action.triggerCondition);
//        this.addAction(new TriggeredAction(
//            action.action,
//            action.triggerCondition
//        ));
//    }
//
//    public ActionLoop addAction(org.firstinspires.ftc.teamcode.blackIce.action.Action action) {
//        return this
//            .withLoop(action::update)
//            .withInitialize(action::reset);
//    }
//    public ActionLoop addActions(List<org.firstinspires.ftc.teamcode.blackIce.action.Action> actions) {
//        for (org.firstinspires.ftc.teamcode.blackIce.action.Action action : actions) {
//            addAction(action);
//        }
//        return this;
//    }
//
//    public ActionLoop withCancelCondition(Condition cancelCondition) {
//        this.cancelCondition = this.cancelCondition.or(cancelCondition);
//        return this;
//    }
//    public ActionLoop withLoop(Action onLoop) {
//        this.onLoop = this.onLoop.andThen(onLoop);
//        return this;
//    }
//    public ActionLoop withInitialize(Action onInitialize) {
//        this.onInitialize = this.onInitialize.andThen(onInitialize);
//        return this;
//    }
//    public ActionLoop withCondition(Condition condition) {
//        this.finishCondition = this.finishCondition.and(condition);
//        return this;
//    }
//    public ActionLoop withTrigger(Condition trigger, Action action) {
//        return this.addAction(new TriggeredAction(action, trigger));
//    }
//
//    @Override
//    public void initialize() {
//        onInitialize.execute();
//    }
//
//    @Override
//    public void loop() {
//        onLoop.execute();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return finishCondition.isTrue();
//    }
//
//    @Override
//    public boolean isCanceled() {
//        return cancelCondition.isTrue() || isCanceled;
//    }
//
//    @Override
//    public void cancel() {
//        this.isCanceled = true;
//    }
//}
