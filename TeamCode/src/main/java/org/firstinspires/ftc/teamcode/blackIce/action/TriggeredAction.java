package org.firstinspires.ftc.teamcode.blackIce.action;

import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Condition;
import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Action;

/**
 * An action that only runs every time a condition is triggered to be true.
 *
 * <pre><code>
 * path.addAction(new SingleAction(
 *      () -> slide.isRaised()
 *      () -> telemetry.addLine("The slide has finished raising!")
 * ));
 * </code></pre>
 */
public class TriggeredAction implements org.firstinspires.ftc.teamcode.blackIce.action.Action {
    protected final Action action;
    protected final Condition triggerCondition;

    public TriggeredAction(Action action, Condition triggerCondition) {
        this.action = action;
        this.triggerCondition = triggerCondition;
    }

    @Override
    public void update() {
        if (isTriggered()) {
            action.execute();
        }
    }

    public boolean isTriggered() {
        return this.triggerCondition.isTrue();
    }
}
