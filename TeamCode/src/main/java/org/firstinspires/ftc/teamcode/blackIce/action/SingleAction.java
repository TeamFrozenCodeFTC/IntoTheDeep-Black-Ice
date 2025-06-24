package org.firstinspires.ftc.teamcode.blackIce.action;

import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Condition;
import org.firstinspires.ftc.teamcode.blackIce.action.lambda.Action;

/**
 * An action that runs only once after a condition is true.
 *
 * <pre><code>
 * path.addAction(new SingleAction(
 *      () -> slide.isRaised()
 *      () -> telemetry.addData("The slide has finished raising!")
 * ));
 * </code></pre>
 */
public class SingleAction extends TriggeredAction {
    private boolean hasRan = false;

    public SingleAction(Condition triggerCondition, Action action) {
        super(action, triggerCondition);
    }

    public boolean hasRan() {
        return hasRan;
    }

    @Override
    public void update() {
        if (isTriggered() && !hasRan()) {
            action.execute();
            hasRan = true;
        }
    }

    @Override
    public void reset() {
        hasRan = false;
    }
}
