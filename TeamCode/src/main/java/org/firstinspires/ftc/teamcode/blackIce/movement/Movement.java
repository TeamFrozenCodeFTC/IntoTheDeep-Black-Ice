package org.firstinspires.ftc.teamcode.blackIce.movement;

import org.firstinspires.ftc.teamcode.blackIce.Condition;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;

/**
 * Returned by calling `.build()` on a {@link MovementBuild} or {@link Path}
 * Only Handles abstract looping and updating, no core logic.
 */
public abstract class Movement {
    // HACK: This class is abstract and uses anonymous classes
    // to avoid boilerplate of {@code Runnable.run()}

    /**
     * Gets called once in every loop.
     */
    public static Runnable globalLoopMethod = () -> {};

    /**
     * Used for stopping macros in tele-op.
     * Usually a gamepad button like {@code () -> gamepad1.x}
     * When true, the movement will cancel.
     */
    public static Condition macroCancelCondition = () ->
        Follower.opMode.gamepad1.left_stick_y != 0
            || Follower.opMode.gamepad1.right_stick_y != 0;
    // only y sticks, for performance?
    // if want only when >20% instead of != 0% have to use absolute value

    private Condition waitUntilCondition = () -> true;
    private Runnable loopMethod = () -> {};

    /**
     * Wait for the Movement to be completed.
     */
    public void waitForMovement() {
        start();

        while ((!isFinished() || !waitUntilCondition.condition()) && !macroCancelCondition.condition()) {
            update();
            globalLoopMethod.run();
            loopMethod.run();
        }

        finish();
    }

    /**
     * Wait for the conditionMethod to be true and Movement to be completed.
     * Holds the end position until the condition is true.
     *
     * @param holdEndUntilTrue Holds the end position until the condition is true.
     * <pre><code>
     * .waitUntil(() -> slideIsRaised())
     * // Will hold the end position until the slide is raised.
     * </code></pre>
     * <p>
     * If no end-holding condition is needed see {@link Movement#waitForMovement()}
     */
    public void waitUntil(Condition holdEndUntilTrue) {
        waitUntilCondition = holdEndUntilTrue;
        waitForMovement();
    }

    /**
     * Add a loop method to the movement.
     *
     * @param newLoopMethod A function that gets called every loop. This is useful for things like
     *                       when a linear slide reaches a certain height, a claw is suppose to open.
     * <p>
     * If a loop method is not needed, {@link Movement#waitForMovement()} will simply start the Movement
     * normally.
     */
    public void withLoop(Runnable newLoopMethod) {
        loopMethod = newLoopMethod;
        waitForMovement();
    }

    /**
     * Whether the movement is finished or not. Includes {@code opModeIsActive()} check.
     */
    public abstract boolean isFinished();

    /**
     * Start the movement. Necessary inorder for {@link Movement#update} to work properly.
     */
    public abstract Movement start();

    /**
     * Finish the movement. Ends wheel powers.
     */
    public abstract void finish();

    /**
     * Update the robot's position and move the robot towards its target.
     * This can also be called for holding positions.
     */
    public abstract void update();
}
