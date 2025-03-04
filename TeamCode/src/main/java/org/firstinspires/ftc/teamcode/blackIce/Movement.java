package org.firstinspires.ftc.teamcode.blackIce;

public class Movement {
    private final Condition isFinished;
    private final Runnable start;
    private final Runnable finish;
    private final Runnable update;

    public Movement(Condition isFinished, Runnable start, Runnable finish, Runnable update) {
        this.isFinished = isFinished;
        this.start = start;
        this.finish = finish;
        this.update = update;
    }

    /**
     * Wait for the Movement to be completed with an extra condition and method that gets called
     * every loop.
     *
     * @param extraCondition An extra {@link Condition} that that has to be satisfied before
     *                      the movement can exit.
     *                      The movement will continue holding while the condition is false.
     *                      Returning true will allow the movement to exit.
     * @param loopMethod A function gets called every loop. This is useful for things like
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If no loop method and no extra condition is needed see {@link Movement#waitForMovement()}
     */
    public void waitForMovement(Condition extraCondition, Runnable loopMethod) {
        start();

        while (!isFinished() && extraCondition.condition()) {
            update();
            loopMethod.run();
        }

        finish();
    }

    /**
     * Wait for the Movement to be completed.
     */
    public void waitForMovement() {
        waitForMovement(() -> true, () -> {});
    }

    /**
     * Wait for the Movement to be completed.
     *
     * @param extraCondition False will continue holding the position,
     *                       true will allow exit
     * <p>
     * If no extra condition is needed see {@link MovementBuild#waitForMovement()}
     */
    public void waitForMovement(Condition extraCondition) {
        waitForMovement(extraCondition, () -> {});
    }

    /**
     * Wait for the Movement to be completed with an extraCondition .
     *
     * @param loopMethod A function gets called every loop. This is useful for things like
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If loop method is not needed see {@link MovementBuild#waitForMovement()}
     */
    public void waitForMovement(Runnable loopMethod) {
        waitForMovement(() -> true, loopMethod);
    }

    public boolean isFinished() {
        return isFinished.condition();
    }

    public Movement start() {
        start.run();
        return this;
    }

    public void finish() {
        finish.run();
    }

    public void update() {
        update.run();
    }
}
