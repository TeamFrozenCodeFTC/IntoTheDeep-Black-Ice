package org.firstinspires.ftc.teamcode.blackIce;
//
//public abstract class Movement {
//    /**
//     * Wait for the Movement to be completed.
//     */
//    public void waitForMovement() {
//        waitForMovement(() -> true, () -> {});
//    }
//
//    /**
//     * Wait for the Movement to be completed.
//     *
//     * @param extraCondition False will continue holding the position,
//     *                       true will allow exit
//     * <p>
//     * If no extra condition is needed see {@link Point#waitForMovement()}
//     */
//    public void waitForMovement(Condition extraCondition) {
//        waitForMovement(extraCondition, () -> {});
//    }
//
//    /**
//     * Wait for the Movement to be completed with an extraCondition .
//     *
//     * @param extraCondition False will continue holding the position,
//     *                       True will allow exit
//     * @param loopMethod A function gets called every loop. This is useful for things like
//     *                       when a linear slide reaches a certain height, a claw opens.
//     * <p>
//     * If no loop method and no extra condition is needed see {@link Point#waitForMovement()}
//     */
//    public void waitForMovement(Condition extraCondition, Runnable loopMethod) {
//        start();
//
//        while (!isFinished() && extraCondition.condition()) {
//            update();
//            loopMethod.run();
//        }
//
//        finish();
//    }
//
//    /**
//     * Wait for the Movement to be completed with an extraCondition .
//     *
//     * @param loopMethod A function gets called every loop. This is useful for things like
//     *                       when a linear slide reaches a certain height, a claw opens.
//     * <p>
//     * If loop method is not needed see {@link Point#waitForMovement()}
//     */
//    public void waitForMovement(Runnable loopMethod) {
//        waitForMovement(() -> true, loopMethod);
//    }
//
//    public abstract boolean isFinished();
//    public abstract Movement start();
//    public abstract void finish();
//    public abstract void update();
//}


public class Movement {
    Condition isFinished;
    Runnable start;
    Runnable finish;
    Runnable update;

    public Movement(Condition isFinished, Runnable start, Runnable finish, Runnable update) {
        this.isFinished = isFinished;
        this.start = start;
        this.finish = finish;
        this.update = update;
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
     * If no extra condition is needed see {@link Point#waitForMovement()}
     */
    public void waitForMovement(Condition extraCondition) {
        waitForMovement(extraCondition, () -> {});
    }

    /**
     * Wait for the Movement to be completed with an extraCondition .
     *
     * @param extraCondition False will continue holding the position,
     *                       True will allow exit
     * @param loopMethod A function gets called every loop. This is useful for things like
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If no loop method and no extra condition is needed see {@link Point#waitForMovement()}
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
     * Wait for the Movement to be completed with an extraCondition .
     *
     * @param loopMethod A function gets called every loop. This is useful for things like
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If loop method is not needed see {@link Point#waitForMovement()}
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
