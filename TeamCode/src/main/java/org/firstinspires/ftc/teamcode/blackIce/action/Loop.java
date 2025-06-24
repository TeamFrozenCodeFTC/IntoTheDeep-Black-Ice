package org.firstinspires.ftc.teamcode.blackIce.action;

/**
 * <pre><code>
 * initialize()
 * while (!isFinished() && !isCanceled()) {
 *     loop();
 * }
 * </code></pre>
 */
public interface Loop {
    void initialize();
    void loop();
    boolean isFinished(); // continues the loop until this condition is true
    boolean isCanceled(); // cancel the loop when the condition is true,
    void cancel();
    
    default boolean isRunning() {
        return !isFinished() && !isCanceled();
    }
}
