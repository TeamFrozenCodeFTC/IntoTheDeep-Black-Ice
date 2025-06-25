package org.firstinspires.ftc.teamcode.blackIce.action;

@FunctionalInterface
public interface Action {
    void execute();
    
    /**
     * Executes the action if the condition is true.
     */
    default void executeWhen(boolean condition) {
        if (condition) {
            execute();
        }
    }
    
    /**
     * Combines this action with another action, executing both in sequence.
     */
    default Action andThen(Action other) {
        if (other == EMPTY) {
            return this;
        }
        if (this == EMPTY) {
            return other;
        }
        
        return () -> {
            this.execute();
            other.execute();
        };
    }
    
    Action EMPTY = () -> {};
}
