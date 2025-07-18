package org.firstinspires.ftc.teamcode.blackIce.util;

/**
 * Advancer is a utility class that helps in iterating through an array of steps, keeping track
 * of the index, current item, and when it is completed.
 * <p>
 * Used in pathExecutor to manage the progression through the segments of a path
 * and used in the Follower to manage the progression through the paths it is following.
 */
public class Advancer<T> {
    private final T[] steps;
    private int index = 0;
    public final int amountOfSteps;
    
    @SafeVarargs
    public Advancer(T... steps) {
        this.steps = steps;
        this.amountOfSteps = steps.length;
    }
    
    public T current() {
        return steps[index];
    }
    
    public boolean advance() {
        if (isDone()) return false;
        index++;
        return true;
    }
    
    public boolean isDone() {
        return index >= steps.length - 1;
    }
    
    public int getIndex() {
        return index;
    }
}
