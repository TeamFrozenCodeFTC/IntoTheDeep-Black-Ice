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
    private T current;

    public Advancer(T[] steps) {
        this.steps = steps;
        this.current = steps[index];
    }
    
    public T current() {
        return current;
    }
    
    public boolean advance() {
        if (!isDone()) {
            index++;
            current = steps[index];
            return true;
        }
        return false;
    }
    
    public boolean isDone() {
        return index >= steps.length - 1;
    }
    
    public int getIndex() {
        return index;
    }
    
    public int size() {
        return steps.length;
    }
}

