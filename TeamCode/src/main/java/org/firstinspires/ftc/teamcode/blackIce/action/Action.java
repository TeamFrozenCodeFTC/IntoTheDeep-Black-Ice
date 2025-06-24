package org.firstinspires.ftc.teamcode.blackIce.action;

public interface Action {
    /**
     * The action ran every loop.
     */
    void update();

    default void reset() {};
}
