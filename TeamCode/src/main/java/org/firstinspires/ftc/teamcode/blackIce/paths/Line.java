package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Target;

/**
 * A straight-line path for the robot to follow.
 * <p>
 * This more optimal when high precision is required or when obstacles are nearby.
 * Simply going toward an single point can make the robot overshoot if used after a change in direction.
 */
public class Line extends BezierCurve {

    /**
     * Constructs a straight-line path from the robot's previous position to the given endpoint.
     * <p>
     * See {@link Line} for more details on its advantages and use cases.
     */
    public Line(double endX, double endY) {
        super(new double[][] {
            {Target.previousX, Target.previousY}, //TODO cannot do, previous needs be used in build function
            {endX, endY}
        });
    }
}
