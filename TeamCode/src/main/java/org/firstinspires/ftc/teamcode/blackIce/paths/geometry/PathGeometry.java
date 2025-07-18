package org.firstinspires.ftc.teamcode.blackIce.paths.geometry;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.MathFunctions;

/**
 * A immutable, stateless part of a path, purely positional. It only knows it's tangent heading.
 */
public interface PathGeometry {
    PathPoint computeClosestPathPointTo(Vector robotPosition, double startingGuess);

    default double computeDistanceRemainingAt(double t) {
        return (1 - t) * length();
    };
    // TODO be apart of pathPoint
    
    double length();

    PathPoint getEndPathPoint();
    
    static double clampT(double t) {
        return MathFunctions.clamp0To1(t);
    }
}

