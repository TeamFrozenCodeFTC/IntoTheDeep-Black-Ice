package org.firstinspires.ftc.teamcode.blackIce.paths.segments;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.math.MathFunctions;

/**
 * A immutable, stateless part of a path, purely positional. It only knows it's tangent heading.
 */
public interface PathSegment {
    SegmentPoint calculateClosestPointTo(Vector robotPosition, double startingGuess);
    Vector calculatePointAt(double t);
    
    double length();
    Vector getEndPoint();
    Vector getEndTangent();
    
    enum Type {
        CURVE,
        LINE,
        POINT
    }
    Type getSegmentType();
    
    default SegmentPoint getEndSegmentPoint() {
        return new SegmentPoint(1, this.getEndTangent(), this.getEndPoint());
    }
    
    static double clampT(double t) {
        return MathFunctions.clamp0To1(t);
    }
}

