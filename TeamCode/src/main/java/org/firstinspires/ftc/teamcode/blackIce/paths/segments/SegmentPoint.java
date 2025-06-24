package org.firstinspires.ftc.teamcode.blackIce.paths.segments;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 * A point on a segment with a tValue (percent along segment) and tangent.
 */
public class SegmentPoint {
    private final double tValue;
    private final Vector tangentVector;
    private final Vector point;
    private final boolean isAtEnd;

    public SegmentPoint(
        double t,
        Vector tangent,
        Vector point
    ) {
        this.tValue = t;
        this.tangentVector = tangent;
        this.point = point;
        this.isAtEnd = t >= 1;
    }
    
    public boolean isAtEnd() {
        return isAtEnd;
    }
    
    public Vector getPoint() {
        return point;
    }
    
    public Vector getTangentVector() {
        return tangentVector;
    }
    
    public double getTValue() {
        return tValue;
    }
}
