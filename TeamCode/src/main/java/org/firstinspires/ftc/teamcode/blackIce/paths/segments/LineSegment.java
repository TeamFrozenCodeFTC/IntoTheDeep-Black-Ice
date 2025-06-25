package org.firstinspires.ftc.teamcode.blackIce.paths.segments;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 * Represents a linear Bezier curve (a straight line) between two points.
 * This is a special case of a Bezier curve with two control points which
 * has enhanced performance for straight lines. Lines can get exact points unlike curves.
 */
public class LineSegment implements PathSegment {
    private final double length;
    private final Vector startPoint;
    private final Vector endPoint;
    private final Vector tangent;

    /**
     * Constructs a line between two points.
     * <p>
     * Points must be at least 1e-6 away from each other.
     */
    public LineSegment(Vector start, Vector end) {
        this.startPoint = start;
        this.endPoint = end;
        this.length = startPoint.distanceTo(endPoint);
        this.tangent = endPoint.subtract(startPoint).normalized();
        
        if (this.length < 1e-6) {
            throw new IllegalArgumentException("Line too small with length of " + this.length +
                ". Make the points further away from each other by at least 1e-6.");
        }
    }

    @Override
    public Vector calculatePointAt(double t) {
        return startPoint.add(tangent.times(t * length));
    }
    
    @Override
    public SegmentPoint calculateClosestPointTo(Vector point, double startingGuess) {
        Vector startToPoint = point.subtract(startPoint);
 
        double projection = startToPoint.dotProduct(tangent);
        double t = projection / length;
        t = PathSegment.clampT(t);
        
        Vector closestPoint = calculatePointAt(t);
        return new SegmentPoint(t, tangent, closestPoint);
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public Vector getEndPoint() {
        return endPoint;
    }

    @Override
    public Vector getEndTangent() {
        return tangent;
    }
    
    @Override
    public Type getSegmentType() {
        return PathSegment.Type.LINE;
    }
}

