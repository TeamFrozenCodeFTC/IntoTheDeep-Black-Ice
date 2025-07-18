package org.firstinspires.ftc.teamcode.blackIce.paths.geometry;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

/**
 * Represents a linear Bezier curve (a straight line) between two points.
 * This is a special case of a Bezier curve with two control points which
 * has enhanced performance for straight lines. Lines can get exact points unlike curves.
 */
public class Line implements PathGeometry {
    private final double length;
    private final Vector startPoint;
    private final Vector tangent;
    private final PathPoint endPathPoint;

    /**
     * Constructs a line between two points.
     * <p>
     * Points must be at least 1e-6 away from each other.
     */
    public Line(Vector start, Vector end) {
        this.startPoint = start;
        this.length = startPoint.distanceTo(end);
        this.tangent = end.minus(startPoint).normalized();
        this.endPathPoint = new PathPoint(1, tangent, end);
        
        if (this.length < 1e-6) {
            throw new IllegalArgumentException("Line too small with length of " + this.length +
                ". Make the points further away from each other by at least 1e-6.");
        }
    }

    public Vector computePointAt(double t) {
        return startPoint.plus(tangent.times(t * length));
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double startingGuess) {
        Vector startToPoint = point.minus(startPoint);
 
        double projection = startToPoint.dotProduct(tangent);
        double t = projection / length;
        t = PathGeometry.clampT(t);
        
        Vector closestPoint = computePointAt(t);
        Logger.debug("internal tangent calculateClosestPointTo", tangent);
        return new PathPoint(t, tangent, closestPoint);
    }

    @Override
    public double length() {
        return length;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return endPathPoint;
    }
}

