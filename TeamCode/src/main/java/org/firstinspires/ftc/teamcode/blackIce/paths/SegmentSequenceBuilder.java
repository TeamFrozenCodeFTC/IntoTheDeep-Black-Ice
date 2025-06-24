package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.LineSegment;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.PathSegment;

import java.util.ArrayList;

/**
 * Build segments
 * A group of Paths that are chained together taking the previous target position as the input
 * for the next Path.
 * Create a path from segments (no heading involved). Use follower methods to add heading
 */
public class SegmentSequenceBuilder { // PathBuilder
    private Vector previousTargetPoint;
    private final PathBehavior defaultPathBehavior;
    private final ArrayList<PathSegment> segments = new ArrayList<>();
    
    public SegmentSequenceBuilder(Vector startingPose, PathBehavior defaultPathBehavior) {
        this.defaultPathBehavior = defaultPathBehavior;
        this.previousTargetPoint = startingPose;
    }
    
    public Path build() {
        Path path = new Path(segments.toArray(new PathSegment[0]));
        defaultPathBehavior.configure(path);
        return path;
    }
    
    public SegmentSequenceBuilder bezierCurve(double[][] controlPoints) {
        BezierCurve curve = new BezierCurve(controlPoints);
        segments.add(new BezierCurve(controlPoints));
        previousTargetPoint = curve.getEndPoint();
        return this;
    }

    /**
     * Creates a Bezier curve path segment that is approximated by a series of line segments.
     * This is theoretically a lot faster because it is precalculated and will never skip part of
     * the path because it is sequential.
     *
     * @param inchPerPoint the distance between each point in inches. Usually 1-2 inches.
     */
    public SegmentSequenceBuilder linedBezierCurve(double[][] controlPoints,
                                                   double inchPerPoint) {
        BezierCurve curve = new BezierCurve(controlPoints);
        
        int numPoints = (int) Math.ceil(curve.length() / inchPerPoint);
        if (numPoints < 2) numPoints = 2;
        Vector[] points = new Vector[numPoints + 1];
        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            points[i] = curve.calculatePointAt(t);
        }
        for (int i = 0; i < points.length - 1; i++) {
            LineSegment segment = new LineSegment(points[i], points[i + 1]);
            segments.add(segment);
        }
        previousTargetPoint = points[points.length - 1];
        
        return this;
    }
    
    public SegmentSequenceBuilder toPoint(Vector point) {
        LineSegment segment = new LineSegment(previousTargetPoint, point);
        segments.add(segment);
        previousTargetPoint = point;
        return this;
    }
    
    /**
     * Creates a path that follows a line from the robot's previous target point to the given x.
     */
    public SegmentSequenceBuilder toX(double x) {
        return toPoint(previousTargetPoint.withX(x));
    }
    
    /**
     * Creates a path that follows a line from the robot's previous target point to the given y.
     */
    public SegmentSequenceBuilder toY(double y) {
        return toPoint(previousTargetPoint.withY(y));
    }
    
    public Vector getPreviousTargetPoint() {
        return previousTargetPoint;
    }
    
    public SegmentSequenceBuilder setCurrentPoint(Vector previousTargetPoint) {
        this.previousTargetPoint = previousTargetPoint;
        return this;
    }
}
