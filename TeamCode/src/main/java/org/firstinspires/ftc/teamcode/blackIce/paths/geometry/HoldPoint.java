package org.firstinspires.ftc.teamcode.blackIce.paths.geometry;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

public class HoldPoint implements PathGeometry {
    private final PathPoint targetPathPoint;
    
    public HoldPoint(Vector targetPoint) {
        this.targetPathPoint = new PathPoint(1, new Vector(0,0), targetPoint);
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double startingGuess) {
        return targetPathPoint;
    }
    
    @Override
    public double computeDistanceRemainingAt(double t) {
        return PathGeometry.super.computeDistanceRemainingAt(t); // target - current, Follower
        // .getInstance() hack?
    }
    
    @Override
    public double length() {
        return 0;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return targetPathPoint;
    }
}
