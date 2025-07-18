package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathPoint;

public class PathState { // pass around with PathBehavior
    public final double distanceRemaining;
    public final double distanceTraveled;
    
    public final double currentTValue;

    public final double tangentialVelocity;

    public final PathPoint closestPathPointToRobot;
    public final PathPoint closestPathPointToPredictedStop;
    public final PathPoint closestPathPointToPredictedStop2;
    
    public PathState(double distanceRemaining, double percentAlongPath,
                     double distanceTraveledAlongPath,
                     double tangentialVelocity,
                     PathPoint closestPointToRobot, PathPoint closestPointToPredictedStop,
                     PathPoint closestPathPointToPredictedStop2) {
        this.distanceRemaining = distanceRemaining;
        this.currentTValue = percentAlongPath;
        this.distanceTraveled = distanceTraveledAlongPath;
        this.tangentialVelocity = tangentialVelocity;
        this.closestPathPointToRobot = closestPointToRobot;
        this.closestPathPointToPredictedStop = closestPointToPredictedStop;
        this.closestPathPointToPredictedStop2 = closestPathPointToPredictedStop2;
    }
}