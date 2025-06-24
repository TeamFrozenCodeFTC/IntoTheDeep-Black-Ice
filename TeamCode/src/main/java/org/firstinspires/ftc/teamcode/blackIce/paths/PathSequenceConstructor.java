package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 * Keeps track of the previous target heading and position to create one path after another in the
 * Follower.
 */
public class PathSequenceConstructor {
    private final SegmentSequenceBuilder segmentSequenceBuilder;
    private double previousTargetHeading;
    
    public PathSequenceConstructor(Pose startingPose, PathBehavior defaultPathBehavior) {
        this.segmentSequenceBuilder = new SegmentSequenceBuilder(startingPose.getPosition(), defaultPathBehavior);
        this.previousTargetHeading = startingPose.getHeading();
    }
    
    /**
     * Create one path with multiple segments or curves.
     * Useful if you want to interpolate the heading along the entire path instead of per segment
     * or if you just want the same behavior for a group of segments.
     */
    public SegmentSequenceBuilder buildSegments() {
        return segmentSequenceBuilder;
    }

    private Vector getPreviousTargetPoint() {
        return segmentSequenceBuilder.getPreviousTargetPoint();
    }
    
    public Path toPoint(Vector targetPoint) {
        return segmentSequenceBuilder
            .toPoint(targetPoint)
            .build()
            .setConstantHeading(previousTargetHeading);
    }
    
    public Path toPose(Pose targetPose) {
        Path path = segmentSequenceBuilder
            .toPoint(targetPose.getPosition())
            .build()
            .setConstantHeading(targetPose.getHeading());
        previousTargetHeading = targetPose.getHeading();
        return path;
    }
    
    public Path toPose(Pose targetPose, double distanceToCompleteTurn) {
        Path path = segmentSequenceBuilder
            .toPoint(targetPose.getPosition())
            .build();
        double finishT = distanceToCompleteTurn == 0 ? 0 : path.length / distanceToCompleteTurn;
        path.setLinearHeadingInterpolation(
            previousTargetHeading,
            targetPose.getHeading(),
            finishT
        );
        previousTargetHeading = targetPose.getHeading();
        return path;
    }
    
    public Path toX(double x) {
        return segmentSequenceBuilder
            .toX(x)
            .build()
            .setConstantHeading(previousTargetHeading);
    }
    
    public Path toY(double y) {
        return segmentSequenceBuilder
            .toY(y)
            .build()
            .setConstantHeading(previousTargetHeading);
    }
    
    public Path toPoseInterpolated(Pose targetPose) {
        return toPose(targetPose)
            .setLinearHeadingInterpolation(
                previousTargetHeading,
                targetPose.getHeading()
            );
    }
    
    public Path turnTo(double heading) {
        return segmentSequenceBuilder
            .toPoint(getPreviousTargetPoint())
            .build()
            .setConstantHeading(heading);
    }
    
    public Path curve(double[][] controlPoints) {
        return segmentSequenceBuilder
            .bezierCurve(controlPoints)
            .build();
    }
    
    public Path linedCurve(double[][] controlPoints) {
        return segmentSequenceBuilder
            .linedBezierCurve(controlPoints, 2)
            .build();
    }
    
    public void setCurrentPose(Pose pose) {
        this.previousTargetHeading = pose.getHeading();
        this.segmentSequenceBuilder.setCurrentPoint(pose.getPosition());
    }
}
