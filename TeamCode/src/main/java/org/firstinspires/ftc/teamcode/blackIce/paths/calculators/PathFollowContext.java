package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathExecutor;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.SegmentPoint;

public class PathFollowContext {
    public final MotionState motionState;
    public final Path path;
    public final SegmentPoint closestPoint;
    public final SegmentPoint closestPointToRobot;
    public final double targetVelocity;
    public final double targetAcceleration;
    public final HeadingInterpolator headingInterpolator;
    public final PathExecutor pathExecutor;
    public final double distanceToEnd;

    public PathFollowContext(MotionState motionState, Path currentPath,
                             SegmentPoint closestPoint, SegmentPoint closestPointToRobot, PathExecutor pathExecutor,
                             double distanceToEnd) {
        this.motionState = motionState;
        this.path = currentPath;
        this.closestPoint = closestPoint;
        this.targetVelocity = currentPath.getMaxVelocity();
        this.targetAcceleration = currentPath.getAcceleration();
        this.headingInterpolator = currentPath.getHeadingInterpolator();
        this.closestPointToRobot = closestPointToRobot;
        this.pathExecutor = pathExecutor;
        this.distanceToEnd = distanceToEnd;
    }
}