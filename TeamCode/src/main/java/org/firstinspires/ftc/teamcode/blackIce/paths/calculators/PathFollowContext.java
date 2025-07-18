package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.motion.MotionState;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.follower.PathExecutor;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathPoint;

// PathState
@Deprecated
public class PathFollowContext {
    public final MotionState motionState;
    public final Path path;
    public final PathPoint closestPoint;
    public final PathPoint closestPointToRobot;
    //public final HeadingInterpolator headingInterpolator;
    public final PathExecutor pathExecutor;
    public final double distanceToEnd;

    public PathFollowContext(MotionState motionState, Path currentPath,
                             PathPoint closestPoint, PathPoint closestPointToRobot, PathExecutor pathExecutor,
                             double distanceToEnd) {
        this.motionState = motionState;
        this.path = currentPath;
        this.closestPoint = closestPoint;
//        this.headingInterpolator = currentPath.getHeadingInterpolator();
        this.closestPointToRobot = closestPointToRobot;
        this.pathExecutor = pathExecutor;
        this.distanceToEnd = distanceToEnd;
    }
}