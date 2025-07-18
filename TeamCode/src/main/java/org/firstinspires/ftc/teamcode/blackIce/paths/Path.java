package org.firstinspires.ftc.teamcode.blackIce.paths;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathPoint;

/**
 * An immutable path with geometry of where the robot should go and behavior of how the Follower
 * should follow the path.
 */
public class Path {
    public final PathGeometry geometry;
    public final HeadingInterpolator headingInterpolator;
    public final ImmutablePathBehavior behavior;

    public final Pose endPose;

    public Path(PathGeometry geometry, HeadingInterpolator headingInterpolator,
                ImmutablePathBehavior behavior) {
        this.geometry = geometry;
        this.headingInterpolator = headingInterpolator;
        this.behavior = behavior;
        PathPoint endPathPoint = geometry.getEndPathPoint();
        this.endPose = new Pose(
            endPathPoint.getPoint(),
            headingInterpolator.interpolate(endPathPoint)
        );
    }

    @Nullable
    public Double computeTargetVelocityAt(double distanceRemaining) {
        if (behavior.velocityProfile == null) {
            return null;
        }
        return behavior.velocityProfile.computeTargetVelocity(geometry.length(), distanceRemaining);
    }
    
    public Path withBehavior(PathBehavior behavior) {
        return new Path(geometry, headingInterpolator,
            this.behavior.toBuilder().mergeWith(behavior).build());
    }
    
    // todo timeouts
}