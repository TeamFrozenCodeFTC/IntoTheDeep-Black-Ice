package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.segments.SegmentPoint;

/**
 * Heading interpolation for a path.
 */
@FunctionalInterface
public interface HeadingInterpolator {
    /**
     * (bezierPoint) -> heading
     */
    double interpolate(SegmentPoint curvePoint);

    /**
     * Offsets the heading interpolator by a given amount.
     */
    default HeadingInterpolator offset(double angle) {
        double headingOffsetRadians = Math.toRadians(angle);
        return curvePoint -> this.interpolate(curvePoint) + headingOffsetRadians;
    }
    
    /**
     * Reverses or rotates the heading interpolator by 180 degrees.
     */
    default HeadingInterpolator reverse() {
        return offset(180);
    }
    
    /**
     * The robot will always be facing the direction of the path. This is the default behavior.
     */
    HeadingInterpolator tangent = curvePoint -> curvePoint.getTangentVector().calculateRadians();
    
    /**
     * A constant heading along a path.
     */
    static HeadingInterpolator constant(double heading) {
        double headingRadians = Math.toRadians(heading);
        return curvePoint -> headingRadians;
    }

    /**
     * The robot will transition from the start heading to the end heading.
     */
    static HeadingInterpolator linear(double startHeading, double endHeading) {
        return linear(startHeading, endHeading, 1.00);
    }
    
    /**
     * @param startHeading Heading at the start (degrees)
     * @param endHeading Heading at the end (degrees)
     * @param finishT The t parameter (0-1) at which the heading should reach endHeadingDeg
     */
    static HeadingInterpolator linear(double startHeading, double endHeading, double finishT) {
        return curvePoint -> {
            double t = Math.min(curvePoint.getTValue() / finishT, 1.0);
            double startHeading_ = Math.toRadians(startHeading);
            double endHeading_ = Math.toRadians(endHeading);
            double deltaHeading = AngleUnit.RADIANS.normalize(endHeading_ - startHeading_);
            return startHeading_ + deltaHeading * t;
        };
    }
    
    /**
     * The robot will always be facing the the given curvePoint.
     */
    static HeadingInterpolator facingPoint(Vector facingPoint) {
        return curvePoint -> {
            Vector direction = facingPoint.minus(curvePoint.getPoint());
            return direction.calculateRadians();
        };
    }
}
