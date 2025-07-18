package org.firstinspires.ftc.teamcode.blackIce.paths;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.blackIce.action.Action;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.Line;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.HoldPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Keeps track of the previous target heading and position to create one path after another in the
 * Follower.
 */
public class PathSequence {
    private static PathBehavior globalDefaultBehavior = new PathBehavior();
    
    public static void setGlobalDefaultBehavior(PathBehavior behavior) {
        globalDefaultBehavior = behavior;
    }
    
    private final PathBehavior defaultPathBehavior;
    
    private final List<DeferredPath> paths = new ArrayList<>();
    private final @Nullable Pose startingPose;
    
    private @Nullable DeferredPath currentPath = null;
    
    public PathSequence(@Nullable Pose startingPose, PathBehavior defaultPathBehavior) {
        this.startingPose = startingPose;
        this.defaultPathBehavior = globalDefaultBehavior.mergeWith(defaultPathBehavior);
    }
    
    public PathSequence() {
        this(null, new PathBehavior());
    }
    
    public PathSequence(PathBehavior defaultPathBehavior) {
        this(null, defaultPathBehavior);
    }
    
    public Path[] asArray(Pose startPose) {
        return build(startPose).getPaths();
    }
    
    public Path[] asArray() {
        return build().getPaths();
    }
    
    private PathSequence addPath(DeferredPath path) {
        paths.add(path);
        currentPath = path;
        return this;
    }
    
    /**
     * This is called internally at runtime once when a path is being followed.
     * TODO this method can be optimized by prebuilding all paths except the first because the
     * it is the only path required to potentially need a startPose from the follower.
     * OR can also preload the order of all run paths but really hard to manage that.
     */
    public ImmutablePathSequence build(Pose startPose) {
        Pose previousEndPose = startPose;
        
        Path[] paths = new Path[this.paths.size()];
        int i = 0;
        for (DeferredPath draft : this.paths) {
            Path path = draft.toPath(previousEndPose);
            previousEndPose = path.endPose;
            paths[i++] = path;
        }
        
        return new ImmutablePathSequence(paths);
    }
    
    public ImmutablePathSequence build() {
        if (startingPose == null) {
            throw new IllegalArgumentException("No starting pose for this PathSequence");
        }
        return build(startingPose);
    }
    
    public boolean hasStartingPose() {
        return startingPose != null;
    }
    
    public PathSequence addPath(Path path) {
        return addPath(new DeferredPath(
            previousPoint -> path.geometry,
            previousHeading -> path.headingInterpolator,
            path.behavior.toBuilder(),
            path.endPose.getHeading()
        ));
    }
    
    public PathSequence addPathSequence(PathSequence other) {
        for (DeferredPath draft : other.paths) {
            PathBehavior behaviorCopy = draft.behavior.clone();
            DeferredPath copy = new DeferredPath(draft.geometrySupplier, draft.headingInterpolatorSupplier,
                                                 behaviorCopy, draft.targetHeading);
            paths.add(copy);
        }
        return this;
    }
    
    public PathSequence addPath(PathGeometry geometry) {
        return addPath(start -> geometry);
    }
    
    public PathSequence addPath(DeferredPath.GeometrySupplier geometry) {
        return addPath(new DeferredPath(
            geometry,
            HeadingInterpolator::constant,
            defaultPathBehavior,
            null
        ));
    }
    
    public PathSequence addAction(Action action) {
        withBehavior(new PathBehavior().onFinish(action));
        preventCurrentPathEditing();
        return this;
    }
    
    /**
     * Sets the behavior of the last created path.
     */
    public PathSequence withBehavior(PathBehavior behavior) {
        if (currentPath == null) {
            throw new IllegalArgumentException("Cannot use this method before adding a path.");
        }
        currentPath.overrideBehavior(behavior);
        return this;
    }
    
    // TODO make vector only internal any only accept like new Pose(3, 5) as a point
    public PathSequence lineTo(Vector point) {
        return addPath(previousPoint -> new Line(previousPoint, point));
    }
    
    public PathSequence lineTo(double x, double y) {
        return lineTo(new Vector(x, y));
    }

    public PathSequence lineTo(Pose pose) {
        return lineTo(pose.getPosition()).withConstantHeading(pose.getHeading());
    }
    
    public PathSequence lineTo(double x, double y, double heading) {
        return lineTo(new Pose(x, y, heading));
    }
    
    /**
     * Set the function that tells the robot what heading it should be, at a given point.
     */
    public PathSequence withHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        getCurrentPath().setHeadingInterpolatorSupplier(previousHeading -> headingInterpolator);
        return this;
    }
    
    /**
     * Set the headingInterpolator based off the previous heading from the last path.
     */
    public PathSequence withHeadingInterpolatorSupplier(DeferredPath.HeadingInterpolatorSupplier headingInterpolator) {
        getCurrentPath().setHeadingInterpolatorSupplier(headingInterpolator);
        return this;
    }
    
    // setLinearHeadingInterpolation
    public PathSequence withHeadingInterpolationTo(double heading) {
        return withHeadingInterpolatorSupplier(previousHeading -> HeadingInterpolator.linear(previousHeading, heading));
    }
    
    // setConstantHeading
    public PathSequence withConstantHeading(double constantHeading) {
        return withHeadingInterpolator(HeadingInterpolator.constant(constantHeading));
    }
    
    public PathSequence withTangentHeading() {
        return withHeadingInterpolator(HeadingInterpolator.tangent);
    }
    
    public PathSequence withHeadingInterpolation() {
        return withHeadingInterpolationTo(requireTargetHeading());
    }
    
    public PathSequence withConstantHeading() {
        return withConstantHeading(requireTargetHeading());
    }
    
    // TODO setHeadingInterpolator for whole path
    
    // TODO turn tangent and then back to a heading
    // TODO turn at end of path (finishByT or use distanceRemaining/distance left on path
    
    private double requireTargetHeading() {
        Double heading = getCurrentPath().targetHeading;
        if (heading == null) {
            throw new IllegalStateException("No heading available from path endpoint.");
        }
        return heading;
    }
    
    /**
     * Stops at the end of the last added path.
     */
    public PathSequence stop() {
        getCurrentPath().behavior.stopAtEnd();
        return this;
    }

    public PathSequence turnTo(double heading) { // TODO slower turning with interpolation or lower power
        addPath(HoldPoint::new)
            .withConstantHeading(heading)
            .stop();
        preventCurrentPathEditing();
        return this;
    }
    
    private void preventCurrentPathEditing() {
        currentPath = null;
    }
    
    /**
     * Creates a path that follows a line from the robot's previous target point to the given x.
     */
    public PathSequence lineToX(double x) {
        return addPath(previousPoint -> new Line(previousPoint, previousPoint.withX(x)));
    }
    
    /**
     * Creates a path that follows a line from the robot's previous target point to the given y.
     */
    public PathSequence lineToY(double y) {
        return addPath(previousPoint -> new Line(previousPoint, previousPoint.withY(y)));
    }
  
//    public PathSequence setCurrentPose(Pose currentPose) {
//        this.previousEndPose = currentPose;
//        drafts.add(new DeferredPath())
//        return this;
//    }
    
    // interface FollowableStep
    
    /**
     * A mutable path that can be put with any other path prior seamlessly.
     */
    private static class DeferredPath {
        @FunctionalInterface
        public interface GeometrySupplier {
            PathGeometry get(Vector previousPoint);
        }
        
        @FunctionalInterface
        public interface HeadingInterpolatorSupplier {
            HeadingInterpolator get(double previousHeading);
        }
        
        private final GeometrySupplier geometrySupplier;
        private HeadingInterpolatorSupplier headingInterpolatorSupplier;
        private PathBehavior behavior;
        
        public @Nullable Double targetHeading;
        
        // .lineTo(new Pose(5,5,90)).withHeading(60).linearInterpolation()

        DeferredPath(GeometrySupplier geometrySupplier,
                     HeadingInterpolatorSupplier headingInterpolatorSupplier,
                     PathBehavior behavior,
                     @Nullable Double targetHeading) {
            this.geometrySupplier = geometrySupplier;
            this.headingInterpolatorSupplier = headingInterpolatorSupplier;
            this.behavior = behavior;
            this.targetHeading = targetHeading;
        }
        
        void overrideBehavior(PathBehavior behavior) {
            this.behavior = behavior.mergeWith(behavior);
        }
        
        void setHeadingInterpolatorSupplier(HeadingInterpolatorSupplier headingInterpolatorSupplier) {
            this.headingInterpolatorSupplier = headingInterpolatorSupplier;
        }
        
        Path toPath(Pose previousPose) {
            return new Path(
                geometrySupplier.get(previousPose.getPosition()),
                headingInterpolatorSupplier.get(previousPose.getHeading()),
                behavior.build()
            );
        }
    }
    
    private DeferredPath getCurrentPath() {
        if (currentPath == null) {
            throw new IllegalStateException("There is no current path to use this method on.");
        }
        return currentPath;
    }
    
//    public PathSequence curveTo(Vector targetPoint, Vector controlPoint) {
//        return addPath(new DeferredPath(
//            previousPoint -> new BezierCurve(new double[]{{}})
//        ));
//    };
    
    // TODO toPoint



//    public SegmentSequenceBuilder bezierCurve(double[][] controlPoints, PathBehaviorConfig behavior) {
//
//        segments.add(new BezierCurve(controlPoints));
//        paths = paths.add(new Path(new BezierCurve(controlPoints)))
//        previousTargetPoint = curve.getEndPoint();
//        return this;
//    }

//    /**
//     * Creates a Bezier curve path segment that is approximated by a series of line segments.
//     * This is theoretically a lot faster because it is precalculated and will never skip part of
//     * the path because it is sequential.
//     *
//     * @param inchPerPoint the distance between each point in inches. Usually 1-2 inches.
//     */
//    public SegmentSequenceBuilder linedBezierCurve(double[][] controlPoints,
//                                                   double inchPerPoint) {
//        BezierCurve curve = new BezierCurve(controlPoints);
//
//        int numPoints = (int) Math.ceil(curve.length() / inchPerPoint);
//        if (numPoints < 2) numPoints = 2;
//        Vector[] points = new Vector[numPoints + 1];
//        for (int i = 0; i <= numPoints; i++) {
//            double t = (double) i / numPoints;
//            points[i] = curve.calculatePointAt(t);
//        }
//        for (int i = 0; i < points.length - 1; i++) {
//            Line segment = new Line(points[i], points[i + 1]);
//            segments.add(segment);
//        }
//        previousTargetPoint = points[points.length - 1];
//
//        return this;
//    }
}