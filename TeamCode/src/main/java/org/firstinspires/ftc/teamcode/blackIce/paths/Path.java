package org.firstinspires.ftc.teamcode.blackIce.paths;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.DriveCorrection;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

/**
 * Takes a path of points, calculates the desired heading at each point,
 * and creates a Path made of Movements to each point which follow the path.
 */
public class Path implements Cloneable {
    public boolean stopAtEndOfPath = true;

    private MovementProperties pathMovementProperties = (x) -> x;
    private MovementProperties endPointMovementProperties = (x) -> x;

    public final double[][] points;
    double[] headings;

    double[] endPoint;
    double endingHeading;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;
        this.endPoint = new double[] {
            this.points[points.length - 1][0],
            this.points[points.length - 1][1]
        };
        this.headings = calculateHeadings(0);
    }

    private double[] calculateHeadings(double headingOffset) {
        double[] headings = new double[this.points.length - 1];

        for (int i = 0; i < this.points.length - 1; i++) {
            double[] point = this.points[i];

            double[] pointHeading;

            int lookAheadIndex = i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING;
            if (lookAheadIndex >= this.points.length) {
                pointHeading = this.endPoint;
            } else {
                pointHeading = this.points[lookAheadIndex];
            }

            headings[i] = Math.toDegrees(
                Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
            ) + headingOffset;
        }
        return headings;
    }

    /**
     * Make the robot have a constant heading along the path.
     */
    public Path setConstantHeading(double newConstantHeading) {
        headings = new double[this.points.length];
        for (int i = 0; i < this.points.length; i++) {
            headings[i] = newConstantHeading;
        }
        return this;
    }

    /**
     * Make the robot continue its momentum at the end of the path.
     */
    public Path setToContinueMomentumAfter() {
        stopAtEndOfPath = false;
        return this;
    }

    /**
     * Make the robot stop at the end of the path (this is the default).
     */
    public Path setToStopAtEndOfPath() {
        stopAtEndOfPath = true;
        return this;
    }

    /**
     * Set specific movement properties of the Path.
     * <pre><code>
     * .setPathProperties((movement) -> movement
     *     .setMaxVelocity(...)
     *     .setMaxPower(...)
     *     // Many other methods
     * )
     * </code></pre>
     */
    public Path setPathProperties(MovementProperties setProperties) {
        pathMovementProperties = setProperties;
        return this;
    }

    /**
     * Set specific movement properties for stopping and holding the end of Path.
     * <pre><code>
     * .setEndPointHoldProperties((movement) -> movement
     *     .setMaxVelocity(...)
     *     .setMaxPower(...)
     *     // Many other methods
     * )
     * </code></pre>
     */
    public Path setEndPointHoldProperties(MovementProperties setProperties) {
        endPointMovementProperties = setProperties;
        return this;
    }

    /**
     * Make the robot have a offset heading along the path.
     */
    public Path setHeadingOffset(double newHeadingOffset) {
        headings = calculateHeadings(newHeadingOffset);
        return this;
    }

    /**
     * Make the robot smoothly transition from one heading to another along the path.
     */
    public Path setLinearHeadingInterpolation(double startingHeading, double endingHeading) {
        return setLinearHeadingInterpolation(startingHeading, endingHeading, 1.00);
    }

    /**
     * Make the robot smoothly transition from one heading to another along the path.
     *
     * @param finishByPercent The percent of the path to finish the rotation.
     *                       .80 would make the robot turn for 80% of the path.
     */
    public Path setLinearHeadingInterpolation(
        double startingHeading,
        double endingHeading,
        double finishByPercent
    ) {
        headings = new double[this.points.length];

        for (int i = 0; i < this.points.length; i++) {
            int numPointsMinusOne = this.points.length - 1;
            double adjustedFinish = (1 - finishByPercent) * numPointsMinusOne;
            double t = Math.min(1, (double) i / (numPointsMinusOne - adjustedFinish));
            headings[i] = startingHeading + t * (endingHeading - startingHeading);
        }
        return this;
    }

    private int i = 1;
    private boolean holdingEndPosition = false;
    private boolean finished = false;

    private void updatePath() {
        if (holdingEndPosition) {
            if (holdEndPosition.isFinished()) {
                finished = true;
            }
            holdEndPosition.update();
            return;
        }

        Follower.telemetry.addData("i", i);
        Follower.telemetry.update();

        Target.updatePosition();

        double inchesLeftOnPath = (this.points.length - i) * Constants.Curve.INCHES_PER_POINT;
//        if (inchesLeftOnPath < Util.getVectorMagnitude(
//            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
//            startHoldingEndPosition();
//            return;
//        }

        boolean endPointIsWithinBrakingDistance = Odometry.pointIsWithinBrakingDistance(
            this.endPoint[0],
            this.endPoint[1]
        );

        if (inchesLeftOnPath < 24 && endPointIsWithinBrakingDistance) {
            startHoldingEndPosition();
            return;
        }

        movements[i].waitForMovement();

        this.i += 1;
        if (i >= this.points.length - 1) { // -1
            startHoldingEndPosition();
        }
    }

    private Movement holdEndPosition;

    private void startHoldingEndPosition() {
        holdEndPosition.start();

        holdingEndPosition = true;
    }

    private Movement[] movements;

    private void buildMovements() {
        movements = new Movement[this.points.length - 1];

        for (int i = 1; i < this.points.length; i++) {
            double[] point = this.points[i];
            movements[i - 1] = pathMovementProperties.setProperties(
                    MovementBuilder.moveThrough(point[0], point[1], this.headings[i - 1])
                )
                .setDriveCorrection(DriveCorrection.fullPower)
                .setIsAtGoal(() -> {
                    boolean isPastPoint = (Target.y - Odometry.y - Odometry.yBrakingDistance)
                        * Target.yDelta
                        + (Target.x - Odometry.x - Odometry.xBrakingDistance)
                        * Target.xDelta < 0;

                    return isPastPoint;
                    //                if (Vector.getMagnitude(xPower, yPower) < 1) {
                    //                    break;
                    //                }
                })
                .build();
        }
    }

    /**
     * Finish building the path. This should be the last method called.
     * If you want to inherit methods
     */
    public Movement build() {
        this.endingHeading = this.headings[headings.length - 1];

        buildMovements();
        buildEndPointMovement();

        return new Movement() {
            @Override
            public boolean isFinished() {
                return finished;
            }

            @Override
            public Movement start() {
                i = 1;
                holdingEndPosition = false;
                finished = false;
                Target.setTarget(Target.previousHeading, Path.this.points[0][0], Path.this.points[0][1]); //?
                timer.reset();
                return this;
            }

            @Override
            public void finish() {
                holdEndPosition.finish();
            }

            @Override
            public void update() {
                updatePath();
            }
        };
    }

    private void buildEndPointMovement() {
        if (stopAtEndOfPath) {
            holdEndPosition = endPointMovementProperties.setProperties(
                MovementBuilder.stopAtPosition(
                    endPoint[0],
                    endPoint[1],
                    endingHeading
                )).build();
        }
        else {
            holdEndPosition = pathMovementProperties.setProperties(
                MovementBuilder.moveThrough(
                    endPoint[0],
                    endPoint[1],
                    endingHeading
                )).setToContinuePowerAfter().build();
        }
    }

    @NonNull
    @Override
    public Path clone() {
        try {
            return (Path) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}