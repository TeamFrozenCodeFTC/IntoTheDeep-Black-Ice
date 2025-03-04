package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

public class MovementBuilder {
    /**
     * Move the robot through a target point without stopping at it.
     *
     * <p>
     * <h5>How does it work?</h5>
     * <ul>
     * <li>This method travels towards the point using a simple proportional control (error * constant).</li>
     * <li>The robot predicts its position based on the braking distance
     * for determining if it has passed the target. This allows it to quickly change
     * direction without overshooting.</li>
     * <li>To determine if it has passed the target, it constructs a plane
     * perpendicular to the line connecting the previous target and the new target.</li>
     * </ul>
     *
     * @return A {@code Movement} object configured to move through the target.
     */
    public static MovementBuild moveThrough(double x, double y, double heading) {
        return new MovementBuild(x, y, heading) {
            @Override
            protected MovementBuild getThis() {
                return this;
            }

            private double xSign;
            private double ySign;
            private double slope;

            protected void start() {
                double targetYError = Target.previousY - Target.y;
                double targetXError = Target.previousX - Target.x;
                xSign = Math.signum(targetXError);
                ySign = Math.signum(targetYError);
                slope = (Target.x == Target.previousX) ? 0 : targetYError / targetXError;

                super.start();
            }

            @Override
            boolean isAtGoal() {
                double predictedXError = Target.xError - Odometry.xBrakingDistance;
                double predictedYError = Target.yError - Odometry.yBrakingDistance;

                if (Target.x == Target.previousX) {
                    return ySign * predictedYError >= 0;
                }

                return -xSign * predictedXError <= slope * xSign * predictedYError;
            }
        }
            .setHeadingCorrection(HeadingCorrection.locked)
            .setDriveCorrection(DriveCorrection.proportional)
            .continuePowerAfter();
    }

    /**
     * Move the robot to target point and stop.
     * <p>
     * <h5>How does it work?</h5>
     * <ul>
     * <li>This method travels towards the point
     * using a simple proportional control (error * constant).</li>
     * <li>The robot predicts its position based on the braking distance,
     * allowing the robot maintain full power for as long as possible,
     * only braking at the optimal point. The braking distance also prevents overshooting.</li>
     *
     * @return A {@code Movement} object configured to stop at the target.
     *
     * @see MovementBuild#stopAtPosition
     */
    public static MovementBuild stopAtPosition(double x, double y, double heading) {
        return new MovementBuild(x, y, heading) {
            @Override
            protected MovementBuild getThis() {
                return this;
            }

            private double consideredStoppedVelocity = 1;

            /**
             * Set the maximum velocity that the robot considers at rest.
             *
             * @param newConsideredStoppedVelocity (inches/second).
             * A lower value will make the robot take more time to stop more accurately.
             * A higher value will make the robot slow down less and carry more of its momentum.
             * Should be not less than 0.01 inches/second.
             * <p>
             * If you don't want the robot to slow down use {@link MoveThrough}
             */
            public MovementBuild setConsideredStoppedVelocity(double newConsideredStoppedVelocity) {
                consideredStoppedVelocity = newConsideredStoppedVelocity;
                return getThis();
            }

            @Override
            boolean isAtGoal() {
                return Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
                    && Odometry.velocity < consideredStoppedVelocity;
            }
        }
            .setHeadingCorrection(HeadingCorrection.locked)
            .setDriveCorrection(DriveCorrection.stopAtTarget);
    }
}

