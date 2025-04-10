package org.firstinspires.ftc.teamcode.blackIce.movement;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.DriveCorrection;
import org.firstinspires.ftc.teamcode.blackIce.HeadingCorrection;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.Vector;
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

            {
                setHeadingCorrection(HeadingCorrection.locked);
                setDriveCorrection(() ->
                    Drive.fieldVectorToLocalWheelPowers(
                        (Target.xError - Odometry.xBrakingDistance),
                        (Target.yError - Odometry.yBrakingDistance)
                    )
                );
                setTotalCorrection(() -> Drive.power(
                    Drive.combineMax(
                        getDrivePowerCorrection(),
                        getHeadingPowerCorrection(),
                        1
                    )
                ));

                // Makes robot move always full power
                setDrivePowerScaling(drivePowers -> Vector.scaleToMax(drivePowers, 1));

                setToContinuePowerAfter();

                isAtGoal = () -> {
                    boolean isPastPoint = (Target.yError - Odometry.yBrakingDistance)
                        * Target.yDelta
                        + (Target.xError - Odometry.xBrakingDistance)
                        * Target.xDelta < 0;

                    return isPastPoint;

//                if (Vector.getMagnitude(xPower, yPower) < 1) {
//                    break;
//                }
                };
            }
        };
    }

    // TODO create methods that detect being stuck/lining up with walls etc

    /**
     * Same as {@link MovementBuilder#stopAtPosition(double, double, double)} but with
     * {@code consideredStoppedVelocity}
     *
     * @param consideredStoppedVelocity the velocity that the robot ends the Movement at.
     * Lower values will make the robot stop more accurately but may waste time.
     * Higher values can be useful for just a quick slow down.
     * This value is not needing when holding positions.
     */
    public static MovementBuild stopAtPosition(double x, double y, double heading,
                                                double consideredStoppedVelocity) {
        return new MovementBuild(x, y, heading) {
            @Override
            protected MovementBuild getThis() {
                return this;
            }

            {
                isAtGoal = () -> Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
                    && Odometry.velocity < consideredStoppedVelocity;

                setTotalCorrection(() -> Drive.power(
                    Drive.combineMax(
                        getDrivePowerCorrection(),
                        getHeadingPowerCorrection(),
                        1
                    )
                ));

                setHeadingCorrection(HeadingCorrection.locked);
                setDriveCorrection(DriveCorrection.stopAtTarget);
                setToBrakeAfter();
            }
        };
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
     * @see MovementBuild#stopAtPosition
     */
    public static MovementBuild stopAtPosition(double x, double y, double heading) {
        return stopAtPosition(x, y, heading, 1);
    }
}

