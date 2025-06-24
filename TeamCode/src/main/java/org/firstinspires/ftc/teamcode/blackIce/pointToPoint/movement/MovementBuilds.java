//package org.firstinspires.ftc.teamcode.blackIce.movement;
//
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.DriveCorrection;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.HeadingCorrection;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;
//import org.firstinspires.ftc.teamcode.blackIce.kinematics.Kinematics;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//
//public final class MovementBuilds {
//    /**
//     * Move the robot through a target point without stopping at it.
//     *
//     * <p>
//     * <h5>How does it work?</h5>
//     * <ul>
//     * <li>This method travels towards the point using a simple proportional control (error * constant).</li>
//     * <li>The robot predicts its position based on the braking distance
//     * for determining if it has passed the target. This allows it to quickly change
//     * direction without overshooting.</li>
//     * <li>To determine if it has passed the target, it constructs a plane
//     * perpendicular to the line connecting the previous target and the new target.</li>
//     * </ul>
//     *
//     * @return A {@code Movement} object configured to move through the target.
//     */
//    public static MovementBuild moveThrough(double x, double y, double heading) {
//        return new MovementBuild(x, y, heading) {
//            @Override
//            protected MovementBuild getThis() {
//                return this;
//            }
//
//            {
//                setHeadingCorrection(HeadingCorrection.locked);
////                setDriveCorrection(() ->
////                    DriveVectors.fieldVectorToLocalWheelPowers(
////                        (Target.xError - Odometry.xBrakingDistance),
////                        (Target.yError - Odometry.yBrakingDistance)
////                    )
////                );
//                setDriveCorrection(() ->
//                    DrivePowers.fromFieldVector(
//                        (Target.xError - PinpointLocalizer.xBrakingDistance),
//                        (Target.yError - PinpointLocalizer.yBrakingDistance)
//                    )
//                );
////                setTotalCorrection(() -> Drive.power(
////                    DrivePowers.normalizeDown(DrivePowers.combine(
////                        getDrivePowerCorrection(),
////                        getHeadingPowerCorrection()
////                    ))
////                ));
//                setTotalCorrection((driveCorrection, headingCorrection) ->
//                    driveCorrection
//                        .add(headingCorrection)
//                        .normalizeDown()
//                );
//
//                // Makes robot move always full power
//                setDrivePowerScaling(drivePowers
//                    -> drivePowers.scaleMaxTo(1));
//
//                setToContinuePowerAfter();
//
//                setIsAtGoal(() -> {
//                    boolean isPastPoint;
//                    isPastPoint = (Target.yError - PinpointLocalizer.yBrakingDistance)
//                        * Target.yDelta
//                        + (Target.xError - PinpointLocalizer.xBrakingDistance)
//                        * Target.xDelta < 0;
//
//                    return isPastPoint;
//
////                if (Vector.getMagnitude(xPower, yPower) < 1) {
////                    break;
////                }
//                });
//            }
//        };
//    }
//
//    // TODO create methods that detect being stuck/lining up with walls etc
//
//    /**
//     * Same as {@link MovementBuilds#stopAtPosition(double, double, double)} but with
//     * {@code consideredStoppedVelocity}
//     *
//     * @param consideredStoppedVelocity the velocity that the robot ends the Movement at.
//     * Lower values will make the robot stop more accurately but may waste time.
//     * Higher values can be useful for just a quick slow down.
//     * This value is not needing when holding positions.
//     */
//    public static MovementBuild stopAtPosition(double x, double y, double heading,
//                                                double consideredStoppedVelocity) {
//        return new MovementBuild(x, y, heading) {
//            @Override
//            protected MovementBuild getThis() {
//                return this;
//            }
//
//            {
//                // Makes update function continue supply power to wheels after finished
//                // to hold the end position
//                holdEndPositionAfterFinish = true;
//
//                setIsAtGoal(
//                    () -> Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
//                    && PinpointLocalizer.velocity < consideredStoppedVelocity
//                );
//
////                setTotalCorrection(() -> Drive.power(
////                    DrivePowers.normalizeDown(DrivePowers.combine(
////                        getDrivePowerCorrection(),
////                        getHeadingPowerCorrection()
////                    ))
////                ));
//
//                setTotalCorrection((driveCorrection, headingCorrection) ->
//                    driveCorrection
//                        .add(headingCorrection)
//                        .normalizeDown()
//                );
//
//                setHeadingCorrection(HeadingCorrection.locked);
//                setDriveCorrection(DriveCorrection.stopAtTarget);
//                setToBrakeAfter();
//            }
//        };
//    }
//
//    /**
//     * Move the robot to target point and stop.
//     * <p>
//     * <h5>How does it work?</h5>
//     * <ul>
//     * <li>This method travels towards the point
//     * using a simple proportional control (error * constant).</li>
//     * <li>The robot predicts its position based on the braking distance,
//     * allowing the robot maintain full power for as long as possible,
//     * only braking at the optimal point. The braking distance also prevents overshooting.</li>
//     *
//     * @return A {@code Movement} object configured to stop at the target.
//     * @see MovementBuilds#stopAtPosition
//     */
//    public static MovementBuild stopAtPosition(double x, double y, double heading) {
//        return stopAtPosition(x, y, heading, 1);
//    }
//
//    /**
//     * Move the robot to target point and stop.
//     * <p>
//     * <h5>How does it work?</h5>
//     * <ul>
//     * <li>This method travels towards the point
//     * using a simple proportional control (error * constant).</li>
//     * <li>The robot predicts its position based on the braking distance,
//     * allowing the robot maintain full power for as long as possible,
//     * only braking at the optimal point. The braking distance also prevents overshooting.</li>
//     *
//     * @return A {@code Movement} object configured to stop at the target.
//     * @see MovementBuilds#stopAtPosition
//     */
//    public static MovementBuild stopAtPosition(double x, double y, double heading) {
//        return stopAtPosition(x, y, heading, 1);
//    }
//
//    /**
//     * Same as {@link MovementBuilds#stopAtPosition(double, double, double)} but with
//     * {@code consideredStoppedVelocity}
//     *
//     * @param consideredStoppedVelocity the velocity that the robot ends the Movement at.
//     * Lower values will make the robot stop more accurately but may waste time.
//     * Higher values can be useful for just a quick slow down.
//     * This value is not needing when holding positions.
//     */
//    public static MovementBuild stopAtPosition2(double x, double y, double heading,
//                                               double consideredStoppedVelocity) {
//        return new MovementBuild(x, y, heading) {
//            @Override
//            protected MovementBuild getThis() {
//                return this;
//            }
//
//            {
//                // Makes update function continue supply power to wheels after finished
//                // to hold the end position
//                holdEndPositionAfterFinish = true;
//
//                setIsAtGoal(
//                    () -> Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
//                        && PinpointLocalizer.velocity < consideredStoppedVelocity
//                );
//
//                setDriveCorrection(() -> {
//                    Vector offset = Vector.calculateForwardAndLateralOffset(
//                        new Vector(PinpointLocalizer.x, PinpointLocalizer.y),
//                        PinpointLocalizer.heading,
//                        new Vector(Target.x, Target.y)
//                    );
//                    Vector velocityTargetToStopAtEnd = new Vector(
//                        Kinematics.getCurrentVelocityToStopWithDeceleration(
//                            predictedRobotVelocity.x,
//                            forwardBrakingDeceleration,
//                            offset.x
//                        ),
//                        Kinematics.getCurrentVelocityToStopWithDeceleration(
//                            predictedRobotVelocity.y,
//                            lateralBrakingDeceleration,
//                            offset.y
//                        )
//                    );
//
//                    Vector velocityError = velocityTargetToStopAtEnd.minus(robotVelocity);
//                    Vector velocityPower = velocityError.dot(
//                        1 / MAX_FORWARD_VELOCITY, 1 / MAX_LATERAL_VELOCITY);
//                    return DrivePowers.fromRobotVector(velocityPower);
//                });
//
//                setDrivePowerScaling((drivePowers) -> drivePowers.scale(2));
//
////                setTotalCorrection(() -> Drive.power(
////                    DrivePowers.normalizeDown(DrivePowers.combine(
////                        getDrivePowerCorrection(),
////                        getHeadingPowerCorrection()
////                    ))
////                ));
//
//                setTotalCorrection((driveCorrection, headingCorrection) ->
//                    driveCorrection
//                        .add(headingCorrection)
//                        .normalizeDown()
//                );
//
//                setHeadingCorrection(HeadingCorrection.locked);
//                setDriveCorrection(DriveCorrection.stopAtTarget);
//                setToBrakeAfter();
//            }
//        };
//    }
//
//}
//
