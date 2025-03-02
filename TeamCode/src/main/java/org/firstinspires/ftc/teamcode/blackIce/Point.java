package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

public class Point extends MovementBuilder<Point> {
    private double x;
    private double y;
    private double heading;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    protected Point getThis() {
        return this;
    }

    /**
     * Move the robot through a target point without stopping.
     * <p>
     * To add more customization, use the instance method: {@link Point#moveThrough()}
     */
    public static void moveThrough(double x, double y, double heading) {
        new Point(x, y, heading).moveThrough().build().waitForMovement();
    }

    /**
     * Move the robot through a target point without stopping (keeps the previous heading)
     *
     * @see #moveThrough(double, double, double)
     */
    public static void moveThrough(double x, double y) {
        moveThrough(x, y, Target.previousHeading);
    }

    /**
     * Move the robot to target point and stops.
     * <p>
     * To add more customization, use the instance method: {@link Point#stopAtPosition()}
     */
    public static void stopAtPosition(double x, double y, double heading) {
        new Point(x, y, heading).stopAtPosition().build().waitForMovement();
    }

    /**
     * Move the robot to target point and stops (keeps the previous heading).
     *
     * @see #stopAtPosition(double, double, double)
     */
    public static void stopAtPosition(double x, double y) {
        stopAtPosition(x, y, Target.previousHeading);
    }

    /**
     * Create a new movement. Can be built upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     * <h6>Usage</h6>
     * <pre><code>
     * new Movement(x, y, heading)
     *     .stopAtPosition() // Starts with stopAtPosition
     *     .setMaxVelocity(40) // Sets the maximum velocity to 40 inches/second
     *     // Makes the robot turn instantly instead of over the whole movement
     *     .setHeadingCorrection(movement.headingCorrections.locked)
     *     // Makes the robot hold its position until the vertical slide is raised
     *     .setMovementExit(() -> slide.isRaised())
     *     .run();
     * </code></pre>
     *
     * @see Point#Point(double, double)
     */
    public Point(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;

        setMovementExit(() -> !Target.isNotWithinErrorMargin(Target.defaultErrorMargin));
    }

    public Point(Point copyFrom) {
        this.x = copyFrom.x;
        this.y = copyFrom.y;
        this.heading = copyFrom.heading;

        this.copyProperties(copyFrom);
    }

    public Movement build() {
        return new Movement(this::isFinished, this::start, this::finish, this::update);
    }

    private void finish() {
        if (brakeAfter) {
            Drive.zeroPowerBrakeMode();
        }
        else {
            Drive.zeroPowerFloatMode();
        }
        if (!continuePowerAfter) {
            Drive.zeroPower();
        }
    }

    private void update() {
        Target.updatePosition();
        moveTowardTarget();
    }

    private void start() {
        Target.setTarget(heading, x, y);
        timer.reset();
    }

    private boolean isFinished() {
        return !Follower.opMode.opModeIsActive() || movementExit.condition() || timer.seconds() >= timeoutSeconds;
    }

    private void moveTowardTarget() {
//        double velocityMult = (
//            maxVelocity < 100
//                && (Math.abs(Target.xError - Odometry.xBrakingDistance) > 1
//                || Math.abs(Target.yError - Odometry.yBrakingDistance) > 1))
//            ? (maxVelocity / Odometry.velocity) / 2 : 1;
//        double headingMult = (Odometry.headingVelocity > maxHeadingVelocity)
//            ? (maxHeadingVelocity / Odometry.headingVelocity) : 1;
//
//        Follower.telemetry.addData("velocityMult", velocityMult);
//        Follower.telemetry.addData("maxVel", maxVelocity);
//        Follower.telemetry.addData("velocity", Odometry.velocity);
//        Follower.telemetry.update();

        // if it is not braking (error-brake greater than zero)
        // 0.02 * velocity_error * abs(velocity_error)

//        power = kP * adjusted_error
//        print(power)
//        if abs(power) > 1:
//        power += 0.02 * velocity_error * abs(velocity_error)
//    else:
//        power += 0.0005 * (0 - current_velocity) * abs(current_velocity)

        // if it is not braking (error-brake greater than zero)
        // 0.02 * velocity_error * abs(velocity_error)

//        power = kP * adjusted_error

        Drive.power(Drive.combineMax(
            driveCorrection.calculateDrivePowers(),
            HeadingCorrection.getWheelPowers(headingCorrection),
            maxPower
        ));
    }


    /**
     * Create a new movement (uses previous heading).
     * Can be build upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     *
     * @see Point#Point(double, double, double)
     */
    public Point(double x, double y) {
        this(x, y, Target.previousHeading);
    }

    /**
     * <h5>To fix underline, add arguments</h5>
     * <p>
     * Creates an empty {@link Point}.
     */
    protected Point() {
    }


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
    public Point moveThrough() {
        double targetYError = Target.previousY - Target.y;
        double targetXError = Target.previousX - Target.x;
        double xSign = Math.signum(targetXError);
        double ySign = Math.signum(targetYError);
        double slope = (Target.x == Target.previousX) ? 0 : targetYError / targetXError;

        return getThis()
            .setHeadingCorrection(HeadingCorrection.locked)
            .setDriveCorrection(DriveCorrection.proportional)
            .continuePowerAfter()
            .setMovementExit(() -> {
                double predictedXError = Target.xError - Odometry.xBrakingDistance;
                double predictedYError = Target.yError - Odometry.yBrakingDistance;

                if (Target.x == Target.previousX) {
                    return ySign * predictedYError >= 0;
                }

                return -xSign * predictedXError <= slope * xSign * predictedYError;
            });
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
     * @see Point#stopAtPosition
     */
    public Point stopAtPosition() {
        return getThis()
            .setMovementExit(() ->
                Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
                    && Odometry.velocity < consideredStoppedVelocity)
            .setHeadingCorrection(HeadingCorrection.locked)
            .setDriveCorrection(DriveCorrection.stopAtTarget);
    }


//    @NonNull
//    @Override
//    public Movement clone() {
//        try {
//            return (Movement) super.clone();
//        } catch (CloneNotSupportedException e) {
//            throw new AssertionError(); // This should never happen since we implement Cloneable
//        }
//    }

}
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//        private VoltageSensor myControlHubVoltageSensor;
//        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
// presentVoltage = myControlHubVoltageSensor.getVoltage();

// TODO test clamp and turnToFaceTarget in order to turn and move faster vvv
//    public void moveTowardTarget() {
//        double[] headingPowers = driveCorrection.calculateDrivePowers();
//        double[] drivePowers = HeadingCorrections.getWheelPowers(headingCorrection);
//        Drive.power(
//            Util.normalize(new double[]{ // make this a function in robot.drive TODO
//                Util.clampPower(headingPowers[0] + drivePowers[0]),
//                Util.clampPower(headingPowers[1] + drivePowers[1]),
//                Util.clampPower(headingPowers[2] + drivePowers[2]),
//                Util.clampPower(headingPowers[3] + drivePowers[3])
//            })
//        );
//    }
// moves faster but not necessarily on the path/accurately

//    public Movement moveThrough() {
//        return this
//            .setHeadingCorrection(HeadingCorrections.turnOverMovement)
//            .setDriveCorrection(DriveCorrections.proportional)
//            .setMovementExit(() -> {
//                boolean pastY;
//                boolean pastX;
//
//                if (Target.previousY < Target.y) {
//                    pastY = Odometry.y > Target.y - Odometry.yBrakingDistance;
//                }
//                else if (Target.previousY == Target.y) {
//                    pastY = true;
//                }
//                else {
//                    pastY = Odometry.y < Target.y - Odometry.yBrakingDistance;
//                }
//
//                if (Target.previousX < Target.x) {
//                    pastX = Odometry.x > Target.x - Odometry.xBrakingDistance;
//                }
//                else if (Target.previousX == Target.x) {
//                    pastX = true;
//                }
//                else {
//                    pastX = Odometry.x < Target.x - Odometry.xBrakingDistance;
//                }
//                return pastY && pastX;
//
//                // have parallel to previous point
//            });
//    }

//public Movement moveTo(double brakePercent) {
//    return this
//        .setHeadingCorrection(HeadingCorrections.turnOverMovement)
//        .setDriveCorrection(() -> new double[]{
//            Target.xError - Odometry.xBrakingDistance * brakePercent,
//            Target.yError - Odometry.yBrakingDistance * brakePercent,
//        })
//        .setMovementExit(() ->
//            Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin));
//}
