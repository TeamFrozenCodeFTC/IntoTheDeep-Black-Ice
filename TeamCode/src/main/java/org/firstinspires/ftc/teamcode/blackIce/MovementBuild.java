package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MovementBuild extends BaseMovementBuild<MovementBuild> {
    private double x;
    private double y;
    private double heading;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean brakeAfter = true;
    private boolean continuePowerAfter = false;

//    /**
//     * Move the robot through a target point without stopping.
//     * <p>
//     * To add more customization, use the instance method: {@link Point#moveThrough()}
//     */
//    public static void moveThrough(double x, double y, double heading) {
//        new Point(x, y, heading).moveThrough().build().waitForMovement();
//    }
//
//    /**
//     * Move the robot through a target point without stopping (keeps the previous heading)
//     *
//     * @see #moveThrough(double, double, double)
//     */
//    public static void moveThrough(double x, double y) {
//        moveThrough(x, y, Target.previousHeading);
//    }
//
//    /**
//     * Move the robot to target point and stops.
//     * <p>
//     * To add more customization, use the instance method: {@link Point#stopAtPosition()}
//     */
//    public static void stopAtPosition(double x, double y, double heading) {
//        new Point(x, y, heading).stopAtPosition().build().waitForMovement();
//    }
//
//    /**
//     * Move the robot to target point and stops (keeps the previous heading).
//     *
//     * @see #stopAtPosition(double, double, double)
//     */
//    public static void stopAtPosition(double x, double y) {
//        stopAtPosition(x, y, Target.previousHeading);
//    }

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
     * Default is stopAtPosition
     *
     * @see MovementBuild#MovementBuild(double, double)
     */
    public MovementBuild(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    private boolean keepPreviousHeading = false;

    /**
     * Create a new movement (uses previous heading).
     * Can be build upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     *
     * @see MovementBuild#MovementBuild(double, double, double)
     */
    public MovementBuild(double x, double y) {
        this.x = x;
        this.y = y;
        this.keepPreviousHeading = true;
    }

    // have positions reusable?
//    public Point(Point copyFrom) {
//        this.x = copyFrom.x;
//        this.y = copyFrom.y;
//        this.heading = copyFrom.heading;
//
//        this.copyProperties(copyFrom);
//    }

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

    void start() {
        if (keepPreviousHeading) {
            Target.setTarget(x, y);
        }
        else {
            Target.setTarget(heading, x, y);
        }
        timer.reset();
    }

    abstract boolean isAtGoal();

    private boolean isFinished() {
        return !Follower.opMode.opModeIsActive()
            || (movementExit.condition() && isAtGoal())
            || timer.seconds() >= timeoutSeconds;
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
     * <h5>To fix underline, add arguments</h5>
     * <p>
     * Creates an empty {@link MovementBuild}.
     */
    protected MovementBuild() {
    }

    /**
     * Turns on zero power float mode after reaching the target.
     * This makes the robot glide with its momentum.
     */
    public MovementBuild floatAfter() {
        brakeAfter = false;
        continuePowerAfter = false;
        return getThis();
    }

    /**
     * Turns on zero power brake mode after reaching the target.
     */
    public MovementBuild brakeAfter() {
        brakeAfter = true;
        continuePowerAfter = false;
        return getThis();
    }

    /**
     * Make the robot continue the supplied power to the wheels after reaching the target.
     */
    public MovementBuild continuePowerAfter() {
        brakeAfter = false;
        continuePowerAfter = true;
        return getThis();
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
