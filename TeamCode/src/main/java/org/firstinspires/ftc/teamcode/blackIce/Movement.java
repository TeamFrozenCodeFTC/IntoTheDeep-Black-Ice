package org.firstinspires.ftc.teamcode.blackIce;

public class Movement extends Follower<Movement> {
    @Override
    protected Movement getThis() {
        return this;
    }

    /**
     * Move the robot through a target point without stopping.
     * <p>
     * To add more customization, use the instance method: {@link Movement#moveThrough()}
     */
    public static void moveThrough(double x, double y, double heading) {
        new Movement(x, y, heading).moveThrough().waitForMovement();
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
     * To add more customization, use the instance method: {@link Movement#stopAtPosition()}
     */
    public static void stopAtPosition(double x, double y, double heading) {
        new Movement(x, y, heading).stopAtPosition().waitForMovement();
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
     * Create a new movement. Can be build upon to add more functionality and customization.
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
     * @see Movement#Movement(double, double)
     */
    public Movement(double x, double y, double heading) {
        Target.setTarget(heading, x, y);

        setMovementExit(() -> !Target.isNotWithinErrorMargin(Target.defaultErrorMargin));

        timer.reset();
    }

    /**
     * Create a new movement (uses previous heading).
     * Can be build upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     *
     * @see Movement#Movement(double, double, double)
     */
    public Movement(double x, double y) {
        this(x, y, Target.previousHeading);
    }

//    private PathFollower pathFollower;

//    /**
//     * Create a new path movement.
//     * Can be build upon to add more functionality and customization.
//     * Don't forget to call {@code .run()} after you construct your movements.
//     *
//     * @see Movement#Movement(double, double, double)
//     */
//    public Movement(Path path) {
//        PathFollower pathFollower = new PathFollower(path);
//        this.path = path;
//    }

    /**
     * <h5>To fix underline, add arguments</h5>
     * <p>
     * Creates an empty {@link Movement}.
     */
    protected Movement() {
    }



//    private void moveTowardTarget() {
//        double velocityMult = (Odometry.velocity > maxVelocity)
//            ? (maxVelocity / Odometry.velocity) * 0.5 : 1;
//        double headingMult = (Odometry.headingVelocity > maxHeadingVelocity)
//            ? (maxHeadingVelocity / Odometry.headingVelocity) : 1;
//
//        Drive.power(Drive.combineMax(
//            Drive.multiply(driveCorrection.calculateDrivePowers(), velocityMult),
//            Drive.multiply(HeadingCorrection.getWheelPowers(headingCorrection), headingMult),
//            maxPower
//        ));
//    }


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
