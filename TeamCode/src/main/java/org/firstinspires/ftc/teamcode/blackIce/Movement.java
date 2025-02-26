package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

// kye_anderson_2009, alexreman45
public class Movement {
    /**
     * Move the robot through a target point without stopping.
     * <p>
     * To add more customization, use the instance method: {@link Movement#moveThrough()}
     */
    public static void moveThrough(double x, double y, double heading) {
        new Movement(x, y, heading).moveThrough().run();
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
        new Movement(x, y, heading).stopAtPosition().run();
    }

    /**
     * Move the robot to target point and stops (keeps the previous heading).
     *
     * @see #stopAtPosition(double, double, double)
     */
    public static void stopAtPosition(double x, double y) {
        stopAtPosition(x, y, Target.previousHeading);
    }

    private HeadingCorrection headingCorrection;
    private DriveCorrection driveCorrection;
    private MovementExit movementExit;

    private double maxPower = 1;
    private double maxVelocity = 100; // inch/second
    private double maxHeadingVelocity = 999; // degrees/second
    private double consideredStoppedVelocity = 1;

    private boolean brakeAfter = true;
    private boolean continuePowerAfter = true;

    public Path path = null;

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

    /**
     * Create a new path movement.
     * Can be build upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     *
     * @see Movement#Movement(double, double, double)
     */
    public Movement(Path path) {
        this.path = path;
    }

    // Credit to alexreman45 for supporting and finding bug of NullExpestionError about Bezier Curves
    /**
     * <h5>To fix underline, add arguments</h5>
     * <p>
     * Creates an empty {@link Movement}.
     */
    protected Movement() {
    }

    /**
     * Turns on zero power brake mode after reaching the target.
     */
    public Movement brakeAfter() {
        brakeAfter = true;
        continuePowerAfter = false;
        return this;
    }

    /**
     * Turns on zero power float mode after reaching the target.
     * This makes the robot glide with its momentum.
     */
    public Movement floatAfter() {
        brakeAfter = false;
        continuePowerAfter = false;
        return this;
    }

    /**
     * Make the robot continue the supplied power to the wheels after reaching the target.
     */
    public Movement continuePowerAfter() {
        brakeAfter = false;
        continuePowerAfter = true;
        return this;
    }

    /**
     * Set the maximum velocity that the robot considers at rest.
     * Useful for different accuracies of {@link Movement#stopAtPosition}.
     */
    public Movement setConsideredStoppedVelocity(double newConsideredStoppedVelocity) {
        consideredStoppedVelocity = newConsideredStoppedVelocity;
        return this;
    }

    /**
     * Set a maximum velocity the robot can travel (is not perfectly accurate).
     *
     * @param newMaxVelocity inches/second (312 rpm goes 40-60 inches/second)
     */
    public Movement setMaxVelocity(double newMaxVelocity) {
        maxVelocity = newMaxVelocity;
        return this;
    }

    /**
     * Set a maximum velocity the robot can turn (is not perfectly accurate).
     *
     * @param newMaxHeadingVelocity degrees/second
     */
    public Movement setMaxHeadingVelocity(double newMaxHeadingVelocity) {
        maxHeadingVelocity = newMaxHeadingVelocity;
        return this;
    }

    /**
     * Set the kind of {@link HeadingCorrection} that is responsible for turning the robot.
     * <h6>Usage</h6>
     * {@code .setHeadingCorrection(HeadingCorrection.x)}
     * where x is the type of heading correction.
     * <p>
     */
    public Movement setHeadingCorrection(HeadingCorrection newHeadingCorrection) {
        headingCorrection = newHeadingCorrection;
        return this;
    }

    /**
     * Set the kind of {@link DriveCorrection}
     * that is responsible for moving the robot toward the target.
     * <h6>Usage</h6>
     * {@code .setDriveCorrection(DriveCorrection.x)}
     * where x is the type of drive correction.
     * <p>
     */
    public Movement setDriveCorrection(DriveCorrection newDriveCorrection) {
        driveCorrection = newDriveCorrection;
        return this;
    }

    /**
     * Set the kind of {@link MovementExit#condition()}
     * that is responsible for telling the movement when its reached its goal.
     *
     * @param newMovementExit {@code .setMovementExit(() -> {return ...})}
     * (has to return a boolean)
     *
     * <h6>Examples</h6>
     * Continues to hold the position until it is within the error margin and the slide is raised:
     * <pre><code>
     * .setMovementExit(() -> Target.isWithinBrakingErrorMargin() && slide.isRaised)}
     * </code></pre>
     */
    public Movement setMovementExit(MovementExit newMovementExit) {
        movementExit = newMovementExit;
        return this;
    }

    /**
     * Set the maximum power the robot can move at.
     *
     * @param newMaxPower A number 0 to 1. {@code 0.5} -> 50% power
     */
    public Movement setMaxPower(double newMaxPower) {
        maxPower = newMaxPower;
        return this;
    }

    private void moveTowardTarget() {
        double velocityMult = (Odometry.velocity > maxVelocity)
            ? (maxVelocity / Odometry.velocity) * 0.5 : 1;
        double headingMult = (Odometry.headingVelocity > maxHeadingVelocity)
            ? (maxHeadingVelocity / Odometry.headingVelocity) : 1;

        Drive.power(Drive.combineMax(
            Drive.multiply(driveCorrection.calculateDrivePowers(), velocityMult),
            Drive.multiply(HeadingCorrection.getWheelPowers(headingCorrection), headingMult),
            maxPower
        ));
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
    public Movement moveThrough() {
        double targetYError = Target.previousY - Target.y;
        double targetXError = Target.previousX - Target.x;
        double xSign = Math.signum(targetXError);
        double ySign = Math.signum(targetYError);
        double slope = (Target.x == Target.previousX) ? 0 : targetYError / targetXError;

        return this
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
     * @see Movement#stopAtPosition
     */
    public Movement stopAtPosition() {
        return this
            .setMovementExit(() ->
                Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
                    && Odometry.velocity < consideredStoppedVelocity)
            .setHeadingCorrection(HeadingCorrection.turnOverMovement)
            .setDriveCorrection(DriveCorrection.stopAtTarget);
    }

    /**
     * Run the movement (has a 5 second timeout).
     *
     * @see Movement#runTimeout
     */
    public void run() {
        runTimeout(5);
    }

    public boolean isCompleted() {
        return robot.isNotInterrupted()
                && !movementExit.condition()
            }
    /**
     * Run the movement with a timeout.
     *
     * @param timeout The timeout seconds.
     */
    public void runTimeout(double timeout) {
        // Hacky
        if (path != null) {
            path.runCurve(this);
            return;
        }

        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        Robot robot = Robot.getInstance();
        robot.loopUpdate();
        while (
            robot.isNotInterrupted()
                && !movementExit.condition()
                && timer.seconds() < timeout
        ) {
            moveTowardTarget();

            //robot.telemetry.addData("VEL", Odometry.velocity);
            robot.opMode.telemetry.addData("x pos", Odometry.x);
            robot.opMode.telemetry.addData("y pos", Odometry.y);
//            robot.telemetry.addData("x braking distance", Odometry.xBrakingDistance);
//            robot.telemetry.addData("y braking distance", Odometry.yBrakingDistance);
//            robot.telemetry.addData("x vel", builder.robot.odometry.xVelocity);
//            robot.telemetry.addData("y vel", builder.robot.odometry.yVelocity);
            robot.opMode.telemetry.update();

            robot.loopUpdate();
        }

        if (brakeAfter) {
            Drive.zeroPowerBrakeMode();
            Drive.zeroPower();
        }
        else if (!continuePowerAfter) {
            Drive.zeroPowerFloatMode();
            Drive.zeroPower();
        }
    }
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
