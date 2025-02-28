package org.firstinspires.ftc.teamcode.blackIce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathFollower;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

public abstract class Follower<SubclassType extends Follower<SubclassType>> {
    HeadingCorrection headingCorrection;
    DriveCorrection driveCorrection;
    Condition movementExit;

    double maxPower = 1;
    double maxVelocity = 100; // inch/second
    double maxHeadingVelocity = 999; // degrees/second
    double consideredStoppedVelocity = 1;
    double timeoutSeconds = 5;
    private static LinearOpMode opMode;

    final ElapsedTime timer = new ElapsedTime();

    boolean brakeAfter = true;
    boolean continuePowerAfter = false;

    public SubclassType copyProperties(SubclassType properties) {
        properties.brakeAfter = brakeAfter;
        properties.continuePowerAfter = continuePowerAfter;
        return properties
            .setConsideredStoppedVelocity(consideredStoppedVelocity)
            .setMaxPower(maxPower)
            .setMaxHeadingVelocity(maxHeadingVelocity)
            .setTimeoutSeconds(timeoutSeconds)
            .setMaxVelocity(maxVelocity)
            .setDriveCorrection(driveCorrection)
            .setMovementExit(movementExit)
            .setHeadingCorrection(headingCorrection);
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
    public SubclassType moveThrough() {
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
     * @see Movement#stopAtPosition
     */
    public SubclassType stopAtPosition() {
        return getThis()
            .setMovementExit(() ->
                Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)
                    && Odometry.velocity < consideredStoppedVelocity)
            .setHeadingCorrection(HeadingCorrection.turnOverMovement)
            .setDriveCorrection(DriveCorrection.stopAtTarget);
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
     * Wait for the Movement to be completed.
     */
    public void waitForMovement() {
        waitForMovement(() -> true, () -> {});
    }

    /**
     * Wait for the Movement to be completed.
     *
     * @param extraCondition False will continue holding the position,
     *                       true will allow exit
     * <p>
     * If no extra condition is needed see {@link Movement#waitForMovement()}
     */
    public void waitForMovement(Condition extraCondition) {
        waitForMovement(extraCondition, () -> {});
    }

    /**
     * Wait for the Movement to be completed with an extraCondition .
     *
     * @param extraCondition False will continue holding the position,
     *                       True will allow exit
     * @param updateHardware A function that updates robot hardware. For example,
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If no updating hardware and no extraCondition is needed see {@link Movement#waitForMovement()}
     */
    public void waitForMovement(Condition extraCondition, UpdateHardware updateHardware) {
        timer.reset();

        Target.updatePosition();
        while (isNotCompleted() && extraCondition.condition()) {
            update();
            updateHardware.update();
        }

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

    /**
     * Wait for the Movement to be completed with an extraCondition .
     *
     * @param updateHardware A function that updates robot hardware. For example,
     *                       when a linear slide reaches a certain height, a claw opens.
     * <p>
     * If updating hardware is not needed see {@link Movement#waitForMovement()}
     */
    public void waitForMovement(UpdateHardware updateHardware) {
        waitForMovement(() -> true, updateHardware);
    }

    public boolean isNotCompleted() {
        return opMode.opModeIsActive()
            && !movementExit.condition()
            && timer.seconds() < timeoutSeconds;
    }

    public void update() {
        Target.updatePosition();
        moveTowardTarget();
    }

    public static MultipleTelemetry telemetry;

    public static void init(LinearOpMode opMode) {
        Follower.opMode = opMode;
        Odometry.init(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    /**
     * Turns on zero power brake mode after reaching the target.
     */
    public SubclassType brakeAfter() {
        brakeAfter = true;
        continuePowerAfter = false;
        return getThis();
    }

    /**
     * Set the kind of {@link Condition#condition()}
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
    public SubclassType setMovementExit(Condition newMovementExit) {
        movementExit = newMovementExit;
        return getThis();
    }

    /**
     * Set the kind of {@link HeadingCorrection} that is responsible for turning the robot.
     * <h6>Usage</h6>
     * {@code .setHeadingCorrection(HeadingCorrection.x)}
     * where x is the type of heading correction.
     * <p>
     */
    public SubclassType setHeadingCorrection(HeadingCorrection newHeadingCorrection) {
        headingCorrection = newHeadingCorrection;
        return getThis();
    }

    /**
     * Set the kind of {@link DriveCorrection}
     * that is responsible for moving the robot toward the target.
     * <h6>Usage</h6>
     * {@code .setDriveCorrection(DriveCorrection.x)}
     * where x is the type of drive correction.
     * <p>
     */
    public SubclassType setDriveCorrection(DriveCorrection newDriveCorrection) {
        driveCorrection = newDriveCorrection;
        return getThis();
    }

    /**
     * Set the maximum power the robot can move at.
     *
     * @param newMaxPower A number 0 to 1. {@code 0.5} -> 50% power
     */
    public SubclassType setMaxPower(double newMaxPower) {
        maxPower = newMaxPower;
        return getThis();
    }

    /**
     * Turns on zero power float mode after reaching the target.
     * This makes the robot glide with its momentum.
     */
    public SubclassType floatAfter() {
        brakeAfter = false;
        continuePowerAfter = false;
        return getThis();
    }

    /**
     * Make the robot continue the supplied power to the wheels after reaching the target.
     */
    public SubclassType continuePowerAfter() {
        brakeAfter = false;
        continuePowerAfter = true;
        return getThis();
    }

    /**
     * Set the Movement's timeout in seconds. Default is 5 seconds.
     */
    public SubclassType setTimeoutSeconds(double newTimeoutSeconds) {
        timeoutSeconds = newTimeoutSeconds;
        return getThis();
    }

    /**
     * Set the maximum velocity that the robot considers at rest.
     * Useful for different accuracies of {@link Movement#stopAtPosition}.
     */
    public SubclassType setConsideredStoppedVelocity(double newConsideredStoppedVelocity) {
        consideredStoppedVelocity = newConsideredStoppedVelocity;
        return getThis();
    }

    /**
     * Set a maximum velocity the robot can travel (is not perfectly accurate).
     *
     * @param newMaxVelocity inches/second (312 rpm goes 40-60 inches/second)
     */
    public SubclassType setMaxVelocity(double newMaxVelocity) {
        maxVelocity = newMaxVelocity;
        return getThis();
    }

    /**
     * Set a maximum velocity the robot can turn (is not perfectly accurate).
     *
     * @param newMaxHeadingVelocity degrees/second
     */
    public SubclassType setMaxHeadingVelocity(double newMaxHeadingVelocity) {
        maxHeadingVelocity = newMaxHeadingVelocity;
        return getThis();
    }

    protected abstract SubclassType getThis();
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
