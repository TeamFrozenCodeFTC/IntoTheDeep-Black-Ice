package org.firstinspires.ftc.teamcode.blackIce;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;


public abstract class MovementBuilder
    <SubclassType extends MovementBuilder<SubclassType>>
    implements Cloneable
{
    protected HeadingCorrection headingCorrection;
    protected DriveCorrection driveCorrection;
    protected Condition movementExit;

    protected double maxPower = 1;
    protected double maxVelocity = 100; // inch/second
    protected double maxHeadingVelocity = 999; // degrees/second
    protected double consideredStoppedVelocity = 1;
    protected double timeoutSeconds = 5;

    protected boolean brakeAfter = true;
    protected boolean continuePowerAfter = false;

    public SubclassType copyProperties(MovementBuilder<?> properties) {
        SubclassType this_ = getThis();
        this_.brakeAfter = properties.brakeAfter;
        this_.continuePowerAfter = properties.continuePowerAfter;
        return this_
            .setConsideredStoppedVelocity(properties.consideredStoppedVelocity)
            .setMaxPower(properties.maxPower)
            .setMaxHeadingVelocity(properties.maxHeadingVelocity)
            .setTimeoutSeconds(properties.timeoutSeconds)
            .setMaxVelocity(properties.maxVelocity)
            .setDriveCorrection(properties.driveCorrection)
            .setMovementExit(properties.movementExit)
            .setHeadingCorrection(properties.headingCorrection);
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
     * Useful for different accuracies of {@link Point#stopAtPosition}.
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

    @NonNull
    @Override
    public SubclassType clone() {
        try {
            @SuppressWarnings("unchecked")
            SubclassType cloned = (SubclassType) super.clone(); // Create shallow copy

            return cloned;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // This should never happen since we implement Cloneable//            throw new AssertionError(); // This should never happen since we implement Cloneable
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
