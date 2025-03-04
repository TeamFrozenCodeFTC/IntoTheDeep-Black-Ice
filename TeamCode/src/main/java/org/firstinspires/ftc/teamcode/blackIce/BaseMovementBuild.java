package org.firstinspires.ftc.teamcode.blackIce;

import androidx.annotation.NonNull;


public abstract class BaseMovementBuild
    <SubclassType extends BaseMovementBuild<SubclassType>>
    implements Cloneable
{
    protected abstract Movement build();

    protected HeadingCorrection headingCorrection;
    protected DriveCorrection driveCorrection;
    protected Condition movementExit = () -> false;

    protected double maxPower = 1;
    protected double maxTurnPower = 1;
    protected double maxVelocity = 100; // inch/second
    protected double maxHeadingVelocity = 999; // degrees/second
    protected double timeoutSeconds = 5;

    /**
     * Copy the properties of another MovementBuild.
     * <p>
     * Note: only copies point-to-point properties, not path properties.
     */
    public SubclassType copyProperties(BaseMovementBuild<?> properties) {
        SubclassType this_ = getThis();
        return this_
            .setMaxPower(properties.maxPower)
            .setMaxHeadingVelocity(properties.maxHeadingVelocity)
            .setTimeoutSeconds(properties.timeoutSeconds)
            .setMaxVelocity(properties.maxVelocity)
            .setDriveCorrection(properties.driveCorrection)
            .setMovementExit(properties.movementExit)
            .setHeadingCorrection(properties.headingCorrection);
    }

    /**
     * Set an extra {@link Condition} that that has to be satisfied before the movement can exit.
     *
     * @param newMovementExit {@code .setMovementExit(() -> {return ...})} (has to return a boolean)
     * The movement will continue holding while the condition is false.
     * Returning true will allow the movement to exit.
     *
     * <h6>Examples</h6>
     * Continues to hold the position until linear slide is raised:
     * <p>
     * {@code .setMovementExit(() -> {return linearSlide.isRaised()})}
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
     * Set the Movement's timeout in seconds. Default is 5 seconds.
     */
    public SubclassType setTimeoutSeconds(double newTimeoutSeconds) {
        timeoutSeconds = newTimeoutSeconds;
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
