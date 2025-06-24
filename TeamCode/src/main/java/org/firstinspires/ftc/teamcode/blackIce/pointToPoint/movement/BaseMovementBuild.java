//package org.firstinspires.ftc.teamcode.blackIce.movement;
//
//import androidx.annotation.NonNull;
//
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.CombineCorrectionPowers;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.DriveCorrection;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.Function;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.HeadingCorrection;
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;
//
///**
// * Handles all of the ".set" build methods and data for Movements and Paths.
// */
//public abstract class BaseMovementBuild
//    <SubclassType extends BaseMovementBuild<SubclassType>>
//    implements Cloneable
//{
//    protected abstract Movement build();
//
//    private HeadingCorrection headingCorrection;
//    private DriveCorrection driveCorrection;
//    protected Condition isAtGoal = () -> true;
//
//    protected double maxPower = 1;
//    protected double maxTurnPower = 1;
//    protected double maxVelocity = 100; // inch/second // get deceleration rates
//    protected double maxHeadingVelocity = 999; // degrees/second
//    protected double timeoutSeconds = 5;
//
//    protected boolean brakeAfter = false;
//    protected boolean continuePowerAfter = false;
//
//    private CombineCorrectionPowers totalCorrection;
//    private Function<DrivePowers, DrivePowers> drivePowerScaling =
//        (drivePowers) -> drivePowers;
//
//    /**
//     * Copy the properties of another MovementBuild.
//     * <p>
//     * Note: only copies point-to-point properties, not path properties.
//     */
//    public SubclassType copyProperties(BaseMovementBuild<?> properties) {
//        SubclassType this_ = getThis();
//        this_.brakeAfter = properties.brakeAfter;
//        this_.continuePowerAfter = properties.brakeAfter;
//        return this_
//            .setMaxPower(properties.maxPower)
//            .setMaxHeadingVelocity(properties.maxHeadingVelocity)
//            .setTimeoutSeconds(properties.timeoutSeconds)
//            .setMaxVelocity(properties.maxVelocity)
//            .setDriveCorrection(properties.driveCorrection)
//            .setIsAtGoal(properties.isAtGoal)
//            .setHeadingCorrection(properties.headingCorrection)
//            .setTotalCorrection(properties.totalCorrection)
//            .setDrivePowerScaling(properties.drivePowerScaling);
//    }
//
//    /**
//     * Turns on zero power float mode after reaching the target.
//     * This makes the robot glide with its momentum.
//     */
//    public SubclassType setToFloatAfter() {
//        brakeAfter = false;
//        continuePowerAfter = false;
//        return getThis();
//    }
//
//    /**
//     * Turns on zero power brake mode and brakes after reaching the target.
//     */
//    public SubclassType setToBrakeAfter() {
//        brakeAfter = true;
//        continuePowerAfter = false;
//        return getThis();
//    }
//
//    /**
//     * Make the robot continue the supplied power to the wheels after reaching the target.
//     */
//    public SubclassType setToContinuePowerAfter() {
//        brakeAfter = false;
//        continuePowerAfter = true;
//        return getThis();
//    }
//
//    private DrivePowers getDrivePowerCorrection() {
//        return drivePowerScaling.run(driveCorrection.calculateDrivePowers());
//    }
//
//    private DrivePowers getHeadingPowerCorrection() {
//        return DrivePowers.turnCounterclockwise(headingCorrection.calculateTurnPower());
//    }
//
//    /**
//     * Moves toward the target.
//     */
//    public void moveTowardTarget() {
//        totalCorrection.combineCorrectionPowers(
//            getDrivePowerCorrection(),
//            getHeadingPowerCorrection()
//        ).applyPowers();
//    }
//
//    /**
//     * Set the function that combines drive correction and heading correction.
//     *
//     * <pre><code>
//     * // Example: Scales the the drive power so that at least one wheel is working at full power.
//     * .setTotalWheelPowers(() -> Drive.power(
//     *     Drive.combineMax(
//     *         getDriveCorrection(),
//     *         Drive.turnCounterclockwise(headingCorrection.calculateTurnPower()),
//     *         1
//     *     )
//     * ))
//     * </code></pre>
//     */
//    public SubclassType setTotalCorrection(CombineCorrectionPowers function) {
//        totalCorrection = function;
//        return getThis();
//    }
//
//    /**
//     * Set the function for scaling the wheel powers.
//     * <pre><code>
//     * // Example: Scales the the drive power so that at least one wheel is working at full power.
//     * .setDrivePowerScaling(wheelPowers -> Vector.scaleToMax(driveDirection, 1))
//     * // General Format:
//     * .setDrivePowerScaling((double[FL, BL, FR, BR] wheelPowers) -> {return Vector.scale(...)})
//     * // the wheelPower arrays usually look something like {2.3, 5.5, 5.5, 2.3} so it is important
//     * // to scale them or else all the wheels would go at full power.
//     * </code></pre>
//     * See {@link Vector} class for common scaling methods.
//     */
//    public SubclassType setDrivePowerScaling(Function<DrivePowers, DrivePowers> function) {
//        drivePowerScaling = function;
//        return getThis();
//    }
//
//    /**
//     * Set the {@link Condition} that tells when the movement is finished.
//     */
//    public SubclassType setIsAtGoal(Condition condition) {
//        isAtGoal = condition;
//        return getThis();
//    }
//
//    /**
//     * Set the kind of {@link HeadingCorrection} that is responsible for turning the robot.
//     * <h6>Usage</h6>
//     * {@code .setHeadingCorrection(HeadingCorrection.x)}
//     * where x is the type of heading correction.
//     * <p>
//     */
//    public SubclassType setHeadingCorrection(HeadingCorrection newHeadingCorrection) {
//        headingCorrection = newHeadingCorrection;
//        return getThis();
//    }
//
//    /**
//     * Set the kind of {@link DriveCorrection}
//     * that is responsible for moving the robot toward the target.
//     * <h6>Usage</h6>
//     * {@code .setDriveCorrection(DriveCorrection.x)}
//     * where x is the type of drive correction.
//     * <p>
//     */
//    public SubclassType setDriveCorrection(DriveCorrection newDriveCorrection) {
//        driveCorrection = newDriveCorrection;
//        return getThis();
//    }
//
//    /**
//     * Set the Movement's timeout in seconds. Default is 5 seconds.
//     */
//    public SubclassType setTimeoutSeconds(double newTimeoutSeconds) {
//        timeoutSeconds = newTimeoutSeconds;
//        return getThis();
//    }
//
//    /**
//     * Set the maximum power the robot can move at.
//     *
//     * @param newMaxPower A number 0 to 1. {@code 0.5} -> 50% power
//     */
//    public SubclassType setMaxPower(double newMaxPower) {
//        maxPower = newMaxPower;
//        return getThis();
//    }
//
//    /**
//     * Set a maximum velocity the robot can travel (is not perfectly accurate).
//     *
//     * @param newMaxVelocity inches/second (312 rpm goes 40-60 inches/second)
//     */
//    public SubclassType setMaxVelocity(double newMaxVelocity) {
//        maxVelocity = newMaxVelocity;
//        return getThis();
//    }
//
//    /**
//     * Set a maximum velocity the robot can turn (is not perfectly accurate).
//     *
//     * @param newMaxHeadingVelocity degrees/second
//     */
//    public SubclassType setMaxHeadingVelocity(double newMaxHeadingVelocity) {
//        maxHeadingVelocity = newMaxHeadingVelocity;
//        return getThis();
//    }
//
//    protected abstract SubclassType getThis();
//
//    @NonNull
//    @Override
//    public SubclassType clone() {
//        try {
//            @SuppressWarnings("unchecked")
//            SubclassType cloned = (SubclassType) super.clone(); // Create shallow copy
//
//            return cloned;
//        } catch (CloneNotSupportedException e) {
//            throw new AssertionError();
//        }
//    }
//}
//
//
////import com.qualcomm.robotcore.hardware.VoltageSensor;
////        private VoltageSensor myControlHubVoltageSensor;
////        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
//// presentVoltage = myControlHubVoltageSensor.getVoltage();
//
//// TODO test clamp and turnToFaceTarget in order to turn and move faster vvv
////    public void moveTowardTarget() {
////        double[] headingPowers = driveCorrection.calculateDrivePowers();
////        double[] drivePowers = HeadingCorrections.getWheelPowers(headingCorrection);
////        Drive.power(
////            Util.normalize(new double[]{ // make this a function in robot.drive TODO
////                Util.clampPower(headingPowers[0] + drivePowers[0]),
////                Util.clampPower(headingPowers[1] + drivePowers[1]),
////                Util.clampPower(headingPowers[2] + drivePowers[2]),
////                Util.clampPower(headingPowers[3] + drivePowers[3])
////            })
////        );
////    }
//// moves faster but not necessarily on the path/accurately
