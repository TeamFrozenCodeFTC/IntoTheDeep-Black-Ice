package org.firstinspires.ftc.teamcode.blackIce.movement;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.Condition;
import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.Target;

/**
 * Handles Point to Point Movements
 */
public abstract class MovementBuild extends BaseMovementBuild<MovementBuild> {
    private final double x;
    private final double y;
    private double heading;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Create a new movement. Can be built upon to add more functionality and customization.
     * Don't forget to call {@code .run()} after you construct your movements.
     *
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
     * @param heading optional double
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

    protected boolean holdEndPosition = false; // OR USE A CONDITION

    private void update() {
        Target.updatePosition();
        finished = !Follower.opMode.opModeIsActive() // TODO more conditions ^^ holdEnd!!
            || isAtGoal.condition()
            || timer.seconds() >= timeoutSeconds;

        if (finished) {// && !holdEndPosition
            return;
        }

        moveTowardTarget();
    }

    public void start() {
        if (keepPreviousHeading) {
            Target.setTarget(x, y);
        }
        else {
            Target.setTarget(heading, x, y);
        }
        timer.reset();

        finished = isAtGoal.condition();
//        finished = !Follower.opMode.opModeIsActive() // TODO more conditions ^^ holdEnd
//            || isAtGoal.condition()
//            || timer.seconds() >= timeoutSeconds;
    }

    public Condition isAtGoal;

    protected boolean finished = false;

    private boolean isFinished() {
        return finished;
    }


    //    /**
//     * Copy the properties of another MovementBuild.
//     * <p>
//     * Note: only copies point-to-point properties, not path properties.
//     */
//    public MovementBuild copyProperties(MovementBuild properties) {
//        MovementBuild this_ = getThis();
//        this_.brakeAfter = properties.brakeAfter;
//        this_.continuePowerAfter = properties.continuePowerAfter;
//        return super.copyProperties(properties);
//    }

//    /**
//     * <h5>To fix underline, add arguments</h5>
//     * <p>
//     * Creates an empty {@link MovementBuild}.
//     */
//    protected MovementBuild(Class<? extends MovementBuild> aClass) {
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
//TODO for StopAtPosition
//protected double consideredStoppedVelocity = 1;
//
///**
// * Set the maximum velocity that the robot considers at rest.
// *
// * @param newConsideredStoppedVelocity (inches/second).
// * A lower value will make the robot take more time to stop more accurately.
// * A higher value will make the robot slow down less and carry more of its momentum.
// * Should be not less than 0.01 inches/second.
// * <p>
// * If you don't want the robot to slow down use {@link MoveThrough}
// */
//public MovementBuild setConsideredStoppedVelocity(double newConsideredStoppedVelocity) {
//    consideredStoppedVelocity = newConsideredStoppedVelocity;
//    return getThis();
//}