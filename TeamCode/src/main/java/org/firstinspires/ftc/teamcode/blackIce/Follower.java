package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

// kye_anderson_2009, alexreman45
public abstract class Follower {
    HeadingCorrection headingCorrection;
    DriveCorrection driveCorrection;
    Condition movementExit;

    double maxPower = 1;
    double maxVelocity = 100; // inch/second
    double maxHeadingVelocity = 999; // degrees/second
    double consideredStoppedVelocity = 1;
    double timeoutSeconds = 5;
    private static LinearOpMode opMode;

    private final ElapsedTime timer = new ElapsedTime();

    boolean brakeAfter = true;
    boolean continuePowerAfter = true;

    public Path path = null;

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

    public static void init(LinearOpMode opMode) {
        Follower.opMode = opMode;
        Odometry.init(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);
    }

//    /**
//     * Run the movement with a timeout.
//     *
//     * @param timeout The timeout seconds.
//     */
//    public void runTimeout(double timeout) {
//        // Hacky
//        if (path != null) {
//            path.runCurve(this);
//            return;
//        }
//
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//
//        Follower.update();
//        while (
//            robot.isNotInterrupted()
//                && !movementExit.condition()
//                && timer.seconds() < timeout
//        ) {
//            moveTowardTarget();
//
//            //robot.telemetry.addData("VEL", Odometry.velocity);
//            robot.opMode.telemetry.addData("x pos", Odometry.x);
//            robot.opMode.telemetry.addData("y pos", Odometry.y);
////            robot.telemetry.addData("x braking distance", Odometry.xBrakingDistance);
////            robot.telemetry.addData("y braking distance", Odometry.yBrakingDistance);
////            robot.telemetry.addData("x vel", builder.robot.odometry.xVelocity);
////            robot.telemetry.addData("y vel", builder.robot.odometry.yVelocity);
//            robot.opMode.telemetry.update();
//
//            robot.loopUpdate();
//        }
//
//        if (brakeAfter) {
//            Drive.zeroPowerBrakeMode();
//        }
//        else {
//            Drive.zeroPowerFloatMode();
//        }
//
//        if (!continuePowerAfter) {
//            Drive.zeroPower();
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
