package org.firstinspires.ftc.teamcode.blackIce;

import static org.firstinspires.ftc.teamcode.Robot.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.blackIce.paths.Path;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

public class Movement {
    public static void moveThrough(double heading, double x, double y) {
        new Movement(x, y, heading).moveThrough().run();
    }
    public static void moveTo(double heading, double x, double y, double brakingPercent) {
        new Movement(x, y, heading).moveTo(brakingPercent).run();
    }
    public static void stopAtPosition(double heading, double x, double y) {
        new Movement(x, y, heading).stopAtPosition().run();
    }
    public static void turnAndMoveThrough(double heading, double x, double y) {
        new Movement(x, y, heading).turnAndMoveThrough().run();
    }

    HeadingCorrection headingCorrection;
    DriveCorrection driveCorrection;
    MovementExit movementExit;

    boolean zeroPowerBrake = true;
    double maxPower = 1;
    double maxVelocity = 100; // 100 inch/second
    double maxHeadingVelocity = 999; // 999 degrees/second
    boolean brakeAfter = true;
    // have these ^^ in  a different class that MovementBuild and Curve inherit from
    // or

    public Path curve = null;

    public Movement(double x, double y, double heading) {
        Target.setTarget(heading, x, y); // does this fix quirk?

        robot.loopUpdate();

        setMovementExit(() -> !Target.isNotWithinErrorMargin(Target.defaultErrorMargin));
    }

    public Movement(Path curve) {
        this.curve = curve;
    }

    public Movement() { // for Curve
    }

    public Movement zeroPowerBrake() {
        zeroPowerBrake = true;
        return this;
    }

    public Movement zeroPowerFloat() {
        zeroPowerBrake = false;
        return this;
    }

    public Movement brakeAfter() {
        brakeAfter = true;
        return this;
    }

    public Movement noBrakeAfter() {
        brakeAfter = false;
        return this;
    }

    public Movement setMaxVelocity(double newMaxVelocity) {
        maxVelocity = newMaxVelocity;
        return this;
    }

    public Movement setMaxHeadingVelocity(double newMaxHeadingVelocity) {
        maxHeadingVelocity = newMaxHeadingVelocity;
        return this;
    }

    public Movement setHeadingCorrection(HeadingCorrection newHeadingCorrection) {
        headingCorrection = newHeadingCorrection;
        return this;
    }

    public Movement setDriveCorrection(DriveCorrection newDriveCorrection) {
        driveCorrection = newDriveCorrection;
        return this;
    }

    public Movement setMovementExit(MovementExit newMovementExit) {
        movementExit = newMovementExit;
        return this;
    }

    public Movement setMaxPower(double newMaxPower) {
        maxPower = newMaxPower;
        return this;
    }

    public void moveTowardTarget() { // 55 - 50 = 5, 60 - 50 = 10, 50/55
        double velocityMult = (Odometry.velocity > maxVelocity) ? (maxVelocity / Odometry.velocity) * 0.5 : 1;
        double headingMult = (Odometry.headingVelocity > maxHeadingVelocity) ? (maxHeadingVelocity / Odometry.headingVelocity) : 1;

//import com.qualcomm.robotcore.hardware.VoltageSensor;
//        private VoltageSensor myControlHubVoltageSensor;
//        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // presentVoltage = myControlHubVoltageSensor.getVoltage();

        Drive.power(Drive.combineMax(
            Drive.multiply(driveCorrection.calculateDrivePowers(), velocityMult),
            Drive.multiply(HeadingCorrections.getWheelPowers(headingCorrection), headingMult),
            maxPower
        ));
    }

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

    public Movement moveThrough() {
        double targetYError = Target.previousY - Target.y;
        double targetXError = Target.previousX - Target.x;
        double xSign = Math.signum(targetXError);

        return this
            .setHeadingCorrection(HeadingCorrections.locked)
            .setDriveCorrection(DriveCorrections.proportional)
            .setMovementExit(() -> {
                robot.telemetry.addData("previousX", Target.previousX);
                robot.telemetry.addData("target x", Target.x);
                robot.telemetry.update();

//                double x = Odometry.x + Odometry.xBrakingDistance;
//                double y = Odometry.y + Odometry.yBrakingDistance;

                double yErrorBraking = Target.yError - Odometry.yBrakingDistance;
                double xErrorBraking = Target.xError - Odometry.xBrakingDistance;

                // TODO simplify math
                // efficient and effective
                // advanced path follower
                // simple and effective
                // weakness - high expectations

                if (Target.x == Target.previousX) {
//                    if (Target.previousY < Target.y) {
//                        return Odometry.y > Target.y;
//                    }
//                    else {
//                        return Odometry.y < Target.y;
//                    }
//                   return Math.signum(Target.y - Target.previousY)
//                       * (Odometry.y + Odometry.yBrakingDistance - Target.y) > 0;
                    return Math.signum(targetYError) * (yErrorBraking) > 0;
                }
                // (o - t) = (-t + o)
               // double slope = (Target.y - Target.previousY) / (Target.x - Target.previousX);

                double slope = (targetYError) / (targetXError);

                return -xSign * xErrorBraking
                    <= slope * xSign * yErrorBraking;

//                return Math.signum(targetXError) * xErrorBraking
//                    <= slope * Math.signum(targetXError) * yErrorBraking;

//                if (Target.x > Target.previousX) {
//                    return -x <= slope * (y - Target.y) - Target.x;
//                }
//                if (Target.x < Target.previousX) { // (Target.x < Target.previousX)
//                    return x <= slope * (-y + Target.y) + Target.x;
//                }
////
//                return true; // should never happen
            });
    }

    public Movement moveTo(double brakePercent) {
        return this
            .setHeadingCorrection(HeadingCorrections.turnOverMovement)
            .setDriveCorrection(() -> new double[]{
                Target.xError - Odometry.xBrakingDistance * brakePercent,
                Target.yError - Odometry.yBrakingDistance * brakePercent,
            })
            .setMovementExit(() ->
                Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin));
    }

    public Movement stopAtCustom() {
        return this
            .setHeadingCorrection(HeadingCorrections.turnOverMovement)
            .setDriveCorrection(() -> {
                if (Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin)) {
                    return new double[]{
                        Target.xError - Odometry.xBrakingDistance,
                        Target.yError - Odometry.yBrakingDistance,
                    };
                } else {
                    return new double[]{
                        Target.xError,
                        Target.yError,
                    };
                }
            });
    }
// prevents overshooting
// starts braking at the most optimal point

    public Movement stopAtPosition() {
        return this
            .setMovementExit(() ->
                Target.isWithinBrakingErrorMargin(Target.defaultErrorMargin) && Odometry.velocity < 1)
            .setHeadingCorrection(HeadingCorrections.turnOverMovement)
            .setDriveCorrection(DriveCorrections.stopAtTarget);
    }

    public Movement turnAndMoveThrough() {
        return this
            .moveThrough()
            .setHeadingCorrection(HeadingCorrections.locked);
    }

    // MAKE SURE YOU RUN YOUR MOVEMENT
    public void run() {
        runTimeout(5);
    }

    public void runTimeout(double timeout) {
        robot.telemetry.addData("curve", curve);
        robot.telemetry.update();
        if (curve != null) {
            curve.runCurve(this);
            return;
        }

        if (zeroPowerBrake) {
            Drive.zeroPowerBrake();
        }
        else {
            Drive.zeroPowerFloat();
        }

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        robot.loopUpdate();
        while (
            robot.isNotInterrupted()
            && !movementExit.condition()
            && timer.seconds() < timeout
        ) {
            moveTowardTarget();

            //robot.telemetry.addData("VEL", Odometry.velocity);
            robot.telemetry.addData("x pos", Odometry.x);
            robot.telemetry.addData("y pos", Odometry.y);
            robot.telemetry.addData("x braking distance", Odometry.xBrakingDistance);
            robot.telemetry.addData("y braking distance", Odometry.yBrakingDistance);
//            robot.telemetry.addData("x vel", builder.robot.odometry.xVelocity);
//            robot.telemetry.addData("y vel", builder.robot.odometry.yVelocity);
            robot.telemetry.update();

            robot.loopUpdate();
        }

        if (brakeAfter) {
            Drive.zeroPower();
        }
    }
}
