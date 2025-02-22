package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.BezierCurve;

public class MovementBuild {
    MovementBuilder builder;
    HeadingCorrection headingCorrection;
    DriveCorrection driveCorrection;
    MovementExit movementExit;

    boolean zeroPowerBrake = true;
    double maxPower = 1;
    double maxVelocity = 100; // 100 inch/second TODO
    double maxHeadingVelocity = 999; // 999 degrees/second TODO

    BezierCurve curve = null;

    public MovementBuild(MovementBuilder builder, double x, double y, double heading) {
        this.builder = builder;

        builder.target.setTarget(heading, x, y); // does this fix quirk?
        builder.robot.loopUpdate();

        setMovementExit(() -> !builder.target.isNotWithinErrorMargin(builder.target.defaultErrorMargin));
    }

    public MovementBuild(MovementBuilder builder, BezierCurve curve) {
        this.builder = builder;
        this.curve = curve;

//        builder.target.setTarget(heading, x, y); // does this fix quirk?
//        builder.robot.loopUpdate();

        setMovementExit(() -> !builder.target.isNotWithinErrorMargin(builder.target.defaultErrorMargin));
    }

    public MovementBuild zeroPowerBrake() {
        zeroPowerBrake = true;
        return this;
    }

    public MovementBuild zeroPowerFloat() {
        zeroPowerBrake = false;
        return this;
    }

    public MovementBuild setMaxVelocity(double newMaxVelocity) {
        maxVelocity = newMaxVelocity;
        return this;
    }

    public MovementBuild setMaxHeadingVelocity(double newMaxHeadingVelocity) {
        maxHeadingVelocity = newMaxHeadingVelocity;
        return this;
    }

    public MovementBuild setHeadingCorrection(HeadingCorrection newHeadingCorrection) {
        headingCorrection = newHeadingCorrection;
        return this;
    }

    public MovementBuild setDriveCorrection(DriveCorrection newDriveCorrection) {
        driveCorrection = newDriveCorrection;
        return this;
    }

    // opposite of exit. keeps the loop running
    public MovementBuild setMovementExit(MovementExit newMovementExit) {
        movementExit = newMovementExit;
        return this;
    }

    public MovementBuild setMaxPower(double newMaxPower) {
        maxPower = newMaxPower;
        return this;
    }

    public boolean isBraking() {
        return builder.robot.movement.target.xError < builder.robot.odometry.xBrakingDistance
           || builder.robot.movement.target.yError < builder.robot.odometry.yBrakingDistance;
    }

    public void moveTowardTarget() { // 55 - 50 = 5, 60 - 50 = 10, 50/55
        double velocityMult = (builder.robot.odometry.velocity > maxVelocity) ? (maxVelocity / builder.robot.odometry.velocity) * 0.5 : 1;
        double headingMult = (builder.robot.odometry.headingVelocity > maxHeadingVelocity) ? (maxHeadingVelocity / builder.robot.odometry.headingVelocity) : 1;

//import com.qualcomm.robotcore.hardware.VoltageSensor;
//        private VoltageSensor myControlHubVoltageSensor;
//        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // presentVoltage = myControlHubVoltageSensor.getVoltage();

        builder.robot.drive.power(builder.robot.drive.combineMax(
            builder.robot.drive.multiply(driveCorrection.calculateDrivePowers(), velocityMult),
            builder.robot.drive.multiply(builder.headingCorrections.getWheelPowers(headingCorrection), headingMult),
            maxPower
        ));
    }

    // TODO test clamp and turnToFaceTarget in order to turn and move faster vvv
//    public void moveTowardTarget_Clamped() {
//        double[] headingPowers = builder.driveCorrections.getWheelPowers(driveCorrection);
//        double[] drivePowers = builder.headingCorrections.getWheelPowers(headingCorrection);
//        builder.robot.drive.power(
//            Util.normalize(new double[]{ // make this a function in robot.drive TODO
//                Util.clampPower(headingPowers[0] + drivePowers[0]),
//                Util.clampPower(headingPowers[1] + drivePowers[1]),
//                Util.clampPower(headingPowers[2] + drivePowers[2]),
//                Util.clampPower(headingPowers[3] + drivePowers[3])
//            })
//        );
//    }

    // moveThrough = movePast

    public MovementBuild moveThrough() {
        return this
            .setHeadingCorrection(builder.headingCorrections.turnOverMovement)
            .setDriveCorrection(builder.driveCorrections.proportional)
            .setMovementExit(() -> {
                boolean pastY;
                boolean pastX;

                if (builder.target.previousY < builder.target.y) {
                    pastY = builder.robot.odometry.y > builder.target.y - builder.robot.odometry.yBrakingDistance;
                }
                else if (builder.target.previousY == builder.target.y) {
                    pastY = true;
                }
                else {
                    pastY = builder.robot.odometry.y < builder.target.y - builder.robot.odometry.yBrakingDistance;
                }

                if (builder.target.previousX < builder.target.x) {
                    pastX = builder.robot.odometry.x > builder.target.x - builder.robot.odometry.xBrakingDistance;
                }
                else if (builder.target.previousX == builder.target.x) {
                    pastX = true;
                }
                else {
                    pastX = builder.robot.odometry.x < builder.target.x - builder.robot.odometry.xBrakingDistance;
                }
                return pastY && pastX;

                // have parallel to previous point
            });
    }

    public MovementBuild moveThrough2() {
        return this
            .setHeadingCorrection(builder.headingCorrections.turnOverMovement)
            .setDriveCorrection(builder.driveCorrections.proportional)
            .setMovementExit(() -> {
                builder.robot.telemetry.addData("previousX", builder.target.previousX);
                builder.robot.telemetry.addData("target x", builder.target.x);
                builder.robot.telemetry.update();

                double x = builder.robot.odometry.x + builder.robot.odometry.xBrakingDistance;
                double y = builder.robot.odometry.y + builder.robot.odometry.yBrakingDistance;

                if (builder.target.x == builder.target.previousX) {
                    if (builder.target.previousY < builder.target.y) {
                        return builder.robot.odometry.y > builder.target.y;
                    }
                    else {
                        return builder.robot.odometry.y < builder.target.y;
                    }
                }
//                movement.moveThrough2(90, 0, 16);
//                movement.moveThrough2(90, 24, 16);
                double slope = (builder.target.y - builder.target.previousY) / (builder.target.x - builder.target.previousX);

                if (builder.target.x > builder.target.previousX) {
                    return -x <= slope * (y - builder.target.y) - builder.target.x;
                }
                if (builder.target.x < builder.target.previousX) {
                    return x <= slope * (-y + builder.target.y) + builder.target.x;
                }

                // x < slope * (-y+builder.target.y) + builder.target.x
                // -x < slope * (y-builder.target.y) - builder.target.x

                // have parallel to previous point
                return true; // error
            });
    }

    public MovementBuild moveTo(double brakePercent) {
        return this
            .setHeadingCorrection(builder.headingCorrections.turnOverMovement)
            .setDriveCorrection(() -> new double[]{
                builder.robot.movement.target.xError - builder.robot.odometry.xBrakingDistance * brakePercent,
                builder.robot.movement.target.yError - builder.robot.odometry.yBrakingDistance * brakePercent,
            })
            .setMovementExit(() ->
                builder.target.isWithinBrakingErrorMargin(builder.target.defaultErrorMargin));
    }

    public MovementBuild stopAtCustom() {
        return this
            .setHeadingCorrection(builder.headingCorrections.turnOverMovement)
            .setDriveCorrection(() -> {
                if (builder.target.isWithinBrakingErrorMargin(builder.target.defaultErrorMargin)) {
                    return new double[]{
                        builder.robot.movement.target.xError - builder.robot.odometry.xBrakingDistance,
                        builder.robot.movement.target.yError - builder.robot.odometry.yBrakingDistance,
                    };
                } else {
                    return new double[]{
                        builder.robot.movement.target.xError,
                        builder.robot.movement.target.yError,
                    };
                }
            });
    }

    public MovementBuild stopAtPosition() {
        return this
            .setHeadingCorrection(builder.headingCorrections.turnOverMovement)
            .setDriveCorrection(builder.driveCorrections.stopAtTarget);
    }

    public MovementBuild turnAndMoveThrough() {
        return this
            .moveThrough()
            .setHeadingCorrection(builder.headingCorrections.locked);
    }

    // MAKE SURE YOU RUN YOUR MOVEMENT
    public void run() {
        runTimeout(5);
    }

    public void runTimeout(double timeout) {
        if (curve != null) {
            curve.run(builder.robot);
            return;
        }

        if (zeroPowerBrake) {
            builder.robot.drive.zeroPowerBrake();
        }
        else {
            builder.robot.drive.zeroPowerFloat();
        }

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        builder.robot.loopUpdate();
        while (
            builder.robot.isNotInterrupted()
            && !movementExit.condition()
            && timer.seconds() < timeout
        ) {
            moveTowardTarget();

//            builder.robot.telemetry.addData("VEL", builder.robot.odometry.velocity);
//            builder.robot.telemetry.addData("x pos", builder.robot.odometry.x);
//            builder.robot.telemetry.addData("y pos", builder.robot.odometry.y);
//            builder.robot.telemetry.addData("x braking distance", builder.robot.odometry.xBrakingDistance);
//            builder.robot.telemetry.addData("y braking distance", builder.robot.odometry.yBrakingDistance);
//            builder.robot.telemetry.addData("x vel", builder.robot.odometry.xVelocity);
//            builder.robot.telemetry.addData("y vel", builder.robot.odometry.yVelocity);
            builder.robot.telemetry.update();

            builder.robot.loopUpdate();
        }

        //builder.robot.drive.zeroPower();
    }
}
