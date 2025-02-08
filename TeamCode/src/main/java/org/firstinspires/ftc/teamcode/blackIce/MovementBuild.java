package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MovementBuild {
    MovementBuilder builder;
    HeadingCorrection headingCorrection;
    DriveCorrection driveCorrection;
    MovementExit movementExit;

    boolean zeroPowerBrake = true;
    double maxPower = 1;

    public MovementBuild(MovementBuilder builder, double x, double y, double heading) {
        this.builder = builder;

        builder.target.setTarget(heading, x, y);
        builder.robot.loopUpdate();

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

    public void moveTowardTarget() {
        builder.robot.drive.power(builder.robot.drive.combineMax(
            builder.driveCorrections.getWheelPowers(driveCorrection),
            builder.headingCorrections.getWheelPowers(headingCorrection),
            maxPower
        ));
    }

//    public void moveTowardTarget_Clamped() {
//        double[] headingPowers = builder.driveCorrections.getWheelPowers(driveCorrection);
//        double[] drivePowers = builder.headingCorrections.getWheelPowers(headingCorrection);
//        builder.robot.drive.power(
//            Util.normalize(new double[]{ // make this a function in robot.drive
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
//        .setMovementExit(() -> {
//            boolean pastY;
//            boolean pastX;
//            if (builder.target.previousY < builder.target.y) {
//                pastY = builder.robot.odometry.y < builder.target.y - builder.robot.odometry.yBrakingDistance;
//            }
//            else if (builder.target.previousY == builder.target.y) {
//                pastY = true;
//            }
//            else {
//                pastY = builder.robot.odometry.y > builder.target.y - builder.robot.odometry.yBrakingDistance;
//            }
//
//            if (builder.target.previousX < builder.target.x) {
//                pastX = builder.robot.odometry.x < builder.target.x - builder.robot.odometry.xBrakingDistance;
//            }
//            else if (builder.target.previousX == builder.target.x) {
//                pastX = true;
//            }
//            else {
//                pastX = builder.robot.odometry.x > builder.target.x - builder.robot.odometry.xBrakingDistance;
//            }
//            return !(pastY && pastX);
//        });
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
        runTimeout(7);
    }

    public void runTimeout(double timeout) {
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

            builder.robot.telemetry.addData("x pos", builder.robot.odometry.x);
            builder.robot.telemetry.addData("y pos", builder.robot.odometry.y);
            builder.robot.telemetry.addData("x braking distance", builder.robot.odometry.xBrakingDistance);
            builder.robot.telemetry.addData("y braking distance", builder.robot.odometry.yBrakingDistance);
            builder.robot.telemetry.addData("x vel", builder.robot.odometry.xVelocity);
            builder.robot.telemetry.addData("y vel", builder.robot.odometry.yVelocity);
            builder.robot.telemetry.update();

            builder.robot.loopUpdate();
        }

        builder.robot.drive.zeroPower();
    }
}
