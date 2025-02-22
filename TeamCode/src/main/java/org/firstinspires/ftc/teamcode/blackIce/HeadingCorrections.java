package org.firstinspires.ftc.teamcode.blackIce;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Util;


public class HeadingCorrections {
    Robot robot;

    public HeadingCorrections(Robot robot) {
        this.robot = robot;
    }

    private double turnOverMovement() {
        double turnPower = robot.movement.target.headingError * Constants.TurnCorrection.TURN_POWER;

        if (robot.movement.target.distanceToTarget < 1 || Math.abs(robot.movement.target.heading - robot.movement.target.previousHeading) < 10) {
            return turnPower;
        }

        double progressFactor = Math.min(1, (robot.movement.target.totalDistanceToTarget - robot.movement.target.distanceToTarget) / robot.movement.target.totalDistanceToTarget
                        + Constants.TurnCorrection.FINISH_TURN_BY_PERCENT);
        return turnPower * progressFactor;
    }
    public HeadingCorrection turnOverMovement = this::turnOverMovement;

    private double locked() {
        return robot.movement.target.headingError * Constants.TurnCorrection.TURN_POWER;
    }
    public HeadingCorrection locked = this::locked;

    private double turnToFaceTarget() {
        double progressFactor = Math.min(1, (robot.movement.target.totalDistanceToTarget - robot.movement.target.distanceToTarget) / robot.movement.target.totalDistanceToTarget);

        if (progressFactor > 0.80) {
            return locked();
        }

        double headingToFaceTarget = Math.toDegrees(Math.atan2(robot.movement.target.yError, robot.movement.target.xError));
        double headingError = Util.simplifyAngle(headingToFaceTarget - robot.movement.robot.odometry.heading);

        return headingError * Constants.TurnCorrection.TURN_POWER;
    }
    public HeadingCorrection turnToFaceTarget = this::turnToFaceTarget;

    public double[] getWheelPowers(HeadingCorrection headingCorrection) {
        return robot.drive.turnCounterclockwise(headingCorrection.calculateTurnPower());
    }
}