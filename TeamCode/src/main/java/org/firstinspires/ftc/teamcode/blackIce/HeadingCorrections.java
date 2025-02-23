package org.firstinspires.ftc.teamcode.blackIce;


import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;


public final class HeadingCorrections {
//    Robot robot;
//
//    public HeadingCorrections(Robot robot) {
//        this.robot = robot;
//    }

    private static final Robot robot = Robot.robot;

    public static HeadingCorrection turnOverMovement = () -> {
        double turnPower = Target.headingError * Constants.TurnCorrection.TURN_POWER;

        if (Target.distanceToTarget < 1 || Math.abs(Target.heading - Target.previousHeading) < 10) {
            return turnPower;
        }

        double progressFactor = Math.min(1, (Target.totalDistanceToTarget - Target.distanceToTarget) / Target.totalDistanceToTarget
            + Constants.TurnCorrection.FINISH_TURN_BY_PERCENT);
        return turnPower * progressFactor;
    };

    public static HeadingCorrection locked = () -> Target.headingError * Constants.TurnCorrection.TURN_POWER;

    public static HeadingCorrection turnToFaceTarget = () -> {
        double progressFactor = Math.min(1, (Target.totalDistanceToTarget - Target.distanceToTarget) / Target.totalDistanceToTarget);

        if (progressFactor > 0.80) {
            return HeadingCorrections.locked.calculateTurnPower();
        }

        double headingToFaceTarget = Math.toDegrees(Math.atan2(Target.yError, Target.xError));
        double headingError = Util.simplifyAngle(headingToFaceTarget - Odometry.heading);

        return headingError * Constants.TurnCorrection.TURN_POWER;
    };

    public static double[] getWheelPowers(HeadingCorrection headingCorrection) {
        return Drive.turnCounterclockwise(headingCorrection.calculateTurnPower());
    }
}