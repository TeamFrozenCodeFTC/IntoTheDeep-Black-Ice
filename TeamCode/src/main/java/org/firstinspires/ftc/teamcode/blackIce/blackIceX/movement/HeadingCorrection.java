package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;


import org.firstinspires.ftc.teamcode.util.Util;

public abstract class HeadingCorrection extends TargetTracker {
    public double power;

    public double turnOverMovement() {
        double turnPower = headingError * Constants.TurnCorrection.TURN_POWER;
        double progressFactor = Math.min(1,
                (totalDistanceToTarget - distanceToTarget) / totalDistanceToTarget
                        + Constants.TurnCorrection.FINISH_TURN_BY_PERCENT);
        return turnPower * progressFactor;
    }

    public double turnToFaceTarget() {
        // !
        double degrees = Math.toDegrees(Math.atan2(yError, xError)) - targetHeading + 90;
        return Util.simplifyAngle(degrees - odometry.heading) * 0.015;
    }

    public double[] applyTurnCorrection(double[] powers) {
        double turnCorrection = turnOverMovement();

        return Util.normalize(new double[] {
                Util.clampPower(-turnCorrection + powers[0]),
                Util.clampPower(-turnCorrection + powers[1]),
                Util.clampPower(+turnCorrection + powers[2]),
                Util.clampPower(+turnCorrection + powers[3]),
        });
    }
}