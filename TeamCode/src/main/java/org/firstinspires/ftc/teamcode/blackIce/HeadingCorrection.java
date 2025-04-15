package org.firstinspires.ftc.teamcode.blackIce;

public interface HeadingCorrection {
    double calculateTurnPower();

//    HeadingCorrection turnOverMovement = () -> {
//        double turnPower = Target.headingError * Constants.TurnCorrection.TURN_POWER;
//
//        double distanceToTarget = Math.sqrt(Math.pow(Target.xError, 2) + Math.pow(Target.yError, 2));
//        if (distanceToTarget < 1 || Math.abs(Target.heading - Target.previousHeading) < 10) {
//            return turnPower;
//        }
//
//        double progressFactor = Math.min(1, (Target.totalDistanceToTarget - distanceToTarget) / Target.totalDistanceToTarget
//            + Constants.TurnCorrection.FINISH_TURN_BY_PERCENT);
//        return turnPower * progressFactor;
//    };

    HeadingCorrection locked = () -> Target.headingError * Constants.TurnCorrection.TURN_POWER;

//    static double[] getWheelPowers(HeadingCorrection headingCorrection) {
//        return Drive.turnCounterclockwise(headingCorrection.calculateTurnPower());
//    }
}
