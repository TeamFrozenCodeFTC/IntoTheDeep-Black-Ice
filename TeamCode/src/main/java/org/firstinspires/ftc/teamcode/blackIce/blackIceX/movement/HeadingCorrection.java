package org.firstinspires.ftc.teamcode.blackIce.blackIceX.movement;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Util;

public class HeadingCorrection {
    Movement movement;

    public HeadingCorrection(Movement movement) {
        this.movement = movement;
    }

    public double power;

    public double turnOverMovement() {
        double turnPower = movement.target.headingError * Constants.TurnCorrection.TURN_POWER;
        double progressFactor = Math.min(1,
                (movement.target.totalDistanceToTarget - movement.target.distanceToTarget) / movement.target.totalDistanceToTarget
                        + Constants.TurnCorrection.FINISH_TURN_BY_PERCENT);
        return turnPower * progressFactor;
    }

    public double locked() {
        return movement.target.headingError * Constants.TurnCorrection.TURN_POWER;
    }

//    public double turnToFaceTarget() {
//        // !
//
//        double degrees = Math.toDegrees(Math.atan2(movement.target.yError, movement.target.xError)) - movement.target.targetHeading + 90;
//        return Util.simplifyAngle(degrees - movement.robot.odometry.heading) * 0.015;
//    }

    public double[] applyTurnCorrection(double[] powers, double turnCorrection) {
//        return Util.normalize(new double[] {
//                Util.clampPower(-turnCorrection + powers[0]),
//                Util.clampPower(-turnCorrection + powers[1]),
//                Util.clampPower(+turnCorrection + powers[2]),
//                Util.clampPower(+turnCorrection + powers[3]),
//        }); // why clamp power?
        return movement.robot.drive.combine(powers, movement.robot.drive.turnCounterclockwise(turnCorrection));
    }
}