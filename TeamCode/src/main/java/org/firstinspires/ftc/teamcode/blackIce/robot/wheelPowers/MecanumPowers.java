package org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

@Deprecated
public class MecanumPowers extends WheelPowers<MecanumPowers> {
    public MecanumPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        super(frontLeft, backLeft, frontRight, backRight);
    }
    
    @Override
    protected MecanumPowers fromComponentArray(double[] components) {
        return new MecanumPowers(components[0], components[1], components[2], components[3]);
    }
    
    public static MecanumPowers backward(double power) {
        return forward(-power);
    }
    public static MecanumPowers turnCounterclockwise(double power) {
        return turnClockwise(-power);
    }
    public static MecanumPowers forward(double power) {
        return new MecanumPowers(power, power, power, power);
    }
    public static MecanumPowers slideLeft(double power) {
        return new MecanumPowers(power, -power, -power, power);
    }
    public static MecanumPowers slideRight(double power) {
        return slideLeft(-power);
    }
    public static MecanumPowers turnClockwise(double power) {
        return new MecanumPowers(power, -power, power, -power);
    }

    /**
     * Takes a robot-relative vector and a turningPower and converts it into wheel powers.
     * <p>
     * This adjusts for all the inconsistencies with the mecanum wheels such as it driving
     * faster forward than laterally.
     */
    public static MecanumPowers simpleAdd(Vector robotVector, double turnPower) {
        MecanumPowers transitionalPowers = fromRobotVector(robotVector);
        
        return transitionalPowers
            .add(MecanumPowers.turnCounterclockwise(turnPower))
            .downscaleMaxTo(1);
    }
    
    public static MecanumPowers prioritizeTransitional(Vector robotVector, double turnPower) {
        return fromRobotVector(robotVector)
            .addAlignedPowers(MecanumPowers.turnCounterclockwise(turnPower))
            .downscaleMaxTo(1);
    }
    
    public static MecanumPowers prioritizeTurning(Vector robotVector, double turnPower) {
        return MecanumPowers.turnCounterclockwise(turnPower)
            .addAlignedPowers(fromRobotVector(robotVector))
            .downscaleMaxTo(1);
    }

    public static MecanumPowers dynamicPrioritizing(Vector robotVector, double turnPower) {
        if (Math.abs(turnPower) < 0.5) {
            return prioritizeTurning(robotVector, turnPower);
        }
        return prioritizeTransitional(robotVector, turnPower);
    }

    private static MecanumPowers fromRobotVector(Vector robotVector) {
        Vector adjustedVector = Follower.getInstance().getDrivetrain().adjustDirectionalEffort(robotVector);
        
        double upRightDirection = -adjustedVector.getY() + adjustedVector.getX();
        double downLeftDirection = -adjustedVector.getY() - adjustedVector.getX();
        
        return new MecanumPowers(
            upRightDirection, downLeftDirection,
            downLeftDirection, upRightDirection
        );
    }
}
