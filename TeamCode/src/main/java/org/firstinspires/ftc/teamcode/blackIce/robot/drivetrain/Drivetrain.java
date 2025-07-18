package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 *
 * Dev Note: Cannot have Drivetrain&lt;T&gt; behavior due to use of wildcard:
 * Drivetrain&lt;?&gt; d = new MecanumDrive();
 * d.applyPowers(d.getPowers()); // d.applyPowers cannot confirm d.getPowers is same type without
 * having the follower have a generic.
 */
public abstract class Drivetrain {
    public abstract void followVector(Vector robotVector, double turningPower);
    public abstract void applyBrakingPowers(Vector robotVector, double turningPower);
    
    public Vector adjustDirectionalEffort(Vector inputEffort) {
        return inputEffort;
    }

    public final DcMotorEx[] motors;
    
    protected Drivetrain(DcMotorEx[] motors) {
        this.motors = motors;
    }
    
    public void zeroPowerBrakeMode() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void zeroPower() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
    
    public void zeroPowerFloatMode() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void disableAll() {
        for (DcMotorEx motor : motors) {
            motor.setMotorDisable();
        }
    }
    public void enableAll() {
        for (DcMotorEx motor : motors) {
            motor.setMotorEnable();
        }
    }
    
    @FunctionalInterface
    public interface WheelPowersProvider<Powers> {
        Powers getPowers(Vector robotVector, double turningPower);
    }
}