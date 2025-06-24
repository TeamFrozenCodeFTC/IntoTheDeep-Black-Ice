package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 *
 * Dev Note: Cannot have Drivetrain&lt;T&gt; behavior due to use of wildcard:
 * Drivetrain&lt;?&gt; d = new MecanumDrive();
 * d.applyPowers(d.getPowers()); // d.applyPowers cannot confirm d.getPowers is same type without
 * having the follower have a generic.
 */
public abstract class Drivetrain {
    /**
     * A type of drivetrain that produces a new instance of Drivetrain given a HardwareMap and
     * configuration.
     */
    @FunctionalInterface
    public interface Factory {
        Drivetrain create(HardwareMap hardwareMap, DrivetrainConfig config);
    }
    
    /**
     * The Mecanum drivetrain type which uses specially angled wheels to enable omnidirectional
     * movement, allowing a robot to move forward, backward, sideways, and rotate—all independently.
     * Best for following paths.
     */
    public static final Factory MECANUM =
        (hardwareMap, config) -> new MecanumDrive(hardwareMap, (MecanumDrivetrainConfig) config);
    
    public abstract void driveTowards(Vector robotVector, double turningPower);
    public abstract void applyBrakingPowers(Vector robotVector, double turningPower);
    
    public Vector adjustDirectionalEffort(Vector inputEffort) {
        return inputEffort;
    }

    protected final DcMotorEx[] motors;
    
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