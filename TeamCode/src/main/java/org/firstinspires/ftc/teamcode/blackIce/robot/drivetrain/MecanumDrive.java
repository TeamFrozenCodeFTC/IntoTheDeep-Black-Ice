package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers.MecanumPowers;

public class MecanumDrive extends Drivetrain {
    WheelPowersProvider<MecanumPowers> brakingPowersProvider;
    WheelPowersProvider<MecanumPowers> acceleratingPowersProvider;
    public final double strafingEffortMultiplier;

    // remove mecanum config and just have mecanum drive
    public MecanumDrive(HardwareMap map, MecanumDriveConfig config) {
        super(initMotors(map, config));
        this.acceleratingPowersProvider = config.acceleratingPowersProvider;
        this.brakingPowersProvider = config.brakingPowersProvider;
        this.strafingEffortMultiplier = config.maxForwardSpeed / config.maxLateralSpeed;
    }
    
    @Override
    public Vector adjustDirectionalEffort(Vector inputEffort) {
        return new Vector(
            inputEffort.getX(),
            inputEffort.getY() * this.strafingEffortMultiplier
        );
    }
    
    @Override
    public void applyBrakingPowers(Vector robotVector, double turningPower) {
        applyPowers(brakingPowersProvider.getPowers(robotVector, turningPower));
    }
    @Override
    public void driveTowards(Vector robotVector, double turningPower) {
        applyPowers(acceleratingPowersProvider.getPowers(robotVector, turningPower));
    }
    
    public void applyPowers(MecanumPowers powers) {
        for (int i = 0; i < powers.getPowers().length; i++) {
            motors[i].setPower(powers.getPowers()[i]);
        }
    }
    
    private static DcMotorEx[] initMotors(HardwareMap map, MecanumDriveConfig config) {
        // TODO test performance difference between DcMotor and DcMotorEx
        DcMotorEx frontLeft = map.get(DcMotorEx.class, config.frontLeftName);
        DcMotorEx backLeft = map.get(DcMotorEx.class, config.backLeftName);
        DcMotorEx frontRight = map.get(DcMotorEx.class, config.frontRightName);
        DcMotorEx backRight = map.get(DcMotorEx.class, config.backRightName);
        
        frontLeft.setDirection(config.frontLeftDirection);
        backLeft.setDirection(config.backLeftDirection);
        frontRight.setDirection(config.frontRightDirection);
        backRight.setDirection(config.backRightDirection);

        return new DcMotorEx[]{ frontLeft, backLeft, frontRight, backRight };
    }
}
