package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers.MecanumPowers;

public class MecanumDrivetrainConfig implements DrivetrainConfig {
    String frontLeftName, backLeftName, frontRightName, backRightName;
    DcMotorSimple.Direction frontLeftDirection, backLeftDirection,
                             frontRightDirection, backRightDirection;
    Drivetrain.WheelPowersProvider<MecanumPowers> brakingPowersProvider =
        MecanumPowers::prioritizeTurning;
    Drivetrain.WheelPowersProvider<MecanumPowers> acceleratingPowersProvider =
        MecanumPowers::simpleAdd;
    // it would be nice if this was separated ^^^
    
    public double maxForwardSpeed = 70;
    public double maxLateralSpeed = 60;
    
    public MecanumDrivetrainConfig() {}
    
    public MecanumDrivetrainConfig frontLeft(String name, DcMotorSimple.Direction direction) {
        this.frontLeftName = name;
        this.frontLeftDirection = direction;
        return this;
    }
    public MecanumDrivetrainConfig backLeft(String name, DcMotorSimple.Direction direction) {
        this.backLeftName = name;
        this.backLeftDirection = direction;
        return this;
    }
    public MecanumDrivetrainConfig frontRight(String name, DcMotorSimple.Direction direction) {
        this.frontRightName = name;
        this.frontRightDirection = direction;
        return this;
    }
    public MecanumDrivetrainConfig backRight(String name, DcMotorSimple.Direction direction) {
        this.backRightName = name;
        this.backRightDirection = direction;
        return this;
    }
    
    public MecanumDrivetrainConfig brakingPowersProvider(Drivetrain.WheelPowersProvider<MecanumPowers> provider) {
        this.brakingPowersProvider = provider;
        return this;
    }
    
    public MecanumDrivetrainConfig acceleratingPowersProvider(Drivetrain.WheelPowersProvider<MecanumPowers> provider) {
        this.acceleratingPowersProvider = provider;
        return this;
    }
}
