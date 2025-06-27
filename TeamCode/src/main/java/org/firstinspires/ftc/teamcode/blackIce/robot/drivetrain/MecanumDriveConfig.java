package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers.MecanumPowers;

public class MecanumDriveConfig {
    String frontLeftName, backLeftName, frontRightName, backRightName;
    DcMotorSimple.Direction frontLeftDirection, backLeftDirection,
                             frontRightDirection, backRightDirection;
    Drivetrain.WheelPowersProvider<MecanumPowers> brakingPowersProvider =
        MecanumPowers::prioritizeTurning;
    Drivetrain.WheelPowersProvider<MecanumPowers> acceleratingPowersProvider =
        MecanumPowers::simpleAdd;
    // it would be nice if this was separated ^^^
    
    double maxForwardSpeed;
    double maxLateralSpeed;
    
    public MecanumDriveConfig() {}
    
    public MecanumDriveConfig frontLeft(String name, DcMotorSimple.Direction direction) {
        this.frontLeftName = name;
        this.frontLeftDirection = direction;
        return this;
    }
    public MecanumDriveConfig backLeft(String name, DcMotorSimple.Direction direction) {
        this.backLeftName = name;
        this.backLeftDirection = direction;
        return this;
    }
    public MecanumDriveConfig frontRight(String name, DcMotorSimple.Direction direction) {
        this.frontRightName = name;
        this.frontRightDirection = direction;
        return this;
    }
    public MecanumDriveConfig backRight(String name, DcMotorSimple.Direction direction) {
        this.backRightName = name;
        this.backRightDirection = direction;
        return this;
    }
    
    public MecanumDriveConfig maxForwardSpeed(double maxForwardSpeed) {
        this.maxForwardSpeed = maxForwardSpeed;
        return this;
    }
    public MecanumDriveConfig maxLateralSpeed(double maxLateralSpeed) {
        this.maxLateralSpeed = maxLateralSpeed;
        return this;
    }
}
