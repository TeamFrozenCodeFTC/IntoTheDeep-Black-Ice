package org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blackIce.robot.wheelPowers.MecanumPowers;

public class MecanumDriveConstants {
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
    
    public MecanumDriveConstants() {}
    
    public MecanumDriveConstants frontLeft(String name, DcMotorSimple.Direction direction) {
        this.frontLeftName = name;
        this.frontLeftDirection = direction;
        return this;
    }
    public MecanumDriveConstants backLeft(String name, DcMotorSimple.Direction direction) {
        this.backLeftName = name;
        this.backLeftDirection = direction;
        return this;
    }
    public MecanumDriveConstants frontRight(String name, DcMotorSimple.Direction direction) {
        this.frontRightName = name;
        this.frontRightDirection = direction;
        return this;
    }
    public MecanumDriveConstants backRight(String name, DcMotorSimple.Direction direction) {
        this.backRightName = name;
        this.backRightDirection = direction;
        return this;
    }
    
    public MecanumDriveConstants maxForwardSpeed(double maxForwardSpeed) {
        this.maxForwardSpeed = maxForwardSpeed;
        return this;
    }
    public MecanumDriveConstants maxLateralSpeed(double maxLateralSpeed) {
        this.maxLateralSpeed = maxLateralSpeed;
        return this;
    }
}
