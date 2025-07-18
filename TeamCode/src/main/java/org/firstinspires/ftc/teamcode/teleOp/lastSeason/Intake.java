package org.firstinspires.ftc.teamcode.teleOp.lastSeason;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intakeExtender;
    public Servo sweeperRotator;
    public DcMotor sweeper;
    
    public Intake(HardwareMap map) {
        intakeExtender = map.get(DcMotor.class, "intakeMotor");
        intakeExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtender.setTargetPosition(0);
        intakeExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtender.setDirection(DcMotorSimple.Direction.REVERSE);
        
        sweeperRotator = map.get(Servo.class, "sweeperRotator");
        sweeper = map.get(DcMotor.class, "sweeper");
    }

    public static final double ARM_MIN_POSITION = .45;
    public static final double ARM_MAX_POSITION = .99;

    public static final int MAX_TICKS = 1244;
    public static final int MIN_TICKS = 0;

    boolean extending = false;
    boolean retracting = false;

    public void retract() {
        intakeExtender.setTargetPosition(MIN_TICKS);
        intakeExtender.setPower(-1);

        retracting = true;
        extending = false;
    }

    public void fullyExtend() {
        intakeExtender.setTargetPosition(MAX_TICKS);
        intakeExtender.setPower(1);

        retracting = false;
        extending = true;
    }

    public void extendTo(int ticks) {
        intakeExtender.setTargetPosition(ticks);
        intakeExtender.setPower(1);

        retracting = false;
        extending = true;
    }

    public void loopUpdate() {
        if (retracting && intakeExtender.getCurrentPosition() < 10) {
            intakeExtender.setPower(0);
        }

        if (extending && intakeExtender.getCurrentPosition() > MAX_TICKS-10) {
            intakeExtender.setPower(0);
        }
    }

    public void armUp() {
        sweeperRotator.setPosition(.65);
    }

    public void armDown() {
        sweeperRotator.setPosition(0.29);
    }

    public void armOut() {
        sweeperRotator.setPosition(ARM_MAX_POSITION);
    }

    public void armIn() {
        sweeperRotator.setPosition(ARM_MIN_POSITION);
    }

    public void armIn2() {
        sweeperRotator.setPosition(.36);
    }

    public void restArm() {
        sweeperRotator.getController().pwmDisable();
    }

    public void spinSweeperIn() {
        sweeper.setPower(0.5);
    }

    public void spinSweeperBy(double power) {
        sweeper.setPower(power);
    }

    public void spinSweeperOut() {
        sweeper.setPower(-0.75);
    }

    public void stopSweeper() {
        sweeper.setPower(0);
    }
}
