//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class Intake {
//    Robot op;
//
//    public Intake(Robot op) {
//        this.op = op;
//    }
//
//    public static final double ARM_MIN_POSITION = .45;
//    public static final double ARM_MAX_POSITION = .99;
//
//    public static final int MAX_TICKS = 1244;
//    public static final int MIN_TICKS = 0;
//
//    boolean extending = false;
//    boolean retracting = false;
//
//    public void retract() {
//        op.intakeExtender.setTargetPosition(MIN_TICKS);
//        op.intakeExtender.setPower(-1);
//
//        retracting = true;
//        extending = false;
//    }
//
//    public void fullyExtend() {
//        op.intakeExtender.setTargetPosition(MAX_TICKS);
//        op.intakeExtender.setPower(1);
//
//        retracting = false;
//        extending = true;
//    }
//
//    public void extendTo(int ticks) {
//        op.intakeExtender.setTargetPosition(ticks);
//        op.intakeExtender.setPower(1);
//
//        retracting = false;
//        extending = true;
//    }
//
//    public void loopUpdate() {
//        if (retracting && op.intakeExtender.getCurrentPosition() < 10) {
//            op.intakeExtender.setPower(0);
//        }
//
//        if (extending && op.intakeExtender.getCurrentPosition() > MAX_TICKS-10) {
//            op.intakeExtender.setPower(0);
//        }
//    }
//
//    public void armUp() {
//        op.sweeperRotator.setPosition(.65);
//    }
//
//    public void armDown() {
//        op.sweeperRotator.setPosition(0.29); // .3
//    }
//
//    public void armOut() {
//        op.sweeperRotator.setPosition(ARM_MAX_POSITION);
//    }
//
//    public void armIn() {
//        op.sweeperRotator.setPosition(ARM_MIN_POSITION);
//    }
//
//    public void armIn2() {
//        op.sweeperRotator.setPosition(.36);
//    }
//
//    public void restArm() {
//        op.sweeperRotator.getController().pwmDisable();
//    }
//
//    public void spinSweeperIn() {
//        op.sweeper.setPower(0.5);
//    }
//
//    public void spinSweeperBy(double power) {
//        op.sweeper.setPower(power);
//    }
//
//    public void spinSweeperOut() {
//        op.sweeper.setPower(-0.75);
//    }
//
//    public void stopSweeper() {
//        op.sweeper.setPower(0);
//    }
//}
