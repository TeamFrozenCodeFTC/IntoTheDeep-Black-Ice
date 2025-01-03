package org.firstinspires.ftc.teamcode;

public class Intake {
    Robot op;

    public Intake(Robot op) {
        this.op = op;
    }

    public static final double ARM_MIN_POSITION = .35;
    public static final double ARM_MAX_POSITION = 0.96;

    public static final int MAX_TICKS = 1500;
    public static final int MIN_TICKS = 0;

    private double targetTicks;

    public void stopExtender() {
        op.intakeExtender.setPower(0);
    }

    public void retract() {
        targetTicks = MIN_TICKS;
        op.intakeExtender.setTargetPosition(MIN_TICKS);
        op.intakeExtender.setPower(-1);

        new Thread(this::waitForExtension).start();
    }

    public void fullyExtend() {
        targetTicks = MAX_TICKS;
        op.intakeExtender.setTargetPosition(MAX_TICKS);
        op.intakeExtender.setPower(1);

        new Thread(this::waitForExtension).start();
    }

    public void waitForExtension() {
        while (op.intakeExtender.isBusy() || op.viperSlideMotor.getCurrentPosition() < targetTicks-10) {
            op.idle();
        }
        op.intakeExtender.setPower(0);
    }

    public void armUp() {
        op.sweeperRotator.setPosition(.7);
    }

    public void armDown() {
        op.sweeperRotator.setPosition(0.04);
    }

    public void armOut() {
        op.sweeperRotator.setPosition(ARM_MAX_POSITION);
    }

    public void armIn() {
        op.sweeperRotator.setPosition(ARM_MIN_POSITION);
    }

    public void spinSweeperIn() {
        op.sweeper.setPower(0.5);
    }

    public void spinSweeperOut() {
        op.sweeper.setPower(-1);
    }

    public void stopSweeper() {
        op.sweeper.setPower(0);
    }
}
